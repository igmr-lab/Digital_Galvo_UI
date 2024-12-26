//Red Wire = 3V3
//Black Wire = Ground
//Orange Wire = SCLK
//Yellow Wire = MOSI
//Green Wire = MISO
//Brown Wire = CS

#include <chrono>
#include <vector>
#include <random>
#include <climits>
#include <algorithm>
#include <functional>

#include <conio.h> // For getch() and kbhit()
#include <io.h>     // For _setmode()
#include <fcntl.h>  // For _O_BINARY

#include <stdint.h>
#include <iostream>
#include <stdio.h>
#include <Windows.h>
#include <time.h>
#include <math.h>
#include "ftd2xx.h"
#include "libmpsse_spi.h"
#include "STM32.h"

// For UART
HANDLE hComm;

// For printing out error message 
void print_and_quit(const char cstring[]) {
    printf("%s\n", cstring);
    getc(stdin);
    exit(1);
}

// Initialize UART 
void Init_UART() {
    BOOL status;
    DCB dcbSerialParams = { 0 };
    COMMTIMEOUTS timeouts = { 0 };

    // Check the Port that is connected to STM32 
    hComm = CreateFile(L"\\\\.\\COM6", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    // Check if the serial port can be open
    if (hComm == INVALID_HANDLE_VALUE) {
        printf("Error in opening serial port\n");
    }
    else {
        printf("Opening serial port successful\n");
    }

    // Check if purging comm is complete
    status = PurgeComm(hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    if (!status) {
        printf("Error in purging comm\n");
        CloseHandle(hComm);
    }

    // Get the comm State
    status = GetCommState(hComm, &dcbSerialParams);
    if (!status) {
        printf("Error in GetCommState\n");
        CloseHandle(hComm);
    }

    // We will be using BaudRate of 9600 for UART communication, the configuration of UART is same with that of STM32
    dcbSerialParams.BaudRate = CBR_9600;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    status = SetCommState(hComm, &dcbSerialParams);

    if (!status) {
        printf("Error in setting DCB structure\n");
        CloseHandle(hComm);
    }

    // How long can UART wait until it completes reception and transmission (It is set to be very long)
    timeouts.ReadIntervalTimeout = 200000;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 200000;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 200000;
    if (!SetCommTimeouts(hComm, &timeouts)) {
        printf("Error in setting timeouts\n");
        CloseHandle(hComm);
    }
}

//Initialize SPI
FT_HANDLE Init_SPI() {
    FT_STATUS status;
    FT_DEVICE_LIST_INFO_NODE channelInfo;
    FT_HANDLE handle;

    DWORD channelCount = 0;

    // Check the number of SPI channels
    status = SPI_GetNumChannels(&channelCount);

    // Check if there is at least one channel
    if (status != FT_OK)
        print_and_quit("Error while checking the number of available MPSSE channels.");
    else if (channelCount < 1)
        print_and_quit("Error: No MPSSE channels are available.");

    printf("There are %d channels available. \n\n", channelCount);

    // Display the information of the channel
    for (unsigned int i = 0; i < channelCount; i++) {
        status = SPI_GetChannelInfo(i, &channelInfo);
        if (status != FT_OK)
            print_and_quit("Error while getting details for an MPSSE channel.");
        printf("Channel number : %d\n", i);
        printf("Description: %s\n", channelInfo.Description);
        printf("Serial Number : %s\n", channelInfo.SerialNumber);
    }

    // Ask user to use which channel he will use
    uint32_t channel = 0;
    printf("\n Enter a channel number to use: ");
    scanf_s("%d", &channel);

    // Open that channel and check if it can be open
    status = SPI_OpenChannel(channel, &handle);
    if (status != FT_OK)
        print_and_quit("Error while opening the MPSSE channel.");

    // Configure the channel. ClockRate and LatencyTimer can be configured differrently but this combination worked optimal for me
    ChannelConfig channelConfig;
    channelConfig.ClockRate = 15000000;
    channelConfig.configOptions = SPI_CONFIG_OPTION_MODE2 | SPI_CONFIG_OPTION_CS_DBUS3 | SPI_CONFIG_OPTION_CS_ACTIVELOW;
    channelConfig.LatencyTimer = 20;

    //   Initialize the channel according to the configuration
    status = SPI_InitChannel(handle, &channelConfig);
    if (status != FT_OK)
        print_and_quit("Error while initializing the MPSSE channel.");
    return handle;
}

//This function is written considering endianness mismatch between SPI and SAI
void reversePacketOrder(UCHAR* buffer, size_t bufferSize) {
    for (size_t i = 0; i < bufferSize; i += 4) {
        for (size_t j = 0; j < 2; ++j) {
            UCHAR temp = buffer[i + j];
            buffer[i + j] = buffer[i + 3 - j];
            buffer[i + 3 - j] = temp;
        }
    }
}

int test(int argc, char** argv)
{
    FT_HANDLE handle;
    FILE* fp;
    FILE* fp2;
    FILE* fr;
    FILE* fr2;

    //Initialize the peripherals
    Init_UART();
    Init_libMPSSE();
    handle = Init_SPI();

    fopen_s(&fp, "data.csv", "w+");
    fopen_s(&fp2, "renew_data.csv", "w+");
    fopen_s(&fr, "EnginePattern_raster_amp1_res512_hexadecimal_downsample.txt", "r");

    FT_STATUS status;
    bool status2;
    DWORD bytesRead;
    DWORD bytesWrite;
    UCHAR StartByte = 0;
    UCHAR RxByte;

    DWORD transferCount = 0;
    LPDWORD ptransferCount = &transferCount;
    DWORD options = SPI_TRANSFER_OPTIONS_SIZE_IN_BYTES | SPI_TRANSFER_OPTIONS_CHIPSELECT_ENABLE | SPI_TRANSFER_OPTIONS_CHIPSELECT_DISABLE;

    int COUNT = 32 * 4;
    const int CHUNK_NUM = 32;
    const int NUM_OF_POSITIONS_PER_CHUNK = 2620;
    int GALVO_DELAY = 5;

    int CHUNK_NUM_CURRENT = 0;
    int COMPARISON_CHUNK_NUM_CURRENT = 0;
    const int MULTIPLIER = 4;

    DWORD input_data[CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK];
    UCHAR tx_buffer[MULTIPLIER * CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK];
    int NUM_OF_BYTES_PER_CHUNK = MULTIPLIER * NUM_OF_POSITIONS_PER_CHUNK;
    UCHAR tx_message[MULTIPLIER * NUM_OF_POSITIONS_PER_CHUNK + MULTIPLIER];
    UCHAR rx_buffer[MULTIPLIER * NUM_OF_POSITIONS_PER_CHUNK + MULTIPLIER];

    int x;
    int y;
    int original_x;
    int original_y;
    int16_t state = 0;
    int16_t payload = NUM_OF_POSITIONS_PER_CHUNK;
    int idle_sent = 0;
    int i = 2;
    int ping = 0;
    int print = 0;
    int32_t msg = ((uint32_t)state << 16) | (uint16_t)payload;
    UCHAR ping_buffer[4];

    int PREP = 1;
    int STREAM = 2;
    int STOP = 3;
    int PING = 4;
    int FF_STREAM = 7;

    int PREP_STATE = 0;
    int SET_STATE = 1;
    int STREAM_STATE = 2;
    int STOP_STATE = 3;
    int PING_STATE = 4;
    int FF_STREAM_STATE = 7;
    int KB_CONSTANT = 48;

    for (int k = 0; k < CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK; ++k) {
        fscanf_s(fr, "%x", &input_data[k]);
    }

    for (int k = 0; k < CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK; ++k) {
        if (k < CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK - GALVO_DELAY) {
            uint32_t data = input_data[k + GALVO_DELAY];
            tx_buffer[4 * k + 3] = (uint8_t)((data >> 24) & 0xFF);
            tx_buffer[4 * k + 2] = (uint8_t)((data >> 16) & 0xFF);
            tx_buffer[4 * k + 1] = (uint8_t)((data >> 8) & 0xFF);
            tx_buffer[4 * k] = (uint8_t)(data & 0xFF);
        }
        else {
            uint32_t data = input_data[(k + GALVO_DELAY) % (CHUNK_NUM * NUM_OF_POSITIONS_PER_CHUNK)];
            tx_buffer[4 * k + 3] = (uint8_t)((data >> 24) & 0xFF);
            tx_buffer[4 * k + 2] = (uint8_t)((data >> 16) & 0xFF);
            tx_buffer[4 * k + 1] = (uint8_t)((data >> 8) & 0xFF);
            tx_buffer[4 * k] = (uint8_t)(data & 0xFF);
        }
    }

    while (1) {
        if (_kbhit()) {
            char ch = _getch();

            std::cout << "Key pressed: " << ch << std::endl;
            std::cout << "msg: " << msg << std::endl;

            if (state == PREP_STATE && ch == (KB_CONSTANT + SET_STATE)) {
                state = SET_STATE;
                idle_sent = 0;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                status = WriteFile(hComm, &msg, 4, &bytesWrite, NULL);

                Sleep(500);
                status = SPI_Write(handle, &tx_buffer[0], NUM_OF_BYTES_PER_CHUNK * 2, ptransferCount, options);
                printf("Buffer Filled \n");
            }

            if (state == SET_STATE && ch == (KB_CONSTANT + STREAM_STATE)) {
                state = STREAM_STATE;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                status = WriteFile(hComm, &msg, 4, &bytesWrite, NULL);
                i = 2;
                printf("Stream Start \n");
            }

            if (state == SET_STATE && ch == (KB_CONSTANT + FF_STREAM)) {
                state = 2;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                status = WriteFile(hComm, &msg, 4, &bytesWrite, NULL);
                i = 2;
                printf("Feedforward Stream Start \n");
                state = FF_STREAM_STATE;
            }

            if ((state == STREAM_STATE || state == FF_STREAM_STATE) && ch == (KB_CONSTANT + STOP_STATE)) {
                state = STOP_STATE;
            }

            if ((state == STREAM_STATE || state == FF_STREAM_STATE) && ch == (KB_CONSTANT + PING_STATE)) {
                ping = 1;
            }

            if ((state == STREAM_STATE || state == FF_STREAM_STATE) && ch == (KB_CONSTANT + 5)){
                state = 5;
            }

            // if ((state == 5) && ch == (KB_CONSTANT + 2)) {
            //     state = 2;
            // }

            if (state == PREP_STATE && ch == (KB_CONSTANT + PING_STATE)) {
                state = PING_STATE;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                status = WriteFile(hComm, &msg, 4, &bytesWrite, NULL);
                status2 = ReadFile(hComm, &ping_buffer[0], 4, &bytesWrite, NULL);
                printf("Ping ID result: %d, %d, %d, %d \n", ping_buffer[0], ping_buffer[1], ping_buffer[2], ping_buffer[3]);
                state = PREP_STATE;
            }

            if (state == SET_STATE && ch == (KB_CONSTANT + PING_STATE)) {
                state = PING_STATE;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                status = WriteFile(hComm, &msg, 4, &bytesWrite, NULL);
                status2 = ReadFile(hComm, &ping_buffer[0], 4, &bytesWrite, NULL);
                printf("Ping ID result: %d, %d, %d, %d \n", ping_buffer[0], ping_buffer[1], ping_buffer[2], ping_buffer[3]);
                state = SET_STATE;
            }

            if (state == 5 && ch == (KB_CONSTANT + PING_STATE)) {
                state = PING_STATE;
                msg = ((uint32_t)state << 16) | (uint16_t)payload;
                status = WriteFile(hComm, &msg, 4, &bytesWrite, NULL);
                status2 = ReadFile(hComm, &ping_buffer[0], 4, &bytesWrite, NULL);
                printf("Ping ID result: %d, %d, %d, %d \n", ping_buffer[0], ping_buffer[1], ping_buffer[2], ping_buffer[3]);
                state = SET_STATE;
            }

        }

        if (state == STREAM_STATE || state == FF_STREAM_STATE || state == 5 || state == STOP_STATE) {

            Sleep(1);
            if (i % 2 == 0) {
                status = SPI_Read(handle, &rx_buffer[0], NUM_OF_BYTES_PER_CHUNK + MULTIPLIER, ptransferCount, options);
            }
            else {
                status = SPI_Read(handle, &rx_buffer[0], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
            }
            if (ping == 1 && i % 2 == 0) {
                if (print == 0) {
                    print = 1;
                }
                else if (print == 1) {
                    print = 0;
                    ping = 0;
                    printf("Ping ID result: %d, %d, %d, %d \n", rx_buffer[NUM_OF_BYTES_PER_CHUNK], rx_buffer[NUM_OF_BYTES_PER_CHUNK + 1], rx_buffer[NUM_OF_BYTES_PER_CHUNK + 2], rx_buffer[NUM_OF_BYTES_PER_CHUNK + 3]);
                }
            }
            Sleep(1);

            if (i == COUNT + 3) {
                break;
            }

            CHUNK_NUM_CURRENT = i % CHUNK_NUM;

            if (i % 2 == 1) {
                status = SPI_Write(handle, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK, ptransferCount, options);
            }
            else {
                if ((state == STREAM_STATE || state == FF_STREAM_STATE) && idle_sent == 0) {
                    memcpy(tx_message, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK);
                    UCHAR extra_bytes[4] = { STREAM, 0, 0, 0 };
                    if (ping == 1) {
                        extra_bytes[0] = PING;
                    }
                    memcpy(&tx_message[NUM_OF_BYTES_PER_CHUNK], extra_bytes, MULTIPLIER);
                    status = SPI_Write(handle, tx_message, NUM_OF_BYTES_PER_CHUNK + MULTIPLIER, ptransferCount, options);

                }
                else if (state == STOP_STATE && idle_sent == 0) {
                    memcpy(tx_message, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK);
                    UCHAR extra_bytes[4] = { STOP, 0, 0, 0 };
                    memcpy(&tx_message[NUM_OF_BYTES_PER_CHUNK], extra_bytes, MULTIPLIER);
                    status = SPI_Write(handle, tx_message, NUM_OF_BYTES_PER_CHUNK + MULTIPLIER, ptransferCount, options);
                    idle_sent = 1;
                }
                else if (state == 5 && idle_sent == 0) {
                    memcpy(tx_message, &tx_buffer[CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK], NUM_OF_BYTES_PER_CHUNK);
                    UCHAR extra_bytes[4] = { 5, 0, 0, 0 };
                    memcpy(&tx_message[NUM_OF_BYTES_PER_CHUNK], extra_bytes, MULTIPLIER);
                    status = SPI_Write(handle, tx_message, NUM_OF_BYTES_PER_CHUNK + MULTIPLIER, ptransferCount, options);
                    idle_sent = 1;
                }
                else if (idle_sent == 1) {
                    if (state == STOP_STATE) {
                        state = 0;
                    }
                    else if (state == 5) {
                        char input = '0';
                        do {
                            if (_kbhit()) {
                                input = _getch();
                            }
                        } while (input != '2'); 
                        if (input == '2') {
                            state = 2;
                            msg = ((uint32_t)state << 16) | (uint16_t)payload;
                            status = WriteFile(hComm, &msg, 4, &bytesWrite, NULL);
                            idle_sent = 0;
                        }
                    }
                }
            }

            if (state == STREAM_STATE || state == STOP_STATE || state == 5) {
                if (i < COUNT + 4) {
                    status2 = ReadFile(hComm, &RxByte, 1, &bytesWrite, NULL);
                }

                for (int j = 0; j < MULTIPLIER * NUM_OF_POSITIONS_PER_CHUNK; j += MULTIPLIER) {
                    x = rx_buffer[j + 3] * 256 + rx_buffer[j + 2];
                    y = rx_buffer[j + 1] * 256 + rx_buffer[j];
                    fprintf(fp, "%d,%d,", x, y);
                }

                fprintf(fp, "\n");
                fprintf(fp2, "\n");
                printf("%d, %d, %d, %d \n", i, state, RxByte, idle_sent);
                i = i + 1;

            }
            else if (state == FF_STREAM_STATE) {
                if (i < COUNT + 4) {
                    status2 = ReadFile(hComm, &RxByte, 1, &bytesWrite, NULL);
                }

                for (int j = 0; j < MULTIPLIER * NUM_OF_POSITIONS_PER_CHUNK; j += MULTIPLIER) {
                    x = rx_buffer[j + 3] * 256 + rx_buffer[j + 2];
                    y = rx_buffer[j + 1] * 256 + rx_buffer[j];
                    fprintf(fp, "%d,%d,", x, y);
                    if (i > CHUNK_NUM + 2) {
                        CHUNK_NUM_CURRENT = (i - 3) % CHUNK_NUM;
                        uint32_t data = input_data[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK + j / 4];
                        original_x = (uint16_t)((data >> 16) & 0xFFFF);
                        original_y = (uint16_t)(data & 0xFFFF);

                        uint32_t error_x = x - original_x;
                        uint32_t error_y = y - original_y;
                        fprintf(fp2, "%d,%d,", error_x, error_y);

                        //tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + j / 2] =
                        //    tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + j / 2] - (x - original_x);

                        //tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + 1 + j / 2] =
                        //    tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + j / 2 + 1] - (y - original_y);

                        int number = (COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_BYTES_PER_CHUNK + j + CHUNK_NUM * NUM_OF_BYTES_PER_CHUNK - 20)
                            % (CHUNK_NUM * NUM_OF_BYTES_PER_CHUNK);

                        //tx_buffer[number] = tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + j / 2 + 1] % 256;
                        //tx_buffer[number + 1] = tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + j / 2 + 1] / 256;
                        //tx_buffer[number + 2] = tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + j / 2] % 256;
                        //tx_buffer[number + 3] = tx_buffer2[COMPARISON_CHUNK_NUM_CURRENT * NUM_OF_POSITIONS_PER_CHUNK * 2 + j / 2] / 256;
                    }
                }

                fprintf(fp, "\n");
                fprintf(fp2, "\n");
                printf("%d, %d, %d, %d \n", i, state, RxByte, idle_sent);
                i = i + 1;

            }
        }
    }

    Cleanup_libMPSSE();
    return 0;
}

int main(int argc, char** argv) {
    test(argc, argv);
}
