#include <stdio.h>
#include <ctype.h>
#include <ftdi.h>

// Struct object of FTDI context.
struct ftdi_context ftdic;
// Buffer to hold MPSSE commands and data to be sent to FT4232H
unsigned char OutputBuffer[1024];
// Buffer to hold Data unsigned chars to be read from FT4232H
unsigned char InputBuffer[1024];
// Value of clock divisor, SCL Frequency = 60MHz/((1+0x0383)*2) = 33KHz
const uint16_t kClockDivisor = 0x0200;
// Default number of times to repeat commands for hold times.
const uint8_t kDefaultRepeat = 4;
// Index of output buffer
uint16_t numberOfBytesToSend = 0;
// Bytes sent
uint16_t numberOfBytesSent = 0;
// Bytes read
uint16_t numberOfBytesRead = 0;
// FTDI Channel
uint8_t channel = 1;
// Debug toggle
uint8_t debug = 1;

int InitializeI2C() {
  unsigned int count;
  char SerialNumBuf[64];
  int commandEchoed;
  int ftStatus = 0;
  int i;

  ftStatus = ftdi_init(&ftdic);
  if(ftStatus < 0) {
    printf("ftdi init failed\n");
    return 0;
  }
  i = (channel == 0) ? INTERFACE_A : INTERFACE_B;
  ftdi_set_interface(&ftdic, i);

  ftStatus = ftdi_usb_open(&ftdic, 0x0403, 0x6011);
  if(ftStatus < 0) {
    printf("Error opening usb device: %s\n", ftdi_get_error_string(&ftdic));
    return 1;
  }

  if(debug)
    printf("Port opened, resetting device...\n");

  // ftStatus |= ftdi_set_baudrate(&ftdic, 115200);

  ftStatus |= ftdi_usb_reset(&ftdic);
  ftStatus |= ftdi_usb_purge_rx_buffer(&ftdic);
  ftStatus |= ftdi_usb_purge_tx_buffer(&ftdic);
  ftStatus |= ftdi_read_data_set_chunksize(&ftdic, 65536);
  ftStatus |= ftdi_write_data_set_chunksize(&ftdic, 65536);
  ftStatus |= ftdi_set_latency_timer(&ftdic, 16);

  if (ftStatus != 0) {
    printf("Issues setting up FTDI: %s\n", ftdi_get_error_string(&ftdic));
  }


  ftdi_set_bitmode(&ftdic, 0xFF, BITMODE_RESET);
  ftdi_set_bitmode(&ftdic, 0xFF, BITMODE_MPSSE);

  numberOfBytesToSend = 0;
  // MPSSE command for disabling clock divide by 5 for 60 MHz master clock.
  OutputBuffer[numberOfBytesToSend++] = 0x8A;
  // MPSSE command for turning off adaptive clocking.
  OutputBuffer[numberOfBytesToSend++] = 0x97;
  // Enable 3 phase data clock. Enables I2C bus to clock data on both clock edges.
  OutputBuffer[numberOfBytesToSend++] = 0x8C;
  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;
  // Set the direction of the lower 8 pins. Force value on bits set as output.
  OutputBuffer[numberOfBytesToSend++] = 0x80;
  // Set SDA and SCL high. Write Protection disabled by SK and DO at bit 1. GPIOL0 at bit 0.
  OutputBuffer[numberOfBytesToSend++] = 0x03;
  // Set SK, DO, GPIOL0 pins as output with bit 1 other pins as input with bit 0.
  OutputBuffer[numberOfBytesToSend++] = 0x13;
  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;
  // SK frequency = 60MHz /((1 + [(1 + 0xValueH*256) | 0xValueL])*2)
  // MPSSE command to set clock divisor.
  OutputBuffer[numberOfBytesToSend++] = 0x86;
  // Set 0xValueL of clock divisor
  OutputBuffer[numberOfBytesToSend++] = kClockDivisor & 0xFF;
  // Set 0xValueH of clock divisor
  OutputBuffer[numberOfBytesToSend++] = (kClockDivisor >> 8) & 0xFF;

  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;

  // Turn off loop back of TDI/TDO connection.
  OutputBuffer[numberOfBytesToSend++] = 0x85;

  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;

  return 0;
}

int SetI2CStart() {
  numberOfBytesToSend = 0;
  int count;
  for(count = 0; count < kDefaultRepeat; ++count) {
    OutputBuffer[numberOfBytesToSend++] = 0x80;
    OutputBuffer[numberOfBytesToSend++] = 0x03;
    OutputBuffer[numberOfBytesToSend++] = 0x13;
  }
  for(count = 0; count < kDefaultRepeat; ++count) {
    OutputBuffer[numberOfBytesToSend++] = 0x80;
    OutputBuffer[numberOfBytesToSend++] = 0x01;
    OutputBuffer[numberOfBytesToSend++] = 0x13;
  }
  OutputBuffer[numberOfBytesToSend++] = 0x80;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x13;

  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;
  return 0;
}

int SetI2CStop() {
  numberOfBytesToSend = 0;
  int count;

  for(count = 0; count < kDefaultRepeat; ++count) {
    OutputBuffer[numberOfBytesToSend++] = 0x80;
    OutputBuffer[numberOfBytesToSend++] = 0x01;
    OutputBuffer[numberOfBytesToSend++] = 0x13;
  }
  for(count = 0; count < kDefaultRepeat; ++count) {
    OutputBuffer[numberOfBytesToSend++] = 0x80;
    OutputBuffer[numberOfBytesToSend++] = 0x03;
    OutputBuffer[numberOfBytesToSend++] = 0x13;
  }

  OutputBuffer[numberOfBytesToSend++] = 0x80;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x10;

  //Send off the commands
  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;
  return 0;
}

int SendByteAndCheckACK(unsigned char data) {
  // numberOfBytesToSend = 0;

  OutputBuffer[numberOfBytesToSend++] = 0x11;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = data;
  OutputBuffer[numberOfBytesToSend++] = 0x80;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x11;

  OutputBuffer[numberOfBytesToSend++] = 0x22;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x87;

  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;

  numberOfBytesRead = ftdi_read_data(&ftdic, InputBuffer, 1);
  int status;
  if((InputBuffer[0] & 0x01) == 0x00) {
    status = 1;
  } else {
    status = 0;
  }
  if(debug) {
    printf("Sent: %d, %02X | Received: %d, %02X\n", numberOfBytesSent, data,
        numberOfBytesRead, InputBuffer[0]);
  }

  OutputBuffer[numberOfBytesToSend++] = 0x80;
  OutputBuffer[numberOfBytesToSend++] = 0x02;
  OutputBuffer[numberOfBytesToSend++] = 0x13;
  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;

  return status;
}

int ReadByte() {
  numberOfBytesToSend = 0;

  OutputBuffer[numberOfBytesToSend++] = 0x80;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x11;

  OutputBuffer[numberOfBytesToSend++] = 0x24;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x00;

  OutputBuffer[numberOfBytesToSend++] = 0x22;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x87;

  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;

  usleep(1);

  numberOfBytesRead = ftdi_read_data(&ftdic, InputBuffer, 2);
  int status;
  unsigned char read_byte = InputBuffer[0];
  if(numberOfBytesRead != 2) {
    return -1;
  }

  return read_byte;
}

void PrintHelp() {
  printf("i2c: Send/Read data over i2c bus using FTDI F4232H port 0 I2C\n");
  printf("usage: i2c [-c <channel>] [-r <bytes>] [<address> <data>]\n");
}

int main(int argc, char *argv[]) {
  int i, arg_index;
  char *arg, *serial;
  int byte = 0;
  int read = 0;

  if(argc < 2) {
    PrintHelp();
    return 1;
  }
  for(arg_index = 1; arg_index < argc; ++arg_index) {
    arg = argv[arg_index];
    // If arg starts with '-', it is a commandline option.
    if(*arg == '-') {
      ++arg;
      ++arg_index;
      if(*arg == 'c') {
        channel = atoi(argv[arg_index]);
      } else if (*arg == 'r') {
        read = atoi(argv[arg_index]);
      } else {
        printf("Unknown option -%c\n", *arg);
        PrintHelp();
        return -1;
      }
    } else {
      break;
    }
  }

  InitializeI2C();
  SetI2CStart();
  arg = argv[arg_index];
  byte = 0;
  if(*arg == '0') {
    ++arg;
  }
  if(*arg == 'x') {
    ++arg;
  }
  while(*arg) {
    if(!isxdigit(*arg)) {
      printf("%c Invalid hex value: %s\n", *arg, argv[i]);
      break;
    }
    byte *= 16;
    *arg = toupper(*arg);
    if(*arg >= 'A') {
      byte += (*arg - 'A' + 10);
    } else {
      byte += (*arg - '0');
    }
    ++arg;
  }

  if (read) {
    printf("=== DOING READ ===\n");
  }

  // R/W bit should be 0
  unsigned char address = byte << 1;
  if(debug)
    printf("Sending Address %02X\n", address);
  int status = SendByteAndCheckACK(address);
  if(debug) {
    if(status) {
      printf("Received Address ACK\n");
    } else {
      printf("^====> Error Address reading ACK\n");
    }
  }

  if (status == 0) {
    SetI2CStop();
    return status;
  }

  for(i = arg_index + 1; i < argc; ++i) {
    arg = argv[i];
    byte = 0;
    if(*arg == '0') {
      ++arg;
    }
    if(*arg == 'x') {
      ++arg;
    }
    while(*arg) {
      if(!isxdigit(*arg)) {
        printf("%c Invalid hex value: %s\n", *arg, argv[i]);
        break;
      }
      byte *= 16;
      *arg = toupper(*arg);
      if(*arg >= 'A') {
        byte += (*arg - 'A' + 10);
      } else {
        byte += (*arg - '0');
      }
      ++arg;
    }
    if(debug) {
      printf("Sending data %02X\n", byte);
    }
    status = SendByteAndCheckACK((unsigned char) byte);
    if(debug) {
      if(status) {
        printf("Received data ACK\n");
      } else {
        printf("^====> Error data reading ACK\n");
      }
    }
  }

  unsigned char returned_data[read];
  if (read > 0) {
    for (i = 0; i < read; i++) {
      SetI2CStart();
      address = address | 0x01;
      if(debug) {
        printf("Sending Read Address %02X\n", address);
      }
      status = SendByteAndCheckACK(address);
      if(debug) {
        if(status) {
          printf("Received Read Address ACK\n");
        } else {
          printf("^====> Error Read Address reading ACK\n");
        }
      }
      int read_byte = ReadByte();
      if (read_byte < 0) {
        printf("^====> Error Reading Byte \n");
      } else {
        printf("read[%d]: %02X\n", i, read_byte);
        returned_data[i] = read_byte;
      }
    }
    for (int i = 0; i < read; i++) {
      printf("returned_data[%d]: %02X\n", i, returned_data[i]);
    }
  }

  SetI2CStop();

  numberOfBytesToSend = 0;
  ftdi_usb_close(&ftdic);
  ftdi_deinit(&ftdic);
  return 0;
}
