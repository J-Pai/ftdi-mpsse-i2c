#include <stdio.h>
#include <ctype.h>
#include <ftdi.h>

// Struct object of FTDI context.
struct ftdi_context ftdic;
// Buffer to hold MPSSE commands and data to be sent to FT4232H
unsigned char OutputBuffer[1024];
// Buffer to hold Data unsigned chars to be read from FT4232H
unsigned char InputBuffer[1024];
// Value of clock divisor, SCL Frequency = 60MHz/(((1+0x012C)*2) ~= 100KHz
const uint32_t kClockDivisor = 0x012C;
// Index of output buffer
uint16_t numberOfBytesToSend = 0;
// Bytes sent
uint16_t numberOfBytesSent = 0;
// Bytes read
uint16_t numberOfBytesRead = 0;
// FTDI Channel
uint8_t channel = 0;
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

  ftStatus |= ftdi_usb_reset(&ftdic);
  ftStatus |= ftdi_usb_purge_rx_buffer(&ftdic);
  ftStatus |= ftdi_usb_purge_tx_buffer(&ftdic);

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
  for(count=0; count < 4; count++) {
    OutputBuffer[numberOfBytesToSend++] = 0x80;
    OutputBuffer[numberOfBytesToSend++] = 0x03;
    OutputBuffer[numberOfBytesToSend++] = 0x13;
  }
  for(count=0; count < 4; count++) {
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
  for(count=0; count<4; count++) {
    OutputBuffer[numberOfBytesToSend++] = 0x80;
    OutputBuffer[numberOfBytesToSend++] = 0x01;
    OutputBuffer[numberOfBytesToSend++] = 0x13;
  }
  for(count=0; count<4; count++) {
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

int SendByteAndCheckACK(unsigned char dataSend) {
  numberOfBytesToSend = 0;
  int ftStatus = 0;
  int r;
  OutputBuffer[numberOfBytesToSend++] = 0x80;
  OutputBuffer[numberOfBytesToSend++] = 0x02;
  OutputBuffer[numberOfBytesToSend++] = 0x13;

  OutputBuffer[numberOfBytesToSend++] = 0x11;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = dataSend;
  OutputBuffer[numberOfBytesToSend++] = 0x80;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x11;

  OutputBuffer[numberOfBytesToSend++] = 0x22;
  OutputBuffer[numberOfBytesToSend++] = 0x00;
  OutputBuffer[numberOfBytesToSend++] = 0x87;
  numberOfBytesSent = ftdi_write_data(&ftdic, OutputBuffer, numberOfBytesToSend);
  numberOfBytesToSend = 0;

  numberOfBytesRead = ftdi_read_data(&ftdic, InputBuffer, 1);
  if(numberOfBytesRead == 0) {
    return 0;
  } else if((InputBuffer[0] & 0x01) == 0x00) {
    r = 1;
  } else {
    r = 0;
  }
  if(debug) {
    printf("Sent: %d, %02X | Received: %d, %02X\n", numberOfBytesSent, dataSend, numberOfBytesRead, InputBuffer[0]);
  }
  return r;
}

int main(int argc, char *argv[]) {
  int i, a;
  char *s, *serial;
  int b = 0;

  if(argc < 2) {
    printf("i2c: Send/Read data over i2c bus using FTDI F4232H port 0 I2C\n");
    printf("usage: i2c [-c <channel>] [<address> <data>]\n");
    return 1;
  }
  for(a = 1; a < argc; a++) {
    s = argv[a];
    if(*s == '-') {	/* This is a command line option */
      s++;
      a++;
      if(*s == 'c')
        channel = atoi(argv[a]);
      else {
        printf("Unknown option -%c\n", *s);
        return -1;
      }
    }
    else
      break;
  }
  InitializeI2C();
  SetI2CStart();
  s = argv[a];
  b = 0;
  if(*s == '0')
    s++;
  if(*s == 'x')
    s++;
  while(*s) {
    if(!isxdigit(*s)) {
      printf("%c Invalid hex value: %s\n", *s, argv[i]);
      break;
    }
    b *= 16;
    *s = toupper(*s);
    if(*s >= 'A')
      b += (*s - 'A' + 10);
    else
      b += (*s - '0');
    s++;
  }
  // R/W bit should be 0
  b = b << 1;
  if(debug)
    printf("Sending Address %02X\n", b);
  b = SendByteAndCheckACK((unsigned char)b);
  if(debug) {
    if(b)
      printf("Received Address ACK\n");
    else
      printf("^====> Error Address reading ACK\n");
  }
  for(i = a + 1; i < argc; i++) {
    s = argv[i];
    b = 0;
    if(*s == '0')
      s++;
    if(*s == 'x')
      s++;
    while(*s) {
      if(!isxdigit(*s)) {
        printf("%c Invalid hex value: %s\n", *s, argv[i]);
        break;
      }
      b *= 16;
      *s = toupper(*s);
      if(*s >= 'A')
        b += (*s - 'A' + 10);
      else
        b += (*s - '0');
      s++;
    }
    if(debug)
      printf("Sending data %02X\n", b);
    b = SendByteAndCheckACK((unsigned char)b);
    if(debug) {
      if(b)
        printf("Received data ACK\n");
      else
        printf("^====> Error data reading ACK\n");
    }
  }
  SetI2CStop();
  numberOfBytesToSend = 0;

  ftdi_usb_close(&ftdic);
  ftdi_deinit(&ftdic);
  return 0;
}
