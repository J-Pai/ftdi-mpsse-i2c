#include <stdio.h>
#include <ctype.h>
#include <ftdi.h>

struct ftdi_context ftdic;
unsigned char OutputBuffer[1024]; // Buffer to hold MPSSE commands and data to be sent to FT4232H
unsigned char InputBuffer[1024];  // Buffer to hold Data unsigned chars to be read from FT4232H
unsigned int dwClockDivisor = 0x00c8; // Value of clock divisor, SCL Frequency = 60/((1+0x0032)*2) (MHz) ~= 400khz
unsigned int dwNumBytesToSend = 0; // Index of output buffer
unsigned int dwNumBytesSent = 0, dwNumBytesRead = 0, dwNumInputBuffer = 0;
int channel = 0;
int debug = 1;	// Debug mode

int InitializeI2C() {
  unsigned int dwCount;
  char SerialNumBuf[64];
  int bCommandEchoed;
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

  // Port opened successfully
  if(debug)
    printf("Port opened, resetting device...\n");

  ftStatus |= ftdi_usb_reset(&ftdic); 			// Reset USB device
  ftStatus |= ftdi_usb_purge_rx_buffer(&ftdic);	// purge rx buffer
  ftStatus |= ftdi_usb_purge_tx_buffer(&ftdic);	// purge tx buffer
  /* Set MPSSE mode */
  ftdi_set_bitmode(&ftdic, 0xFF, BITMODE_RESET);
  ftdi_set_bitmode(&ftdic, 0xFF, BITMODE_MPSSE);

  OutputBuffer[dwNumBytesToSend++] = '\xAA'; 	// Add BAD command 0xxAA
  dwNumBytesSent = ftdi_write_data(&ftdic, OutputBuffer, dwNumBytesToSend);
  dwNumBytesToSend = 0;
  i = 0;
  do {
    dwNumBytesRead = ftdi_read_data(&ftdic, InputBuffer, 2);
    if(dwNumBytesRead < 0) {
      if(debug)
        printf("Error: %s\n", ftdi_get_error_string(&ftdic));
      break;
    }
    if(debug)
      printf("Got %d bytes %02X %02X\n", dwNumBytesRead, InputBuffer[0], InputBuffer[1]);
    if(++i > 5)	/* up to 5 times read */
      break;
  } while (dwNumBytesRead == 0);
  // Check if echo command and bad received
  for (dwCount = 0; dwCount < dwNumBytesRead; dwCount++) {
    if ((InputBuffer[dwCount] == 0xFA) && (InputBuffer[dwCount+1] == 0xAA)) {
      if(debug)
        printf("FTDI synchronized\n");
      bCommandEchoed = 1;
      break;
    }
  }
  if (bCommandEchoed == 0) {
    return 1;
    /* Error, cant receive echo command , fail to synchronize MPSSE interface. */
  }

  dwNumBytesToSend = 0; // Clear index to zero
  OutputBuffer[dwNumBytesToSend++] = 0x8A; // Disable clock divide-by-5 for 60Mhz master clock
  OutputBuffer[dwNumBytesToSend++] = 0x97; // Ensure adaptive clocking is off
  OutputBuffer[dwNumBytesToSend++] = 0x8C; // Enable 3 phase data clocking, data valid on both clock edges for I2C
  dwNumBytesSent = ftdi_write_data(&ftdic, OutputBuffer, dwNumBytesToSend);
  dwNumBytesToSend = 0;
  OutputBuffer[dwNumBytesToSend++] = 0x85; // Ensure internal loopback is off
  OutputBuffer[dwNumBytesToSend++] = 0x80; //Command to set directions of lower 8 pins and force value on bits set as output
  OutputBuffer[dwNumBytesToSend++] = 0x03; //Set SDA, SCL high, WP disabled by SK, DO at bit ‘1’, GPIOL0 at bit ‘0’
  OutputBuffer[dwNumBytesToSend++] = 0x13; //Set SK,DO,GPIOL0 pins as output with bit ’, other pins as input with bit ‘’
  dwNumBytesSent = ftdi_write_data(&ftdic, OutputBuffer, dwNumBytesToSend);
  dwNumBytesToSend = 0;
  // Clear index to zero
  OutputBuffer[dwNumBytesToSend++] = 0x86;
  // Command to set clock divisor
  OutputBuffer[dwNumBytesToSend++] = dwClockDivisor & 0xFF;
  // Set 0xValueL of clock divisor
  OutputBuffer[dwNumBytesToSend++] = (dwClockDivisor >> 8) & 0xFF;
  // Set 0xValueH of clock divisor
  dwNumBytesSent = ftdi_write_data(&ftdic, OutputBuffer, dwNumBytesToSend);
  dwNumBytesToSend = 0;

  return 0;
}

int SetI2CStart() {
  dwNumBytesToSend = 0; //Clear output buffer
  int dwCount;
  for(dwCount=0; dwCount < 4; dwCount++) // Repeat commands to ensure the minimum period of the start hold time is achieved
  {
    OutputBuffer[dwNumBytesToSend++] = 0x80; // Command to set ADbus direction/ data
    OutputBuffer[dwNumBytesToSend++] = 0x03; // Bring data out low (bit 1)
    OutputBuffer[dwNumBytesToSend++] = 0x13; // Set pins o/p except bit 2 (data_in)
  }
  for(dwCount=0; dwCount < 4; dwCount++) // Repeat commands to ensure the minimum period
    // of the start setup time is achieved
  {
    OutputBuffer[dwNumBytesToSend++] = 0x80; // Command to set ADbus direction/ data
    OutputBuffer[dwNumBytesToSend++] = 0x01; // Bring clock line low too
    OutputBuffer[dwNumBytesToSend++] = 0x13; // Set pins o/p except bit 2 (data_in)
  }
  // Turn the LED on by setting port AC6 low
  OutputBuffer[dwNumBytesToSend++] = 0x80; // Command to set ACbus direction and data
  OutputBuffer[dwNumBytesToSend++] = 0x00; // Bit 6 is going low
  OutputBuffer[dwNumBytesToSend++] = 0x13; // Only bit 6 is output
  //Send off the commands
  dwNumBytesSent = ftdi_write_data(&ftdic, OutputBuffer, dwNumBytesToSend);
  dwNumBytesToSend = 0;
  return 0;
}

int SetI2CStop() {
  dwNumBytesToSend = 0; //Clear output buffer
  int dwCount;
  // Initial condition for the I2C Stop - Pull data low (Clock will already be low and is kept low)
  for(dwCount=0; dwCount<4; dwCount++) // Repeat commands to ensure the minimum period of the stop setup time is achieved
  {
    OutputBuffer[dwNumBytesToSend++] = 0x80; // Command to set ADbus direction/data
    OutputBuffer[dwNumBytesToSend++] = 0x01; // put data and clock low
    OutputBuffer[dwNumBytesToSend++] = 0x13; // Set pins o/p except bit 2 (data_in)
  }
  // Clock now goes high (open drain)
  for(dwCount=0; dwCount<4; dwCount++) // Repeat commands to ensure the minimum period
    // of the stop setup time is achieved
  {
    OutputBuffer[dwNumBytesToSend++] = 0x80; // Command to set ADbus direction/data
    OutputBuffer[dwNumBytesToSend++] = 0x03; // put data low, clock remains high
    OutputBuffer[dwNumBytesToSend++] = 0x13; // Set pins o/p except bit 2 (data_in)
  }

  OutputBuffer[dwNumBytesToSend++] = 0x80; // Command to set ADbus direction/data
  OutputBuffer[dwNumBytesToSend++] = 0x00; // both clock and data now high
  OutputBuffer[dwNumBytesToSend++] = 0x10; // Set pins o/p except bit 2 (data_in)

  //Send off the commands
  dwNumBytesSent = ftdi_write_data(&ftdic, OutputBuffer, dwNumBytesToSend);
  dwNumBytesToSend = 0;
  return 0;
}

int SendByteAndCheckACK(unsigned char dataSend) {
  // dwNumBytesToSend = 0; // Clear output buffer
  int ftStatus = 0;
  int r;
  OutputBuffer[dwNumBytesToSend++] = 0x11; // command: clock bytes out MSB first on
  // clock falling edge
  OutputBuffer[dwNumBytesToSend++] = 0x00; //
  OutputBuffer[dwNumBytesToSend++] = 0x00; // Data length of 0x0000 means clock out 1 byte
  OutputBuffer[dwNumBytesToSend++] = dataSend; // Actual byte to clock out
  // Put I2C line back to idle (during transfer) state... Clock line low, Data line high
  OutputBuffer[dwNumBytesToSend++] = 0x80; // Command to set ADbus direction/ data
  OutputBuffer[dwNumBytesToSend++] = 0x00; // Set the value of the pins
  OutputBuffer[dwNumBytesToSend++] = 0x11; // Set pins o/p except bit 2 (data_in)
  // AD0 (SCL) is output driven low
  // AD1 (DATA OUT) is output high (open drain)
  // AD2 (DATA IN) is input (therefore the output value specified is ignored)
  // AD3 to AD7 are inputs driven high (not used in this application)
  OutputBuffer[dwNumBytesToSend++] = 0x22; // Command to clock in bits MSB first
  // on rising edge
  OutputBuffer[dwNumBytesToSend++] = 0x00; // Length of 0x00 means to scan in 1 bit
  // This command then tells the MPSSE to send any results gathered back immediately
  OutputBuffer[dwNumBytesToSend++] = 0x87; //Send answer back immediate command
  dwNumBytesSent = ftdi_write_data(&ftdic, OutputBuffer, dwNumBytesToSend);
  dwNumBytesToSend = 0;
  // Send off the commands
  // Clear output buffer
  // Check if ACK bit received, may need to read more times to get ACK bit or fail if timeout
  dwNumBytesRead = ftdi_read_data(&ftdic, InputBuffer, 1);
  // Read one byte from device receive buffer
  if(dwNumBytesRead == 0) {
    return 0; /* Error reading bit, should not happened if we are connected to FTDI */
  } else if((InputBuffer[0] & 0x01) == 0x00) {
    r = 1;
    // Check ACK bit 0 on data byte read out
  } else {
    r = 0;
  }
  if(debug) {
    printf("Sent: %d, %02X | Received: %d, %02X\n", dwNumBytesSent, dataSend, dwNumBytesRead, InputBuffer[0]);
  }
  OutputBuffer[dwNumBytesToSend++] = 0x80; //Command to set directions of lower 8 pins and force value on bits set as output
  OutputBuffer[dwNumBytesToSend++] = 0x02; //Set SDA high, SCL low, WP disabled by SK at bit '0', DO, GPIOL0 at bit '1'
  OutputBuffer[dwNumBytesToSend++] = 0x13; //Set SK,DO,GPIOL0 pins as output with bit ‘1’, other pins as input with bit ‘0’
  return r;
}

int main(int argc, char *argv[]) {
  int i, a;
  char *s, *serial;
  int b = 0;

  if(argc < 2) {
    printf("i2c: Send data over i2c bus using ftdi F4232H port 0 I2C\n");
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
  b = b << 1;	/* R/W bit should be 0 */
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
    /* Acutaly send bytes */
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
  dwNumBytesToSend = 0;

  ftdi_usb_close(&ftdic);
  ftdi_deinit(&ftdic);
  return 0;
}
