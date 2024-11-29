#include "crc8.h"
#include "remote.h"
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#define MAX_BUF_SZ 256

// Opens the specified serial port, sets it up for binary communication,
// configures its read timeouts, and sets its baud rate.
// Returns a non-negative file descriptor on success, or -1 on failure.
static int open_serial_port(const char *device, uint32_t baud_rate) {
  int fd = open(device, O_RDWR | O_NOCTTY);
  if (fd == -1) {
    perror(device);
    return -1;
  }

  // Flush away any bytes previously read or written.
  int result = tcflush(fd, TCIOFLUSH);
  if (result) {
    perror("tcflush failed"); // just a warning, not a fatal error
  }

  // Get the current configuration of the serial port.
  struct termios options;
  result = tcgetattr(fd, &options);
  if (result) {
    perror("tcgetattr failed");
    close(fd);
    return -1;
  }

  // Turn off any options that might interfere with our ability to send and
  // receive raw binary bytes.
  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  // Set up timeouts: Calls to read() will return as soon as there is
  // at least one byte available or when 100 ms has passed.
  options.c_cc[VTIME] = 1;
  options.c_cc[VMIN] = 0;

  // This code only supports certain standard baud rates. Supporting
  // non-standard baud rates should be possible but takes more work.
  switch (baud_rate) {
  case 4800:
    cfsetospeed(&options, B4800);
    break;
  case 9600:
    cfsetospeed(&options, B9600);
    break;
  case 19200:
    cfsetospeed(&options, B19200);
    break;
  case 38400:
    cfsetospeed(&options, B38400);
    break;
  case 115200:
    cfsetospeed(&options, B115200);
    break;
  default:
    fprintf(stderr, "warning: baud rate %u is not supported, using 9600.\n",
            baud_rate);
    cfsetospeed(&options, B9600);
    break;
  }
  cfsetispeed(&options, cfgetospeed(&options));
  result = tcsetattr(fd, TCSANOW, &options);
  if (result) {
    perror("tcsetattr failed");
    close(fd);
    return -1;
  }
  return fd;
}

// Writes bytes to the serial port, returning 0 on success and -1 on failure.
static int write_port(int fd, uint8_t *buffer, size_t size) {
  ssize_t result = write(fd, buffer, size);
  if (result != (ssize_t)size) {
    perror("failed to write to port");
    return -1;
  }
  return 0;
}

static size_t form_motor_cmd(float pwm, uint8_t *buffer, size_t ptr)
{
  uint16_t pwm_cmd;
  uint8_t cmd;

  pwm_cmd = fabs(pwm) * 65535;
  if (pwm > 0)
    cmd = 0; // forward
  else if (pwm < 0)
    cmd = 1; // backward
  else if (pwm == 0)
    cmd = 2; // idle
  else
    cmd = 3; // brake

  memcpy(buffer + ptr, &pwm_cmd, sizeof(pwm_cmd));
  ptr += sizeof(pwm_cmd);

  memcpy(buffer + ptr, &cmd, sizeof(cmd));
  ptr += sizeof(cmd);

  return ptr;
}

// Reads bytes from the serial port.
// Returns after all the desired bytes have been read, or if there is a
// timeout or other error.
// Returns the number of bytes successfully read into the buffer, or -1 if
// there was an error reading.
static ssize_t read_port(int fd, uint8_t *buffer, size_t size) {
  size_t received = 0;
  while (received < size) {
    ssize_t r = read(fd, buffer + received, size - received);
    if (r < 0) {
      perror("failed to read from port");
      return -1;
    }
    if (r == 0) {
      // Timeout
      break;
    }
    received += r;
  }
  return received;
}

int main(int argc, char const *argv[]) {
  // Choose the serial port name.  If the Jrk is connected directly via USB,
  // Linux USB example:          "/dev/ttyACM0"  (see also: /dev/serial/by-id)
  // macOS USB example:          "/dev/cu.usbmodem001234562"
  const char *device;
  if (argc > 1)
    device = argv[1];
  else
    device = "/dev/ttyACM0";

  uint8_t packet[MAX_BUF_SZ];
  uint8_t kcmd;
  uint8_t buf[MAX_BUF_SZ];
  uint32_t number = 32;
  const void *data;
  size_t data_sz = 0;
  memset(packet, 0, sizeof(packet));

  if (argc > 2) {
    char const *cmd = argv[2];

    if (!strcmp(cmd, "pwm"))
      kcmd = CMD_MOTOR_PWM;
    else
      kcmd = CMD_TYPE_CUST;
  }
  else
    kcmd = CMD_TYPE_CUST;

  if (kcmd == CMD_MOTOR_PWM)
  {
    if (argc > 4)
    {
      size_t ptr = 0;
      ptr = form_motor_cmd(atof(argv[3]), buf, ptr);
      ptr = form_motor_cmd(atof(argv[4]), buf, ptr);

      data = buf;
      data_sz = ptr;
    } else {
      printf("need more parameters");
      return EXIT_FAILURE;
    }
  }
  else
  {
    if (argc > 3)
      number = atoi(argv[3]);
    data = &number;
    data_sz = sizeof(number);
  }

  uint32_t baud_rate = 115200;

  int fd = open_serial_port(device, baud_rate);
  if (fd < 0) {
    return 1;
  }

  // form a packet to be sent:
  size_t sz = 0;
  packet[sz++] = PACK_BEGIN;
  packet[sz++] = 3 + data_sz;    // len of packet
  packet[sz++] = kcmd; // command to be sent
  memcpy(packet + sz, data, data_sz);
  sz += data_sz;
  uint8_t crc = crc8(packet + 2, sz - 2);
  printf("kcmd: %hhu data: %ld crc8: '0x%x'.\n", kcmd, (long)data, crc);
  memcpy(packet + sz, &crc, sizeof(crc));
  sz += sizeof(crc);
  packet[sz++] = PACK_END;

  printf("data sz: %ld\n", (long)data_sz);
  for (size_t i = 0; i < data_sz; ++i)
    printf("0x%x ", ((uint8_t *)data)[i]);
  printf("\n");

  if (!write_port(fd, packet, sz)) {
    printf("written to port sz: %ld\n", (long)sz);
    for (size_t i = 0; i < sz; ++i)
      printf("0x%x ", packet[i]);
    printf("\n");
    ssize_t rec = read_port(fd, packet, sizeof(packet));
    if (rec < 0) {
      printf("Nothing to read from port '%s'.\n", device);
    } else {
      if (rec > 0) {
        packet[rec] = 0;
        printf("received from port '%s', sz: %ld\n", packet, (long)rec);
        for (size_t i = 0; i < rec; ++i)
          printf("0x%x ", packet[i]);
        printf("\n");
      } else
        printf("received nothing from port '%s'.\n", device);
    }
  }

  close(fd);
  return 0;
}
