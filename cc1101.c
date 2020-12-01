#include "cc1101.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

static const char STATES[8][20]   = {
  "IDLE"
  , "RX"
  , "TX"
  , "FSTXON"
  , "CALIBRATE"
  , "SETTLING"
  , "RXFIFO_OVERFLOW"
  , "TXFIFO_UNDERFLOW"
};
static const char STROBES[14][10] = {
  "sres"
  , "sfstxon"
  , "sxoff"
  , "scal"
  , "srx"
  , "stx"
  , "sidle"
  , ""
  ,  "swor"
  , "spwd"
  , "sfrx"
  , "sftx"
  , "sworrst"
  , "snop"
};
static const char STATUS[18][14] = {
  "partnum"
  , "version"
  , "freqest"
  , "lqi"
  , "rssi"
  , "marcstate"
  , "wortime1"
  , "wortime0"
  , "pktstatus"
  , "vco_vc_dac"
  , "txbytes"
  , "rxbytes"
  , "rcctrl1_status"
  , "rcctrl0_status"
};

int spi_dev_file;

int cc1101_init(char* spi_device) {
  __u8 mode, lsb, bits;
  __u32 speed;

  if ((spi_dev_file = open(spi_device, O_RDWR)) < 0 ) {
      printf("Failed to open the spidev0.1\n");
      exit(1);
  }

  // Device defaults
  /* mode = SPI_CPOL|SPI_CS_HIGH|SPI_CPHA|0; */
  mode = SPI_CPOL|SPI_CPHA|0;
  if (ioctl(spi_dev_file, SPI_IOC_WR_MODE, &mode) < 0) {
    perror("Can't set spi mode");
    return -1;
  }

  lsb = 0; // Most signification bit first
  if (ioctl(spi_dev_file, SPI_IOC_WR_LSB_FIRST, &lsb) < 0) {
    perror("Can't set spi lsb");
    return -1;
  }

  bits = 0;
  if (ioctl(spi_dev_file, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
    perror("Can't set spi bits per word");
    return -1;
  }

  speed = 500000;
  if (ioctl(spi_dev_file, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
    perror("Can't set spi max speed");
    return -1;
  }

  cc1101_command_strobe(header_command_sres);
  cc1101_command_strobe(header_command_sidle);
  return 1; // return handle (If implementation with handle)
}

void cc1101_deinit() {
  cc1101_command_strobe(header_command_sres);
  close(spi_dev_file);
};

// 8 bits per word, e.g. N bytes of data will result in N seperate transmissions
// TODO swap len and read
static void spi_access(__u8* data, int len, __u8 *read) {
  int ret;
  __u8 write_buf[len];
  __u8 read_buf[len];
  struct spi_ioc_transfer transfers[len];

  memset(transfers, 0, sizeof transfers);
  memset(write_buf, 0, sizeof write_buf);
  memset(read_buf,  0, sizeof read_buf);

  memcpy(write_buf, data, len);

  /* printf("sending: "); */
  for (size_t i = 0; i < len; ++i) {
    transfers[i].len = 1; /* number of bytes to write */
    transfers[i].cs_change = 0; /* Keep CS activated */
    /* channil[0].word_delay_usecs = 1; */
    transfers[i].delay_usecs = 1;
    transfers[i].speed_hz = 500000;
    transfers[i].bits_per_word = 8;
    transfers[i].rx_buf = (__u64) (read_buf+i);
    transfers[i].tx_buf = (__u64) (write_buf+i);
    /* printf("0x%02X ", write_buf[i]); */
  }// printf("\n");

  if (ret = ioctl(spi_dev_file, SPI_IOC_MESSAGE(len), transfers) < 0) {
    perror("SPI_IOC_MESSAGE");
    return;
  }

  /* printf("read: "); */
  /* for (size_t i = 0; i < len; ++i) { */
  /*   printf("0x%02X ", read_buf[i]); */
  /* } printf("\n"); */

  memcpy(read, read_buf, len);
}

__u8 cc1101_command_strobe(__u8 strobe) {
  /* printf("sending command %s\n", PRETTY_STROBE(strobe)); */
  __u8 read[2];
  __u8 data[2] = {strobe, 0x00};
  spi_access(data, 2, read);

  /* printf("read[0] = 0x%02X, read[1] = 0x%02X\n", read[0], read[1]); */
  printf("\n");

  return read[1];
}

// Transmit len number of bytes TODO add address
void cc1101_write_tx_fifo(__u8* data, size_t len) {
  /* for (size_t i = 0; i < 64; ++i) { */
    __u8 read[len+1];
    __u8 data_w_header[len+1];

    memcpy (data_w_header+1, data, len);
    data_w_header[0] = header_burst_tx_fifo;
    // TODO split into multiple transmission if len exceeds 64 byte

    spi_access(data_w_header, len+1, read);
}

// Return number of bytes read from FIFO or -1 if device in wrong mode
// len: amount of bytes to read from fifo
int cc1101_read_rx_fifo(__u8 *read, size_t len) {
  __u8 ret;
  __u8 fifo_bytes;
  __u8 read_buf[len+1];
  __u8 data[len+1];
  memset(data, 0 , len);
  data[0] = header_burst_rx_fifo;

  __u8 cc1101_get_chip_state();
  // check FIFO for content
  ret = cc1101_command_strobe(header_command_snop);

  // TODO while(IS_STATE(ret, SETTLING)) loop?
  if (!(IS_STATE(ret, RX_MODE) | IS_STATE(ret, RXFIFO_OVERFLOW)))
    return -1;


  spi_access(data, len+1, read_buf);
  printf("0x%02X bytes in RX FIFO\n", fifo_bytes);
  memcpy(read, read_buf+1, len);

  printf("read from fifo: ");
  for(size_t i = 0; i < len; i++) {
    printf("0x%02X ", read[i]);
  } printf("\n");
  /* if (ret & FIFO_BITS >= 0x03) { */
  /*   fifo_bytes = cc1101_read_status_reg(header_status_rxbytes); */
  /* } */
}

//
void cc1101_set_receive() {
  __u8 ret;
  cc1101_get_chip_state();
  /* if (!IS_STATE(ret, SETTLING)) */
  /*   return -1; */

  cc1101_command_strobe(header_command_srx);
  cc1101_get_chip_state();
}

void cc1101_set_transmit() {
  __u8 ret;
  cc1101_get_chip_state();
  /* if (!IS_STATE(ret, SETTLING)) */
  /*   return -1; */

  cc1101_command_strobe(header_command_stx);
  cc1101_get_chip_state();
}

// read status register, use header_status_ definitions from cc1101.h
__u8 cc1101_read_status_reg(__u8 header) {
  __u8 read[2];
  __u8 data[2] = {header, 0x00};
  printf("reading status register %s\n", PRETTY_STATUS(header));
  spi_access(data, 2, read);
  printf("Received 0x%02X\n\n", read[1]);
  return read[1];
}

void cc1101_write_config(__u8 config, __u8 value) {
  __u8 read[2];
  __u8 data[2] = {TRANSACTION(WRITE, SINGLE, config), value};
  spi_access(data, 2, read);
  printf("writing to config 0x%02X value 0x%02X\n\n", config, value);
}

__u8 cc1101_read_config(__u8 config) {
  __u8 read[2];
  __u8 data[2] = {TRANSACTION(READ, SINGLE, config), 0x00};
  spi_access(data, 2, read);
  printf("reading from config 0x%02X value 0x%02X\n\n", config, read[1]);
  return read[1];
}

__u8 cc1101_rx_fifo_bytes() {
  return cc1101_read_status_reg(header_status_rxbytes & 0x7F);
}

__u8 cc1101_tx_fifo_bytes() {
  return cc1101_read_status_reg(header_status_txbytes & 0x7F);
}

__u8 cc1101_get_chip_state() {
  __u8 ret;
  ret = cc1101_command_strobe(header_command_snop);
  printf("status: %s\n\n", PRETTY_STATE(ret));

  return ret & STATE_BITS;
}
