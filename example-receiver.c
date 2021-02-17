#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "cc1101.h"

int main(int argc, char *argv[])
{
  int handle; // handle system not implemented, use structure instead for one session?
  handle = cc1101_init("/dev/spidev0.0");

  if (handle < 0) return 0;
  /* __u8 testdata[4] = {0x74, 0x38, 0x62, 0x11}; */

  while(IS_STATE(cc1101_get_chip_state(), SETTLING)) {
    printf("device is settling\n");
  }

  unsigned char PKTCTRL1_old;
  PKTCTRL1_old = cc1101_read_config(PKTCTRL1);
  cc1101_write_config(PKTCTRL1, PKTCTRL1_old | 0x08 | 0x80); // 0x08 -> auto CRC Flush, 0x40 -> PQT = 2*4 = 8
  cc1101_read_config(PKTCTRL1);

  // enable frequency synthesizer auto-calibration upon entering rx or tx state from idle every third time
  int MCSM0_old;
  MCSM0_old = cc1101_read_config(MCSM0);
  cc1101_write_config(MCSM0, MCSM0_old | 0x30);
  cc1101_read_config(MCSM0);
  cc1101_set_base_freq(1091553);// frequency increment for roughly 433MHz transmission

  // set relative carrier sense threshold to 6dB
  int AGCCTRL1_old;
  AGCCTRL1_old = cc1101_read_config(AGCCTRL1);
  cc1101_write_config(AGCCTRL1, AGCCTRL1_old | 0x20);
  cc1101_read_config(AGCCTRL1);

  // require both 16/16 sync word recognition aswell as positiver carrier sense threshold
  int MDMCFG2_old;
  MDMCFG2_old = cc1101_read_config(MDMCFG2);
  cc1101_write_config(MDMCFG2, MDMCFG2_old | 0x06);
  cc1101_read_config(MDMCFG2);


  cc1101_write_config(IOCFG0, 0x07);

  cc1101_command_strobe(header_command_scal);

  while(IS_STATE(cc1101_get_chip_state(), CALIBRATE)) {
    printf("device is calibrating. wait.\n");
  }

  cc1101_set_receive();

  while(1) {
    int ret;
    int bytes_avail;


    while(IS_STATE(cc1101_get_chip_state(), CALIBRATE)) {
      printf("device is calibrating. wait.\n");
    }

    if (IS_STATE(cc1101_get_chip_state(), RXFIFO_OVERFLOW)) {
      cc1101_command_strobe(header_command_sfrx);
    }

    bytes_avail = cc1101_rx_fifo_bytes();

    if(bytes_avail > 0) {
      __u8 read_buf[bytes_avail]; // TODO module should hide kernel types?
      cc1101_read_rx_fifo(read_buf, bytes_avail);
    }

    if (IS_STATE(cc1101_get_chip_state(), IDLE)) {
      cc1101_set_receive();
    }

    usleep(500e3);
    /* sleep(5); */
    /* printf("Reading from fifo\n"); */

    /* ret = cc1101_read_rx_fifo(read_buf); */
    /* if (ret > 0) { */
    /*   printf("read bytes %d\n", ret); */
    /* } */
    /* printf("Setting receiver\n"); */
    /* cc1101_set_receive(); */
  }

  /* cc1101_transmit(testdata, 4); */

  // How to access configuration register
  /* spi_write(file, TRANSACTION(READ, 0, ADDR), 0x00); */

  // get amount of bytes in tx queue
  /* spi_write(file, header_status_txbytes, 0x00); */
  cc1101_deinit();

  return 0;
}
