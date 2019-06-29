#include "cc1101_func.h"

static uint8_t debug_level = 0;
static int fd_spi;

// int main(void){
//     uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
//     uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
//     uint8_t rx_addr,sender,lqi;
//     int8_t rssi_dbm;
//     uint8_t addr_rx = 3;
//      cc1101_begin(3, 1);
//         if(packet_available()){
//             get_payload(Rx_fifo, pktlen, rx_addr, sender, &rssi_dbm, lqi);
// 			printf("RSSI: %d dBm\n", rssi_dbm);
// 		}
// }

static void pabort(const char *s){
	perror(s);
	abort();
}
static int spi_init(void){
  int ret = 0;
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 0;
  uint32_t speed = 500000;
  int fd = open(SPI_PATH_CC1101, O_RDWR);
  if (fd < 0) pabort("spi open for cc1101\n");
  //Set mode, bits and speed of spi
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1) pabort("can't set spi mode");
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) pabort("can't get spi mode");
	/* bits per word */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) pabort("can't set bits per word");
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) pabort("can't get bits per word");
	/* max speed hz */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) pabort("can't set max speed hz");
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) pabort("can't get max speed hz");

	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
  return fd;
}
void spi_write_strobe(uint8_t spi_instr){
    int ret;
	uint8_t tx[] = {0, 0};
	uint8_t rx[] = {0, 0};
	tx[0] = spi_instr;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = 1,
		.delay_usecs = 0,
		.speed_hz = 0,
		.bits_per_word = 8,
	};
	ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message write strobe");
}
void spi_write_register(uint8_t spi_instr, uint8_t value){
    int ret;
	uint8_t tx[] = {0, 0};
	uint8_t rx[] = {0, 0};
	tx[0] = spi_instr;
    tx[1] = value;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = 2,
		.delay_usecs = 0,
		.speed_hz = 0,
		.bits_per_word = 8,
	};
	ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message write register");
}
void spi_write_burst(uint8_t spi_instr, uint8_t *pArr, uint8_t len){
    int ret;
    uint8_t tbuf[len + 1];
	uint8_t rx[len + 1];
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tbuf,
		.rx_buf = (unsigned long)rx,
		.len = len + 1,
		.delay_usecs = 0,
		.speed_hz = 0,
		.bits_per_word = 8,
	};
	tbuf[0] = spi_instr | WRITE_BURST;
	for (uint8_t i=0; i<len ;i++ )
	{
		tbuf[i+1] = pArr[i];
	}
	ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message write burst");
}
uint8_t spi_read_register(uint8_t spi_instr){
    int ret;
	uint8_t tx[] = {0, 0};
	uint8_t rx[] = {0, 0};
	tx[0] = spi_instr | READ_SINGLE_BYTE;
    tx[1] = 0x00;
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = 2,
		.delay_usecs = 0,
		.speed_hz = 0,
		.bits_per_word = 8,
	};
	ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message read register");
     return rx[1];
}
void spi_read_burst(uint8_t spi_instr, uint8_t *pArr, uint8_t len){
    int ret;
    uint8_t tbuf[len + 1];
	uint8_t rx[len + 1];
	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tbuf,
		.rx_buf = (unsigned long)rx,
		.len = len + 1,
		.delay_usecs = 0,
		.speed_hz = 0,
		.bits_per_word = 8,
	};
	tbuf[0] = spi_instr | READ_BURST;
	ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message read burst");    
    for (uint8_t i=0; i<len ;i++ )
    {
        pArr[i] = rx[i+1];
    }
}
uint8_t sidle(void){
    uint8_t marcstate;
    spi_write_strobe(SIDLE);              //sets to idle first. must be in
    marcstate = 0xFF;                     //set unknown/dummy state value
    while(marcstate != 0x01){              //0x01 = sidle
        marcstate = (spi_read_register(MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in RX
    }
    usleep(100);
    return TRUE;
}
uint8_t receive(void){
    uint8_t marcstate;
    sidle();                              //sets to idle first.
    spi_write_strobe(SRX);                //writes receive strobe (receive mode)
    marcstate = 0xFF;                     //set unknown/dummy state value
    while(marcstate != 0x0D){              //0x0D = RX
        marcstate = (spi_read_register(MARCSTATE) & 0x1F); //read out state of cc1100 to be sure in RX
    }
    usleep(100);
    return TRUE;
}
void reset_cc1101(void){
	spi_write_strobe(SRES);
    sleep(1);
}
void set_mode(uint8_t mode){
    switch (mode){
        case 0x01:  spi_write_burst(WRITE_BURST,cc1100_GFSK_1_2_kb,CFG_REGISTER);   break;
        case 0x02:  spi_write_burst(WRITE_BURST,cc1100_GFSK_38_4_kb,CFG_REGISTER);  break;
        case 0x03:  spi_write_burst(WRITE_BURST,cc1100_GFSK_100_kb,CFG_REGISTER);   break;
        case 0x04:  spi_write_burst(WRITE_BURST,cc1100_MSK_250_kb,CFG_REGISTER);    break;
        case 0x05:  spi_write_burst(WRITE_BURST,cc1100_MSK_500_kb,CFG_REGISTER);    break;
        case 0x06:  spi_write_burst(WRITE_BURST,cc1100_OOK_4_8_kb,CFG_REGISTER);    break;
        default:    spi_write_burst(WRITE_BURST,cc1100_GFSK_100_kb,CFG_REGISTER);   break;
    }
    return;
}
void set_ISM(uint8_t ism_freq){
    uint8_t freq2, freq1, freq0;
    switch (ism_freq){                                                      //loads the RF freq which is defined in cc1100_freq_select
        case 0x01:                                                          //315MHz
                    freq2=0x0C;
                    freq1=0x1D;
                    freq0=0x89;
                    spi_write_burst(PATABLE_BURST,patable_power_315,8);
                    break;
        case 0x02:                                                          //433.92MHz
                    freq2=0x10;
                    freq1=0xB0;
                    freq0=0x71;
                    spi_write_burst(PATABLE_BURST,patable_power_433,8);
                    break;
        case 0x03:                                                          //868.3MHz
                    freq2=0x21;
                    freq1=0x65;
                    freq0=0x6A;
                    spi_write_burst(PATABLE_BURST,patable_power_868,8);
                    break;
        case 0x04:                                                          //915MHz
                    freq2=0x23;
                    freq1=0x31;
                    freq0=0x3B;
                    spi_write_burst(PATABLE_BURST,patable_power_915,8);
                    break;
        default:                                                             //default is 868.3MHz
                    freq2=0x21;
                    freq1=0x65;
                    freq0=0x6A;
                    spi_write_burst(PATABLE_BURST,patable_power_868,8);    //sets up output power ramp register
                    break;
    }
    spi_write_register(FREQ2,freq2);                                         //stores the new freq setting for defined ISM band
    spi_write_register(FREQ1,freq1);
    spi_write_register(FREQ0,freq0);
}
void set_channel(uint8_t channel){
    spi_write_register(CHANNR,channel);   //stores the new channel # in the CC1100
}
void set_output_power_level(int8_t dBm){
    uint8_t pa = 0xC0;
    if      (dBm <= -30) pa = 0x00;
    else if (dBm <= -20) pa = 0x01;
    else if (dBm <= -15) pa = 0x02;
    else if (dBm <= -10) pa = 0x03;
    else if (dBm <= 0)   pa = 0x04;
    else if (dBm <= 5)   pa = 0x05;
    else if (dBm <= 7)   pa = 0x06;
    else if (dBm <= 10)  pa = 0x07;
    spi_write_register(FREND0,pa);
}
void set_myaddr(uint8_t addr){
    spi_write_register(ADDR,addr);          //stores MyAddr in the CC1100
}
uint8_t begin(uint8_t addr, uint8_t channel){
	uint8_t partnum, version;
	uint8_t cc1100_mode_select = 3;
	uint8_t cc1100_freq_select = 2;
	uint8_t cc1100_channel_select = channel;
	reset_cc1101();
	spi_write_strobe(SFTX); 	usleep(100);
	spi_write_strobe(SFRX); 	usleep(100);
	partnum = spi_read_register(PARTNUM); //reads CC1100 partnumber
    version = spi_read_register(VERSION); //reads CC1100 version number 
	if(version == 0x00 || version == 0xFF){
		printf("no CC11xx found!\r\n");
		return FALSE;
	}
	printf("Partnumber: 0x%02X\r\n", partnum);
	printf("Version   : 0x%02X\r\n", version);
	set_mode(cc1100_mode_select);
	set_ISM(cc1100_freq_select);
	set_channel(cc1100_channel_select);
	set_output_power_level(0);
	set_myaddr(addr);
	receive();
}
uint8_t cc1101_begin(uint8_t address, uint8_t channel){
    fd_spi = spi_init();
    if (fd_spi < 0){
        printf("error spi init\n");
        return -1;
    }
    begin(address, channel);
    receive();
}

void show_register_settings(void){
    if(debug_level > 0){
        uint8_t config_reg_verify[CFG_REGISTER],Patable_verify[CFG_REGISTER];

        spi_read_burst(READ_BURST,config_reg_verify,CFG_REGISTER);  //reads all 47 config register from cc1100
        spi_read_burst(PATABLE_BURST,Patable_verify,8);             //reads output power settings from cc1100

        //show_main_settings();
        printf("Config Register:\r\n");

        for(uint8_t i = 0 ; i < CFG_REGISTER; i++)  //showes rx_buffer for debug
        {
            printf("0x%02X ", config_reg_verify[i]);
            if(i==9 || i==19 || i==29 || i==39) //just for beautiful output style
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
        printf("PaTable:\r\n");

        for(uint8_t i = 0 ; i < 8; i++)         //showes rx_buffer for debug
        {
            printf("0x%02X ", Patable_verify[i]);
        }
        printf("\r\n");
    }
}
void rx_fifo_erase(uint8_t *rxbuffer){
    memset(rxbuffer, 0, sizeof(FIFOBUFFER)); //erased the RX_fifo array content to "0"
}
uint8_t rx_payload_burst(uint8_t rxbuffer[], uint8_t *pktlen){
    uint8_t bytes_in_RXFIFO = 0;
    uint8_t res = 0;

    bytes_in_RXFIFO = spi_read_register(RXBYTES);              //reads the number of bytes in RXFIFO

    if((bytes_in_RXFIFO & 0x7F) && !(bytes_in_RXFIFO & 0x80))  //if bytes in buffer and no RX Overflow
    {
        spi_read_burst(RXFIFO_BURST, rxbuffer, bytes_in_RXFIFO);
        *pktlen = rxbuffer[0];
        res = TRUE;
    }
    else
    {
        if(debug_level > 0){
            printf("no bytes in RX buffer or RX Overflow!: ");printf("0x%02X \r\n", bytes_in_RXFIFO);
        }
        sidle();                                                  //set to IDLE
        spi_write_strobe(SFRX);usleep(100);            			//flush RX Buffer
        receive();                                                //set to receive mode
        res = FALSE;
    }

    return res;
}
uint8_t rssi_convert(uint8_t Rssi_hex){
    int8_t rssi_dbm;
    int16_t Rssi_dec;
    Rssi_dec = Rssi_hex;        //convert unsigned to signed
    if(Rssi_dec >= 128){
        rssi_dbm=((Rssi_dec-256)/2)-RSSI_OFFSET_868MHZ;
    }
    else{
        if(Rssi_dec<128){
            rssi_dbm=((Rssi_dec)/2)-RSSI_OFFSET_868MHZ;
        }
    }
    return rssi_dbm;
}
uint8_t lqi_convert(uint8_t lqi){
    return (lqi & 0x7F);
}
uint8_t check_crc(uint8_t lqi){
    return (lqi & 0x80);
}
uint8_t check_acknowledge(uint8_t *rxbuffer, uint8_t pktlen, uint8_t sender, uint8_t my_addr){
    int8_t rssi_dbm;
    uint8_t crc, lqi;

    if((pktlen == 0x05 && \
        rxbuffer[1] == my_addr || rxbuffer[1] == BROADCAST_ADDRESS) && \
        rxbuffer[2] == sender && \
        rxbuffer[3] == 'A' && rxbuffer[4] == 'c' && rxbuffer[5] == 'k')   //acknowledge received!
        {
            if(rxbuffer[1] == BROADCAST_ADDRESS){                           //if receiver address BROADCAST_ADDRESS skip acknowledge
                if(debug_level > 0){
                    printf("BROADCAST ACK\r\n");
                }
                return FALSE;
            }
            rssi_dbm = rssi_convert(rxbuffer[pktlen + 1]);
            lqi = lqi_convert(rxbuffer[pktlen + 2]);
            crc = check_crc(lqi);

            if(debug_level > 0){
                printf("ACK! ");
                printf("RSSI:%i ",rssi_dbm);
                printf("LQI:0x%02X ",lqi);
                printf("CRC:0x%02X\r\n",crc);
            }
            return TRUE;
        }
    return FALSE;
}

uint8_t get_payload(uint8_t rxbuffer[], uint8_t pktlen, uint8_t my_addr, uint8_t sender, int8_t *rssi_dbm, uint8_t lqi){
    uint8_t crc;

    rx_fifo_erase(rxbuffer);                               //delete rx_fifo bufffer

    if(rx_payload_burst(rxbuffer, &pktlen) == FALSE)        //read package in buffer
    {
        rx_fifo_erase(rxbuffer);                           //delete rx_fifo bufffer
        return FALSE;                                    //exit
    }
    else
    {
        my_addr = rxbuffer[1];                             //set receiver address to my_addr
        sender = rxbuffer[2];

        if(check_acknowledge(rxbuffer, pktlen, sender, my_addr) == TRUE) //acknowlage received?
        {
            rx_fifo_erase(rxbuffer);                       //delete rx_fifo bufffer
            return FALSE;                                //Ack received -> finished
        }
        else                                               //real data, and sent acknowladge
        {
            *rssi_dbm = rssi_convert(rxbuffer[pktlen + 1]); //converts receiver strength to dBm
            lqi = lqi_convert(rxbuffer[pktlen + 2]);       //get rf quialtiy indicator
            crc = check_crc(lqi);                          //get packet CRC

            if(debug_level > 0){                           //debug output messages
                if(rxbuffer[1] == BROADCAST_ADDRESS)       //if my receiver address is BROADCAST_ADDRESS
                {
                    printf("BROADCAST message\r\n");
                }

                printf("RX_FIFO:");
                for(uint8_t i = 0 ; i < pktlen + 1; i++)   //showes rx_buffer for debug
                {
                    printf("0x%02X ", rxbuffer[i]);
                }
                printf("| 0x%02X 0x%02X |", rxbuffer[pktlen+1], rxbuffer[pktlen+2]);
                printf("\r\n");

                printf("RSSI:%d ", *rssi_dbm);
                printf("LQI:");printf("0x%02X ", lqi);
                printf("CRC:");printf("0x%02X ", crc);
                printf("\r\n");
            }

            my_addr = rxbuffer[1];                         //set receiver address to my_addr
            sender = rxbuffer[2];                          //set from_sender address

            // if(my_addr != BROADCAST_ADDRESS)               //send only ack if no BROADCAST_ADDRESS
            // {
            //     sent_acknowledge(my_addr, sender);           //sending acknowlage to sender!
            // }

            return TRUE;
        }
        return FALSE;
    }
}
uint8_t packet_available(void){
    //if RF package received
    if(digitalRead(GDO2) == TRUE)
    {
        return TRUE;
    }
    return FALSE;
}