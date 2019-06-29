#include "mcp2515.h"

static uint8_t mcpMode;
static unsigned long  can_id;
static uint8_t  ext_flg;
static uint8_t  rtr;
static uint8_t nReservedTx = 0;
static int fd_spi;

static void pabort(const char *s){
	perror(s);
	abort();
}
static int spi_init(void){
  int ret = 0;
  uint8_t mode = SPI_MODE_0;
  uint8_t bits = 0;
  uint32_t speed = 500000;
  int fd = open(SPI_PATH, O_RDWR);
  if (fd < 0){
        printf("error spi open\n");
        return -1;
  }
  //Set mode, bits and speed of spi
  ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");
	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
	/* bits per word */
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");
	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");
	/* max speed hz */
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");
	printf("spi mode: %d\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);
  return fd;
}
uint8_t spi_readwrite(uint8_t *buf, uint8_t len){
  int ret;
  uint8_t tx[len];
	uint8_t rx[len];

  int i;
  for (i = 0; i < len; i++)
    tx[i] = buf[i];

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)buf,
		.rx_buf = (unsigned long)rx,
		.len = len,
		.delay_usecs = 0,
		.speed_hz = 0,
		.bits_per_word = 8,
	};
	ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1)
		printf("can't send spi message write register");  

  for (i = 0; i < len; i++)
    buf[i] = rx[i];
}
uint8_t mcp2515_readRegister(const uint8_t address){
    uint8_t buf[3] = {MCP_READ, address, 0x00};
    spi_readwrite(buf, 3);
    return ( buf[2] );
}
void mcp2515_setRegister(const uint8_t address, const uint8_t value){
    uint8_t buf[3] = {MCP_WRITE, address, value};
    spi_readwrite(buf, 3);
}
void mcp2515_modifyRegister(const uint8_t address, const uint8_t mask, const uint8_t data){
    uint8_t buf[4] = {MCP_BITMOD, address, mask, data};
    spi_readwrite(buf, 4);   
}
void mcp2515_setRegisterS(const byte address, const byte values[], const byte n){
    uint8_t i;
    uint8_t buf_spi[2+n];
    buf_spi[0] = MCP_WRITE;
    buf_spi[1] = address;
    for (i=0; i<n; i++)
      buf_spi[2 + i] = values[i];
    spi_readwrite(buf_spi, (2 + n));
}
uint8_t mcp2515_readStatus(void){
    uint8_t buf[2] = {MCP_READ_STATUS, 0x00};
    spi_readwrite(buf, 2);
    return (buf[1]);
}
uint8_t getMode(void){
    return mcp2515_readRegister(MCP_CANSTAT) & MODE_MASK;
}
uint8_t txStatusPendingFlag(uint8_t i) {
  switch (i) {
    case 0: return MCP_STAT_TX0_PENDING;
    case 1: return MCP_STAT_TX1_PENDING;
    case 2: return MCP_STAT_TX2_PENDING;
  }
  return 0xff;
}
uint8_t txCtrlReg(uint8_t i) {
  switch (i) {
    case 0: return MCP_TXB0CTRL;
    case 1: return MCP_TXB1CTRL;
    case 2: return MCP_TXB2CTRL;
  }
  return MCP_TXB2CTRL;
}
uint8_t txIfFlag(uint8_t i) {
  switch (i) {
    case 0: return MCP_TX0IF;
    case 1: return MCP_TX1IF;
    case 2: return MCP_TX2IF;
  }
  return 0;
}
uint8_t statusToTxSidh(uint8_t status){
  switch ( status ){
    case MCP_TX0IF : return MCP_TXB0SIDH;
    case MCP_TX1IF : return MCP_TXB1SIDH;
    case MCP_TX2IF : return MCP_TXB2SIDH;
  }
  return 0;
}
uint8_t txSidhToTxLoad(uint8_t sidh) {
  switch (sidh) {
    case MCP_TXB0SIDH: return MCP_LOAD_TX0;
    case MCP_TXB1SIDH: return MCP_LOAD_TX1;
    case MCP_TXB2SIDH: return MCP_LOAD_TX2;
  }
  return 0;
}
uint8_t txSidhToRTS(uint8_t sidh) {
  switch (sidh) {
    case MCP_TXB0SIDH: return MCP_RTS_TX0;
    case MCP_TXB1SIDH: return MCP_RTS_TX1;
    case MCP_TXB2SIDH: return MCP_RTS_TX2;
  }
  return 0;
}
void mcp2515_id_to_buf(const uint8_t ext, const unsigned long id, uint8_t *tbufdata){
    uint16_t canid;
    canid = (uint16_t)(id & 0x0FFFF);
    if ( ext == 1)
    {
        tbufdata[MCP_EID0] = (uint8_t) (canid & 0xFF);
        tbufdata[MCP_EID8] = (uint8_t) (canid >> 8);
        canid = (uint16_t)(id >> 16);
        tbufdata[MCP_SIDL] = (uint8_t) (canid & 0x03);
        tbufdata[MCP_SIDL] += (uint8_t) ((canid & 0x1C) << 3);
        tbufdata[MCP_SIDL] |= MCP_TXB_EXIDE_M;
        tbufdata[MCP_SIDH] = (uint8_t) (canid >> 5 );
    }
    else
    {
        tbufdata[MCP_SIDH] = (byte) (canid >> 3 );
        tbufdata[MCP_SIDL] = (byte) ((canid & 0x07 ) << 5);
        tbufdata[MCP_EID0] = 0;
        tbufdata[MCP_EID8] = 0;
    }
}
void mcp2515_start_transmit(const uint8_t mcp_addr){
    uint8_t buf = txSidhToRTS(mcp_addr);
    spi_readwrite(&buf, 1);
}
void mcp2515_write_id(const uint8_t mcp_addr, const uint8_t ext, const unsigned long id){
    uint8_t tbufdata[4];
    mcp2515_id_to_buf(ext,id,tbufdata);
    mcp2515_setRegisterS(mcp_addr, tbufdata, 4);
}
uint8_t mcp2515_requestNewMode(const uint8_t newmode){
    int i = 0;

    while (i < 200){
        mcp2515_modifyRegister(MCP_CANCTRL, MODE_MASK, newmode);
		uint8_t statReg = mcp2515_readRegister(MCP_CANSTAT);

		if((statReg & MODE_MASK) == newmode)
			return MCP2515_OK;
        usleep(1000);
        i++;
    }
    return MCP2515_FAIL;
}
uint8_t mcp2515_setCANCTRL_Mode(const uint8_t newmode){
    if (getMode() == MODE_SLEEP && newmode != MODE_SLEEP){

        uint8_t wakeIntEnabled = (mcp2515_readRegister(MCP_CANINTE) & MCP_WAKIF);        
        if (!wakeIntEnabled)
            mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, MCP_WAKIF);

        mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, MCP_WAKIF);

        if(mcp2515_requestNewMode(MODE_LISTENONLY) != MCP2515_OK)
			return MCP2515_FAIL;

		if(!wakeIntEnabled)
			mcp2515_modifyRegister(MCP_CANINTE, MCP_WAKIF, 0);
    }
    mcp2515_modifyRegister(MCP_CANINTF, MCP_WAKIF, 0);    
    return mcp2515_requestNewMode(newmode);
}
uint8_t setMode(const uint8_t opMode){
    if(opMode != MODE_SLEEP)
		mcpMode = opMode;
    return mcp2515_setCANCTRL_Mode(opMode);
}

uint8_t init_Mask(uint8_t num, uint8_t ext, unsigned long ulData)
{
    byte res = MCP2515_OK;
    usleep(10);
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (res > 0) {
        usleep(10);
        return res;
    }

    if (num == 0) {
        mcp2515_write_id(MCP_RXM0SIDH, ext, ulData);

    }
    else if (num == 1) {
        mcp2515_write_id(MCP_RXM1SIDH, ext, ulData);
    }
    else res =  MCP2515_FAIL;

    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if (res > 0) {
        usleep(10);
        return res;
    }
    usleep(10);
    return res;
}

uint8_t init_Filt(uint8_t num, uint8_t ext, unsigned long ulData)
{
    byte res = MCP2515_OK;
    usleep(10);
    res = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (res > 0)
    {
        usleep(10);
        return res;
    }

    switch ( num )
    {
      case 0:
        mcp2515_write_id(MCP_RXF0SIDH, ext, ulData);
        break;

      case 1:
        mcp2515_write_id(MCP_RXF1SIDH, ext, ulData);
        break;

      case 2:
        mcp2515_write_id(MCP_RXF2SIDH, ext, ulData);
        break;

      case 3:
        mcp2515_write_id(MCP_RXF3SIDH, ext, ulData);
        break;

      case 4:
        mcp2515_write_id(MCP_RXF4SIDH, ext, ulData);
        break;

      case 5:
        mcp2515_write_id(MCP_RXF5SIDH, ext, ulData);
        break;

      default:
        res = MCP2515_FAIL;
    }
    res = mcp2515_setCANCTRL_Mode(mcpMode);
    if (res > 0){
        usleep(10);
        return res;
    }
    usleep(10);

    return res;
}

unsigned long getCanId(void)
{
    return can_id;
}

uint8_t checkReceive(void)
{
    uint8_t res;
    res = mcp2515_readStatus();                                         // RXnIF in Bit 1 and 0
    return ((res & MCP_STAT_RXIF_MASK)?CAN_MSGAVAIL:CAN_NOMSG);
}

uint8_t readRxTxStatus(void)
{
  uint8_t ret=( mcp2515_readStatus() & ( MCP_STAT_TXIF_MASK | MCP_STAT_RXIF_MASK ) );
  ret=(ret & MCP_STAT_TX0IF ? MCP_TX0IF : 0) |
      (ret & MCP_STAT_TX1IF ? MCP_TX1IF : 0) |
      (ret & MCP_STAT_TX2IF ? MCP_TX2IF : 0) |
      (ret & MCP_STAT_RXIF_MASK); // Rx bits happend to be same on status and MCP_CANINTF
  return ret;
}

void mcp2515_read_canMsg( const uint8_t buffer_load_addr, volatile unsigned long *id, volatile uint8_t *ext, volatile uint8_t *rtrBit, volatile uint8_t *len, volatile uint8_t *buf)        /* read can msg                 */
{
  uint8_t tbufdata[4];
  uint8_t i;
  uint8_t bufspi[14];
  bufspi[0] = buffer_load_addr;
  for (i = 1; i < (14); i++)
    bufspi[i] = 0x00;
  spi_readwrite(bufspi, (14));

  // mcp2515 has auto-increment of address-pointer
  for (i = 0; i < 4; i++) tbufdata[i] = bufspi[i+1];

  *id = (tbufdata[MCP_SIDH] << 3) + (tbufdata[MCP_SIDL] >> 5);
  *ext = 0;
  if ( (tbufdata[MCP_SIDL] & MCP_TXB_EXIDE_M) ==  MCP_TXB_EXIDE_M )
  {
    /* extended id                  */
    *id = (*id << 2) + (tbufdata[MCP_SIDL] & 0x03);
    *id = (*id << 8) + tbufdata[MCP_EID8];
    *id = (*id << 8) + tbufdata[MCP_EID0];
    *ext = 1;
  }

  uint8_t pMsgSize = bufspi[5];
  *len = pMsgSize & MCP_DLC_MASK;
  *rtrBit = (pMsgSize & MCP_RTR_MASK) ? 1 : 0;
  for (i = 0; i < *len && i<CAN_MAX_CHAR_IN_MESSAGE; i++) {
    buf[i] = bufspi[6 + i];
  }
}

uint8_t readMsgBufID(uint8_t status, volatile unsigned long *id, volatile uint8_t *ext, volatile uint8_t *rtrBit, volatile uint8_t *len, volatile uint8_t *buf)
{
  uint8_t rc=CAN_NOMSG;

  if ( status & MCP_RX0IF )                                        // Msg in Buffer 0
  {
    mcp2515_read_canMsg( MCP_READ_RX0, id, ext, rtrBit, len, buf);
    rc = CAN_OK;
  }
  else if ( status & MCP_RX1IF )                                   // Msg in Buffer 1
  {
    mcp2515_read_canMsg( MCP_READ_RX1, id, ext, rtrBit, len, buf);
    rc = CAN_OK;
  }

  if (rc==CAN_OK) {
    rtr=*rtrBit;
    // dta_len=*len; // not used on any interface function
    ext_flg=*ext;
    can_id=*id;
  } else {
    *len=0;
  }

  return rc;
}

uint8_t readMsgBuf(uint8_t *len, uint8_t buf[])
{
    return readMsgBufID(readRxTxStatus(),&can_id,&ext_flg,&rtr,len,buf);
}

uint8_t mcp2515_getNextFreeTXBuf(uint8_t *txbuf_n){
    uint8_t status=mcp2515_readStatus() & MCP_STAT_TX_PENDING_MASK;
    uint8_t i;

    *txbuf_n = 0x00;

    if ( status==MCP_STAT_TX_PENDING_MASK ) return MCP_ALLTXBUSY; 

    for (i = 0; i < MCP_N_TXBUFFERS-nReservedTx; i++)
    {
      if ( (status & txStatusPendingFlag(i) ) == 0 ) {
        *txbuf_n = txCtrlReg(i) + 1;                                   // return SIDH-address of Buffer
        mcp2515_modifyRegister(MCP_CANINTF, txIfFlag(i), 0);
        return MCP2515_OK;                                                 // ! function exit
      }
    }
    return MCP_ALLTXBUSY;
}
void mcp2515_write_canMsg(const uint8_t buffer_sidh_addr, unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, volatile const uint8_t *buf){
  uint8_t load_addr=txSidhToTxLoad(buffer_sidh_addr);

  uint8_t tbufdata[4];
  uint8_t dlc = len | ( rtrBit ? MCP_RTR_MASK : 0 ) ;
  uint8_t i;
  uint8_t buf_spi[6 + len];

  mcp2515_id_to_buf(ext,id,tbufdata);

  buf_spi[0] = load_addr;
  for (i = 0; i < 4; i++) buf_spi[i + 1] = tbufdata[i];
  buf_spi[5] = dlc;
  for (i = 0; i < len && i<CAN_MAX_CHAR_IN_MESSAGE; i++) buf_spi[i + 6] = buf[i];
  spi_readwrite(buf_spi, 6 + len);
  mcp2515_start_transmit( buffer_sidh_addr );
}
uint8_t sendMsg(unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, const uint8_t *buf, uint8_t wait_sent){
    uint8_t res, res1, txbuf_n;
    uint16_t uiTimeOut = 0;

    can_id = id;
    ext_flg = ext;
    rtr = rtrBit;

    do {
        if (uiTimeOut > 0) usleep(10);
        res = mcp2515_getNextFreeTXBuf(&txbuf_n);                       // info = addr.
        uiTimeOut++;
    } while (res == MCP_ALLTXBUSY && (uiTimeOut < TIMEOUTVALUE));

    if (uiTimeOut == TIMEOUTVALUE)
    {
        return CAN_GETTXBFTIMEOUT;                                      // get tx buff time out
    }
    mcp2515_write_canMsg(txbuf_n, id, ext, rtrBit, len, buf);

    if (wait_sent) {
      uiTimeOut = 0;
      do
      {
        if (uiTimeOut > 0) usleep(10);
          uiTimeOut++;
          res1 = mcp2515_readRegister(txbuf_n - 1);  // read send buff ctrl reg
          res1 = res1 & 0x08;
      } while (res1 && (uiTimeOut < TIMEOUTVALUE));

      if (uiTimeOut == TIMEOUTVALUE)                                       // send msg timeout
      {
          return CAN_SENDMSGTIMEOUT;
      }
    }
    return CAN_OK;
}
uint8_t sendMsgBuf(unsigned long id, uint8_t ext, uint8_t len, const uint8_t *buf, uint8_t wait_sent){
    return sendMsg(id,ext,0,len,buf,wait_sent);
}








// uint8_t sendMsgBuf(uint8_t status, unsigned long id, uint8_t ext, uint8_t rtrBit, uint8_t len, volatile const uint8_t *buf)
// {
//     uint8_t txbuf_n=statusToTxSidh(status);

//     if ( txbuf_n==0 ) return CAN_FAILTX; // Invalid status

//     mcp2515_modifyRegister(MCP_CANINTF, status, 0);  // Clear interrupt flag
//     mcp2515_write_canMsg(txbuf_n, id, ext, rtrBit, len, buf);

//     return CAN_OK;
// }















void mcp2515_reset(void){
    uint8_t buf = MCP_RESET;
    spi_readwrite(&buf, 1);
    usleep(10);
}









void mcp2515_initCANBuffers(void){
    uint8_t i, a1, a2, a3;

    a1 = MCP_TXB0CTRL;
    a2 = MCP_TXB1CTRL;
    a3 = MCP_TXB2CTRL;
    for (i = 0; i < 14; i++)                         // in-buffer loop
    {
      mcp2515_setRegister(a1, 0);
      mcp2515_setRegister(a2, 0);
      mcp2515_setRegister(a3, 0);
      a1++;
      a2++;
      a3++;
    }
    mcp2515_setRegister(MCP_RXB0CTRL, 0);
    mcp2515_setRegister(MCP_RXB1CTRL, 0);
}



uint8_t mcp2515_configRate(const uint8_t canSpeed, const uint8_t clockset){
  uint8_t set, cfg1, cfg2, cfg3;
  set = 1;
  switch (clockset){
    case (MCP_16MHz):
      switch (canSpeed){
        case (CAN_5KBPS):
          cfg1 = MCP_16MHz_5kBPS_CFG1;
          cfg2 = MCP_16MHz_5kBPS_CFG2;
          cfg3 = MCP_16MHz_5kBPS_CFG3;
          break;

        case (CAN_10KBPS):
          cfg1 = MCP_16MHz_10kBPS_CFG1;
          cfg2 = MCP_16MHz_10kBPS_CFG2;
          cfg3 = MCP_16MHz_10kBPS_CFG3;
          break;

        case (CAN_20KBPS):
          cfg1 = MCP_16MHz_20kBPS_CFG1;
          cfg2 = MCP_16MHz_20kBPS_CFG2;
          cfg3 = MCP_16MHz_20kBPS_CFG3;
          break;

        case (CAN_25KBPS):
          cfg1 = MCP_16MHz_25kBPS_CFG1;
          cfg2 = MCP_16MHz_25kBPS_CFG2;
          cfg3 = MCP_16MHz_25kBPS_CFG3;
          break;

        case (CAN_31K25BPS):
          cfg1 = MCP_16MHz_31k25BPS_CFG1;
          cfg2 = MCP_16MHz_31k25BPS_CFG2;
          cfg3 = MCP_16MHz_31k25BPS_CFG3;
          break;

        case (CAN_33KBPS):
          cfg1 = MCP_16MHz_33kBPS_CFG1;
          cfg2 = MCP_16MHz_33kBPS_CFG2;
          cfg3 = MCP_16MHz_33kBPS_CFG3;
          break;

        case (CAN_40KBPS):
          cfg1 = MCP_16MHz_40kBPS_CFG1;
          cfg2 = MCP_16MHz_40kBPS_CFG2;
          cfg3 = MCP_16MHz_40kBPS_CFG3;
          break;

        case (CAN_50KBPS):
          cfg1 = MCP_16MHz_50kBPS_CFG1;
          cfg2 = MCP_16MHz_50kBPS_CFG2;
          cfg3 = MCP_16MHz_50kBPS_CFG3;
          break;

        case (CAN_80KBPS):
          cfg1 = MCP_16MHz_80kBPS_CFG1;
          cfg2 = MCP_16MHz_80kBPS_CFG2;
          cfg3 = MCP_16MHz_80kBPS_CFG3;
          break;

        case (CAN_83K3BPS):
          cfg1 = MCP_16MHz_83k3BPS_CFG1;
          cfg2 = MCP_16MHz_83k3BPS_CFG2;
          cfg3 = MCP_16MHz_83k3BPS_CFG3;
          break;

        case (CAN_95KBPS):
          cfg1 = MCP_16MHz_95kBPS_CFG1;
          cfg2 = MCP_16MHz_95kBPS_CFG2;
          cfg3 = MCP_16MHz_95kBPS_CFG3;
          break;

        case (CAN_100KBPS):
          cfg1 = MCP_16MHz_100kBPS_CFG1;
          cfg2 = MCP_16MHz_100kBPS_CFG2;
          cfg3 = MCP_16MHz_100kBPS_CFG3;
          break;

        case (CAN_125KBPS):
          cfg1 = MCP_16MHz_125kBPS_CFG1;
          cfg2 = MCP_16MHz_125kBPS_CFG2;
          cfg3 = MCP_16MHz_125kBPS_CFG3;
          break;

        case (CAN_200KBPS):
          cfg1 = MCP_16MHz_200kBPS_CFG1;
          cfg2 = MCP_16MHz_200kBPS_CFG2;
          cfg3 = MCP_16MHz_200kBPS_CFG3;
          break;

        case (CAN_250KBPS):
          cfg1 = MCP_16MHz_250kBPS_CFG1;
          cfg2 = MCP_16MHz_250kBPS_CFG2;
          cfg3 = MCP_16MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS):
          cfg1 = MCP_16MHz_500kBPS_CFG1;
          cfg2 = MCP_16MHz_500kBPS_CFG2;
          cfg3 = MCP_16MHz_500kBPS_CFG3;
          break;

        case (CAN_666KBPS):
          cfg1 = MCP_16MHz_666kBPS_CFG1;
          cfg2 = MCP_16MHz_666kBPS_CFG2;
          cfg3 = MCP_16MHz_666kBPS_CFG3;
          break;

        case (CAN_1000KBPS):
          cfg1 = MCP_16MHz_1000kBPS_CFG1;
          cfg2 = MCP_16MHz_1000kBPS_CFG2;
          cfg3 = MCP_16MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          break;
      }
      break;

    case (MCP_8MHz) :
      switch (canSpeed)
      {
        case (CAN_5KBPS) :
          cfg1 = MCP_8MHz_5kBPS_CFG1;
          cfg2 = MCP_8MHz_5kBPS_CFG2;
          cfg3 = MCP_8MHz_5kBPS_CFG3;
          break;

        case (CAN_10KBPS) :
          cfg1 = MCP_8MHz_10kBPS_CFG1;
          cfg2 = MCP_8MHz_10kBPS_CFG2;
          cfg3 = MCP_8MHz_10kBPS_CFG3;
          break;

        case (CAN_20KBPS) :
          cfg1 = MCP_8MHz_20kBPS_CFG1;
          cfg2 = MCP_8MHz_20kBPS_CFG2;
          cfg3 = MCP_8MHz_20kBPS_CFG3;
          break;

        case (CAN_31K25BPS) :
          cfg1 = MCP_8MHz_31k25BPS_CFG1;
          cfg2 = MCP_8MHz_31k25BPS_CFG2;
          cfg3 = MCP_8MHz_31k25BPS_CFG3;
          break;

        case (CAN_40KBPS) :
          cfg1 = MCP_8MHz_40kBPS_CFG1;
          cfg2 = MCP_8MHz_40kBPS_CFG2;
          cfg3 = MCP_8MHz_40kBPS_CFG3;
          break;

        case (CAN_50KBPS) :
          cfg1 = MCP_8MHz_50kBPS_CFG1;
          cfg2 = MCP_8MHz_50kBPS_CFG2;
          cfg3 = MCP_8MHz_50kBPS_CFG3;
          break;

        case (CAN_80KBPS) :
          cfg1 = MCP_8MHz_80kBPS_CFG1;
          cfg2 = MCP_8MHz_80kBPS_CFG2;
          cfg3 = MCP_8MHz_80kBPS_CFG3;
          break;

        case (CAN_100KBPS) :
          cfg1 = MCP_8MHz_100kBPS_CFG1;
          cfg2 = MCP_8MHz_100kBPS_CFG2;
          cfg3 = MCP_8MHz_100kBPS_CFG3;
          break;

        case (CAN_125KBPS) :
          cfg1 = MCP_8MHz_125kBPS_CFG1;
          cfg2 = MCP_8MHz_125kBPS_CFG2;
          cfg3 = MCP_8MHz_125kBPS_CFG3;
          break;

        case (CAN_200KBPS) :
          cfg1 = MCP_8MHz_200kBPS_CFG1;
          cfg2 = MCP_8MHz_200kBPS_CFG2;
          cfg3 = MCP_8MHz_200kBPS_CFG3;
          break;

        case (CAN_250KBPS) :
          cfg1 = MCP_8MHz_250kBPS_CFG1;
          cfg2 = MCP_8MHz_250kBPS_CFG2;
          cfg3 = MCP_8MHz_250kBPS_CFG3;
          break;

        case (CAN_500KBPS) :
          cfg1 = MCP_8MHz_500kBPS_CFG1;
          cfg2 = MCP_8MHz_500kBPS_CFG2;
          cfg3 = MCP_8MHz_500kBPS_CFG3;
          break;

        case (CAN_1000KBPS) :
          cfg1 = MCP_8MHz_1000kBPS_CFG1;
          cfg2 = MCP_8MHz_1000kBPS_CFG2;
          cfg3 = MCP_8MHz_1000kBPS_CFG3;
          break;

        default:
          set = 0;
          break;
      }
      break;

    default:
      set = 0;
      break;
  }

  if (set) {
    mcp2515_setRegister(MCP_CNF1, cfg1);
    mcp2515_setRegister(MCP_CNF2, cfg2);
    mcp2515_setRegister(MCP_CNF3, cfg3);
    return MCP2515_OK;
  }
  else {
    return MCP2515_FAIL;
  }
}

uint8_t mcp2515_init(uint8_t speedset, const uint8_t clockset){
    uint8_t ret;

    mcp2515_reset();//+

    ret = mcp2515_setCANCTRL_Mode(MODE_CONFIG);
    if (ret > 0)
        printf("Enter setting mode fail\n");
    else
        printf("Enter setting mode success\n");    
    usleep(10);

    ret = mcp2515_configRate(speedset, clockset);
    if (ret > 0)
        printf("Enter setting rate fail\n");
    else
        printf("Enter setting rate success\n");    
    usleep(10);

    if (ret == MCP2515_OK){
        // init canbuffers
        mcp2515_initCANBuffers();
        // interrupt mode
        mcp2515_setRegister(MCP_CANINTE, MCP_RX0IF | MCP_RX1IF);
              // enable both receive-buffers to receive messages with std. and ext. identifiers and enable rollover
        mcp2515_modifyRegister( MCP_RXB0CTRL, 
                                MCP_RXB_RX_MASK | MCP_RXB_BUKT_MASK,
                                MCP_RXB_RX_STDEXT | MCP_RXB_BUKT_MASK);
        mcp2515_modifyRegister( MCP_RXB1CTRL, MCP_RXB_RX_MASK, MCP_RXB_RX_STDEXT);
    }

    ret = setMode(MODE_NORMAL);
    if (ret > 0)
        printf("Enter normal mode fail\n");
    else
        printf("Enter normal mode success\n");
    usleep(10);
    return ret;
}

uint8_t can_begin(uint8_t speedset, const uint8_t clockset){
    fd_spi = spi_init();
    if (fd_spi < 0){
        printf("error spi init\n");
        return -1;
    }
    uint8_t ret = mcp2515_init(speedset, clockset);
    return ((ret == MCP2515_OK) ? CAN_OK : CAN_FAILINIT);
}

