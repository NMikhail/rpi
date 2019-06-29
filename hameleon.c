#include "hameleon.h"



uint8_t JoysticMsg[8]   = {0}; uint8_t JoysticOk    = 0;
uint8_t telega1Msg[8]   = {0}; uint8_t telegaNumOk  = 0;
uint8_t telega2Msg[8]   = {0};
uint8_t telega3Msg[8]   = {0};
uint8_t telega4Msg[8]   = {0};
uint8_t slaveMsg[8]     = {0}; uint8_t slaveOk      = 0;
uint8_t masterMsg[8]    = {0}; uint8_t masterOk     = 0;

uint8_t buf[8]          = {0};    
uint8_t buf_current[8]  = {0}; 

pthread_mutex_t can_mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t cc1101_mutex = PTHREAD_MUTEX_INITIALIZER;

uint8_t CURR_ADDR = TAIL_ADDR;

uint8_t mode_switch = 0;



float KP, KD, KI;           //Коэффициенты ПИД регулятора
float err_delay[10] = {0}; //Линия задержки для диф. ошибки, увеличивает диф ошибку для более точного регулирования

//Поток обработки сообщений CAN
void *read_CAN_thread(void *v){
    uint8_t buf[8];
    uint8_t len;
    unsigned long canId;
    while(1){
        usleep(50);
        if(CAN_MSGAVAIL == checkReceive()){ 
            readMsgBuf(&len, buf);
            canId = getCanId();
            pthread_mutex_lock( &can_mutex );
            switch  (canId){
                case TAIL_ADDR: memcpy(slaveMsg, buf, 8);   slaveOk = 1; break;
                case HEAD_ADDR: memcpy(masterMsg, buf, 8); masterOk = 1; break;
                case 0x001: memcpy(telega1Msg, buf, 8); telegaNumOk = 1; break;
                case 0x002: memcpy(telega2Msg, buf, 8); telegaNumOk = 2; break;
                case 0x003: memcpy(telega3Msg, buf, 8); telegaNumOk = 3; break;
                case 0x004: memcpy(telega4Msg, buf, 8); telegaNumOk = 4; break;
                case 0x040: memcpy(JoysticMsg, buf, 8); JoysticOk = 1; break;
                default: break;
            }
            pthread_mutex_unlock( &can_mutex );                       
        }            
    }    
    pthread_exit( 0 );
}
//Поток обработки RSSI с радиомаяка
int8_t rssi_dbmi = -128;
uint8_t rssi_flag_ok = 0;
void *read_CC1101_thread(void *v){
    uint8_t Tx_fifo[FIFOBUFFER], Rx_fifo[FIFOBUFFER];
    uint8_t My_addr, Tx_addr, Rx_addr, Pktlen, pktlen, Lqi, Rssi;
    uint8_t rx_addr,sender,lqi;
    while(1){
        usleep(50);
        if(packet_available()){
            pthread_mutex_lock( &cc1101_mutex );
			get_payload(Rx_fifo, pktlen, rx_addr, sender, &rssi_dbmi, lqi);
			rssi_flag_ok = 1;
            pthread_mutex_unlock( &cc1101_mutex );
		}
    }    
    pthread_exit( 0 );
}
int main(int argc, char **argv){
    uint8_t mode_SM;
    int i = 0;
    //Инициализация выводов
    if (wiringPiSetup() < 0) {
        printf("WiringPi Setup ERROR\n");
        return 0;
    }
    //УЗ датчики
    int trig, echo;
    //Передний левый датчик
    trig = 28; echo = 29;
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    //левый датчик
    trig = 26; echo = 27;
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    //правый датчик
    trig = 24; echo = 25;
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    //Передний правый датчик
    trig = 7; echo = 0;
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);    
    //Переключатель режимов
    pinMode(SWITCH_PIN1,    INPUT);    
    pinMode(SWITCH_PIN2,    INPUT);
    pullUpDnControl(SWITCH_PIN1, PUD_UP);
    pullUpDnControl(SWITCH_PIN2, PUD_UP);
    pinMode(LED_HANDLE,     OUTPUT);
    pinMode(LED_AUTO,       OUTPUT);
    pinMode(LED_AUTO_BACK,  OUTPUT);

    digitalWrite(LED_HANDLE,    LOW);
    digitalWrite(LED_AUTO,      LOW);
    digitalWrite(LED_AUTO_BACK, LOW);

    if (argc != 5){
        printf("**********************\n");
        printf("* Usage %s 0|1 KP KD KI\n", argv[0]);
        printf("* 0 - slave (tail), 1 - master (head)\n");
        //printf("* 0 - handle, 1 - auto forward, 2 - auto reverse\n");
        printf("**********************\n");
        return 0;       
    }
    mode_SM = strtoul(argv[1], NULL, 0);
    KP = strtof(argv[2], NULL);
    KD = strtof(argv[3], NULL);
    KI = strtof(argv[4], NULL);
    //mode_switch = strtoul(argv[2], NULL, 0);
    //Инициализация контроллера CAN
    while (i < CNT_CAN_INIT){
        if (can_begin(CAN_500KBPS, MCP_16MHz) == CAN_OK)
            break;
        i++;
        usleep(1000);
    }
    //Поток на чтение собщений CAN
    pthread_t th_read_can;
    pthread_create( &th_read_can, 0, read_CAN_thread, 0 );
    //Определение режима мастер или слейв
    if (mode_SM)
        CURR_ADDR = HEAD_ADDR;
    else
        CURR_ADDR = TAIL_ADDR;

    while (1){
        if (mode_SM){
            printf("Master mode OK.\n");
            mastermode();
        } else {
            printf("Slave mode OK.\n");
            slavemode();
        }
    }
}
static void pabort(const char *s){
	perror(s);
	abort();
}
//************************************************
//      Описание            |   PIN2    |   PIN1    |
//Ручной режим              |    L      |    L      |
//Автоматический вперед     |    L      |    H      |
//Автоматический назад      |    H      |    L      |      
// void mastermode(uint8_t *mode){
//     while(1){
//         if      (digitalRead(SWITCH_PIN2) == LOW    && digitalRead(SWITCH_PIN1) == LOW)
//             handleMode();
//         else if (digitalRead(SWITCH_PIN2) == LOW    && digitalRead(SWITCH_PIN1) == HIGH)
//             automaticForwardMode();
//         else if (digitalRead(SWITCH_PIN2) == HIGH   && digitalRead(SWITCH_PIN1) == LOW)
//             automaticReverseMode();

//         if (getMsgFromSlave()){
//             *mode = SLAVE_MOD;
//             break;
//         }
//         usleep(5);
//     }
// }
void mastermode(){

    direction_t Vec_direction;    
    direction_t *pVec_direction = &Vec_direction;
    pthread_t th_read_cc1101;
    uint8_t pin1 = 0;
    uint8_t pin2 = 0;
    //Инициализация CC1101
    cc1101_begin(3,1);  
    printf("cc1101 init for head > addr: 3, channel: 1\n");
    //Поток на чтение сообщений CC1101
    pthread_create( &th_read_cc1101, 0, read_CC1101_thread, 0 );

    int i;
    
    while(1){
        pin2 = digitalRead(SWITCH_PIN2);
        pin1 = digitalRead(SWITCH_PIN1);

        if ((pin2 == HIGH) && (pin1 == HIGH)){
            sendToSlave(SLAVE_MOD);
            handleMode();
            digitalWrite(LED_HANDLE,    HIGH);
            digitalWrite(LED_AUTO,      LOW);
            digitalWrite(LED_AUTO_BACK, LOW);

            memset(err_delay, 0, 10);
        }
        else if(pin1 == LOW){
            sendToSlave(SLAVE_MOD);
            automaticMode(pVec_direction, MASTER_MOD);
            digitalWrite(LED_HANDLE,    LOW);
            digitalWrite(LED_AUTO,      HIGH);
            digitalWrite(LED_AUTO_BACK, LOW);
        }
        else if (pin2 == LOW){
            sendToSlave(MASTER_MOD);
            digitalWrite(LED_HANDLE,    LOW);
            digitalWrite(LED_AUTO,      LOW);
            digitalWrite(LED_AUTO_BACK, HIGH);
            for (i = 0; i < 1000; i++)
                usleep(1000);
            memset(err_delay, 0, 10);
        }
        usleep(1000);    
    }
}

void slavemode(void){
    int i;
    direction_t Vec_direction;    
    direction_t *pVec_direction = &Vec_direction;
    pthread_t th_read_cc1101;
    //Инициализация CC1101
    cc1101_begin(27,10);
    printf("cc1101 init for tail > addr: 27, channel: 10\n");
    //Поток на чтение сообщений CC1101
    pthread_create( &th_read_cc1101, 0, read_CC1101_thread, 0 );
    uint8_t mode_master = 0;
    while (1){          
        getMsgFromMaster(&mode_master);
        if(mode_master){
            automaticMode(pVec_direction, SLAVE_MOD);
        }
        for (i = 0; i < 500; i++)
                usleep(1000);
    }
}


//***************Управление режимами ручным и автоматическими*********
//****Ручной режим****
void handleMode(void){
    //uint8_t buf_new[8] = {0};
    //uint8_t buf_old[8];
    //memcpy(buf_old, buf, 8);
    JoystickMsgf(buf);
    //Плавное изменение скорости
    //ChangeSpeed(buf, buf_old);
  //  uint8_t tmptmp[8];
 //   CurrentMsg(tmptmp);
    //CurrentCorrect(buf_new, buf, buf_current);    
    SetSpeedMsg(buf);
    
}

void automaticMode(direction_t *pVec, uint8_t mode_master){
    long start_time, end_time;
    int msg, i, delay_ms;
    float PID, P, D, I;
    start_time = micros();
    // if (pVec->cart_head.dist_cm_front <= 100)
    //     pVec->cart_head.moment  = 0.0;
    // else
    if (mode_master) 
        pVec->cart_head.moment  = avrgSpeed;
    else
        pVec->cart_head.moment  = avrgSpeed;
    //Уровень сигнала ниже порога, датчик вне зоны, продолжаем движение
    if (rssi_dbmi < RSSI_STOP_THRESHOLD){
        probe_ultrasound(CM_STOP_THRESHOLD, pVec);
        for (i = 0; i < 9; i++)
            err_delay[i] = err_delay[i + 1];
        err_delay[9] = pVec->cart_head.min_distance - distanceToWall;
        // float err_sum = 0;
        // err_sum = err_sum + (err + err_old) / 2;
        // P = KP * err;
        // D = KD * (err - err_old);
        // I = KI * err_sum;
        // PID = P + I + D;1/KI * err_sum 
        PID = KP * (err_delay[9] + KD * (err_delay[0] - err_delay[9]));
        if      (PID > 1)   PID = 1;
        else if (PID < -1)  PID = -1;
        else                PID = PID;
        pVec->cart_head.speed_wheel_l = (uint16_t)pVec->cart_head.moment * (1 - PID);
        pVec->cart_head.speed_wheel_r = (uint16_t)pVec->cart_head.moment * (1 + PID);       
    }
    probe_beacon(RSSI_STOP_THRESHOLD, pVec);
    convertSpeed2Msg(pVec,buf,mode_master);
    SetSpeedMsg(buf);
    //Время измерения
    // for (delay_ms = 0; delay_ms < 100; delay_ms++)
    //     usleep(1000);
    end_time = micros();
    printf("PID: %2.5f, ", PID);
    printf("min_distance: %3.1f, ", pVec->cart_head.min_distance);
    printf("speed_left: %d, speed_right: %d, ", buf[6], buf[7]);
    printf("Time of meas: %2.4f ms\n", (float)(end_time - start_time)/1000);
}
//********************************************************************
void sendToSlave(uint8_t mode){
    uint8_t tmpbuf[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, mode};
    sendMsgBuf(HEAD_ADDR, 0, 8, tmpbuf, 1);    
}

void getMsgFromMaster(uint8_t *mode){
    pthread_mutex_lock( &can_mutex );
    if (masterOk){
        masterOk = 0;
        if (masterMsg[0] == 0x30)
            *mode = masterMsg[7];
    }
    pthread_mutex_unlock( &can_mutex );
}
uint8_t getMsgFromSlave(void){
    pthread_mutex_lock( &can_mutex );
    if (slaveOk){
        slaveOk = 0;
        if ((slaveMsg[0] == 0x30) && (slaveMsg[7] == 0x00)){
            uint8_t tmpbuf[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
            sendMsgBuf(HEAD_ADDR, 0, 8, tmpbuf, 1);
            pthread_mutex_unlock( &can_mutex ); 
            return 1;      
        }
    }
    pthread_mutex_unlock( &can_mutex );     
    return 0;  
}

void JoystickMsgf(uint8_t *tmpbuf){
    pthread_mutex_lock( &can_mutex );
    if (JoysticOk){
        JoysticOk = 0;
        memcpy(tmpbuf, JoysticMsg, 8);
    }
    pthread_mutex_unlock( &can_mutex );
}
uint8_t CurrentMsg(uint8_t *tmpbuf){
    int ret = 0;
    pthread_mutex_lock( &can_mutex );
    if (telegaNumOk != 0)
        ret = 1;
    switch (telegaNumOk){
        case 0x01:  memcpy(tmpbuf, telega1Msg, 8); telegaNumOk = 0; break;
        case 0x02:  memcpy(tmpbuf, telega2Msg, 8); telegaNumOk = 0; break;
        case 0x03:  memcpy(tmpbuf, telega3Msg, 8); telegaNumOk = 0; break;
        case 0x04:  memcpy(tmpbuf, telega4Msg, 8); telegaNumOk = 0; break;
        default: telegaNumOk = 0; break;
    }
    // printf("Current overpower errL: %d, errR: %d,  left: %d, rigth: %d\n", tmpbuf[4], tmpbuf[5], tmpbuf[6], tmpbuf[7]);
    // if (tmpbuf[7] > CURRENTTRUESHOLD || tmpbuf[6] > CURRENTTRUESHOLD ){
    //     printf("Current overpower left: %d, rigth: %d\n", tmpbuf[6], tmpbuf[7]);
    //     digitalWrite(LED_OVERPOWER, HIGH);
    // } else {
    //     digitalWrite(LED_OVERPOWER, LOW);
    // }
    
    pthread_mutex_unlock( &can_mutex );
    return ret;
}
void CurrentCorrect(uint8_t *newdata, uint8_t *speedvalue, uint8_t *currvalue){
    if ((currvalue[6] > CURRENTTRUESHOLD) || (currvalue[7] > CURRENTTRUESHOLD)){
        memcpy(newdata,speedvalue, 6);
        newdata[6] = speedvalue[6] * 0.7;
        newdata[7] = speedvalue[7] * 0.7;
    } else {
        memcpy(newdata,speedvalue, 8);
    }
}
void SetSpeedMsg(uint8_t *speedvalue){
    uint8_t tmpbuf[8] = {0x01, speedvalue[1], speedvalue[2], speedvalue[3], speedvalue[4], speedvalue[5], speedvalue[6], speedvalue[7]};
    sendMsgBuf(HEAD_ADDR, 0, 8, tmpbuf, 1); //Скорость 1 тележки
    tmpbuf[0] = 0x02; tmpbuf[6] = speedvalue[6] * 1; tmpbuf[7] = speedvalue[7] * 1;
    sendMsgBuf(HEAD_ADDR, 0, 8, tmpbuf, 1); //Скорость 2 тележки
    tmpbuf[0] = 0x03; tmpbuf[6] = speedvalue[6] * 1; tmpbuf[7] = speedvalue[7] * 1;
    sendMsgBuf(HEAD_ADDR, 0, 8, tmpbuf, 1); //Скорость 3 тележки
    tmpbuf[0] = 0x04; tmpbuf[6] = speedvalue[6] * 1; tmpbuf[7] = speedvalue[7] * 1;
    sendMsgBuf(HEAD_ADDR, 0, 8, tmpbuf, 1); //Скорость 4 тележки
}

void probe_beacon(int level_of_threshold, direction_t *pVec){
    pthread_mutex_lock( &cc1101_mutex );
    if (rssi_flag_ok){
        if (rssi_dbmi > level_of_threshold){
            pVec->cart_head.speed_wheel_l = pVec->cart_head.speed_wheel_l * 0;
            pVec->cart_head.speed_wheel_r = pVec->cart_head.speed_wheel_r * 0;            
        }
        rssi_flag_ok = 0;
    } else {        
        rssi_dbmi = -128;
    }
    
    pthread_mutex_unlock( &cc1101_mutex );
}
void convertSpeed2Msg(direction_t *pVec, uint8_t *pbuf,uint8_t mode_master){
    memset(pbuf, 0, 8);
    if (mode_master){
        pbuf[4] = 0x01;   pbuf[6] = (uint8_t) (pVec->cart_head.speed_wheel_l);
        pbuf[5] = 0x01;   pbuf[7] = (uint8_t) (pVec->cart_head.speed_wheel_r);

    } else {
        pbuf[4] = 0x02;   pbuf[6] = (uint8_t) (pVec->cart_head.speed_wheel_l);
        pbuf[5] = 0x02;   pbuf[7] = (uint8_t) (pVec->cart_head.speed_wheel_r);
    }
}
//перевод скорости в абсолютное знаковое число от -255 до 255;
void speed2signValue(uint8_t *SpeedValue,int16_t *SpeedLeft, int16_t *SpeedRight){
    switch (SpeedValue[5]){
        case 0: *SpeedLeft = 0;              break;
        case 1: *SpeedLeft = SpeedValue[7];  break;
        case 2: *SpeedLeft = -SpeedValue[7]; break;
        default:*SpeedLeft = 0;              break;
    }
    switch (SpeedValue[4]){
        case 0: *SpeedRight = 0;              break;
        case 1: *SpeedRight = SpeedValue[6];  break;
        case 2: *SpeedRight = -SpeedValue[6]; break;
        default:*SpeedRight = 0;              break;
    }
}
uint8_t searchNewSpeed(int16_t *newSpeed,int16_t *oldSpeed){
    //#define максимальное приращение скорости за единицу времени, чем больше, тем выше переходной ток,
    //Чем меньше тем дольше время реакции на управление
    int16_t delta_speed = 0x10;
    if (*newSpeed > *oldSpeed){
        if (*newSpeed > (*oldSpeed + delta_speed)){
            *oldSpeed = *oldSpeed + delta_speed;
        } else {
            *oldSpeed = *newSpeed;
        }            
    } else if (*newSpeed < *oldSpeed){
        if (*newSpeed < (*oldSpeed - delta_speed)){
            *oldSpeed = *oldSpeed - delta_speed;
        } else {
            *oldSpeed = *newSpeed;
        }             
    } else if (*newSpeed == *oldSpeed)
        return 1;
    return 0;
}
//****Обратное преобразование из абсолютной знаковой скорости в относительную
void signValue2Speed(uint8_t *SpeedValue, int16_t SpeedLeft, int16_t SpeedRight){
    SpeedValue[5] = (SpeedLeft > 0) ? 1 : ((SpeedLeft < 0) ? 2 : 0);
    SpeedValue[7] = (SpeedLeft > 0) ? (SpeedLeft & 0xFF) : ((-SpeedLeft) & 0xFF);
    SpeedValue[4] = (SpeedRight > 0) ? 1 : ((SpeedRight < 0) ? 2 : 0);
    SpeedValue[6] = (SpeedRight > 0) ? (SpeedRight & 0xFF) : ((-SpeedRight) & 0xFF);
}
//****плавное изменение скорости ручного режима****
void ChangeSpeed(uint8_t *newSpeedValue, uint8_t *oldSpeedValue){
    uint8_t SpeedValue[8] = {0};
    int16_t oldSpeedLeft, oldSpeedRight, newSpeedLeft, newSpeedRight;
    uint8_t left_ok = 0, right_ok = 0;
    //#define Период коррекции скорости в мкс
    uint16_t TimeCorrectionSpeed = 1000;
/***************************************************
 *  Номер бита  |               Описание            
 *      0       |   
 *      1       |
 *      2       |
 *      3       |
 *      4       |Направление правого колеса 0 - стоп, 1 - вперед, 2 - назад
 *      5       |Направление левого колеса  0 - стоп, 1 - вперед, 2 - назад
 *      6       |Скорость правого колеса    0 - минимум, 255 - максимум
 *      7       |Скорость левого колеса     0 - минимум, 255 - максимум
 * *************************************************/    
    speed2signValue(oldSpeedValue, &oldSpeedLeft, &oldSpeedRight);
    speed2signValue(newSpeedValue, &newSpeedLeft, &newSpeedRight);
    while (1){
        left_ok = searchNewSpeed(&newSpeedLeft, &oldSpeedLeft);
        right_ok = searchNewSpeed(&newSpeedRight, &oldSpeedRight);

        signValue2Speed(SpeedValue, oldSpeedLeft, oldSpeedRight);
        SetSpeedMsg(SpeedValue);

        if ((right_ok == 0x01) & (left_ok == 0x01))
            break;
        else
            usleep(TimeCorrectionSpeed);
        //  Debug
        // printf("Speed l: 0x%04X r: 0x%04X ", speed_left, speed_right);
        // printf("Speed old l: 0x%04X r: 0x%04X ", oldspeed_left, oldspeed_right);
        // printf("Speed new l: 0x%04X r: 0x%04X \n", newspeed_left, newspeed_right);

        // printf("speed_joy: ");
        // for(int i = 0; i<8; i++)
        //     printf("0x%02X\t",new_speedvalue[i]);
        // printf("\n");        
    }
}


