#ifndef HAMELEON_H
#define HAMELEON_H
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <wiringPi.h>
#include "alltype.h"
#include "ultrasound.h"
#include "mcp2515.h"
#include <pthread.h>
#include "cc1101_func.h"

#define LED_HANDLE  1   //Зеленый светодиод запуска программы и ручного режима
#define LED_AUTO    3
#define LED_AUTO_BACK   2
#define SWITCH_PIN1 4   //  SWITCH_PIN2 |   SWITCH_PIN1 |   Режим
#define SWITCH_PIN2 5   //      HIGH    |       HIGH    |    Ручной
                        //      HIGH    |       LOW     |   Автомат вперед
                        //      LOW     |       HIGH    |   Автомат назад
#define SLAVE_MOD   0
#define MASTER_MOD  1

#define HEAD_ADDR   0x10
#define TAIL_ADDR   0x00

#define CURRENTTRUESHOLD 30 //Порог в 30 Ампер

#define distanceToWall  150         //Расстояние до стены (см), линия по которой двигается объект
#define avrgSpeed       40          //Средняя скорость при движении по прямой (диапазон 0 - 255)
#define CM_STOP_THRESHOLD       70  //Расстояние до препятствия для полной остановки (сантиметры)
#define RSSI_STOP_THRESHOLD     -30 //Расстояние до человека (уровень сигнала метки dBm) для полной остановки (определяется экспериментально)
#define DISTANCE_TO_LABEL_INF   2000//Расстояние до метки, когда считается что ее нет.(Условные единицы, определяется экспериментально)
//Уровни переключателя режимов
#define AUTO_CTRL           0
#define MAN_CTRL            1
//Номера выводов GPIO для wiringPi
#define CNTRL_SWITCH        1   //Переключатель режимов


void mastermode(void);
void slavemode(void);

void handleMode(void);
void automaticMode(direction_t *pVec, uint8_t mode);
// void automaticReverseMode(void);
void sendToSlave(uint8_t mode);
//void getMsgFromSlave(void);
void getMsgFromMaster(uint8_t *mode);
void JoystickMsgf(uint8_t *tmpbuf);
uint8_t CurrentMsg(uint8_t *tmpbuf);
void CurrentCorrect(uint8_t *newdata, uint8_t *speedvalue, uint8_t *currvalue);
void SetSpeedMsg(uint8_t *speedvalue);
void ChangeSpeed(uint8_t *newSpeedValue, uint8_t *oldSpeedValue);

//void probe_ultrasound(int level_of_threshold, direction_t *pVec);
void probe_beacon(int level_of_threshold, direction_t *pVec);
void convertSpeed2Msg(direction_t *pVec, uint8_t *buf, uint8_t mode);
#endif