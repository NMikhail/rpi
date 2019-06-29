#include "ultrasound.h"

float getDistanceCm(int TRIG, int ECHO) {
    long ping = 0;
    long pong = 0;
    long timeout = 20000; // 0.5 сек ~ 171 м

    // генерация импульса длительностью 10 мкс
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);
    ping = micros(); 
    // ждём первый ответ датчика или тайм аут.
    while (digitalRead(ECHO) == LOW ){
        pong = micros();
        usleep(50);
        if ((pong - ping) > timeout){
                printf("out of range\n");
                return -400;
        }
    }
    ping = micros();
    // ждём второй ответ датчика или тайм аут
    while (digitalRead(ECHO) == HIGH){
        pong = micros();
        usleep(50);
        if ((pong - ping) > timeout){
                printf("out of range\n");
                return 400;
        }
    }
    pong = micros();
    float distance = ((float)(pong - ping)) * 0.017150;
    //Задержка для выравнивания времени выполнения
    while ((pong - ping) < timeout){        
        usleep(50);
        pong = micros();
    }

    return (distance);
}

void getDistanceCmAll(ultrasound_gpio_t *ultrasound_gpio, direction_t *pVec){
    long time_start, time_stop;
    long ping_front, ping_left, ping_right;
    long pong_front, pong_left, pong_right;
    long timeout = 20000; // 0.5 сек ~ 171 м
    unsigned char start_front = 0, start_left = 0, start_right = 0;
    unsigned char stop_front = 0, stop_left = 0, stop_right = 0;
    if (wiringPiSetup() == -1) {
        printf("WiringPi Setup ERROR\n");
    }
    pinMode(ultrasound_gpio->front_trig,    OUTPUT);
    pinMode(ultrasound_gpio->front_echo,    INPUT);
    pinMode(ultrasound_gpio->left_trig,     OUTPUT);
    pinMode(ultrasound_gpio->left_echo,     INPUT);
    pinMode(ultrasound_gpio->right_trig,    OUTPUT);
    pinMode(ultrasound_gpio->right_echo,    INPUT);

    // генерация импульса длительностью 10 мкс
    digitalWrite(ultrasound_gpio->front_trig, HIGH);
    digitalWrite(ultrasound_gpio->left_trig, HIGH);
    digitalWrite(ultrasound_gpio->right_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(ultrasound_gpio->front_trig, LOW);
    digitalWrite(ultrasound_gpio->left_trig, LOW);
    digitalWrite(ultrasound_gpio->right_trig, LOW);

    time_start = micros();
    time_stop = time_start;
    while ((time_stop - time_start) < timeout){
        if ((digitalRead(ultrasound_gpio->front_echo) != LOW) && (!start_front)){
            start_front = 1;
            ping_front  = micros();
        }else if((digitalRead(ultrasound_gpio->front_echo) == LOW) && start_front){            
            start_front = 0;
            pong_front  = micros();
            pVec->cart_head.dist_cm_front = ((float) (pong_front - ping_front) * 0.017150);
        }
        if ((digitalRead(ultrasound_gpio->left_echo) != LOW) && (!start_left)){
            start_left = 1;
            ping_left  = micros();
        }else if((digitalRead(ultrasound_gpio->left_echo) == LOW) && (start_left)){            
            start_left = 0;
            pong_left  = micros();
            pVec->cart_head.dist_cm_left = ((float) (pong_left - ping_left) * 0.017150);
        }
        if ((digitalRead(ultrasound_gpio->right_echo) != LOW) && (!start_right)){
            start_right = 1;
            ping_right  = micros();
        }else if((digitalRead(ultrasound_gpio->right_echo) == LOW) && (start_right)){            
            start_right = 0;
            pong_right  = micros();
            pVec->cart_head.dist_cm_right = ((float) (pong_right - ping_right) * 0.017150);
        }
        usleep(50);
        time_stop = micros();
    }
}
float slow_right = 0.9;
void moment2speed(direction_t *pVec){
    if (pVec->cart_head.ratio == 0){
        pVec->cart_head.speed_wheel_l = pVec->cart_head.moment * 32767;
        pVec->cart_head.speed_wheel_r = pVec->cart_head.moment * 32767 * slow_right;
    } else if (pVec->cart_head.ratio > 0){
        pVec->cart_head.speed_wheel_l = pVec->cart_head.moment * 32767;// - pVec->cart_head.moment * pVec->cart_head.ratio * 32767; //0 * pVec->cart_head.ratio;// * (1 - pVec->cart_head.ratio );
        pVec->cart_head.speed_wheel_r = pVec->cart_head.moment * 32767 * slow_right;// + pVec->cart_head.moment * pVec->cart_head.ratio * 40960;        
    } else if (pVec->cart_head.ratio < 0){
        pVec->cart_head.speed_wheel_l = pVec->cart_head.moment * 32767; //- pVec->cart_head.moment * pVec->cart_head.ratio * 40960;
        pVec->cart_head.speed_wheel_r = pVec->cart_head.moment * 32767 * slow_right;// + pVec->cart_head.moment * slow_right * pVec->cart_head.ratio * 32767;//  * 0 * -pVec->cart_head.ratio;// * (1 + pVec->cart_head.ratio );       
    }
    //debug
    printf("moment: %2.6f ratio: %2.6f l: %d, r: %d ", pVec->cart_head.moment,  pVec->cart_head.ratio, pVec->cart_head.speed_wheel_l, pVec->cart_head.speed_wheel_r);
}

int distance2direction(int level_of_threshold, direction_t *pVec){
    //Определение путевой скорости
    // if (pVec->cart_head.dist_cm_front <= level_of_threshold)
    //     pVec->cart_head.moment  = 0.0;
    // else
    //     pVec->cart_head.moment  = 1.0;

//Для одной стенки 
    //Минимальное расстояние до стены (независимо от угла)
    pVec->cart_head.min_distance = pVec->cart_head.dist_cm_left;
    if (pVec->cart_head.min_distance > pVec->cart_head.dist_cm_right)
        pVec->cart_head.min_distance = pVec->cart_head.dist_cm_right;
    if (pVec->cart_head.min_distance > pVec->cart_head.dist_cm_front_right)
        pVec->cart_head.min_distance = pVec->cart_head.dist_cm_front_right;
    if (pVec->cart_head.min_distance > pVec->cart_head.dist_cm_front)
        pVec->cart_head.min_distance = pVec->cart_head.dist_cm_front;
    //Загрубление показаний датчика (куча помех, мертвая зона 10 см.)
    pVec->cart_head.min_distance = roundf(pVec->cart_head.min_distance / DEATH_ZONE_ULTRASOUND_CM) * DEATH_ZONE_ULTRASOUND_CM;
    //moment2speed(pVec);
    return 0;
}



int probe_ultrasound(int level_of_threshold, direction_t *pVec){
    int trig, echo;
    trig = 28; echo = 29;
    pVec->cart_head.dist_cm_front = getDistanceCm(trig, echo);
    trig = 7; echo = 0;
    pVec->cart_head.dist_cm_front_right = getDistanceCm(trig, echo);
    trig = 26; echo = 27;
    pVec->cart_head.dist_cm_left = getDistanceCm(trig, echo);
    trig = 24; echo = 25;
    pVec->cart_head.dist_cm_right = getDistanceCm(trig, echo);
    distance2direction(level_of_threshold, pVec);
    return 0;
}

// int probe_ultrasound(int level_of_threshold, direction_t *pVec){
//     int trig, echo;
//     ultrasound_gpio_t ultrasound_gpio;
//     ultrasound_gpio.front_echo  = 29;
//     ultrasound_gpio.front_trig  = 28;
//     ultrasound_gpio.left_echo   = 27;
//     ultrasound_gpio.left_trig   = 26;
//     ultrasound_gpio.right_echo  = 25;
//     ultrasound_gpio.right_trig  = 24;
//     getDistanceCmAll(&ultrasound_gpio, pVec);
//     return ( distance2direction(level_of_threshold, pVec) );
// }