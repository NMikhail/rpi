#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <stdio.h>
#include <stdlib.h>
#include <wiringPi.h>
#include <unistd.h>
#include "alltype.h"

int probe_ultrasound(int level_of_threshold, direction_t *pVec);

#endif