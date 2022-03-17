#ifndef __NCP5623C_H__
#define __NCP5623C_H__
#include <stm32f4xx.h>

#define RGB_PERCENTAGE_MAX 100.0f

void rgb_light(float red,float green,float blue); //percentage of r,g,b
#endif
