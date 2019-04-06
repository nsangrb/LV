#include <math.h>
#include "stm32f4xx_hal.h"
#include "main.h"
#include <string.h>
#include <stdbool.h>

bool Init_GPRS();
bool Get_Method(char url[],char data[]);
bool Post_Method(char url[],char data[]);
