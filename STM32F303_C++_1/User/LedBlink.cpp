/*
 * LedBlink.cpp
 *
 *  Created on: 2018/03/04
 *      Author: stern
 */

#include "LedBlink.hpp"
#include "stm32f3xx_hal.h"
//
//LedBlink::LedBlink() {
//	// TODO Auto-generated constructor stub
//
//}
//
//LedBlink::~LedBlink() {
//	// TODO Auto-generated destructor stub
//}


void LedBlink::toggle() {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    HAL_Delay(100);
}
