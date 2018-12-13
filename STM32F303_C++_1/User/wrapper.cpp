/*
 * wrapper.cpp
 *
 *  Created on: 2018/03/04
 *      Author: stern
 */

#include "wrapper.hpp"
#include "LedBlink.hpp"

void cpploop(void) {
    LedBlink instance;

    instance.toggle();
}
