/*
 * register_map.h
 *
 *  Created on: Dec 23, 2025
 *      Author: GITEX
 */

#ifndef INC_REGISTER_MAP_H_
#define INC_REGISTER_MAP_H_

#include <stdint.h>

#define HOLDING_REG_COUNT 16


#define REG_TARGET_RPM           0
#define REG_TARGET_POSITION      1
#define REG_MODE                 2
#define REG_ACTUAL_RPM           3
#define REG_ACTUAL_POSITION      4
#define REG_PWM_OUTPUT           5


extern uint16_t holding_regs[HOLDING_REG_COUNT];


#endif /* INC_REGISTER_MAP_H_ */
