#ifndef _SERIAL_INPUTS_H_
#define _SERIAL_INPUTS_H_

#include "arm_data_struct.h"

/* read serial input from raspberry pi:
 *  end-effector goal position: (x, y)
 *  end-effector goal velocity: (v_x, v_y)
 * overall parse a string of floats separated by commas: x, y, v_x, v_y
 */
 
rec_arm_data *read_arm_goals();

/* read initial arm pose for pre-game calibration
 * v_x and v_y are dummy values
 */
pregame_arm_data *read_arm_pregame();

#endif
