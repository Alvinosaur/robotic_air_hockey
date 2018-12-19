#include <stdlib.h>

#include "serial_inputs.h"

const byte buff_size = 64;
char raw_input[buff_size];

pregame_arm_data *read_arm_pregame(){
  byte inc = 0;
  while (!Serial.available());  // block until receiving data from rpi
  while (Serial.available() > 0 && inc < buff_size-1) {
    raw_input[inc] = Serial.read();
    inc ++;
  }
  raw_input[inc] = 0;  // end-of-string
  pregame_arm_data *arm_data = malloc(sizeof(pregame_arm_data));

  char *partOfString;

  // parse first value: x position
  partOfString = strtok(raw_input, ",");
  float arm0_x = atof(partOfString);     // convert this part to an integer
  arm_data->arm0_x = arm0_x;

  // parse second value: y position
  partOfString = strtok(NULL, ",");
  float arm0_y = atof(partOfString);     // convert this part to a float
  arm_data->arm0_y = arm0_y;

  // parse second value: y position
  partOfString = strtok(NULL, ",");
  float arm1_x = atof(partOfString);     // convert this part to a float
  arm_data->arm1_x = arm1_x;

  // parse second value: y position
  partOfString = strtok(NULL, ",");
  float arm1_y = atof(partOfString);     // convert this part to a float
  arm_data->arm1_y = arm1_y;

  return rec_data;
}

rec_arm_data *read_arm_goals(){
  byte inc = 0;
  while (!Serial.available());  // block until receiving data from rpi
  while (Serial.available() > 0 && inc < buff_size-1) {
    raw_input[inc] = Serial.read();
    inc ++;
  }
  raw_input[inc] = 0;  // end-of-string
  rec_arm_data *rec_data = (rec_arm_data*)malloc(sizeof(rec_arm_data));
  char *partOfString;

  // parse first value: x position
  partOfString = strtok(raw_input, ",");
  float arm_x = atof(partOfString);     // convert this part to an integer
  rec_data->arm_x = arm_x;

  // parse second value: y position
  partOfString = strtok(NULL, ",");
  float arm_y = atof(partOfString);     // convert this part to a float
  rec_data->arm_y = arm_y;

  // parse third value: x velocity
  partOfString = strtok(NULL, ",");
  float arm_vx = atof(partOfString);     // convert this part to a float
  rec_data->arm_vx = arm_vx;

  // parse fourth value: y velocity
  partOfString = strtok(NULL, ",");
  float arm_vy = atof(partOfString);     // convert this part to a float
  rec_data->arm_vy = arm_vy;

  return rec_data;
}
