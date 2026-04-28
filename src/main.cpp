#include <Arduino.h>
SET_LOOP_TASK_STACK_SIZE( 16*1024 );

#include "main_robot.h"

void setup() {
    robot_setup();
}

void loop() {
    robot_loop();
}
