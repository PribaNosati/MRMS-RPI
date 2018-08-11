/**
Purpose: various test programs for MRMS Arduino projects
@author MRMS team
@version 0.1 2018-08-10
Licence: You can use this code any way you like.
*/

#include "Robot.h"

///Configuration
const int thresh = 20; /// Canny algorithm threshold
const bool saveImages = false; /// For tests later
Robot::State state = Robot::TEST_UART_MESSAGES; /// Check Robot::State to see all the options


int main(int argc, char *argv[])
{
    Robot robot(state, thresh, saveImages); /// Object robot
    robot.run(); /// Start the program
    return 0;
}



