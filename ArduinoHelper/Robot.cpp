#include <iostream>
#include "Robot.h"
#include <wiringPi.h>

using namespace std;

/** Constructor
@param state - initial state
@param thresh - OpenCV Canny's threshold
@param saveImages - save to disk
*/
Robot::Robot(State stateNow, int thresh, bool saveImages){
    state = stateNow;
    camera = new Camera(thresh, saveImages);
    uart = new UART();
    message = new Message();
}

Robot::~Robot(){
}

/** Start and choose action
*/
void Robot::run(){
    if (state == TEST_UART)
        uartTest();
    else if (state == TEST_UART_MESSAGES)
        uartMessagesTest();
    else if (state == FIND_CIRCLES)
        camera->findCirclesUsingTrackbars();
    else if (state == CALIBRATE_BALL)
        camera->calibrateBall();
    else if (state == CROSSING_SINGLE)
        camera->crossing(true);
    else if (state == CROSSING_CONTINUOUS)
        camera->crossing(false);
    else
        exit(9);
}

/** Reads messages and triggers appropriate actions
@param verbose - detailed output
 */
void Robot::uartMessagesInboundHandle(bool verbose){
    if (uart->available()){
        Message message = uart->readMessage(verbose);
        uint8_t messageId = message.readUInt8();
        switch (messageId) {
            case 'I': /// New state: IDLE
                stateSet(IDLE);
                break;
            case 'l': /// Impossible for RPI
                exit(11);
            case 'L': /// New state: LINE
                stateSet(LINE);
                break;
            case 'r': /// Impossible for RPI
                exit(12);
            case 'R': /// New state: RED_ROOM
                stateSet(RED_ROOM);
                break;
            default: // RPI cannot command Arduino to change state
                cerr << "Impossible message id: " << messageId << endl;
                exit(11);
        }
    }
}

/** Part of the test initiated from Arduino UART.ino in UART library.
*/
void Robot::uartMessagesTest(){
    uint32_t lastSentMs = 0;
    int16_t x = 40;
    while (state != IDLE){
        switch(state){
            case LINE:
                if (millis() - lastSentMs > 100){ /// Every 100 ms
                    lastSentMs = millis();

                    x += rand() % 11 - 5; /// Add between -5 and 5
                    if (x < 0)
                        x = 0;
                    if (x > 80)
                        x = 80;

                    /// Construct and send a message: new x position
                    message->reset();
                    message->append((uint8_t)'l');
                    message->append((uint16_t)x);
                    uart->write(*message, true);
                }
                break;
            case RED_ROOM:
                break;
            case TEST_UART_MESSAGES:
                break;
            default:
                exit(8);
        }
        uartMessagesInboundHandle(true);
    }
}


/** Part of the test initiated from Arduino MRMS_Line_RPI in MRMS_Line_RPI library.
*/
void Robot::uartTest(){
    while (true){
        if (uart->available()){
            cout << uart->read();
            cout.flush();
        }
    }
}
