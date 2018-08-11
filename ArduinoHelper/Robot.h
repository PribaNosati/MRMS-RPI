#ifndef ROBOT_H
#define ROBOT_H

#include "Camera.h"
#include "UART.h"

class Robot
{
    public:
        /// State machine pattern
        enum State {
            /// Tests
            FIND_CIRCLES, CALIBRATE_BALL, CROSSING_SINGLE, CROSSING_CONTINUOUS, TEST_STORED_IMAGES,
            TEST_CAMERA_IMAGES, TEST_UART, TEST_UART_MESSAGES,
            /// Run states
            IDLE, LINE, RED_ROOM};

        /** Constructor
        @param state - initial state
        @param thresh - OpenCV Canny's threshold
        @param saveImages - save to disk
        */
        Robot(State state = IDLE, int thresh = 100, bool saveImages = false);

        /** Destructor
        */
        virtual ~Robot();

        /** Start
        */
        void run();

        /** Get state
        @return - current state
        */
        State stateGet(){ return state;}

        /** Set state
        @param newState - new state
        */
        void stateSet(State newState){ state = newState;}

        /** Reads messages and triggers appropriate actions
        @param verbose - detailed output
        */
        void uartMessagesInboundHandle(bool verbose = false);

        /** Part of the test initiated from Arduino UART.ino in UART library.
        */
        void uartMessagesTest();

        /** Part of the test initiated from Arduino MRMS_Line_RPI in MRMS_Line_RPI library.
        */
        void uartTest();

    private:
        Camera *camera; /// RPI camera
        State state; /// Robot's state - according to State Machine pattern
        UART *uart; /// Serial port
        Message *message; /// Current (or last) message
};

#endif // ROBOT_H
