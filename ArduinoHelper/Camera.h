#ifndef CAMERA_H_INCLUDED
#define CAMERA_H_INCLUDED
#include <raspicam/raspicam_cv.h>
#include <vector>
#include <string>

using namespace cv;

/** Class for all of the computer vision methods
*/
class Camera{
    public:
        /** Constructor
        @param thresh - Canny threshold.
        @param saveImages - save images to disk.
        */
        Camera(int thresh = 100, bool saveImages = false);

        /** Destructor
        */
        ~Camera();

        /** Find HSV parameters to maximize number of found circles. Warning: this is no desired result for finding a single ball. To calibrate a sinle ball,
        a viable solution would be to put the ball in a predefined position and then find the values that yield only a single, biggest shape.
        */
		void calibrateBall();

		/** Camera captures one image.
        */
        void capture();

        /** Detect a geen marker in RoboCup Line crossing.
        @param display - display picture by picture. A key must be pressed to advance. Otherwise a continuous flow with FPS indicated.
        */
        void crossing(bool display = true);

        /** Use trackbars to define HSV (hue, saturation, value) parameters and watch the detected circles changing.
        @param lowH - Hsv lower limit
        @param highH - Hsv upper limit
        @param lowS - hSv lower limit
        @param highS - hSv upper limit
        @param lowV - hsV lower limit
        @param highV - hsV upper limit
        */
		void findCirclesUsingTrackbars(int lowH = -1, int highH = -1, int lowS = -1, int highS = -1, int lowV = -1, int highV = -1);

		/** Detect circles using predefined HSV values (no trackbars).
        @param lowH - Hsv lower limit
        @param highH - Hsv upper limit
        @param lowS - hSv lower limit
        @param highS - hSv upper limit
        @param lowV - hsV lower limit
        @param highV - hsV upper limit
        @param display - break and show results for each image
        @param circleCount - number of circles found.
        */
        void findCircles(int lowH, int highH, int lowS, int highS, int lowV, int highV,  bool display, int &circleCount);

        /** Frames Per Second
        */
        void fps();

        /** A way of testing program with not live images. Instead, read images from disk. Record a few hunders images and run this test each time You change the
        program to be sure the change didn't break something.
        */
        void unitTest();

    private:
        uint32_t cnt = 0;/// FPS counter
        raspicam::RaspiCam_Cv* pRaspiCam; /// Camera object
        uint32_t lastCameraMs; /// Last image capture time
        uint32_t lastFpsDisplayMs = 0; /// Last FPS display time
        uint16_t lastImageNumber = 0;  /// Used for storing images to disk
        Mat srcImage; /// Raw picture, as camera captured it.
        bool saveImages; /// Saving captured images to disk.
        uint32_t startMs; /// Program start time, used for FPS calculation
        int thresh; /// Threshold for Canny algorithm.

        /** Keep on capturing until a non-empty picture appears.
        */
        void waitForCapture();
};

#endif // CAMERA_H_INCLUDED
