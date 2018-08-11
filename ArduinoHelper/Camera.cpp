#include "Camera.h"
#include <ctime>
#include <dirent.h>
#include <iostream>
#include <math.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <raspicam/raspicam.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <wiringPi.h>

using namespace std;
using namespace cv;

/** Constructor
@param runtType - chosen program's behavior.
@param thresh - Canny threshold.
@param saveImages - save images to disk.
*/
Camera::Camera(int threshold, bool saveImagesNow){
    pRaspiCam = new raspicam::RaspiCam_Cv();// Sprema se pointer koji pokazuje na kameru. Pomoću njega se kasnije uvijek poziva ova kamera.
    thresh = threshold;
    saveImages = saveImagesNow;
    lastCameraMs = 0;
    startMs = 0;

    cout << "Camera..." << flush;

    /// Camera's parameters
    pRaspiCam->set(CV_CAP_PROP_FORMAT, CV_8UC3);
    const bool HIGH_RES = false;//todo
    pRaspiCam->set(CV_CAP_PROP_FRAME_WIDTH, HIGH_RES ? 1920 : 160);//320, 160
    pRaspiCam->set(CV_CAP_PROP_FRAME_HEIGHT, HIGH_RES ? 1080 : 120);//240, 120

    /// Start the camera
    cout << "opening...";
    if (!pRaspiCam->open())
        cerr<<"Error opening the camera"<<endl;

    cout << "OK" << endl;
}

/** Destructor
*/
Camera::~Camera(){
    cout << "Stop camera..." << endl;
    pRaspiCam->release();
}

/** Find HSV parameters to maximize number of found circles. Warning: this is no desired result for finding a single ball. To calibrate a sinle ball,
a viable solution would be to put the ball in a predefined position and then find the values that yield only a single, biggest shape.
*/
void Camera::calibrateBall(){

    waitForCapture();

    int circleCount;
    int maxCircleCount = -1;
    int extremeLow, extremeHigh;

    for (int low = 0; low <= 255; low += 10){
        for (int high = low + 10; high <= 255; high += 10){
            findCircles(0, 255, 0, 255, low, high, false, circleCount);
            if (circleCount > maxCircleCount){
                maxCircleCount = circleCount;
                extremeLow = low;
                extremeHigh = high;
            }
            cout << low << "-" << high << ": " << circleCount << endl;
        }
    }

}

/** Camera captures one image.
*/
void Camera::capture(){
    if ((millis() - lastCameraMs)  > 30 ){/// 33 FPS
        pRaspiCam->grab(); /// Take a picture
        pRaspiCam->retrieve(srcImage); /// Copy it to srcImage

        lastCameraMs = millis();
    }
}

/** Detect a geen marker in RoboCup Line crossing.
@param display - display picture by picture. A key must be pressed to advance. Otherwise a continuous flow with FPS indicated.
*/
void Camera::crossing(bool display){

    ///Change these values to detech green only:
    uint8_t lowH = 40;
    uint8_t highH = 80;
    uint8_t lowS = 00;
    uint8_t highS = 255;
    uint8_t lowV = 40;
    uint8_t highV = 120;

    waitForCapture();
    Mat imgHSV;
    startMs = millis();
    cnt = 0;

    while(true){

        capture();

        /// Crop the picture, remove upper part.
        uint16_t yStart = srcImage.rows * 0.35;
        Rect roi(0, yStart, srcImage.cols, srcImage.rows - yStart);
        srcImage = srcImage(roi);

        /// Change colorspace to HSV (hue, saturation, value).
        cvtColor(srcImage, imgHSV, COLOR_BGR2HSV);

        /// Separate the green part.
        Mat imgThresholdGreen;
        inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgThresholdGreen);

        /// Erode and dilate the image to delete small islands inside and outside.
        erode(imgThresholdGreen, imgThresholdGreen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
        dilate(imgThresholdGreen, imgThresholdGreen, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

        /// Separata black parts.
        Mat imgThresholdBlack;
        inRange(imgHSV, Scalar(0, 0, 0), Scalar(179, 255, 50), imgThresholdBlack);

        /// Canny - find edges
        Mat canny_output;//Izlazna matrica
        vector<vector<Point> > contours;//Sve konture
        vector<Vec4i> hierarchy;
        Canny( imgThresholdGreen, canny_output, thresh, thresh*2, 3 );

        /// Find (all) contours
        findContours(canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

        /// Check every countour
        for( uint16_t i = 0; i< contours.size(); i++ ) //Po svi konturama slike.
        {
            /// Calculate moments: center of gravity and area
            Moments oMoments = moments(contours[i]);
            double dM01 = oMoments.m01;
            double dM10 = oMoments.m10;
            double area = oMoments.m00;
            int cX = dM10 / area; /// Gravity centre's x
            int cY = dM01 / area; /// y

            /// If area is big enough, it can be a marker
            if (area > srcImage.cols * srcImage.rows /8000){

                /// Write some text to label the marker
                putText(srcImage, "Marker", Point(cX - 10, cY), FONT_HERSHEY_COMPLEX_SMALL, 0.8, Scalar(0, 255, 0), 0.6, CV_AA);

                /// Draw this contour (in green).
                drawContours(srcImage, contours, i, Scalar(0, 255, 0), 2, 8, hierarchy, 0, Point() );

                /// Draw 3 red circles, designating the black-check areas.
                uint8_t dX = srcImage.cols * 0.16;
                uint8_t dY = srcImage.rows * 0.28;
                circle(srcImage, Point(cX - dX, cY), 2, Scalar(0, 0, 255));
                circle(srcImage, Point(cX + dX, cY), 2, Scalar(0, 0, 255));
                circle(srcImage, Point(cX, cY - dY), 2, Scalar(0, 0, 255));

                if (imgThresholdBlack.at<uint8_t>(Point(cX, cY - dY)) == 255) /// If point above is black, this can be a marker
                    if (imgThresholdBlack.at<uint8_t>(Point(cX - dX, cY)) == 255){ /// if the one to the left is also black, this is a right marker.
                        cout << "Right marker" << endl;
                        break;
                    }
                    else if (imgThresholdBlack.at<uint8_t>(Point(cX + dX, cY)) == 255){/// if the one to the right is also black, this is a left marker.
                        cout << "Left marker" << endl;
                        break;
                    }
            }
        }

        /// Display all thw windows
        if (display){
            imshow("Original", srcImage); /// Original image
            moveWindow("Original", 500, 35);
            imshow("ThresholdedGreen", imgThresholdGreen); /// Green part
            moveWindow("ThresholdedGreen", 500, 540);
            imshow("ThresholdedBlack", imgThresholdBlack); /// Black part
            moveWindow("ThresholdedBlack", 1100, 35);
            imshow("ThresholdedBoth", imgThresholdBlack | imgThresholdGreen); /// Green and black parts
            moveWindow("ThresholdedBoth", 1100, 540);

            /// Wait for a key. If Esc, exit the program.
            uint8_t ch = waitKey(0);
            if (ch == 'q' || ch == 27)//esc
                exit(0);
        }
        else
            fps(); /// Display FPS
    }
}

/** Use trackbars to define HSV (hue, saturation, value) parameters and watch the detected circles changing.
@param lowH - Hsv lower limit
@param highH - Hsv upper limit
@param lowS - hSv lower limit
@param highS - hSv upper limit
@param lowV - hsV lower limit
@param highV - hsV upper limit
*/
void Camera::findCirclesUsingTrackbars(int lowH, int highH, int lowS, int highS, int lowV, int highV){

    /// Default values:
    if (lowH == -1) lowH = 0;
    if (highH == -1) highH = 179;
    if (lowS == -1) lowS = 0;
    if (highS == -1) highS = 255;
    if (lowV == -1) lowV = 60;
    if (highV == -1) highV = 147;

    /// Window for trackbars
    namedWindow("Control", WINDOW_AUTOSIZE);

    /// Add trackbars to the window above.
    createTrackbar("LowH", "Control", &lowH, 179);
    createTrackbar("HighH", "Control", &highH, 179);
    createTrackbar("LowS", "Control", &lowS, 255);
    createTrackbar("HighS", "Control", &highS, 255);
    createTrackbar("LowV", "Control", &lowV, 255);
    createTrackbar("HighV", "Control", &highV, 255);

    /// Continuously capture pictures and find circles. After each step, You can alter HSV parameters.
    int circleCount;
    for (int i = 0; i < 1000; i++){
        capture();
        cout << "Frame " << i << " captured" << endl;
        findCircles(lowH, highH, lowS, highS, lowV, highV, true, circleCount);
    }
}

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
void Camera::findCircles(int lowH, int highH, int lowS, int highS, int lowV, int highV,  bool display, int &numberOfCircles){

    /// Limits:
    if (highH > 179) highH = 179;
    if (highS > 255) highS = 255;
    if (highV > 255) highV = 255;

    if (srcImage.empty())
        return;

    /// Convert image to HSV space (hue, saturation, value).
    Mat imgHSV;
    cvtColor(srcImage, imgHSV, COLOR_BGR2HSV);

    /// 2 structures to hod images with separated colors (thresholded).
    Mat imgThresholded;
    Mat imgThresholded2;

    /// Separate colors
    if(lowH > highH) /// A more complicated case when end and beginning part of H is requested.
    {
        inRange(imgHSV, Scalar(0, lowS, lowV), Scalar(highH, highS, highV), imgThresholded);
        inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(179, highS, highV), imgThresholded2);
        imgThresholded = imgThresholded + imgThresholded2;
    }
    else
        inRange(imgHSV, Scalar(lowH, lowS, lowV), Scalar(highH, highS, highV), imgThresholded);

    /// Remove small islands.
    erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
    dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

    /// Hough Circles transform - check OpenCV documentation.
    vector<Vec3f> circles;
    HoughCircles(imgThresholded, circles, HOUGH_GRADIENT, 1,
                 imgThresholded.rows/3,  // change this value to detect circles with different distances to each other
                 100, 20, 20, 0 // change the last two parameters
            // (min_radius & max_radius) to detect larger circles
    );
    numberOfCircles = circles.size();

    /// Display all the circles
    if (display){
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);

            /// Draw centre on the source image.
            circle( srcImage, center, 1, Scalar(0,100,100), 3, LINE_AA);

            /// Draw circular outline
            int radius = c[2];
            circle( srcImage, center, radius, Scalar(255,0,255), 3, LINE_AA);
        }

        /// Display all the windows
        imshow("Original", srcImage); /// Original image
        moveWindow("Original", 500, 35);
        imshow("Thresholded", imgThresholded); /// Thresholded
        moveWindow("Thresholded", 500, 540);

        /// Wait for a key. If Esc, exit the program.
        uint8_t ch = waitKey(0);
        if (ch == 'q' || ch == 27)//esc
            exit(0);
       //imshow("Thresholded2", imgThresholded + imgThresholoded2);
    }
}

/** Frames Per Second
*/
void Camera::fps(){
    cnt++;
    if (millis() - lastFpsDisplayMs > 10000) {//Svakih 10 sec ispis FPSa, broja slika u sekundi.
		cout << endl << cnt << " image in " << ((millis() - startMs)/1000) << " sec: "<< (round(cnt / ((millis() - startMs) / 1000.0))) << " FPS." << endl;
        lastFpsDisplayMs = millis();
	}
}


/** A way of testing program with not live images. Instead, read images from disk. Record a few hunders images and run this test each time You change the
program to be sure the change didn't break something.
*/
void Camera::unitTest(){
    const bool verbose = false;
    DIR *dir;
    struct dirent *ent;
    if ((dir = opendir ("/home/pi/images/")) != NULL) {/// Check directory's existence. You can use some other path.

        RNG rng(12345); /// Random generator

        /// Print all the files and directories within directory.
        while ((ent = readdir (dir)) != NULL) {
            cout << ent->d_name << endl; /// Files' name.

            String imgName = (String)"/home/pi/images/" + ent->d_name;
            srcImage = imread(imgName.c_str(), IMREAD_COLOR);   /// Read the image
            if(!srcImage.data) /// It must not be empty.
            {
                cout <<  "Could not open or find the image" << std::endl ;
                return;
            }

            /// The missing part here is Your test.
        }
        closedir (dir); /// Close the opened directory.
    } else {
        /// Could not open directory.
        perror ("Directory error.");
        return;
    }
}

/** Keep on capturing until a non-empty picture appears.
*/
void Camera::waitForCapture(){
    uint32_t startMs = millis();
    bool ok = false;
    while(millis() - startMs < 2000){
        capture();
        if (!srcImage.empty()) {
            ok = true;
            break;
        }
    }
    if (!ok){
        cout << "No input image!";
        exit (0);
    }
}

