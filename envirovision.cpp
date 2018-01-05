#include <arpa/inet.h>
#include <chrono>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <mutex>
#include <time.h>
#include <thread>
#include <unistd.h>

#include "cscore.h"
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <wiringPi.h>
#include <wiringSerial.h>

using namespace cv;
using namespace std::chrono;


/******OPTIONS*******/
#define CAMERA 1        // Enable Camera
#define IMU 1           // Enable IMU (if CAMERA == 0, main loop will just print RPY values)
#define DEBUG 0         // Enable Debug Messages
#define IMU_CONFIG 0    // Enable IMU configuring (only needs to be run once / or over MT Manager instead)
#define PRINT_FPS 1     // Enable printing of FPS
#define HTTP_STREAM 1   // Enable Camera stream over SSH

#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240

/*********************/
/***** IMU CODE ******/
/*********************/

// IMU thread writes to these variables, main thread reads from them
float pitchAngle = 0;
float rollAngle = 0;
float yawAngle = 0;
std::mutex mut;

// How rotated the cameras are on the Envirobot Frame in radians
const float fixedYaw = 20*M_PI/180.0;

/**
* eulerAngle  Helper struct to help covert hex to 32 bit floating point decimal
*/
union eulerAngle {
    uint8_t hex[4];
    float angle;
};

/**
* floatSwap  Flip bits of floating point value
* @param value  The float to flip
*/
float floatSwap(float value) {
    union v {
        float f;
        unsigned int i;
    };
    union v val;
    val.f = value;
    val.i = htonl(val.i);
    return val.f;
}

/**
* readMessage  Reads the availabe messages from the MTi 1 XSens IMU
* @param fd       The IMU device handle
*        MID      The ID of the byte stream for the message to match
*        dataArr  Stores the read data (*** Caller must handle cleaning up "*dataArr" after ***)
* @return bool  Returns True if message read properly, False otherwise
*/
bool readMessage(int fd, uint8_t MID, uint8_t **dataArr) {
    int checksum = 0;
    int count = 0;
    bool midMatches = false;
    int dataLength = 0;
    int totalLength = 0;
    bool allocated = false;
    
    // Loop while data still available and message is not complete, note that we do not receive
    // totalLength until count == 3, hence the seemingly redundant if clause
    while(serialDataAvail(fd) || (count <=3 || count <= totalLength)) {
        int data = serialGetchar(fd);
#if DEBUG
        std::cout << std::hex << data << " ";
#endif
        // Wait for start of message (0xFA)
        if (count == 0 && data != 250) {
            continue;
        } else if (count > 0) {
            //Start adding to checksum (skipping 0xFA)
            checksum = checksum + data;
            if (count == 2) {
                // 3rd byte is message ID
                midMatches = (MID == data);
            } else if (count == 3) {
                // 4th byte is data length
                dataLength = data;
                totalLength = 4+data+1;
                // Allocate array to store output data
                if (dataLength > 0) {
                    *dataArr = new uint8_t[dataLength];
                    allocated = true;
                }
            } else if (count >= 4 && count < totalLength-1) {
                // Store data
                (*dataArr)[count-4] = (uint8_t)data;
            } else if (count == totalLength-1) {
                // End of message, check the checksum
                bool valid = ((checksum & 0x11) == 0x00);
#if DEBUG
                if (!midMatches) std::cout << " MID Mismatch ";
                if (valid)
                    std::cout << "CHECKSUM OK";
                else
                    std::cout << "CHECKSUM FAIL";
                std::cout << std::endl;
                fflush(stdout);
#endif
                // If checksum is valid is MID matches, return true, otherwise false
                if (midMatches && valid) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        // Increment byte count
        count++;
    }
}

/**
* setupIMU  Connect IMU and setup configuration parameters
* @return int  Device handle of the connected IMU, -1 if none
*/
int setupIMU() {
    int fd;
    if ((fd = serialOpen("/dev/ttyS1", 115200)) < 0) {
        std::cout << "unable to open serial device" << std::endl;
        return -1;
    }
    if (wiringPiSetup() == -1) {
        std::cout << "wiring pi setup failed" << std::endl;
        return -1;
    }
    std::cout << "IMU setup success" << std::endl;

    // Mssages to send to the IMU
    char reset[] = {0xFA, 0xFF, 0x40, 0x00, 0xC1};
    char selfTest[] = {0xFA, 0xFF, 0x24, 0x00, 0xDD};
    char wakeup[] = {0xFA, 0xFF, 0x3F, 0x00, 0xC2};
    char measurementMode[] = {0xFA, 0xFF, 0x10, 0x00, 0xF1};
    char configMode[] = {0xFA, 0xFF, 0x30, 0x00, 0xD1};
    char baud_rate[] = {0xFA, 0xFF, 0x18, 0x01, 0x0C, 0xE6};
    char ack_mids[] = {0x41, 0x3E, 0x31, 0X19, 0x11};
    char *config_messages[] = {reset, wakeup, configMode, baud_rate, measurementMode};

#if IMU_CONFIG
    // Configure IMU settings by sending messages from config_messages and waiting for corresnding ACKs
    int setupPhase = 0;
    while(setupPhase < 5) {
        std::cout << setupPhase << " " <<  std::endl;
        serialPuts(fd, config_messages[setupPhase]);
        serialFlush(fd);
        delay(100);
        uint8_t *data = nullptr;
        bool ack = readMessage(fd, ack_mids[setupPhase], &data);
        if (data != nullptr) {
            delete[] data;
            data = nullptr;
        }
        if (ack) {
            setupPhase++;
#if DEBUG
            switch(setupPhase) {
                case 1:
                    std::cout << "RESET ACK" << std::endl;
                    break;
                case 2:
                    std::cout << "WAKEUP ACK" << std::endl;
                    break;
                case 3:
                    std::cout << "CONFIG ACK" << std::endl;
                    break;
                case 4:
                    std::cout << "BAUD RATE ACK" << std::endl;
                    break;
                case 5:
                    std::cout << "MEASUREMENT ACK" << std::endl;
                    break;
            }
#endif
        }
    }
#endif
    return fd;
}

/**
* imuMain  Main measurement loop for IMU
* @param fd  Handle for the IMU device returned by setupIMU
*/
void imuMain(int fd) {
    char mt_data_mid = 0x36;
    // MEASUREMENT PHASE
    eulerAngle pitch;
    eulerAngle roll;
    eulerAngle yaw;
    for (;;) {
        uint8_t *data = nullptr;
        bool success = readMessage(fd, mt_data_mid, &data);
        if (success) {
            pitch.hex[0] = data[3];
            pitch.hex[1] = data[4];
            pitch.hex[2] = data[5];
            pitch.hex[3] = data[6];
            roll.hex[0] = data[7];
            roll.hex[1] = data[8];
            roll.hex[2] = data[9];
            roll.hex[3] = data[10];
            yaw.hex[0] = data[11];
            yaw.hex[1] = data[12];
            yaw.hex[2] = data[13];
            yaw.hex[3] = data[14];
            mut.lock();
            pitchAngle  = floatSwap(pitch.angle);
            rollAngle = floatSwap(roll.angle);
            yawAngle = floatSwap(yaw.angle);
            mut.unlock();
        }
        if (data != nullptr) {
            delete[] data;
            data = nullptr;
        }
    }
}

/*************************/
/****** CAMERA CODE ******/
/*************************/

const int width = FRAME_WIDTH;
const int height = FRAME_HEIGHT;

//Variables for camera undistortion
Mat map1, map2;

/**
* setupCamera  Setup camera parameters and load instrinsics and distortion coefficients for fisheye undistortion
*/
void setupCamera() {
    // Left camera
    cs::UsbCamera camera0{"usbcam0", 0};
    camera0.SetVideoMode(cs::VideoMode::kYUYV, width, height, 30);
    camera0.GetProperty("white_balance_temperature_auto").Set(0);
    camera0.GetProperty("white_balance_temperature").Set(4000);
    camera0.GetProperty("brightness").Set(50);
    camera0.GetProperty("exposure_auto").Set(1);
    camera0.GetProperty("hue").Set(50);
    camera0.GetProperty("saturation").Set(25);
    camera0.GetProperty("contrast").Set(50);
    camera0.GetProperty("gamma").Set(100);
    camera0.GetProperty("sharpness").Set(12);
    camera0.GetProperty("exposure_absolute").Set(12);
    std::cout << "Settings for /dev/video0" << std::endl;
    std::cout << "Brightness " << camera0.GetProperty("brightness").Get() << std::endl;
    std::cout << "White Balance Auto " << camera0.GetProperty("white_balance_temperature_auto").Get() << std::endl;
    std::cout << "White Balance Temp " << camera0.GetProperty("white_balance_temperature").Get() << std::endl;
    std::cout << "Exposure Auto " << camera0.GetProperty("exposure_auto").Get() << std::endl;
    std::cout << "Hue " << camera0.GetProperty("hue").Get() << std::endl;
    std::cout << "Saturation " << camera0.GetProperty("saturation").Get() << std::endl;
    std::cout << "Contrast " << camera0.GetProperty("contrast").Get() << std::endl;
    std::cout << "Gamma " << camera0.GetProperty("gamma").Get() << std::endl;
    std::cout << "Sharpness " << camera0.GetProperty("sharpness").Get() << std::endl;
    std::cout << "Exposure " << camera0.GetProperty("exposure_absolute").Get() << std::endl;
    // We have to disconnect if we want to use OpenCV's video writer later
    camera0.DisconnectCamera();
    
    // Right camera
    cs::UsbCamera camera1{"usbcam1", 1};
    camera1.SetVideoMode(cs::VideoMode::kYUYV, width, height, 30);
    camera1.GetProperty("white_balance_temperature_auto").Set(0);
    camera1.GetProperty("white_balance_temperature").Set(4000);
    camera1.GetProperty("brightness").Set(50);
    camera1.GetProperty("exposure_auto").Set(1);
    camera1.GetProperty("hue").Set(50);
    camera1.GetProperty("saturation").Set(25);
    camera1.GetProperty("contrast").Set(50);
    camera1.GetProperty("gamma").Set(100);
    camera1.GetProperty("sharpness").Set(12);
    camera1.GetProperty("exposure_absolute").Set(12);
    
    std::cout << "Brightness " << camera1.GetProperty("brightness").Get() << std::endl;
    std::cout << "White Balance Auto " << camera1.GetProperty("white_balance_temperature_auto").Get() << std::endl;
    std::cout << "White Balance Temp " << camera1.GetProperty("white_balance_temperature").Get() << std::endl;
    std::cout << "Exposure Auto " << camera1.GetProperty("exposure_auto").Get() << std::endl;
    std::cout << "Hue " << camera1.GetProperty("hue").Get() << std::endl;
    std::cout << "Saturation " << camera1.GetProperty("saturation").Get() << std::endl;
    std::cout << "Contrast " << camera1.GetProperty("contrast").Get() << std::endl;
    std::cout << "Gamma " << camera1.GetProperty("gamma").Get() << std::endl;
    std::cout << "Sharpness " << camera1.GetProperty("sharpness").Get() << std::endl;
    std::cout << "Exposure " << camera1.GetProperty("exposure_absolute").Get() << std::endl;
    // We have to disconnect if we want to use OpenCV's video writer later
    camera1.DisconnectCamera();

    // Load distortion instrinsics from file
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    FileStorage fs("calibration.xml", FileStorage::READ);
    fs["intrinsic"] >> intrinsic;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
    double balance = 1.0;
    Mat newIntrinsic = getOptimalNewCameraMatrix(intrinsic, distCoeffs, Size(width,height), balance, Size(width,height));
    cv::initUndistortRectifyMap(intrinsic, distCoeffs, Mat::eye(3,3,CV_32F), newIntrinsic, cv::Size(width,height), CV_16SC2, map1, map2 );
}


// Some globals that only need to be computed once
Mat xAxis, yAxis, zAxis, eye, yawLeft, yawRight, WZLeft, WZRight;
bool yawInit = false;

/**
* initYawMats  Initialize global matrices for image rotation that only need to be calculated once (fixed yaw)
*/
void initYawMats() {
    Mat xAxis_init = (Mat_<double>(3,1) << 1, 0, 0);
    Mat yAxis_init = (Mat_<double>(3,1) << 0, 1, 0);
    // Camera faces down the negative Z-axis!
    Mat zAxis_init = (Mat_<double>(3,1) << 0, 0, -1);
    Mat eye_init = (Mat_<double>(3,3) << 1, 0, 0,
                                0, 1, 0,
                                0, 0, 1);
    Mat yawLeft_init = (Mat_<double>(3,3) <<  cos(fixedYaw),  0, sin(fixedYaw),
                                     0,              1, 0,
                                    -sin(fixedYaw),  0, cos(fixedYaw));

    Mat yawRight_init = (Mat_<double>(3,3) <<  cos(-fixedYaw),  0, sin(-fixedYaw),
                                         0,            1, 0,
                                     -sin(-fixedYaw),  0, cos(fixedYaw));
   
    // Invert the z axis from the yaw transformation and calculate roll around axis in world space
    Mat globalZLeft = yawLeft_init.inv() * zAxis_init;
    double ux = globalZLeft.at<double>(0,0);
    double uy = globalZLeft.at<double>(0,1);
    double uz = globalZLeft.at<double>(0,2);
 
    Mat WZLeft_init =(Mat_<double>(3,3) << 0, -uz, uy,
                 uz, 0, -ux,
                 -uy, ux, 0);
    
    Mat globalZRight = yawRight_init.inv() * zAxis_init;
    ux = globalZRight.at<double>(0,0);
    uy = globalZRight.at<double>(0,1);
    uz = globalZRight.at<double>(0,2);
    
    Mat WZRight_init = (Mat_<double>(3,3) << 0, -uz, uy,
                  uz, 0, -ux,
                  -uy, ux, 0);
    
    xAxis = xAxis_init.clone();
    yAxis = yAxis_init.clone();
    zAxis = zAxis_init.clone();
    eye = eye_init.clone();
    yawLeft = yawLeft_init.clone();
    yawRight = yawRight_init.clone();
    WZLeft = WZLeft_init.clone();
    WZRight = WZRight_init.clone();
}

/**
* rotateIms  Rotate images to adjust for pitch and roll
* @param curPitch  Current pitch angle detected by IMU
*        curRoll  Current roll angle detected by IMU
*        leftImage  Left Camera image to rotate
*        rightImage Right Camera iamge to rotate
*/
void rotateIms(float curPitch, float curRoll, Mat& leftImage, Mat& rightImage) {

    if (!yawInit) {
        initYawMats();
        yawInit = true;
    }
    
    // Convert to radians
    curPitch = -curPitch*M_PI/180.0;
    curRoll = -curRoll*M_PI/180.0;
   
    Mat rollLeft = eye+sin(curRoll)*WZLeft + (2*pow(sin(curRoll/2),2))*WZLeft*WZLeft;
    Mat rollRight = eye+sin(curRoll)*WZRight + (2*pow(sin(curRoll/2),2))*WZRight*WZRight;
    
    // Invert the x axis from the combined yaw-roll transformation and calculate pitch around axis in world space
    Mat globalRLeft = (rollLeft*yawLeft).inv() * xAxis;
    double ux = globalRLeft.at<double>(0,0);
    double uy = globalRLeft.at<double>(0,1);
    double uz = globalRLeft.at<double>(0,2);
    Mat WXLeft =(Mat_<double>(3,3) << 0, -uz, uy,
                 uz, 0, -ux,
                 -uy, ux, 0);
    Mat pitchLeft = eye+sin(curPitch)*WXLeft + (2*pow(sin(curPitch/2),2))*WXLeft*WXLeft;
    Mat leftCameraFullRotMat = pitchLeft*rollLeft*yawLeft;
    
    Mat globalRRight = (rollRight*yawRight).inv() * xAxis;
    ux = globalRRight.at<double>(0,0);
    uy = globalRRight.at<double>(0,1);
    uz = globalRRight.at<double>(0,2);
    Mat WXRight =(Mat_<double>(3,3) << 0, -uz, uy,
                  uz, 0, -ux,
                  -uy, ux, 0);
    Mat pitchRight = eye+sin(curPitch)*WXRight + (2*pow(sin(curPitch/2),2))*WXRight*WXRight;
    Mat rightCameraFullRotMat = pitchRight*rollRight*yawRight;
    
    Mat rotatedXAxisLeft = leftCameraFullRotMat*xAxis;
    Mat rotatedYAxisLeft = leftCameraFullRotMat*yAxis;
    Mat rotatedZAxisLeft = leftCameraFullRotMat*zAxis;
    
    Mat rotatedXAxisRight = rightCameraFullRotMat*xAxis;
    Mat rotatedYAxisRight = rightCameraFullRotMat*yAxis;
    Mat rotatedZAxisRight = rightCameraFullRotMat*zAxis;
 
    // Norms are 1 so the ratio itself is the distance
    double planeDistanceToXZLeft = rotatedXAxisLeft.at<double>(1,0)/rotatedYAxisLeft.at<double>(1,0);
    double planeDistanceToXZRight = rotatedXAxisRight.at<double>(1,0)/rotatedYAxisRight.at<double>(1,0);
    double rollAngleLeft = atan2(planeDistanceToXZLeft,1.0)*180/M_PI;
    double rollAngleRight = atan2(planeDistanceToXZRight,1.0)*180/M_PI;
    
    // Get rotation matrices and perform rotations on image
    Mat M_left = getRotationMatrix2D(Point2f(width/2,height/2),rollAngleLeft,1);
    Mat M_right = getRotationMatrix2D(Point2f(width/2,height/2),rollAngleRight,1);
    warpAffine(leftImage,leftImage,M_left,Size(width,height));
    warpAffine(rightImage,rightImage,M_right,Size(width,height));
}

/************************/
/****** MAIN METHOD *****/
/************************/

int main() {
#if IMU
    // Initialize IMU and configure
    int fd = setupIMU();
    // Start measurement thread
    std::thread imu_thread(imuMain, fd);
#if CAMERA == 0
    for (;;) {
       std::cout << "Device: " << fd << " Pitch: " << pitchAngle << " Roll: " << rollAngle << " Yaw: " << yawAngle << std::endl;
    }
#endif
#endif
#if CAMERA
    // FPS clock
    double elapsedTime;
    high_resolution_clock::time_point curTime;
    int frameCount = 0;
    
    // Setup Camera parameters
    setupCamera();
    
#if HTTP_STREAM
    // Setup Server
    cs::CvSource cvsource{"cvsource", cs::VideoMode::kMJPEG, width*2, height/2, 30};
    cs::MjpegServer cvMjpegServer{"cvhttpserver", 8000};
    cvMjpegServer.SetSource(cvsource);
#endif

    // Setup Video capture
    VideoCapture cap0(0);
    VideoCapture cap1(1);
    cap0.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap0.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    cap1.set(CV_CAP_PROP_FRAME_WIDTH, width);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    
    // Get and process images
    cv::Mat left_image;
    cv::Mat right_image;
    Mat combined(height/2, width*2, CV_8UC3);
    for (;;) {
        curTime = high_resolution_clock::now();
        cap0 >> left_image;
        cap1 >> right_image;
        
        // Undistort images
        Mat left_image_undistorted;
        Mat right_image_undistorted;
        remap( left_image, left_image_undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT );
        remap( right_image, right_image_undistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT );
        
        // Rotate Images to account for pitch and roll
        mut.lock();
        float curPitch = pitchAngle;
        float curRoll = rollAngle;
        mut.unlock();
        rotateIms(curPitch, curRoll, left_image_undistorted, right_image_undistorted);
        
        Mat left_image_sub(left_image_undistorted, Rect(0, height/4, width, height/2));
        Mat right_image_sub(right_image_undistorted, Rect(0, height/4, width, height/2));
        
        //----TODO: Process sub images with visual network here----//
        
#if PRINT_FPS
        duration<double> time_span = duration_cast<duration<double> >(high_resolution_clock::now() - curTime);
        elapsedTime += time_span.count();
        if (elapsedTime > 1.0) {
            elapsedTime = 0;
            std::cout << "FPS: " << frameCount << std::endl;
            frameCount = 0;
            curTime = high_resolution_clock::now();
        }
        frameCount++;
#endif
#if HTTP_STREAM
        // Now send info to streaming server for viewing
        Mat left(combined, Rect(0, 0, width, height/2));
        Mat right(combined, Rect(width, 0, width, height/2));
        left_image_sub.copyTo(left);
        right_image_sub.copyTo(right);
        cvsource.PutFrame(combined);
#endif
    }
#endif
    return 0;
}
