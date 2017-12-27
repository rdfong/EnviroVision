//
//  main.cpp
//  CameraCalibrate
//
//  Created by Roger Fong on 10/27/17.
//

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace cv;
using namespace std;
enum MODE{AUTO, MANUAL};

//In AUTO mode we automatically find chessboard corners with OpenCV, in MANUAL we can click on the images to choose corner points manually
MODE mode = AUTO;

//These needs to be configure based on your checkoard and input image
const int numBoards = 20;
const int numCornersHor = 15;
const int numCornersVer = 9;
const int width = 1000;
const int height = 750;

//The folder where calibrations images are located
const char *imageDir = "../GoProData/";

//Image format of calibration images
// NOTE: Images are expected to be labeled as 1.imageFormat, 2.imageFormat, etc...
const char *imageFormat = ".JPG";

//The name of the calibration coefficients file to either write to or read from if already existing
const char *calibrationFile = "calibration.xml"


const int numSquares = numCornersHor * numCornersVer;
const Size board_sz = Size(numCornersHor, numCornersVer);

struct OnClickParams {
    vector<Point2f> *corners;
    Mat gray_image;
    Size board_sz;
    std::function<void(InputOutputArray, Size, InputArray, bool)> drawCorners;
    std::function<void(const String&, InputArray)> draw;
};

bool is_file_exist(const char *fileName)
{
    std::ifstream infile(fileName);
    return infile.good();
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          OnClickParams *params = (OnClickParams*)userdata;
          params->corners->push_back(Point2f(x,y));
         
          Mat temp_image = params->gray_image.clone();
          params->drawCorners(temp_image, params->board_sz, *(params->corners), true);
          imshow("Corners", temp_image);
     }
}

int main(int argc, const char * argv[]) {
    std::vector<std::vector<Point3f> > object_points;
    std::vector<std::vector<Point2f> > image_points;

    std::vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f((int)(j/numCornersHor), j%numCornersHor, 0.0f));
    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    std::vector<Mat> rvecs;
    std::vector<Mat> tvecs;
    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;
    
    // First try to load camera parameters from file
    if (is_file_exist("calibration.xml")) {
        FileStorage fs("calibration.xml", FileStorage::READ);
        fs["intrinsic"] >> intrinsic;
        fs["distCoeffs"] >> distCoeffs;
        fs.release();
    } else {
        // If not found, find checker board points for each image, find coefficients, write to file
        for (int i = 1; i <=numBoards; i++) {
            std::vector<Point2f> corners;
            std::string filename = std::string(imageDir + std::to_string(i) + imageFormat);
            std::cout << filename <<std::endl;
            Mat curImage = imread(filename.c_str());
            
            if(!curImage.data )  // Check for invalid input
            {
                std::cout <<  "Could not open or find the image" << std::endl ;
                return -1;
            }

            Mat gray_image;
            cvtColor(curImage, gray_image, CV_BGR2GRAY);
            Mat temp_gray = gray_image.clone();
            resize(gray_image, gray_image, Size(gray_image.cols * 0.5,gray_image.rows * 0.5), 0, 0, CV_INTER_LINEAR);
            resize(gray_image, gray_image, Size(gray_image.cols * 2,gray_image.rows * 2), 0, 0, CV_INTER_LINEAR);
            
            //Blurring helped detect corners somewhat, you may want to play around with the number of blurs
            for (int i = 0; i < 3; i++) {
                GaussianBlur(gray_image, temp_gray, cv::Size(0, 0), 3);
                addWeighted(gray_image, 1.5, temp_gray, -0.5, 0, gray_image);
            }
            
            if (mode == AUTO) {
                //This doesn't work all the time, may want to use MANUAL mode if the corner images don't look good
                bool found = findChessboardCorners(gray_image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            } else if (mode == MANUAL) {
                //Just choose points by hand here for each image
                namedWindow("Corners", 1);
                OnClickParams cp;
                cp.board_sz = board_sz;
                cp.gray_image = gray_image;
                cp.corners = &corners;
                cp.drawCorners = drawChessboardCorners;
                setMouseCallback("Corners", CallBackFunc, &cp);
                imshow("Corners", gray_image);
                waitKey(0);
            }
            // Refine corner estimate
            cornerSubPix(gray_image, corners, Size(10, 10), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_sz, corners, true);
            image_points.push_back(corners);
            object_points.push_back(obj);
            imshow("Corners", gray_image);
            waitKey(0);
        }
        
        //Calibration coeffcient gathering
        calibrateCamera(object_points, image_points, Size(width,height), intrinsic, distCoeffs, rvecs, tvecs);
        FileStorage fs("calibration.xml", FileStorage::WRITE);
        fs << "intrinsic" << intrinsic;
        fs << "distCoeffs" << distCoeffs;
        fs.release();
    }
    
    // ---Undistort the calibration images for verification ---//
    
    // Set balance = 1.0 to receive the full uncropped image
    double balance = 1.0;
    Mat newIntrinsic = getOptimalNewCameraMatrix(intrinsic, distCoeffs, Size(width,height), balance, Size(width,height));
    Mat map1, map2;
    cv::initUndistortRectifyMap(intrinsic, distCoeffs, Mat::eye(3,3, CV_32F), newIntrinsic, cv::Size(width,height), CV_16SC2, map1, map2 );
    
    for (int i = 1; i <= numBoards; i++) {
        std::string filename = std::string(imageDir + std::to_string(i) + imageFormat);
        Mat curImage = imread(filename.c_str());
        Mat imageUndistorted = cv::Mat::zeros(curImage.size(), curImage.type());
        remap( curImage, imageUndistorted, map1, map2, INTER_LINEAR, BORDER_CONSTANT );
        
        //Instead of the two previous function calls, you could also call this instead of initUndistortRectifyMap and remap, though separating the two is more efficient
        //undistort(curImage, imageUndistorted, intrinsic, distCoeffs, newIntrinsic);
        
        //Display both original and undistorted image
        imshow("Orig", curImage);
        imshow("Undistorted", imageUndistorted);
        waitKey(0);
    }
    return 0;
}
