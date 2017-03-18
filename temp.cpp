//
// Created by daman on 19/11/16.
//


/* Use edge detection
 * Do Hough transform
 * Do Prob Hough transform
 * TODO: splines, warning system, integrate with object detection.
 * */

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "temp.h"

using namespace cv;

int main(int argc, char** argv){
    int houghVote = 200;
//    VideoCapture capture(atoi(argv[1]));
    Mat orig;

    //Video
    CvCapture* input_video = cvCreateFileCapture("/home/daman/CV/vehicle-detection-master/car.mov");
    CvSize video_size;
    video_size.height = (int)cvGetCaptureProperty(input_video,CV_CAP_PROP_FRAME_HEIGHT);//取得视频的高
    video_size.width = (int)cvGetCaptureProperty(input_video,CV_CAP_PROP_FRAME_WIDTH);
    double fps = cvGetCaptureProperty(input_video,CV_CAP_PROP_FPS);
    int vfps = 1000 / fps;


    namedWindow("Original Image");
    namedWindow("Hough");
    namedWindow("HoughBin");
    namedWindow("HoughP");
    namedWindow("HoughPBin");
    namedWindow("H&HP");
    namedWindow("Final");
    namedWindow("bin");

    /*Main Loop*/
    while(true){
//        capture >> orig;
        orig = cvQueryFrame(input_video);
        if(orig.empty()){
            break;
        }

        Mat gray;
        cvtColor(orig, gray, CV_RGB2GRAY);

        //Get region of interest
        Mat ROIimg;
        Rect roi(0, 2 * (orig.rows/3), orig.cols - 1, orig.rows/3);
        ROIimg = gray(roi);

        //Get edges
        Mat contours, contoursInv;
        Canny(ROIimg,contours,50,250);

        // Do hough
        LaneDetection laneDetection ;
        laneDetection.setShift(2* (orig.rows/3));
        Mat hough(ROIimg.size(),CV_8U,Scalar(255));
        ROIimg.copyTo(hough);
        Mat houghBin(orig.size(),CV_8U,Scalar(0));
        laneDetection.getHoughLines(hough, houghBin, contours);

        // Do probabilistic hough
        laneDetection.setLineLengthAndGap(60,10);
        laneDetection.setMinVote(100);  //30 for bitwise and
        laneDetection.setMinLength(75.0);
        Mat houghP(ROIimg.size(),CV_8U,Scalar(255));
        ROIimg.copyTo(houghP);
        Mat houghPBin(orig.size(),CV_8U,Scalar(0));
        laneDetection.getHoughPLines(houghP, houghPBin, contours);

        //Combine hough & houghP
        Mat HandHP(ROIimg.size(),CV_8U,Scalar(0));
        bitwise_or(houghPBin, houghBin, HandHP);

        //Probabilistic hough gives a better estimate. Use that as final result.
        Mat final(orig.size(),CV_8U,Scalar(0));
        cvtColor(houghBin, houghBin, CV_GRAY2RGB);
        add(orig, houghBin, final);

        //Show binary image
        Mat binary(orig.size(),CV_8U,Scalar(0));
        laneDetection.genBinImg(binary);

        imshow("Original Image",orig);
        imshow("Hough", hough);
        imshow("HoughBin", houghBin);
        imshow("HoughP", houghP);
        imshow("HoughPBin", houghPBin);
        imshow("H&HP", HandHP);
        imshow("Final", final);
        imshow("bin", binary);
        waitKey(2);
    }
}
