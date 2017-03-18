#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "main.h"

using namespace cv;

int main(int argc, char** argv){

    int houghVote = 200;
//    VideoCapture capture(atoi(argv[1]));
    Mat orig;

    //Video
    CvCapture* input_video = cvCreateFileCapture("/home/daman/videoplayback.mp4");
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
        Mat ROIimg1;
        Mat ROIimg2;
        Mat ROIimg3;
        Rect roi1(0, 7 * (gray.rows/8), gray.cols - 1, gray.rows/8);
        Rect roi2(0, 6 * (gray.rows/8), gray.cols - 1, gray.rows/8);
        Rect roi3(0, 5 * (gray.rows/8), gray.cols - 1, gray.rows/8);
        ROIimg1 = gray(roi1);
        ROIimg2 = gray(roi2);
        ROIimg3 = gray(roi3);

        //Get edges
        Mat contours1, contours2, contours3;
        Canny(ROIimg1,contours1, 50, 250);
        Canny(ROIimg2, contours2, 50, 250);
        Canny(ROIimg3, contours3, 50, 250);

        // Do hough
//        LaneDetection laneDetection ;
//        laneDetection.setShift(2* (orig.rows/3));
//        Mat hough(ROIimg.size(),CV_8U,Scalar(255));
//        ROIimg.copyTo(hough);
//        Mat houghBin(orig.size(),CV_8U,Scalar(0));
//        laneDetection.getHoughLines(hough, houghBin, contours);

        // Do probabilistic hough

        LaneDetection laneDetection;
        laneDetection.setShift(7* (gray.rows/8));
        laneDetection.setLineLengthAndGap(60,10);
        laneDetection.setMinVote(80);  //30 for bitwise and
        laneDetection.setMinLength(10.0);
        Mat houghP1(ROIimg1.size(),CV_8U,Scalar(255));
        ROIimg1.copyTo(houghP1);
        Mat houghPBin1(orig.size(),CV_8U,Scalar(0, 0, 0));
        laneDetection.getHoughPLines(houghP1, houghPBin1, contours1, 25.0);

        laneDetection.setShift(6* (gray.rows/8));
        Mat houghP2(ROIimg2.size(),CV_8U,Scalar(255));
        ROIimg2.copyTo(houghP2);
        Mat houghPBin2(orig.size(),CV_8U,Scalar(0, 0, 0));
        laneDetection.getHoughPLines(houghP2, houghPBin2, contours2, 20.0);

        laneDetection.setShift(5* (gray.rows/8));
        Mat houghP3(ROIimg3.size(),CV_8U,Scalar(255));
        ROIimg3.copyTo(houghP3);
        Mat houghPBin3(orig.size(),CV_8U,Scalar(0, 0, 0));
        laneDetection.getHoughPLines(houghP3, houghPBin3, contours3, 20.0);

        Mat group(orig.size(),CV_8U,Scalar(0));
        add(houghPBin1, houghPBin2, group);
        add(group, houghPBin3, group);

        //Combine hough & houghP
//        Mat HandHP(ROIimg.size(),CV_8U,Scalar(0));
//        bitwise_or(houghPBin, houghBin, HandHP);

        //Probabilistic hough gives a better estimate. Use that as final result.
        Mat final(orig.size(),CV_8U,Scalar(0));
        cvtColor(group, group, CV_GRAY2RGB);
        add(group, orig, final);

        //Show binary image
        Mat binary(orig.size(),CV_8U,Scalar(0));
        laneDetection.genBinImg(binary);

        imshow("Original Image",orig);
//        imshow("Hough", hough);
//        imshow("HoughBin", houghBin);
        imshow("HoughP", houghPBin3);
        imshow("HoughPBin", houghPBin1);
        imshow("ROI1", houghPBin2);
        imshow("Final", final);
//        imshow("bin", group);
        waitKey(2);
    }
}