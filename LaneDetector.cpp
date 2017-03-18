//
// Created by daman on 02/12/16.
//

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "LaneDetector.h"

using namespace cv;

int main(int argc, char** argv){
    Mat orig;
    namedWindow("Test");
    namedWindow("Binary");
    LaneDetector laneDetector;

    //Camera feed
//    VideoCapture input(atoi(argv[1]));

    //Video file
    CvCapture* input = cvCreateFileCapture("/home/daman/videoplayback.mp4");
    while(true){
        //Camera feed
//        input >> orig;

        //Video feed
        orig = cvQueryFrame(input);
        if(orig.empty()){
            break;
        }

        //Get grayscale image
        Mat gray;
        cvtColor(orig, gray, CV_RGB2GRAY);

        //For curves
        laneDetector.setCurveImg(orig);

        //Define different ROIs
        Mat roi1, roi2, roi3, roi4;
        int roiCount = 4;
        int roiDist = (roiCount * (roiCount + 1)) / 2;

        Rect roiRect1(0, (roiDist*2 - roiCount - 0) * (orig.rows/(2 * roiDist)), orig.cols - 1, (roiCount - 0) * (orig.rows/(2 * roiDist)));
        Rect roiRect2(0, (roiDist*2 - roiCount - 1) * (orig.rows/(2 * roiDist)), orig.cols - 1, (roiCount - 1) * (orig.rows/(2 * roiDist)));
        Rect roiRect3(0, (roiDist*2 - roiCount - 2) * (orig.rows/(2 * roiDist)), orig.cols - 1, (roiCount - 2) * (orig.rows/(2 * roiDist)));
        Rect roiRect4(0, (roiDist*2 - roiCount - 3) * (orig.rows/(2 * roiDist)), orig.cols - 1, (roiCount - 3) * (orig.rows/(2 * roiDist)));

        roi1 = gray(roiRect1);
        roi2 = gray(roiRect2);
        roi3 = gray(roiRect3);
        roi4 = gray(roiRect4);

        //Get contours. Lane detection(1).
        Mat cont1, cont2, cont3, cont4;
        Canny(roi1, cont1, 150, 250);
        Canny(roi2, cont2, 150, 250);
        Canny(roi3, cont3, 150, 250);
        Canny(roi4, cont4, 150, 250);

        laneDetector.setDeltaRho(1.0);
        laneDetector.setDeltaTheta(1.0);
        vector<cv::Vec4i> combinedLines;

        //Hough for roi1
        laneDetector.setImgShift((roiDist*2 - roiCount - 0) * (orig.rows/(2 * roiDist)));
        laneDetector.setThetaLane(25.0);
        laneDetector.setMinVote(30);
        laneDetector.setMinLength((roiCount - 0) * (orig.rows/(2 * roiDist)) / 3);
        laneDetector.setMaxGap(10);

        Mat hough1;
        roi1.copyTo(hough1);
        Mat hough1Bin(orig.size(), CV_8UC3, Scalar(0, 0, 0));
        laneDetector.getHoughPLines(hough1, hough1Bin, cont1, (roiCount - 0) * (orig.rows/(2 * roiDist)),
                                    (roiDist*2 - roiCount - 0) * (orig.rows/(2 * roiDist)), 0, Scalar(0, 255, 0));


        //Hough for roi2
        laneDetector.setImgShift((roiDist*2 - roiCount - 1) * (orig.rows/(2 * roiDist)));
        laneDetector.setThetaLane(25.0);
        laneDetector.setMinVote(30);
        laneDetector.setMinLength((roiCount - 1) * (orig.rows/(2 * roiDist)) / 3);
        laneDetector.setMaxGap(10);

        Mat hough2;
        roi2.copyTo(hough2);
        Mat hough2Bin(orig.size(), CV_8UC3, Scalar(0, 0, 0));
        laneDetector.getHoughPLines(hough2, hough2Bin, cont2, (roiCount - 1) * (orig.rows/(2 * roiDist)),
                                    (roiDist*2 - roiCount - 1) * (orig.rows/(2 * roiDist)), 1, Scalar(0, 255, 0));


        //Hough for roi3
        laneDetector.setImgShift((roiDist*2 - roiCount - 2) * (orig.rows/(2 * roiDist)));
        laneDetector.setThetaLane(20.0);
        laneDetector.setMinVote(20);
        laneDetector.setMinLength((roiCount - 2) * (orig.rows/(2 * roiDist)) / 4);
        laneDetector.setMaxGap(10);

        Mat hough3;
        roi3.copyTo(hough3);
        Mat hough3Bin(orig.size(), CV_8UC3, Scalar(0, 0, 0));
        laneDetector.getHoughPLines(hough3, hough3Bin, cont3, (roiCount - 2) * (orig.rows/(2 * roiDist)),
                                    (roiDist*2 - roiCount - 2) * (orig.rows/(2 * roiDist)), 2, Scalar(0, 255, 0));


        //Hough for roi4
        laneDetector.setImgShift((roiDist*2 - roiCount - 3) * (orig.rows/(2 * roiDist)));
        laneDetector.setThetaLane(15.0);
        laneDetector.setMinVote(25);
        laneDetector.setMinLength((roiCount - 3) * (orig.rows/(2 * roiDist)) / 6);
        laneDetector.setMaxGap(10);

        Mat hough4;
        roi4.copyTo(hough4);
        Mat hough4Bin(orig.size(), CV_8UC3, Scalar(0, 0, 0));
        laneDetector.getHoughPLines(hough4, hough4Bin, cont4, (roiCount - 3) * (orig.rows/(2 * roiDist)),
                                    (roiDist*2 - roiCount - 3) * (orig.rows/(2 * roiDist)), 3, Scalar(0, 255, 0));

        //Combine hough lines. Approximate curve.
        combinedLines = laneDetector.combineLines();
        Mat test;
        orig.copyTo(test);
        laneDetector.displayLines(test, combinedLines, Scalar(255, 0, 0));

        //Generate binary mask
        Mat binary(orig.size(), CV_8U, Scalar(0));
        if(combinedLines.size() > 5){
            Point ul(combinedLines.at(0)[0], combinedLines.at(0)[1]);
            Point ll(combinedLines.at(2)[2], combinedLines.at(2)[3]);
            Point ur(combinedLines.at(3)[0], combinedLines.at(3)[1]);
            Point lr(combinedLines.at(5)[2], combinedLines.at(5)[2]);
            laneDetector.genBinImg(binary, ul, ll, ur, lr);
        }

        Mat final(orig.size(), CV_8UC3, Scalar(0,0,0));
        add(hough1Bin, hough2Bin, final);
        add(final, hough3Bin, final);
        add(final, hough4Bin, final);
        add(final, orig, final);

        imshow("Test", test);
        imshow("Binary", laneDetector.getCurveImg());
        waitKey(10);
    }
    return 0;
}

