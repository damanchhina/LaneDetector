//
// Created by daman on 02/12/16.
//

#ifndef LANEDETECTOR_LANEDETECTOR_H
#define LANEDETECTOR_LANEDETECTOR_H

#define PI 3.1415926

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

class LaneDetector {
private:
    int minVote;
    int maxGap;
    int imgShift;
    double minLength;
    double deltaRho;
    double deltaTheta;
    float thetaLane;
    cv::Mat curveImg;

    std::vector<cv::Vec4i> linesP;
    std::vector<cv::Vec4i> roiTrack[4];

public:
    LaneDetector() : minVote(10), minLength(1.0), imgShift(0), maxGap(0), deltaRho(0.), deltaTheta(0.),
                        thetaLane(0.0){}

    void setMinVote(int minVote) {
        LaneDetector::minVote = minVote;
    }

    void setMaxGap(int maxGap) {
        LaneDetector::maxGap = maxGap;
    }

    void setMinLength(double minLength) {
        LaneDetector::minLength = minLength;
    }

    void setDeltaRho(double deltaRho) {
        LaneDetector::deltaRho = deltaRho;
    }

    void setDeltaTheta(double deltaTheta) {
        LaneDetector::deltaTheta = deltaTheta;
    }

    void setImgShift(int imgShift) {
        LaneDetector::imgShift = imgShift;
    }

    void setThetaLane(float thetaLane) {
        LaneDetector::thetaLane = thetaLane;
    }

    void setCurveImg(const cv::Mat &curveImg) {
        curveImg.copyTo(LaneDetector::curveImg);
    }

    const cv::Mat &getCurveImg() const {
        return curveImg;
    }

    //Lane recognition(2) and tracking(3).
    void getHoughPLines(cv::Mat &roi, cv::Mat &roiBin, cv::Mat &contours, int lineLength, int startY,
                       int roiNo,  cv::Scalar color = cv::Scalar(255, 255, 255)){
        linesP.clear();
        cv::HoughLinesP(contours,linesP,deltaRho,deltaTheta,minVote, minLength, maxGap);
        std::vector<cv::Vec4i> lines = averageLine(roi.cols/2);
        std::cout<<"lines size " << lines.size()<< " " << roiTrack[roiNo].size()<<"\n";
        if(lines.size() > 1){
            std::vector<cv::Vec4i> roiTemp;
            cv::Point pt1(lines.at(0)[0], lines.at(0)[1]);
            cv::Point pt2(lines.at(0)[2], lines.at(0)[3]);
            cv::Point pt3(lines.at(0)[0], lines.at(0)[1] + imgShift);
            cv::Point pt4(lines.at(0)[2], lines.at(0)[3] + imgShift);

            cv::Point pt5(lines.at(1)[0], lines.at(1)[1]);
            cv::Point pt6(lines.at(1)[2], lines.at(1)[3]);
            cv::Point pt7(lines.at(1)[0], lines.at(1)[1] + imgShift);
            cv::Point pt8(lines.at(1)[2], lines.at(1)[3] + imgShift);

            double thetaDegL = (180/PI) * atan(getSlope(pt3.x, pt3.y, pt4.x, pt4.y));
            double thetaDegR = (180/PI) * atan(getSlope(pt7.x, pt7.y, pt8.x, pt8.y));

            if((thetaDegL > -2*thetaLane && thetaDegL < -thetaLane) ){
                std::cout<<"thetaL "<<roiNo << " " << thetaDegL<<"\n";
                line(roi, pt1, pt2, cv::Scalar(255), 5);
                roiTemp.push_back(customLine(roiBin, pt3, pt4, lineLength, startY, color));
//                roiTemp.push_back(cv::Vec4i(pt3.x, pt3.y, pt4.x, pt4.y));
            }

            if((thetaDegR > thetaLane && thetaDegR < 2*thetaLane ) ){
                std::cout<<"thetaR "<<roiNo << " " << thetaDegR<<"\n";
                line(roi, pt5, pt6, cv::Scalar(255), 5);
                roiTemp.push_back(customLine(roiBin, pt7, pt8, lineLength, startY, color));
//                roiTemp.push_back(cv::Vec4i(pt7.x, pt7.y, pt8.x, pt8.y));
            }
            //shift lines
            lines.at(0)[1] += imgShift;
            lines.at(0)[3] += imgShift;
            lines.at(1)[1] += imgShift;
            lines.at(1)[3] += imgShift;
            if(roiTemp.size() > 1){
                copyVec(roiTemp, roiTrack[roiNo]);
            }
        }
        else if(roiTrack[roiNo].size() > 1){
            printf("Got here %d\n", roiTrack[roiNo].size());

            cv::Point pt1(roiTrack[roiNo].at(0)[0], roiTrack[roiNo].at(0)[1]);
            cv::Point pt2(roiTrack[roiNo].at(0)[2], roiTrack[roiNo].at(0)[3]);
            cv::Point pt3(roiTrack[roiNo].at(0)[0], roiTrack[roiNo].at(0)[1]);
            cv::Point pt4(roiTrack[roiNo].at(0)[2], roiTrack[roiNo].at(0)[3]);

            cv::Point pt5(roiTrack[roiNo].at(1)[0], roiTrack[roiNo].at(1)[1]);
            cv::Point pt6(roiTrack[roiNo].at(1)[2], roiTrack[roiNo].at(1)[3]);
            cv::Point pt7(roiTrack[roiNo].at(1)[0], roiTrack[roiNo].at(1)[1]);
            cv::Point pt8(roiTrack[roiNo].at(1)[2], roiTrack[roiNo].at(1)[3]);

            double thetaDegL = (180/PI) * atan(getSlope(pt3.x, pt3.y, pt4.x, pt4.y));
            double thetaDegR = (180/PI) * atan(getSlope(pt7.x, pt7.y, pt8.x, pt8.y));

//            if((thetaDegL > -2*thetaLane && thetaDegL < -thetaLane) ){
                std::cout<<"thetaL "<<roiNo << " " << thetaDegL<<"   TRACK \n";
                line(roi, pt1, pt2, cv::Scalar(255), 5);
                customLine(roiBin, pt3, pt4, lineLength, startY, color);
//            }

//            if((thetaDegR > thetaLane && thetaDegR < 2*thetaLane  ) ) {
                std::cout<<"thetaR "<<roiNo << " " << thetaDegR<<"   TRACK \n";
                line(roi, pt5, pt6, cv::Scalar(255), 5);
                customLine(roiBin, pt7, pt8, lineLength, startY, color);
//            }
        }
        /*else{
            std::vector<cv::Vec4i>::const_iterator iterator= linesP.begin();
            while (iterator!=linesP.end()) {
                cv::Point pt1((*iterator)[0], (*iterator)[1]);
                cv::Point pt2((*iterator)[2], (*iterator)[3]);
                cv::Point pt3((*iterator)[0], (*iterator)[1] + imgShift);
                cv::Point pt4((*iterator)[2], (*iterator)[3] + imgShift);
                double thetaDeg = (180/PI) * atan(getSlope(pt3.x, pt3.y, pt4.x, pt4.y));
                if((thetaDeg > thetaLane || thetaDeg < -thetaLane ) ){
                    line(roi, pt1, pt2, cv::Scalar(255), 5);
                    customLine(roiBin, pt3, pt4, lineLength, startY, color);
                }
                iterator++;
            }
            return linesP;
        }*/
    }

    void copyVec(std::vector<cv::Vec4i> &vec1, std::vector<cv::Vec4i> &vec2){
        vec2.clear();
        std::vector<cv::Vec4i>::const_iterator iterator= vec1.begin();
        while(iterator != vec1.end()){
            vec2.push_back(*iterator);
            iterator++;
        }
//        printf("%d %d %d %d\n", roiTrack[0].size(), roiTrack[1].size(), roiTrack[2].size(), roiTrack[3].size());
    }

    std::vector<cv::Vec4i> averageLine(int midX){
        cv::Point leftUp(0,0);
        cv::Point leftDown(0,0);
        cv::Point rightUp(0,0);
        cv::Point rightDown(0,0);
        int sizeLeft, sizeRight;
        sizeLeft = 0;
        sizeRight = 0;
        std::vector<cv::Vec4i> averageLines;

        if(linesP.size() < 4){
            ;
        }
        else{
            std::vector<cv::Vec4i>::const_iterator iterator= linesP.begin();
            while(iterator != linesP.end()){
                cv::Point pt1((*iterator)[0], (*iterator)[1]);
                cv::Point pt2((*iterator)[2], (*iterator)[3]);

                //Left side or right side. Takes care of lines that are in middle.
                if(pt1.x < midX && pt2.x < midX){
                    if(pt1.y > pt2.y){
                        leftUp.x += pt2.x;
                        leftUp.y += pt2.y;

                        leftDown.x += pt1.x;
                        leftDown.y += pt1.y;
                    }
                    else{
                        leftUp.x += pt1.x;
                        leftUp.y += pt1.y;

                        leftDown.x += pt2.x;
                        leftDown.y += pt2.y;
                    }
                    sizeLeft++;
                }
                else if(pt1.x > midX && pt2.x > midX){
                    if(pt1.y > pt2.y){
                        rightUp.x += pt2.x;
                        rightUp.y += pt2.y;

                        rightDown.x += pt1.x;
                        rightDown.y += pt1.y;
                    }
                    else{
                        rightUp.x += pt1.x;
                        rightUp.y += pt1.y;

                        rightDown.x += pt2.x;
                        rightDown.y += pt2.y;
                    }
                    sizeRight++;
                }
                iterator++;
            }
            if(sizeLeft > 0){
//                std::cout<<"SL "<<sizeLeft <<"\n";
                leftUp.x /= sizeLeft;
                leftUp.y /= sizeLeft;
                leftDown.x /= sizeLeft;
                leftDown.y /= sizeLeft;
            }

            if(sizeRight > 0){
//                std::cout<<"SR " << sizeRight<<"\n";
                rightUp.x /= sizeRight;
                rightUp.y /= sizeRight;
                rightDown.x /= sizeRight;
                rightDown.y /= sizeRight;
            }

            averageLines.push_back(cv::Vec4i(leftUp.x, leftUp.y, leftDown.x, leftDown.y));
            averageLines.push_back(cv::Vec4i(rightUp.x, rightUp.y, rightDown.x, rightDown.y));
        }
        return averageLines;
    }

    std::vector<cv::Vec4i> combineLines(){
        std::vector<cv::Vec4i> combinedLines;
        if(roiTrack[0].size() > 0 && roiTrack[1].size() > 0 && roiTrack[2].size() > 0){
            combinedLines.push_back(roiTrack[2].at(0));
            combinedLines.push_back(cv::Vec4i(roiTrack[2].at(0)[2], roiTrack[2].at(0)[3], roiTrack[1].at(0)[2], roiTrack[1].at(0)[3]));
            combinedLines.push_back(cv::Vec4i(roiTrack[1].at(0)[2], roiTrack[1].at(0)[3], roiTrack[0].at(0)[2], roiTrack[0].at(0)[3]));
//            getBezier(curveImg, roiTrack[2].at(0)[0], roiTrack[2].at(0)[1], roiTrack[1].at(0)[2], roiTrack[1].at(0)[3], roiTrack[0].at(0)[2], roiTrack[0].at(0)[3]);
            if(roiTrack[0].size() > 1 && roiTrack[1].size() > 1 && roiTrack[2].size() > 1){
                combinedLines.push_back(roiTrack[2].at(1));
                combinedLines.push_back(cv::Vec4i(roiTrack[2].at(1)[2], roiTrack[2].at(1)[3], roiTrack[1].at(1)[2], roiTrack[1].at(1)[3]));
                combinedLines.push_back(cv::Vec4i(roiTrack[1].at(1)[2], roiTrack[1].at(1)[3], roiTrack[0].at(1)[2], roiTrack[0].at(1)[3]));
            }
        }
        return combinedLines;
    }

    void displayLines(cv::Mat &img, std::vector<cv::Vec4i> lines, cv::Scalar color = cv::Scalar(255,255,255)){
        std::vector<cv::Vec4i>::const_iterator iterator= lines.begin();
        while (iterator!=lines.end()) {
            cv::Point pt1((*iterator)[0], (*iterator)[1]);
            cv::Point pt2((*iterator)[2], (*iterator)[3]);
            line(img, pt1, pt2, color, 5);
            iterator++;
        }
    }

    double getSlope(int x0, int y0, int x1, int y1){
        return (double)(y1-y0)/(x1-x0);
    }

    cv::Vec4i customLine(cv::Mat &img, cv::Point a, cv::Point b,int distInY, int startY, cv::Scalar color = cv::Scalar(255)){
        double slope = getSlope(a.x, a.y, b.x, b.y);

        cv::Point p(0, startY), q(img.cols, startY + distInY);

        p.x = -(a.y - p.y) / slope + a.x;
        q.x = -(b.y - q.y) / slope + b.x;

        line(img,p,q,color,5);
        return cv::Vec4i(p.x, p.y, q.x, q.y);
    }

    void genBinImg(cv::Mat &img, cv::Point upLeft, cv::Point downLeft, cv::Point upRight, cv::Point downRight){
        cv::Point points[4];
        int i = 0;
        int npt = 4;

        points[0] = upLeft;
        points[1] = upRight;
        points[2] = downRight;
        points[3] = downLeft;

        const cv::Point *ppt = points;
        cv::fillConvexPoly(img, ppt, npt, cv::Scalar(255), 8);
    }

    int getPt( int n1 , int n2 , float perc )
    {
        int diff = n2 - n1;

        return n1 + ( diff * perc );
    }

    void getBezier(cv::Mat img, int x1, int y1, int x2, int y2, int x3, int y3){
        int xa, xb, ya, yb, x, y;
        for( float i = 0 ; i < 1 ; i += 0.01 )
        {
            // The Green Line
            xa = getPt( x1 , x2 , i );
            ya = getPt( y1 , y2 , i );
            xb = getPt( x2 , x3 , i );
            yb = getPt( y2 , y3 , i );

            // The Black Dot
            x = getPt( xa , xb , i );
            y = getPt( ya , yb , i );

            img.at<uchar>(x, y, 2) = 255;
        }
    }
};


#endif //LANEDETECTOR_LANEDETECTOR_H
