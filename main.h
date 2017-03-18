#ifndef MAIN_H
#define MAIN_H

#define PI 3.1415926

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

void traverse(cv::Mat &img){
    for(int i = img.rows - 1; i > -1; i--){
        for(int j = img.cols -1; j > -1; j--){
            if((i+j) % 2 == 0)
                img.at<uchar>(i, j, 0) = 0;
        }
    }
}

/*
 * Edge linking by:
 *  1. Starting point scan
 *  2. Edge tracing
 * Parameters to consider:
 *  1. Orientation
 *  2. Length
 *  3. Width
 *  */
void edgeLinking(cv::Mat &img){

    //get starting point
    int i = 0;
    for(int j = img.cols; j > -1; j--){
        for(int i = img.rows; i > -1; i--){
            if((int)img.at<uchar>(i,j, 0) == 255){
                //traceEdge(img, i, j);
            }
        }
    }
}

void traceEdge(cv::Mat img, int x, int y){

}

class LaneDetection {

private:
    // lines for Hough
    std::vector<cv::Vec2f> lines;

    // lines for Probabilistic Hough
    std::vector<cv::Vec4i> linesP;
    std::vector<cv::Vec4i> twoLines;

    // Initial vote for hough transform
    int houghVote = 200;

    // Accumulator dimensions for probabilistic hough
    double deltaRho;
    double deltaTheta;

    // minimum number of votes that a line
    // must receive before being considered
    int minVote;

    // min length for a line
    double minLength;

    // max allowed gap along the line
    double maxGap;

    // distance to shift the drawn lines down when using a ROI
    int shift;

public:
    LaneDetection() : deltaRho(1), deltaTheta(PI/180), minVote(10), minLength(0.), maxGap(0.), shift(0) {}

    void setAccResolution(double dRho, double dTheta) {
        deltaRho= dRho;
        deltaTheta= dTheta;
    }

    void setMinVote(int minv) {
        minVote= minv;
    }

    void setLineLengthAndGap(double length, double gap) {
        minLength= length;
        maxGap= gap;
    }

    void setShift(int imgShift) {
        shift = imgShift;
    }

    void setMinLength(double minLength) {
        LaneDetection::minLength = minLength;
    }

    /*Calculate hough lines from edge image, add them to image(hough) which is
     * a copy of the original grayscale image with the dimensions of the ROI.
     * Also add to image(houghBin) which has dimensions of original image and
     * eventually turns out to be a binary image with hough lines.
     * */
    void getHoughLines(cv::Mat &hough, cv::Mat &houghBin, cv::Mat &contours){
        if (houghVote < 1) {
            houghVote = 200;
        } else{ houghVote += 25;}
        // loop to get the most prominent lines
        while (lines.size() < 4 and houghVote > 0){
            HoughLines(contours,lines,1,PI/180, houghVote);
            houghVote -= 5;
        }
        std::vector<cv::Vec2f>::const_iterator iterator= lines.begin();
        while (iterator!=lines.end()) {
            float rho = (*iterator)[0];
            float theta = (*iterator)[1];
            float thetaDeg = theta * 180/PI;
            if ((thetaDeg > 115.0) || (thetaDeg < 65.0)) {
                std::cout << " theta1 = " << theta * 180/PI<< "\n";
                double a = cos(theta), b = sin(theta);
                double x0 = a*rho, y0 = b*rho;
                cv::Point pt1, pt2, pt3, pt4;
                pt1.x = cvRound(x0 + 1000*(-b));
                pt1.y = cvRound(y0 + 1000*(a));
                pt2.x = cvRound(x0 - 1000*(-b));
                pt2.y = cvRound(y0 - 1000*(a));

                pt3.x = cvRound(x0 + 1000*(-b));
                pt3.y = cvRound(y0 + 1000*(a) + shift);
                pt4.x = cvRound(x0 - 1000*(-b));
                pt4.y = cvRound(y0 - 1000*(a) + shift);
//                cv::Point pt1(rho / cos(theta), 0);
//                cv::Point pt2((rho - hough.rows * sin(theta)) / cos(theta), hough.rows);
//                cv::Point pt3(2* (rho / cos(theta)), shift);
//                cv::Point pt4(0, houghBin.rows);
                line(hough, pt1, pt2, cv::Scalar(255), 5);
                customLine(houghBin, pt3, pt4, houghBin.rows/9);
            }
            iterator++;
        }
    }

    /*Calculate hough lines from edge image, add them to image(hough) which is
     * a copy of the original grayscale image with the dimensions of the ROI.
     * Also add to image(houghBin) which has dimensions of original image and
     * eventually turns out to be a binary image with hough lines.
     * */
    void getHoughPLines(cv::Mat &houghP, cv::Mat &houghPBin, cv::Mat &contours, float theta){
        linesP.clear();
        twoLines.clear();
        cv::HoughLinesP(contours,linesP,deltaRho,deltaTheta,minVote, minLength, maxGap);
        std::vector<cv::Vec4i>::const_iterator iterator= linesP.begin();
        while (iterator!=linesP.end()) {

            cv::Point pt1((*iterator)[0], (*iterator)[1]);
            cv::Point pt2((*iterator)[2], (*iterator)[3]);
            cv::Point pt3((*iterator)[0], (*iterator)[1] + shift);
            cv::Point pt4((*iterator)[2], (*iterator)[3] + shift);
            double thetaDeg = (180/PI) * atan(getSlope(pt3.x, pt3.y, pt4.x, pt4.y));
            if((thetaDeg > theta || thetaDeg < -theta ) ){
                twoLines.push_back(*iterator);
                std::cout << "theta2: " << thetaDeg << "\n";
                line(houghP, pt1, pt2, cv::Scalar(255), 5);
                customLine(houghPBin, pt3, pt4, houghPBin.rows/8, cv::Scalar(255));
            }
            iterator++;
        }
    }

    void genBinImg(cv::Mat &img){
        cv::Point points[4];
        int i = 0;
        int npt[] = {4};
        if(twoLines.size() > 1) {
            std::vector<cv::Vec4i>::const_iterator iterator = twoLines.begin();
            for (int j = 0; j < 2; j++) {
                points[i].x = cvRound((*iterator)[0]);
                points[i].y = cvRound((*iterator)[1] + shift);
                i++;
                points[i].x = cvRound((*iterator)[2]);
                points[i].y = cvRound((*iterator)[3] + shift);
                i++;
                iterator++;
            }
            const cv::Point *ppt[1] = {points};
            cv::fillPoly(img, ppt, npt, 1, cv::Scalar(255, 255, 255), 8);
        }
    }

    // Optional TODOs: implement predictor - kalaman tracker.Do Inverse Projection mapping.

    double getSlope(int x0, int y0, int x1, int y1){
        return (double)(y1-y0)/(x1-x0);
    }

    /*Draws a custom length line on img from points a and b.
     * */
    void customLine(cv::Mat &img, cv::Point a, cv::Point b,int distInY = 0, cv::Scalar color = cv::Scalar(255)){
        double slope = getSlope(a.x, a.y, b.x, b.y);

        cv::Point p(0,img.rows - distInY - (7* (img.rows/8) - shift)), q(img.cols, img.rows - (7* (img.rows/8) - shift));

        p.x = -(a.y - p.y) / slope + a.x;
        q.x = -(b.y - q.y) / slope + b.x;

        line(img,p,q,color,5);
    }

//    cv::Mat getSpline(cv::Mat &img, std::vector<cv::Vec4i> lines, int binSize)
//    {
//        cv::Mat splineImg(img);
//        std::vector<vec3> points;
//        std::vector<cv::Vec4i>::const_iterator it2= lines.begin();
//
//        std::cout << "GOT HERE00000";
//        while (it2!=lines.end()) {
//            vec3 p1((*it2)[0], (*it2)[1]+shift, 0);
//            vec3 p2((*it2)[2], (*it2)[3]+shift, 0);
//
//            points.push_back(p1);
//            points.push_back(p2);
//            ++it2;
//        }
//
//        std::cout << "GOT HERE11111";
//        //separate points into bins with y
//        std::vector<vec3> bins[img.rows/binSize + 1];
//        vec3 point;
//        std::vector<vec3>::const_iterator it3= points.begin();
//        while(it3!=points.end())
//        {
//            point = vec3((*it3).x, (*it3).y, (*it3).z);
//            int bin = (int) (point.y / binSize);
//            bins[bin].push_back(point);
//            ++it3;
//        }
//        for(int i = 0; i < (img.rows/binSize + 1); i++)
//        {
//            if(!bins[i].empty())
//            {
//                std::sort(bins[i].begin(), bins[i].end(), mycomp);
////                bins[i][0].y = 0;
////                bins[i][bins[i].size() - 1].y = img.rows;
//                if(spline) {delete spline;}
//                spline = new CRSpline();
//                for(int j = 0; j< bins[i].size(); j++)
//                {
//                    spline -> AddSplinePoint(bins[i][j]);
//                    std::cout << "x: " << bins[i][j].x << "   y: " << bins[i][j].y << "\n";
//                }
//                std::cout << "\n\n\n";
//                vec3 rv_last(0,0,0);
//                for(int j = 0; j < img.cols; j++)
//                {
//                    float t= solveForX(j);
//                    vec3 rv = spline->GetInterpolatedSplinePoint(t);
//                    if(i>1)
//                    {
//                        cv::line(splineImg,cv::Point(rv.x,rv.y),cv::Point(rv_last.x,rv_last.y),cv::Scalar(0,0,255),5);
//                    }
//                    rv_last=rv;
//                }
//            }
//        }
//        return splineImg;
//    }
};

#endif