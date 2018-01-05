/* Header file that declares all methods used to compute the essential matrix
Author: Sasidharan Mahalingam
Date Created: 18th December 2017 */


#ifndef EPIPOLES_H
#define EPIPOLES_H

#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <math.h>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/stitching.hpp"

using namespace cv;
using namespace cv::xfeatures2d;


using namespace cv;
using namespace std;

typedef std::vector<std::string> stringvec;

//function that caluculates corresponding matching points in the given images
void compute_epipoles(stringvec &);

//function that draws lines when points and the corresponding lines are given
void draw_lines(Mat, Mat, std::vector<Point3f>, std::vector<Point2f>, std::vector<Point2f>, std::vector<uchar>);

//function that draws the conjugate epipolar lines
template <typename T>
static void drawEpipolarLines(const std::string& title, const cv::Mat F,
                const cv::Mat& img1, const cv::Mat& img2,
                const std::vector<cv::Point_<T>> points1,
                const std::vector<cv::Point_<T>> points2,
                const float inlierDistance = -1);

//function that calculates the distance of a point from a given line
template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line);

//function that projects the rotated and translated points of one image onto another image
template <typename T>
static void projectgivenPoints(const std::string& title, const cv::Mat R, const cv::Mat F, const cv::Mat E, const cv::Mat t,
                const cv::Mat& img1, const cv::Mat& img2,
                const std::vector<cv::Point_<T>> points1,
                const std::vector<cv::Point_<T>> points2,
                const float inlierDistance);

void calculateRT(const Mat& E, Mat& R, Mat& t);

//function that captures mouse pointer to select corresponding points in both the images
void CallBackFunction(int event, int x, int y, int flags, void * userdata);

//function that gets corresponding points in the left and right immage
void getPoints(std::vector<Point2f>& points1, std::vector<Point2f>& points2, Mat left_img, Mat right_img);

#endif /* end of include guard:  */
