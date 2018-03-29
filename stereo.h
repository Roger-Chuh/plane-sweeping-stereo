/*Header file that contains functions used to find stereo matching
Author: Sasidharan Mahalingam
Date Created: March 10 2018 */

#ifndef STEREO_H
#define STEREO_H
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
#include <Eigen/Dense>
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/stitching.hpp"

using namespace cv;
using namespace std;
using namespace Eigen;
typedef std::vector<std::string> stringvec;

void find_alpha_gamma(const std::vector<Mat>, std::vector<Mat>, vector<Mat> &, vector<double> &);

void find_plane_sets(const Mat, const Mat, std::vector<Mat> &);

void find_homographies(const std::vector<Mat> P1, const std::vector<Mat> P2, std::vector<Mat> & H);

void find_depth(stringvec &v, std::vector<Mat> , std::vector<Mat> , std::vector<Mat> &);

#endif
