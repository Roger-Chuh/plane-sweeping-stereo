/*cpp file that has the definitions of the funstion that calculate the essential matrix linking the given images
Author: Sasidharan Mahalingam
Date Created: 18th December 2017 */

#include "epipoles.h"




/*
Function that computes and draws the epipolar lines in two images associated to each other by a fundamental matrix

\param title     Title of the window to display
\param F         Fundamental matrix
\param img1      First image
\param img2      Second image
\param points1   Set of points in the first image
\param points2   Set of points in the second image matching to the first set
\param inlierDistance      Points with a high distance to the epipolar lines are not displayed. If it is negative, all points are displayed
*/
template <typename T>
static void drawEpipolarLines(const std::string& title, const cv::Mat F,
                const cv::Mat& img1, const cv::Mat& img2,
                const std::vector<cv::Point_<T>> points1,
                const std::vector<cv::Point_<T>> points2,
                const float inlierDistance)
{
  CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
  cv::Mat outImg(img1.rows, img1.cols*2, CV_8UC3);
  cv::Rect rect1(0,0, img1.cols, img1.rows);
  cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
  /*
   * Allow color drawing
   */
  if (img1.type() == CV_8U)
  {
    cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
    cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
  }
  else
  {
    img1.copyTo(outImg(rect1));
    img2.copyTo(outImg(rect2));
  }
  std::vector<cv::Vec<T,3>> epilines1, epilines2;
  cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
  cv::computeCorrespondEpilines(points2, 2, F, epilines2);

  CV_Assert(points1.size() == points2.size() &&
        points2.size() == epilines1.size() &&
        epilines1.size() == epilines2.size());

  cv::RNG rng(0);
  int flag = 0;
  for(size_t i=0; i<points1.size(); i++)
  {
    flag = 0;
    if(inlierDistance > 0)
    {
      if(distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
        distancePointLine(points2[i], epilines1[i]) > inlierDistance)
      {
        //The point match is no inlier
        //continue;
        flag = 1;
      }
    }
    /*
     * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
     */
     if(!flag)
     {
        cv::Scalar color(rng(256),rng(256),rng(256));
        cv::Scalar p_color(0, 256, 256);
        cv::line(outImg(rect2),
        cv::Point(0,-epilines1[i][2]/epilines1[i][1]),
        cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]),
        color);
        cv::circle(outImg(rect1), points1[i], 8, p_color, -1, CV_AA);

        cv::line(outImg(rect1),
        cv::Point(0,-epilines2[i][2]/epilines2[i][1]),
        cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),
        color);
        cv::circle(outImg(rect2), points2[i], 8, p_color, -1, CV_AA);
    }
    else
    {
      cv::Scalar p_color(0, 0, 256);
      cv::circle(outImg(rect1), points1[i], 8, p_color, -1, CV_AA);
      cv::circle(outImg(rect2), points2[i], 8, p_color, -1, CV_AA);
    }

  }
  namedWindow(title,  WINDOW_NORMAL);
  resizeWindow(title, 3000, 1500);
  cv::imshow(title, outImg);
  cv::waitKey(0);
  destroyAllWindows();
}

template <typename T>
static float distancePointLine(const cv::Point_<T> point, const cv::Vec<T,3>& line)
{
  //Line is given as a*x + b*y + c = 0
  return fabsf(line(0)*point.x + line(1)*point.y + line(2))
      / std::sqrt(line(0)*line(0)+line(1)*line(1));
}


/*
Function that computes and projects the points on the left image into the right image

\param title     Title of the window to display
\param R         Rotation matrix
\param t         Translation vector
\param img1      First image
\param img2      Second image
\param points1   Set of points in the first image
\param points2   Set of points in the second image matching to the first set
\param inlierDistance      Points with a high distance to the epipolar lines are not displayed. If it is negative, all points are displayed
*/
template <typename T>
static void projectgivenPoints(const std::string& title, const cv::Mat R, const cv::Mat F, const cv::Mat E, const cv::Mat t,
                const cv::Mat& img1, const cv::Mat& img2,
                const std::vector<cv::Point_<T>> points1,
                const std::vector<cv::Point_<T>> points2,
                const float inlierDistance)
{
  CV_Assert(img1.size() == img2.size() && img1.type() == img2.type());
  cv::Mat outImg(img1.rows, img1.cols*2, CV_8UC3);
  cv::Rect rect1(0,0, img1.cols, img1.rows);
  cv::Rect rect2(img1.cols, 0, img1.cols, img1.rows);
  /*
   * Allow color drawing
   */
  if (img1.type() == CV_8U)
  {
    cv::cvtColor(img1, outImg(rect1), CV_GRAY2BGR);
    cv::cvtColor(img2, outImg(rect2), CV_GRAY2BGR);
  }
  else
  {
    img1.copyTo(outImg(rect1));
    img2.copyTo(outImg(rect2));
  }
  std::vector<cv::Vec<T,3>> epilines1, epilines2;
  cv::computeCorrespondEpilines(points1, 1, F, epilines1); //Index starts with 1
  cv::computeCorrespondEpilines(points2, 2, F, epilines2);

  CV_Assert(points1.size() == points2.size() &&
        points2.size() == epilines1.size() &&
        epilines1.size() == epilines2.size());

  cv::RNG rng(0);
  int flag = 0;
  std::vector<Point3f> h_points;
  std::vector<Point2f> projected_points1;
  //Converting the set of corresponding points in the left image to homogenous co-ordinates
  convertPointsToHomogeneous(points1, h_points);
  std::vector<Point2f> transf_points1;
  //Getting the intrinsic camera matrix calculated during camera calibration
  string intr_param_filename = "camera_params.xml";
  Mat intrinsic, Rvec = Mat::zeros(3,1,CV_64FC1), dist = Mat::zeros(1,5,CV_64FC1);
  FileStorage intr_fs(intr_param_filename, FileStorage::READ);
  intr_fs["camera_matrix"] >> intrinsic;
  double f = intrinsic.at<double>(0,0);
  intr_fs.release();
  //Converting the rotation matrix into a rotation vector
  Rodrigues(R, Rvec);
  cout << "Rotation vector is:\n" << Rvec << endl;
  projectPoints(h_points, Rvec, t, intrinsic, dist, projected_points1);
  cout << "Points are:\n" << points2 << endl;
  cout << "Projected points are:\n" << projected_points1 << endl;
  for(size_t i=0; i<points1.size(); i++)
  {
    flag = 0;
    if(inlierDistance > 0)
    {
      if(distancePointLine(points1[i], epilines2[i]) > inlierDistance ||
        distancePointLine(points2[i], epilines1[i]) > inlierDistance)
      {
        //The point match is no inlier
        //continue;
        flag = 1;
      }
    }
    /*
     * Epipolar lines of the 1st point set are drawn in the 2nd image and vice-versa
     */
     if(!flag)
     {
        cv::Scalar color(rng(256),rng(256),rng(256));
        cv::Scalar p_color(0, 255, 255);
        cv::Scalar color2(255, 0, 0);
        /*cv::line(outImg(rect2),
        cv::Point(0,-epilines1[i][2]/epilines1[i][1]),
        cv::Point(img1.cols,-(epilines1[i][2]+epilines1[i][0]*img1.cols)/epilines1[i][1]),
        color);*/
        cv::circle(outImg(rect1), points1[i], 8, p_color, -1, CV_AA);

        /*
        cv::line(outImg(rect1),
        cv::Point(0,-epilines2[i][2]/epilines2[i][1]),
        cv::Point(img2.cols,-(epilines2[i][2]+epilines2[i][0]*img2.cols)/epilines2[i][1]),
        color);*/
        cv::circle(outImg(rect2), points2[i], 8, p_color, -1, CV_AA);
        //Projecting the rotated and translated points in the first image onti the second image
        //Point2f center_pt;
        //Mat eucl_point, tmper, h_pt = Mat::zeros(3,1,CV_64FC1);
        //h_pt.at<double>(0,0) = h_points[i].x;
        //h_pt.at<double>(1,0) = h_points[i].y;
        //h_pt.at<double>(2,0) = h_points[i].z;
        //transpose(h_points[i], h_pt);
        //cout << "Homogenous point is :\n" << h_pt << endl;
        //cout << "Rotation Matrix is:\n" << R << endl;
        //cout << "Traslation vector is:\n" << t << endl;

        //convertPointsFromHomogeneous(tmper, eucl_point);
        //cout << "Rototranslated eucliedian point is:\n" << eucl_point << endl;
        //tmper = Rcalc * h_pt + tcalc;
        //cout << "Calculated homogenous point is:\n" << tmper << endl;
        //convertPointsFromHomogeneous(tmper, eucl_point);
        //cout << "Calculated eucliedian point is:\n" << eucl_point << endl;
        //center_pt.x = eucl_point.at<double>(0,0);
        //center_pt.y = eucl_point.at<double>(1,0);
        cout << projected_points1[i] << endl;
        cv::circle(outImg(rect2), projected_points1[i], 8, color2, -1, CV_AA);
    }
    else
    {
      cv::Scalar p_color(0, 0, 255);
      cv::circle(outImg(rect1), points1[i], 8, p_color, -1, CV_AA);
      cv::circle(outImg(rect2), points2[i], 8, p_color, -1, CV_AA);
    }

  }
  namedWindow(title,  WINDOW_NORMAL);
  resizeWindow(title, 3000, 1500);
  cv::imshow(title, outImg);
  cv::waitKey(0);
  destroyAllWindows();
}


/* Function that draws the epipolar line on img1, for points on img2 using the calculated fundamental matrix
Usage:
Mat img1: Mat object that stores image on which the epipolar line has to be drawn
Mat img2: Mat object that stores the image that has the points for which the epipoles has to be drawn
std::vector<Point2f> lines: Vector of
Returns:
does not return anything
*/

void draw_lines(Mat img1, Mat img2, std::vector<Point3f> lines, std::vector<Point2f> pts1, std::vector<Point2f> pts2, std::vector<uchar> mask)
{
  unsigned int r,c;
  r = img1.rows;
  c = img1.cols;
  cvtColor(img1, img1, CV_GRAY2BGR);
  cvtColor(img2, img2, CV_GRAY2BGR);
  cv::Scalar color;
  /*std::vector<Point3f>::const_iterator i = lines.begin();
  std::vector<Point2f>::const_iterator j = pts1.begin();
  std::vector<Point2f>::const_iterator k = pts2.begin();
  std::vector<Point1d>::const_iterator l = mask.begin();*/
  unsigned long int sz = lines.size();
  for( int itr = 0; itr < sz; itr++)
  {
    if( mask[itr] == 0)
    {
      color = (255, 255, 255);
    }
    else
    {
      color = (255, 255, 0);
    }
    cv::line(img1, cv::Point(0, -(lines[itr].z / lines[itr].y)), cv::Point(c, -((lines[itr].z + lines[itr].x * c) / lines[itr].y)), color);
    cv::circle(img1, pts1[itr], 3, color);
    cv::circle(img2, pts2[itr], 3, color);
  }
  namedWindow("Image 1", CV_WINDOW_FREERATIO | CV_GUI_NORMAL);
  resizeWindow("Image 1", 500, 600);
  imshow( "Image 1", img1 );
  namedWindow("Image 2", CV_WINDOW_FREERATIO | CV_GUI_NORMAL);
  resizeWindow("Image 2", 500, 600);
  imshow( "Image 2", img2 );
  waitKey(0);
  destroyAllWindows();
  cout << "a\n";
}



void calculateRT(const Mat& E, Mat& R, Mat& t)
{
  Mat e, w, Wt, u, vt, W = Mat::zeros(3,3,CV_64FC1);
  e = -E;
  SVDecomp(e, w, u, vt);
  W.at<double>(0,1) = -1;
  W.at<double>(1,0) = 1;
  W.at<double>(2,2) = -1;
  transpose(W, Wt);
  R = u * Wt * vt;
  t = u.col(2);
}


/*Function that captures the mouse pointer
Usage:
Parameters:
int event: denotes the type of event (left click, right click, middle click, etc.)
int x: x co-ordinate of the mouse position
int y: y co-ordinate of the mouse position
Returns:
does not return anything */
void CallBackFunction(int event, int x, int y, int flags, void * param)
{
  std::vector<Point2f> * userdata = (std::vector<Point2f> *) param;
  Point2f tmp;
  if  ( event == EVENT_LBUTTONDOWN )
  {
       cout << "Left button of the mouse is clicked - position captured (" << x << ", " << y << ")" << endl;
       tmp.x = x;
       tmp.y = y;
       userdata->push_back(tmp);
  }
}


/*Function that implements the process of selecting corresponding points inn the left and right image
Usage:
Parameters
std::vector<Point2f>& points1: Vector that holds the points selected in the left image (passed by reference)
std::vector<Point2f>& points2: Vector that holds the points selected in the right image (passed by reference)
Mat left_img: Mat object that stores the left image
Mat right_img: Mat object that stores the right image
Returns:
does not return anything*/

void getPoints(std::vector<Point2f>& points1, std::vector<Point2f>& points2, Mat left_img, Mat right_img)
{
  cout << "Select corresponding points on the left and right image.\n" << "Press Enter or Space key when done.\n";
  while(1)
  {
      cv::Scalar p_color(0, 255, 255);
      char ch;
      cout << "Select point of left image.\n";

      namedWindow("left image", WINDOW_NORMAL);
      resizeWindow("left image", 1000, 1000);
      for (auto & point: points1)
      {
        cv::circle(left_img, point, 8, p_color, -1, CV_AA);
      }
      setMouseCallback("left image", CallBackFunction, & points1);
      imshow("left image", left_img);
      waitKey(0);
      cout << "Selected point on left image.\n" << "Now select corresponding point on right image";
      destroyAllWindows();
      namedWindow("right image", WINDOW_NORMAL);
      resizeWindow("right image", 1000, 1000);
      moveWindow("right image", 1100,0);
      for (auto & point: points2)
      {
        cv::circle(right_img, point, 8, p_color, -1, CV_AA);
      }
      setMouseCallback("right image", CallBackFunction, & points2);
      imshow("right image", right_img);
      waitKey(0);
      cout << "Selected point on right image.\n";
      destroyAllWindows();
      cout << "Select one more set of corresponding points ? <y/n>:";
      cin >> ch;
      if( ch == 'n')
        break;
  }
  cout << "Selected left points are:\n" << points1 <<"Selected right points are:\n" << points2;
}



/*Function that finding matching points between the two goven images
Usage:
Parameters:
vector<string> v: a vector of strings that holds the paths of the images to be used to compute the essential matrix
Returns:
does not return anything */

void compute_epipoles(stringvec &v )
{
  if(v.size() <2)
  {
    cout << "Folder empty";
    return;
  }
  vector<string>::const_iterator i = v.begin();
  vector<string>::const_iterator j = i + 1;
  //creating and opening a xml file to store fundamental matrix values
  //string essential_mat_file = "essential_matrix.xml";
  string fundamental_mat_file = "fundamental_matrix.xml";
  string extrinsic_params_file = "extrinsic_parameters.xml";
  FileStorage fs(fundamental_mat_file, FileStorage::WRITE);
  FileStorage fs_extr(extrinsic_params_file, FileStorage::WRITE);
  vector<Mat> F_list;
  unsigned int sz = v.size();
  unsigned int no_of_images = 3, k = 1;
  Mat K, Kt;
  FileStorage fs1("camera_params.xml", FileStorage::READ);
  fs1["camera_matrix"] >> K;
  fs1.release();
  cout << "The intrinsic matrix is:\n" << K << endl;
  transpose(K, Kt);
  cout << "The transpose pf the intrinsic matrix is:\n" << Kt << endl;
  for(; i != v.end(); i++, j = (++j), k++)
  {
    Mat img_1, img_2;
    if (k == 3)
    {
      //Since in the last iterations the left and right images are reversed img_object should always point to the left image
      //img_1 = imread( *(v.begin()), CV_LOAD_IMAGE_GRAYSCALE );
      img_1 = imread( *(v.begin()));
      //img_2 = imread( *i, CV_LOAD_IMAGE_GRAYSCALE );
      img_2 = imread( *i);
    }
    else
    {
      //img_1 = imread( *i, CV_LOAD_IMAGE_GRAYSCALE );
      //img_2 = imread( *j, CV_LOAD_IMAGE_GRAYSCALE );
      img_1 = imread( *i );
      img_2 = imread( *j );
    }

    //checking if both the images are readable
    if( !img_1.data || !img_2.data )
    {
      std::cout<< " --(!) Error reading images " << std::endl;
      return;
    }

    //commenting to implement manual selection of points
    /*
    cout << "Starting to find similar features in the two images using SURF detector...\n";
    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 400;
    Ptr<SURF> detector = SURF::create();
    detector->setHessianThreshold(minHessian);
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    detector->detectAndCompute( img_1, Mat(), keypoints_1, descriptors_1 );
    detector->detectAndCompute( img_2, Mat(), keypoints_2, descriptors_2 );
    //-- Step 2: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    matcher.match( descriptors_1, descriptors_2, matches );
    double max_dist = 0; double min_dist = 100;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_1.rows; i++ )
    { double dist = matches[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
    }
    cout << "-- Max dist : " << max_dist << endl;
    cout << "-- Min dist : " << min_dist << endl;
    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< DMatch > good_matches;
    cout << "Selecting good matches...\n";
    for( int i = 0; i < descriptors_1.rows; i++ )
    { if( matches[i].distance <= max(2*min_dist, 0.02) )
      { good_matches.push_back( matches[i]); }
    }
    //-- Draw only "good" matches
    Mat img_matches;
    drawMatches( img_1, keypoints_1, img_2, keypoints_2,
                 good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                 vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    //for( int i = 0; i < (int)good_matches.size(); i++ )
    //{
    //  cout << "-- Good Match [" <<  i << "] Keypoint 1: " <<  good_matches[i].queryIdx << "-- Keypoint 2: " << good_matches[i].trainIdx << "\n";
    //}


    cout << "Found matches. Displaying the selected points...\n";
    namedWindow("Good Matches", WINDOW_NORMAL);
    resizeWindow("Good Matches", 3000, 1500);
    imshow( "Good Matches", img_matches );
    waitKey(0);
    destroyAllWindows();
    //-- Localize the object
    std::vector<Point2f> points1;
    std::vector<Point2f> points2;
    for( size_t i = 0; i < good_matches.size(); i++ )
    {
      //-- Get the keypoints from the good matches
      points1.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
      points2.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
    }


    */

    std::vector<Point2f> points1;
    std::vector<Point2f> points2;
    getPoints(points1, points2, img_1, img_2);
    Mat F, E;
    std::vector<uchar> mask(points1.size());
    cout << points1.size() << '\n';
    cout << "Calculating Fundamental Matrix...\n";
    F = findFundamentalMat(points1, points2, RANSAC, 3, 0.99);
    F_list.push_back(F);
    string F_mat_name = "F" + to_string(k);
    //cout << H_mat_name << endl;
    fs << F_mat_name << F;
    cout << "Calculated and stored the fundamental matrix. The fundamental matrix is:\n" << F << endl;

    cout << "Calculating the Essential matrix...\n";
    string intr_param_filename = "camera_params.xml";
    Mat intrinsic;
    FileStorage intr_fs(intr_param_filename, FileStorage::READ);
    intr_fs["camera_matrix"] >> intrinsic;
    double f = intrinsic.at<double>(0,0);
    E = findEssentialMat(points1, points2, f, Point2d(0, 0), RANSAC, 0.999, 1.0);
    string E_mat_name = "E" + to_string(k);
    fs << E_mat_name << E;
    cout << "Calculated and stored the essential matrix. The essential matrix is:\n" << E << endl;
    cout << "Starting to calculate conjugate epipolar lines..\n\n";
    //find and store the epipolar lines
    std::vector<Point3f> lines1, lines2;
    computeCorrespondEpilines(points1, 1, F, lines2);
    computeCorrespondEpilines(points2, 2, F, lines1);
    cout << "Drawing epipolar lines and displaying them...\n";
    drawEpipolarLines("epipolar lines", F, img_1, img_2, points1, points2, 2);
    //draw_lines(img_object, img_scene, lines1, obj, scene, mask);
    //draw_lines(img_scene, img_object, lines2, scene, obj, mask);
    cout << "Decomposing Essential Matrix...\n";
    Mat R1, R2, t;
    decomposeEssentialMat(E, R1, R2, t);
    cout << "a\n";
    string R1_mat_name = "R1" + to_string(k);
    string R2_mat_name = "R2" + to_string(k);
    string t_mat_name = "t" + to_string(k);
    fs_extr << R1_mat_name << R1;
    fs_extr << R2_mat_name << R2;
    fs_extr << t_mat_name << t;
    cout << "Calculated and stored the rotation matrices and translation vectors. The first rotation matrix is:\n" << R1 << endl << "The second rotation matrix is:\n" << R2 << endl << "The translation vector is:\n" << t << endl;
    cout << "Checking correctness of the first rotational matrix...\n";
    //Checking if R1*R1(transpose) = I and det(R1) = 1
    Mat R_check, Rt;
    double Rdet = 0.0;
    transpose(R1, Rt);
    R_check = Rt * R1;
    cout << "R1 * R1transpose is:\n" << R_check << endl;
    Rdet = determinant(R1);
    cout << "Determinant of R1 is:\n" << Rdet << endl;
    //Checking if R2*R2(transpose) = I and det(R2) = 1
    Rdet = 0.0;
    transpose(R2, Rt);
    R_check = Rt * R2;
    cout << "R2 * R2transpose is:\n" << R_check << endl;
    Rdet = determinant(R2);
    cout << "Determinant of R2 is:\n" << Rdet << endl;
    Mat Rcalc, tcalc;
    calculateRT(E, Rcalc, tcalc);
    cout << "Calculated Rotation Matrix is:\n" << Rcalc << endl;
    cout << "Calculated Translation Matrix is:\n" << tcalc << endl;

    //Checking if R2*R2(transpose) = I and det(R2) = 1
    Rdet = 0.0;
    transpose(Rcalc, Rt);
    R_check = Rt * Rcalc;
    cout << "Rcalc * Rcalctranspose is:\n" << R_check << endl;
    Rdet = determinant(Rcalc);
    cout << "Determinant of Rcalc is:\n" << Rdet << endl;
    projectgivenPoints("projected points", Rcalc, F, E, tcalc, img_1, img_2, points1, points2, 2);

  }
  fs.release();
  fs_extr.release();
}
