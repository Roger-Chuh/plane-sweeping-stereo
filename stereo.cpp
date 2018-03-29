/*cpp file that has the definitions for the function in epipoles.h
Author: Sasidharan Mahalingam
Date Created: March 10 2018 */

#include "stereo.h"

/*Function that scales the translation vectors
Usage:
Parameters:
R_list: vector of matrices that contains all the rotation matrices
t_list: vector of matrices that contains the unscaled translation vectors
st_list: vector of matrices that contains the scaled translation vectors
min_params: vector of doubles that contains the values of alpha and gamma
*/

void find_alpha_gamma(const std::vector<Mat> R_list, std::vector<Mat> t_list, vector<Mat> &st_list, vector<double> & min_params)
{
  vector<Mat> rescaled_t_vec, tmp(t_list.begin(),t_list.end());
  for (auto &j : t_list)
  {
    //normalize each translation vector
    normalize(j, j, 1);
  }

  //express all the translation vector in the first camera's frame of reference

  //Rotating r12 to express it in the frame of reference of the first camera
  cout << "R12\n" << R_list[0] << endl;
  cout << "R21\n" << R_list[0].t() << endl;
  rescaled_t_vec.push_back(R_list[0].t() * t_list[0]);
  //Rotating r23 to express it in the frame of reference of the first camera
  cout << "R13\n" << R_list[2] << endl;
  cout << "R31\n" << R_list[2].t() << endl;
  rescaled_t_vec.push_back(R_list[2].t() * t_list[1]);
  //Changin r13 to r31
  cout << "t_list" << t_list[2] << endl;
  t_list[2] = -(t_list[2]);
  cout << "t_list" << t_list[2] << endl;
  //Rotating r31 to express it in the frame of reference of the first camera
  rescaled_t_vec.push_back(R_list[2].t() * t_list[2]);
  for (auto & ele : rescaled_t_vec)
    cout << ele << endl;
  double a11, a12, a21, a22, b1, b2;
  Mat t;
  multiply(rescaled_t_vec[1],rescaled_t_vec[1],t);
  cout << "r23 " << rescaled_t_vec[1].dot(rescaled_t_vec[1]) << endl;
  cout << "r23 " << sum(t) << endl;
  cout << "r31 " << rescaled_t_vec[2].dot(rescaled_t_vec[2]) << endl;
  a11 = 2.0 * static_cast<double>(rescaled_t_vec[1].dot(rescaled_t_vec[1]));
  a12 = static_cast<double>(rescaled_t_vec[2].dot(rescaled_t_vec[1]) + rescaled_t_vec[1].dot(rescaled_t_vec[2]));
  a21 = static_cast<double>(rescaled_t_vec[1].dot(rescaled_t_vec[2]) + rescaled_t_vec[2].dot(rescaled_t_vec[1]));
  a22 = 2.0 * static_cast<double>(rescaled_t_vec[2].dot(rescaled_t_vec[2]));
  b1 = -static_cast<double>(rescaled_t_vec[0].dot(rescaled_t_vec[1]) + rescaled_t_vec[1].dot(rescaled_t_vec[0]));
  b2 = -static_cast<double>(rescaled_t_vec[0].dot(rescaled_t_vec[2]) + rescaled_t_vec[2].dot(rescaled_t_vec[0]));
  cout << "a11" << a11 << endl;
  cout << "a12" << a12 << endl;
  cout << "a21" << a21 << endl;
  cout << "a22" << a22 << endl;
  Eigen::MatrixXd A(2,2);
  Eigen::VectorXd b(2);
  A(0,0) = a11;
  A(0,1) = a12;
  A(1,0) = a21;
  A(1,1) = a22;
  b << b1, b2;
  cout << "The matrix A is:\n" << A << endl;
  cout << "The vector b is:\n" << b << endl;
  VectorXd x(2);
  x = A.colPivHouseholderQr().solve(b);
  cout << "The solution is:\n" << x << endl;
  double relative_error = (A*x - b).norm() / b.norm(); // norm() is L2 norm
  cout << "The relative error is:\n" << relative_error << endl;
  cout << "Sum of unscaled translation vectors: " << rescaled_t_vec[0] + rescaled_t_vec[1] + rescaled_t_vec[2] << endl;
  rescaled_t_vec[1] = rescaled_t_vec[1] * x[0];
  rescaled_t_vec[2] = rescaled_t_vec[2] * x[1];
  cout << "Sum of scaled translation vectors: " << rescaled_t_vec[0] + rescaled_t_vec[1] + rescaled_t_vec[2] << endl;
  //st_list.push_back(t_list[0]);
  //st_list.push_back(t_list[1] * x[0]);
  //st_list.push_back(t_list[2] * x[1]);
  st_list = rescaled_t_vec;
  min_params.push_back(x[0]);
  min_params.push_back(x[1]);
}

/* Function that gives the set of sweeping planes for a given Rotation matrix and translation vector
Usage:
Parameters:
R: Rotational matrix (3x3)
t: Translation vector (3x1)
plane_set: vector of matrices that represent the planes
*/
void find_plane_sets(const Mat R, const Mat t, std::vector<Mat> & plane_set)
{
  Mat A;
  hconcat(R,t,A);
  string param_filename = "camera_params.xml";
  string intr_param_filename = "camera_params.xml";
  Mat intrinsic;
  //get intrinsic matrix and focal length from the xml
  FileStorage intr_fs(intr_param_filename, FileStorage::READ);
  intr_fs["camera_matrix"] >> intrinsic;
  double f = intrinsic.at<double>(0,0);
  intr_fs.release();
  Mat plane_eq = Mat::zeros(1,4,CV_64FC1);
  plane_eq.at<double>(0,2) = 1;
  for (double i = 1; i  <= 5  ; i= i + 0.02)
  {
    Mat P;
    double tmp = f / static_cast<double>(i * i);
    plane_eq.at<double>(0,3) = tmp;
    vconcat(A,plane_eq,P);
    plane_set.push_back(P);
  }
}


/* Function that gives the set of homographies given corresponding sweeping planes
Usage:
Parameters:
P1: vector of Matrices that gives the equations of planes w.r.t the first camera refernce
P2: vector of Matrices that gives the equations of planes w.r.t the second camera refernce
H: vector of Matrices that stores the homographies that relate the points of the plane in the second camera
   to points on the same plane in the first camera
*/

void find_homographies(const std::vector<Mat> P1, const std::vector<Mat> P2, std::vector<Mat> & H)
{
  vector<Mat>::const_iterator itr1,itr2;
  CV_Assert(P1.size() == P2.size());
  for (itr1 = P1.begin(), itr2 = P2.begin(); itr1 < P1.end(); itr1++, itr2++)
  {
    Mat h2, h1 = Mat::zeros(3,3,CV_64FC1);
    cout << "P1:\n" << (*itr1) << endl << "P2:\n" << (*itr2) << endl << "inv(P2):\n" << (*itr2).inv() << endl;
    h2 = (*itr1) * ((*itr2).inv());
    cout << "P1(inv(P2)):\n" << h2 << endl;
    for (int i = 0; i <= 2; i++)
    {
      for (int j = 0; j <= 2; j++)
      {
        if ((i == 2) && (j == 2))
            h1.at<double>(i,j) = h2.at<double>(i+1,j+1) / h2.at<double>(3,3);
        else
        {
          if(i == 2)
            h1.at<double>(i,j) = h2.at<double>(i+1,j) / h2.at<double>(3,3);
          else
          {
            if(j == 2)
              h1.at<double>(i,j) = h2.at<double>(i,j+1) / h2.at<double>(3,3);
            else
              h1.at<double>(i,j) = h2.at<double>(i,j) / h2.at<double>(3,3);
          }
        }
      }
    }
    H.push_back(h1);
  }
}

/*Function that calculates the depth given two images
Usage:
*/
void find_depth(stringvec &v, std::vector<Mat> R_list, std::vector<Mat> t_list, std::vector<Mat> & depth_image)
{
    vector<Mat> plane_set1,plane_set2,H_set,warped_imgs;
    if(v.size() <2)
    {
      cout << "Folder empty";
      return;
    }
    vector<string>::const_iterator i = v.begin();
    vector<string>::const_iterator j = i + 1;
    int k = 0;
    while(j < v.end())
    {
      Mat I = Mat::eye(3,3,CV_64FC1);
      Mat z = Mat::zeros(3,1,CV_64FC1);
      Mat img1, img2;
      img1 = imread( *i );
      img2 = imread( *j );
      //checking if both the images are readable
      if( !img1.data || !img2.data )
      {
        std::cout<< " --(!) Error reading images " << std::endl;
        return;
      }
      namedWindow("left image", CV_WINDOW_FREERATIO | CV_GUI_NORMAL);
      resizeWindow("left image", 1000, 700);
      moveWindow("left image", 500,500);
      imshow( "left image", img1 );
      namedWindow("right image", CV_WINDOW_FREERATIO | CV_GUI_NORMAL);
      resizeWindow("right image", 1000, 700);
      moveWindow("right image", 2000,500);
      imshow( "right image", img2 );
      waitKey(0);
      destroyAllWindows();
      cout << "Finding plane sets of right image...\n";
      find_plane_sets(I,z,plane_set1);
      cout << "The calculated plane sets of the left image are:\n";
      for (auto & plane : plane_set1)
        cout << plane << endl;
      Mat R,t;
      transpose(R_list[k], R);
      t = - t_list[k];
      cout << "The rotation matrices linking the right image with the left image is:\n";
      cout << R << endl;
      cout << "The scaled translation vector linking right image with the left image is:\n";
      cout << t << endl;
      cout << "Finding plane sets of right image...\n";
      find_plane_sets(R, t, plane_set2);
      cout << "The calculated plane sets of the right image are:\n";
      for (auto & plane : plane_set2)
        cout << plane << endl;
      cout << "Now calculating the homography matrices...\n";
      find_homographies(plane_set1, plane_set2, H_set);
      cout << "The calculated homography are:\n";
      for (auto & h : H_set)
        cout << h << endl;
      cout << "Calculated homography matrices. Now warping images...\n";
      for (auto & H : H_set )
      {
        Mat warped_img;
        warpPerspective(img2, warped_img, H, img1.size());
        namedWindow("Warped Image",  WINDOW_NORMAL);
        resizeWindow("Warped Image", 1000, 700);
        cv::imshow("Warped Image", warped_img);
        cv::waitKey(200);
        destroyAllWindows();
        warped_imgs.push_back(warped_img);
      }
      double min_val = 999, min_d = 0;
      Mat tmp_d_image = Mat::zeros(img1.rows,img1.cols,CV_8UC1);
      for (int u = 0; u < img1.rows; u++)
      {
        for (int v = 0; v < img1.cols; v++)
        {
          //cout << "u: " << u << "\t v: " << v;
          for (int d = 0; d < warped_imgs.size(); d++)
          {
            double diff;
            cv::Vec3b color = img1.at<cv::Vec3b>(u,v) - warped_imgs[d].at<cv::Vec3b>(u,v);
            //cout << "color: " << color << endl;
            diff = static_cast<double>(color[0] + color[1] + color[2]);
            //cout << "diff: " << diff << endl;
            if (diff < min_val)
            {
              min_val = diff;
              min_d = d;
            }
          }
          //cout << "min distance: " << min_d << endl;
          tmp_d_image.at<uchar>(u,v) = 255 / static_cast<int>(warped_imgs.size()) * (min_d + 1);
        }
      }
      cout << "depth calculated\n";
      namedWindow("Depth Image",  WINDOW_NORMAL);
      resizeWindow("Depth Image", 1000, 700);
      cv::imshow("Depth Image", tmp_d_image);
      cv::waitKey(0);
      destroyAllWindows();
      depth_image = tmp_d_image;
      j++;
    }
}
