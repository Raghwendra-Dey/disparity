#include<iostream>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;


int main()
{
	Mat img_left=imread("left_screenshot_yash.png",0);
	Mat img_right=imread("right_screenshot_yash.png",0);
	//Mat undistorted_left = img_left.clone();
	//Mat undistorted_right = img_right.clone();

	float camera_mat_left[3][3] = {462.901520, 0.0, 407.490466, 0.0, 461.090515, 268.767129, 0.0, 0.0, 1.0};
	float camera_mat_right[3][3] = {470.984593, 0.0, 361.474843, 0.0, 472.153861, 249.673963, 0.0, 0.0, 1.0};

	float dist_left[1][5] = {-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	float dist_right[1][5] = {-0.361678, 0.104504, 0.000347, 0.007105, 0.0};

	Mat intrinsic_left = Mat(3, 3, CV_32FC1, camera_mat_left);
	Mat intrinsic_right = Mat(3, 3, CV_32FC1, camera_mat_right);
	Mat disCoeffs_left = Mat(1, 5, CV_32FC1, dist_left);
	Mat disCoeffs_right = Mat(1, 5, CV_32FC1, dist_right);

/*
	intrinsic_left.at<float>(0,0)=462.901520;//{462.901520, 0.0, 407.490466};
	intrinsic_left.at<float>(0,1)=0.0;//{462.901520, 0.0, 407.490466};
	intrinsic_left.at<float>(0,2)=407.490466;//{462.901520, 0.0, 407.490466};
	intrinsic_left.at<float>(1,0)=0.0;//{0.0, 461.090515, 268.767129};
	intrinsic_left.at<float>(1,1)=461.090515;//{0.0, 461.090515, 268.767129};
	intrinsic_left.at<float>(1,2)=268.767129;//{0.0, 461.090515, 268.767129};
	intrinsic_left.at<float>(2,0)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_left.at<float>(2,1)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_left.at<float>(2,2)=1.0;//{0.0, 0.0, 1.0};

	disCoeffs_left.at<float>(0,0)=-0.399212;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<float>(0,1)=0.142994;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<float>(0,2)=-0.001086;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<float>(0,3)=-0.000506;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<float>(0,4)=0.0;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};

	intrinsic_right.at<float>(0,0)=470.984593;//{470.984593, 0.0, 361.474843};
	intrinsic_right.at<float>(0,1)=0.0;//{470.984593, 0.0, 361.474843};
	intrinsic_right.at<float>(0,2)=361.474843;//{470.984593, 0.0, 361.474843};
	intrinsic_right.at<float>(1,0)=0.0;//{0.0, 472.153861, 249.673963};
	intrinsic_right.at<float>(1,1)=472.153861;//{0.0, 472.153861, 249.673963};
	intrinsic_right.at<float>(1,2)=249.673963;//{0.0, 472.153861, 249.673963};
	intrinsic_right.at<float>(2,0)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_right.at<float>(2,1)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_right.at<float>(2,2)=1.0;//{0.0, 0.0, 1.0};

	disCoeffs_right.at<float>(0,0)=-0.361678;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<float>(0,1)=0.104504;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<float>(0,2)=0.000347;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<float>(0,3)=0.007105;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<float>(0,4)=0.0;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
*/

	undistort(img_left, undistorted_left, intrinsic_left, disCoeffs_left, intrinsic_left);
	undistort(img_right, undistorted_right, intrinsic_right, disCoeffs_right, intrinsic_right);

	imshow("undistorted_left",undistorted_left);
	imshow("undistorted_right",undistorted_right);
	waitKey(0);
}