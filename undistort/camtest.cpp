Conversations
	
	
	
Yash Soni
	
Undistorted Image
 - -- Yash Soni 1st Year Undergraduate Student Mathematics and Computing IIT Kharagpur
Attachments
test.cpp
camtest.cpp
	
	May 5
2.61 GB (17%) of 15 GB used
Manage
Terms · Privacy · Program Policies
Last account activity: 2 days ago
Details

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "optorusb.h"
#include "optorcam.h"
#include "optorimu.h"
#include <ctime>

using namespace std;
using namespace cv;
bool close_img_viewer=false;
bool visensor_Close_IMU_viewer=false;

// 当前左右图像的时间戳
timeval left_stamp,right_stamp;

/**
 * @brief opencv_showimg
 * @return
 */
time_t t1,t2;
int i=0;
void *opencv_showimg(void*)
{
	Mat img_left(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
	Mat img_left_undistorted(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
	//img_left_undistorted = img_left.clone();
	double left_timestamp;
	Mat img_right(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
	Mat img_right_undistorted(Size(visensor_img_width(),visensor_img_height()),CV_8UC1);
	//img_right_undistorted = img_right.clone();
	double right_timestamp;
	visensor_imudata img_imudata;

	float camera_mat_left[3][3] = {462.901520, 0.0, 407.490466, 0.0, 461.090515, 268.767129, 0.0, 0.0, 1.0};
	float camera_mat_right[3][3] = {470.984593, 0.0, 361.474843, 0.0, 472.153861, 249.673963, 0.0, 0.0, 1.0};

	float dist_left[1][5] = {-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	float dist_right[1][5] = {-0.361678, 0.104504, 0.000347, 0.007105, 0.0};

	Mat intrinsic_left = Mat(3, 3, CV_32FC1, camera_mat_left);
	Mat intrinsic_right = Mat(3, 3, CV_32FC1, camera_mat_right);
	Mat disCoeffs_left = Mat(1, 5, CV_32FC1, dist_left);
	Mat disCoeffs_right = Mat(1, 5, CV_32FC1, dist_right);
/*
	Mat intrinsic_left = Mat(3, 3, CV_32FC1);
	Mat intrinsic_right = Mat(3, 3, CV_32FC1);
	Mat disCoeffs_left = Mat(1, 5, CV_32FC1);
	Mat disCoeffs_right = Mat(1, 5, CV_32FC1);
	intrinsic_left.at<uchar>(0,0)=462.901520;//{462.901520, 0.0, 407.490466};
	intrinsic_left.at<uchar>(0,1)=0.0;//{462.901520, 0.0, 407.490466};
	intrinsic_left.at<uchar>(0,2)=407.490466;//{462.901520, 0.0, 407.490466};
	intrinsic_left.at<uchar>(1,0)=0.0;//{0.0, 461.090515, 268.767129};
	intrinsic_left.at<uchar>(1,1)=461.090515;//{0.0, 461.090515, 268.767129};
	intrinsic_left.at<uchar>(1,2)=268.767129;//{0.0, 461.090515, 268.767129};
	intrinsic_left.at<uchar>(2,0)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_left.at<uchar>(2,1)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_left.at<uchar>(2,2)=1.0;//{0.0, 0.0, 1.0};

	disCoeffs_left.at<uchar>(0,0)=-0.399212;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<uchar>(0,1)=0.142994;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<uchar>(0,2)=-0.001086;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<uchar>(0,3)=-0.000506;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
	disCoeffs_left.at<uchar>(0,4)=0.0;//{-0.399212, 0.142994, -0.001086, -0.000506, 0.0};

	intrinsic_right.at<uchar>(0,0)=470.984593;//{470.984593, 0.0, 361.474843};
	intrinsic_right.at<uchar>(0,1)=0.0;//{470.984593, 0.0, 361.474843};
	intrinsic_right.at<uchar>(0,2)=361.474843;//{470.984593, 0.0, 361.474843};
	intrinsic_right.at<uchar>(1,0)=0.0;//{0.0, 472.153861, 249.673963};
	intrinsic_right.at<uchar>(1,1)=472.153861;//{0.0, 472.153861, 249.673963};
	intrinsic_right.at<uchar>(1,2)=249.673963;//{0.0, 472.153861, 249.673963};
	intrinsic_right.at<uchar>(2,0)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_right.at<uchar>(2,1)=0.0;//{0.0, 0.0, 1.0};
	intrinsic_right.at<uchar>(2,2)=1.0;//{0.0, 0.0, 1.0};

	disCoeffs_right.at<uchar>(0,0)=-0.361678;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<uchar>(0,1)=0.104504;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<uchar>(0,2)=0.000347;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<uchar>(0,3)=0.007105;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	disCoeffs_right.at<uchar>(0,4)=0.0;//{-0.361678, 0.104504, 0.000347, 0.007105, 0.0};

	//intrinsic_left = {{462.901520, 0.0, 407.490466},{0.0, 461.090515, 268.767129},{0.0, 0.0, 1.0}};
	//disCoeffs_left = {-0.399212, 0.142994, -0.001086, -0.000506, 0.0};
*/
	//intrinsic_right = {{470.984593, 0.0, 361.474843},{0.0, 472.153861, 249.673963},{0.0, 0.0, 1.0}};
	//	disCoeffs_right = {-0.361678, 0.104504, 0.000347, 0.007105, 0.0};
	while(!close_img_viewer)
	{/*
		if(visensor_is_leftcam_open() && visensor_is_rightcam_open())
		{
			if(visensor_is_left_img_new() && visensor_is_right_img_new())
			{
				visensor_get_left_latest_img(img_left.data,&left_timestamp,&img_imudata);
				//printf("L-Time: %8.6f, IMUTime: %8.6f\n",left_timestamp,img_imudata.timestamp);
				visensor_get_right_latest_img(img_right.data,&right_timestamp);
				//printf("R-Time: %8.6f\n",right_timestamp);
				//img_left_undistorted = img_left.clone();
				if(i==200)
				{
					t1 = time(NULL);
					undistort(img_left, img_left_undistorted, intrinsic_left, disCoeffs_left);
					t2 = time(NULL);
					printf("TIME DIFFERENCE ::::%ld",t2-t1);
					undistort(img_right, img_right_undistorted, intrinsic_right, disCoeffs_right);
					imwrite("left.jpg",img_left_undistorted);
					imwrite("right.jpg",img_right_undistorted);
					imwrite("distorted_left.jpg",img_left);
					imwrite("distorted_right.jpg",img_right);
				}
				i++;
				imshow("distorted_left.jpg",img_left);
				imshow("distorted_right",img_right);
			}
		}*/

	   if(visensor_is_leftcam_open())
        {
            if(visensor_is_left_img_new())
            {
                visensor_get_left_latest_img(img_left.data,&left_timestamp,&img_imudata);
                printf("L-Time: %8.6f, IMUTime: %8.6f\n",left_timestamp,img_imudata.timestamp);
                undistort(img_left, img_left_undistorted, intrinsic_left, disCoeffs_left);
                //imshow("distorted_left",img_left);
                imshow("UNdistorted_left",img_left_undistorted);
            }
        }
       if(visensor_is_rightcam_open())
        {
            if(visensor_is_right_img_new())
            {
                visensor_get_right_latest_img(img_right.data,&right_timestamp);
                printf("R-Time: %8.6f\n",right_timestamp);
                undistort(img_right, img_right_undistorted, intrinsic_right, disCoeffs_right);
                //imshow("distorted_right",img_right);
                imshow("UNdistorted_right",img_right_undistorted);
            }
        }
		waitKey(10);
	}
	pthread_exit(NULL);
}


void* show_imuData(void *)
{
	visensor_imudata imudata;
	while(!visensor_Close_IMU_viewer)
	{
		if(visensor_imu_have_fresh_data())
		{
			visensor_get_imudata_latest(&imudata);
			/*printf("IMUTime:%8.6f, Gyr: %8.4f,%8.4f,%8.4f, Acc: %8.4f,%8.4f,%8.4f, Quat(WXYZ): %8.4f,%8.4f,%8.4f,%8.4f\n",
				   imudata.timestamp,
				   imudata.rx,imudata.ry,imudata.rz,
				   imudata.ax,imudata.ay,imudata.az,
				   imudata.qw,imudata.qx,imudata.qy,imudata.qz);*/
		}
		usleep(100);
	}
	pthread_exit(NULL);
}

int main(int argc, char* argv[])
{
 
	/************************ Start Cameras ************************/
	visensor_load_settings("../optor_VISensor_Setups.txt");
	/*
	// 手动设置相机参数
	visensor_set_current_mode(5);
	visensor_set_auto_EG(0);
	visensor_set_exposure(50);
	visensor_set_gain(200);
	visensor_set_cam_selection_mode(2);
	visensor_set_resolution(false);
	visensor_set_fps_mode(true);
	// 保存相机参数到原配置文件
	visensor_save_current_settings();
	*/

	int r = visensor_Start_Cameras();
	if(r<0)
	{
		printf("Opening cameras failed...\r\n");
		return r;
	}
	/************************** Start IMU **************************/
	int fd=visensor_Start_IMU();
	if(fd<0)
	{
		printf("visensor_open_port error...\r\n");
		return 0;
	}
	printf("visensor_open_port success...\r\n");
	/************************ ************ ************************/

	usleep(100000);

	//Create img_show thread
	pthread_t showimg_thread;
	int temp;
	if(temp = pthread_create(&showimg_thread, NULL, opencv_showimg, NULL))
		printf("Failed to create thread opencv_showimg\r\n");
	//Create show_imuData thread
	pthread_t showimu_thread;
	if(temp = pthread_create(&showimu_thread, NULL, show_imuData, NULL))
		printf("Failed to create thread show_imuData\r\n");

	while(1)
	{
		// Do - Nothing :)
		//cout<<visensor_get_imu_portname()<<endl;
		//cout<<visensor_get_hardware_fps()<<endl;
		sleep(1);
	}

	/* shut-down viewers */
	close_img_viewer=true;
	visensor_Close_IMU_viewer=true;
	if(showimg_thread !=0)
	{
		pthread_join(showimg_thread,NULL);
	}
	if(showimu_thread !=0)
	{
		pthread_join(showimu_thread,NULL);
	}

	/* close cameras */
	visensor_Close_Cameras();
	/* close IMU */
	visensor_Close_IMU();

	return 0;
}
