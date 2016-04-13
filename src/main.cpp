/*
Copyright 2012. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libviso2.
Authors: Andreas Geiger

libviso2 is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or any later version.

libviso2 is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libviso2; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

/*
  Documented C++ sample code of stereo visual odometry (modify to your needs)
  To run this demonstration, download the Karlsruhe dataset sequence
  '2010_03_09_drive_0019' from: www.cvlibs.net!
  Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019
*/
#include<ros/ros.h>
#include<sensor_msgs/Imu.h>
#include<nav_msgs/Odometry.h>
#include<tf/tf.h>
#include<tf2/LinearMath/Transform.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <ctime>
#include <chrono>
#include<thread>
#include<mutex>
#include <vector>
#include <stdint.h>
#include <zed/Camera.hpp>
#include<zed/utils/GlobalDefine.hpp>
#include <viso_stereo.h>
#include <png++/png.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<cv_bridge/cv_bridge.h>
#include<cmath>
#include<sensor_msgs/Imu.h>	
using namespace std;
using namespace sl::zed;

static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =                 
   {{0.1, 0, 0, 0, 0, 0,                                                          
     0, 0.1, 0, 0, 0, 0,                                                          
     0, 0, 0.1, 0, 0, 0,                                                          
     0, 0, 0, 0.17, 0, 0,                                                         
     0, 0, 0, 0, 0.17, 0,                                                         
     0, 0, 0, 0, 0, 0.17 }};     
double imu_yaw = 0;
double first_yaw_reading = 0;
double viso_yaw = 0;
typedef struct image_bufferStruct{
  //unsigned char* data_left, *data_right;
  sl::zed::Mat data_right,data_left;
  std::mutex mutex_input_image;

  int width, height, im_channels;
}image_buffer;

Camera* zed;
image_buffer* buffer;
SENSING_MODE dm_type = SENSING_MODE(1);
bool stop_signal;
int count_run = 0;
bool newFrame = false;
void imu_callback(const sensor_msgs::Imu::ConstPtr& msg){
  tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  tf::Matrix3x3 m(q);
  double roll,pitch,yaw;
  m.getRPY(roll,pitch,yaw);
  if (yaw < 0)
    yaw = 6.283185307179586+yaw;
  imu_yaw = yaw;
 // cout<<"in_sub_got_yaw "<<yaw<<endl;

}
void grab_run(){
  while(!stop_signal){
    if(!zed->grab(dm_type,0,0)){
     // cout<<"processing"<<endl;
      buffer->mutex_input_image.lock();
     // buffer->data_left = zed->retrieveImage(SIDE::LEFT).data;
     // buffer->data_right = zed->retrieveImage(SIDE::RIGHT).data;
      buffer->data_left = zed->retrieveImage(SIDE::LEFT);
      buffer->data_right = zed->retrieveImage(SIDE::RIGHT);
 //     memcpy(buffer->data_left,zed->retrieveImage(SIDE::LEFT).data,buffer->width * buffer->height*buffer->im_channels*sizeof(uchar));
   //   memcpy(buffer->data_right,zed->retrieveImage(SIDE::RIGHT).data,buffer->width * buffer->height*buffer->im_channels*sizeof(uchar));
      newFrame = true;
      buffer->mutex_input_image.unlock();
      newFrame = true;
 //     cv::Mat colorMat_left(buffer->height,buffer->width,CV_8UC4,buffer->data_left,buffer->width*buffer->im_channels*sizeof(uchar));
   //   cv::Mat colorMat_right(buffer->height,buffer->width,CV_8UC4,buffer->data_right,buffer->width*buffer->im_channels*sizeof(uchar));
  //    cv::imshow("left",colorMat_left);
    //  cv::imshow("right",colorMat_right);
   //   char key = cv::waitKey(1);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
//  cout<<newFrame<<endl;

  }
}
int main (int argc, char** argv) {

  // we need the path name to 2010_03_09_drive_0019 as input argument
  if (argc<2) {
  //  cerr << "Usage: ./viso2 path/to/sequence/2010_03_09_drive_0019" << endl;
   // return 1;
  }
  ros::init(argc, argv, "imu_viso_localization");
  ros::NodeHandle node;
  ros::Publisher odom_pub;
  ros::Subscriber imu_sub = node.subscribe("/mavros/imu/data",1,imu_callback); 	
  odom_pub = node.advertise<nav_msgs::Odometry>("imu_viso_localization/odometry",10);
  nav_msgs::Odometry viso_odom; 
  //start the grabbing process from zed 
  stop_signal = false;
  if(argc == 1){
  //  zed = new Camera(HD720);
    zed = new Camera(static_cast<sl::zed::ZEDResolution_mode> (2));
//    zed = new Camera(VGA);
  }
  else
    zed = new Camera(argv[1]);
  int width = zed->getImageSize().width;
  int height = zed->getImageSize().height;

  ERRCODE err = zed->init(MODE::NONE,-1, true);
  cout <<errcode2str(err) <<endl;
  if(err != SUCCESS){
    delete zed;
    return 1;
  }

  buffer = new image_buffer();
  buffer->height = height;
  buffer->width = width;
  buffer->im_channels = 4;

  //visual odometry camera parameters
  sl::zed::StereoParameters* zedParam = zed->getParameters();
  VisualOdometryStereo::parameters param;

  //param.calib.f  = zedParam->LeftCam.fx; // focal length in pixels
  param.calib.f  = 711.554; // focal length in pixels
  //param.calib.cu = zedParam->LeftCam.cx; // principal point (u-coordinate) in pixels
  param.calib.cu = 647.039; // principal point (u-coordinate) in pixels
  param.calib.cv = 180.394; // principal point (v-coordinate) in pixels
//  param.calib.cv = (zedParam->LeftCam.cy)/2; // principal point (v-coordinate) in pixels
  cout<<param.calib.f<<endl;
  cout<<param.calib.cu<<endl;
  cout<<param.calib.cv<<endl;
//  param.calib.cv = 180; // principal point (v-coordinate) in pixels
  param.base     = 0.12; // baseline in meters

  // init visual odometry
  VisualOdometryStereo viso(param);

  // current pose (this matrix transforms a point from the current
  // frame's camera coordinates to the first frame's camera coordinates)
  Matrix pose = Matrix::eye(4);
  tf::Matrix3x3 current_rotation;  
  //create windows for left and right images 
 // cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
 // cv::namedWindow("right",cv:: WINDOW_AUTOSIZE);
  
  //allocate memory for images
  int i = 0;
  uchar *pleft,*pright;
  pleft = new uchar [buffer->width*buffer->height*buffer->im_channels*sizeof(uchar)];
  pright = new uchar [buffer->width*buffer->height*buffer->im_channels*sizeof(uchar)];

  //tranform camera frame to world frame
  double roll,pitch,yaw;
  roll = -1.5707963267948966;                                                                   
  pitch = 0;                                                                   
  yaw = -1.5707963267948966;
  tf::Quaternion quat;
  quat.setRPY(roll,pitch,yaw);
  tf::Transform imu_transform_viso;
  imu_transform_viso.setOrigin(tf::Vector3(0,0,0));
  imu_transform_viso.setRotation(quat);
  //convert images to grayscale
//  cv::Mat greyMat_left,greyMat_right;
//  cv::Mat colorMat_left(height,width,CV_8UC4,pleft,width*buffer->im_channels*sizeof(uchar));
//  cv::Mat colorMat_right(height,width,CV_8UC4,pright,width*buffer->im_channels*sizeof(uchar));

  //run the grabbing thread
  std::thread grab_thread(grab_run);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  
  bool first_frame = false; 
 // uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));  
//  uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
  cout<<width*height*sizeof(uint8_t)<<endl;
  //run viso 
 // uint8_t left_img_data[width*height*sizeof(uint8_t)];  
 // uint8_t right_img_data[width*height*sizeof(uint8_t)];
 cv::Mat colorMat_left_temp(height,width,CV_8UC4);
 cv::Mat colorMat_right_temp(height,width,CV_8UC4);
 bool first_run = true;
 bool got_lost = false;
 double prev_yaw = 0;
  while(1) {
 //   left_img_data[1] = 1;
     if(1) {
      //get lock to copy images
      buffer->mutex_input_image.try_lock();
 //     memcpy(pleft,buffer->data_left,buffer->width * buffer->height*buffer->im_channels*sizeof(uchar));
  //    memcpy(pright,buffer->data_right,buffer->width * buffer->height*buffer->im_channels*sizeof(uchar));
      colorMat_left_temp = slMat2cvMat(buffer->data_left);
      colorMat_right_temp = slMat2cvMat(buffer->data_right);

     // newFrame = false;
      buffer->mutex_input_image.unlock();
      cv::Mat colorMat_left;
     colorMat_left_temp(cv::Rect(0,180,1280,360)).copyTo(colorMat_left);
      cv::Mat colorMat_right;
      colorMat_right_temp(cv::Rect(0,180,1280,360)).copyTo(colorMat_right);
     
//      height = height/2;
      //convert images to grayscale
      cv::Mat greyMat_left,greyMat_right;
    //  cv::Mat colorMat_left(height,width,CV_8UC4,pleft,width*buffer->im_channels*sizeof(uchar));
    //  cv::Mat colorMat_right(height,width,CV_8UC4,pright,width*buffer->im_channels*sizeof(uchar)); 
      // convert input images to uint8_t buffer
      cv::cvtColor(colorMat_right,greyMat_right,CV_RGBA2GRAY);
      cv::cvtColor(colorMat_left,greyMat_left,CV_RGBA2GRAY);
      uint8_t* left_img_data  = greyMat_left.data;  
      uint8_t* right_img_data = greyMat_right.data;
      
    //  cv::Mat right_re_image(height/2,width,CV_8UC1);
   //   cv::Mat left_re_image(height/2,width,CV_8UC1);
    //  cv::resize(greyMat_right,right_re_image,cv::Size(width,height/2),0,0,0);
    //  cv::resize(greyMat_right,left_re_image,cv::Size(width,height/2),0,0,0);
//      greyMat_left.convertTo(greyMat_left,CV_8UC1);
 //     greyMat_right.convertTo(greyMat_right,CV_8UC1);
     // continue;
      int32_t k=0;
    //  cv::imshow("left",left_re_image);
  //    char key = cv::waitKey(1);

     // continue;
    //  uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));  
    //  uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));
//      for (int32_t v=0; v<height; v++) {                                         
//	for (int32_t u=0; u<width; u++) {                                        
//	  left_img_data[k]  =  greyMat_left.at<uchar>(v,u);                          
//	    right_img_data[k] = greyMat_right.at<uchar>(v,u);                          
	//  left_img_data[k]  =  left_re_image.at<uchar>(v,u);                          
	//  right_img_data[k] = right_re_image.at<uchar>(v,u);                          
      //     cout<<int(greyMat_left.at<uchar>(u,v))<<endl;
  //         cout<<int(left_img_data[k])<<endl;
//	    k++;                                                                   
//	}                                                                        
  //    }
    // cv::Mat gr(height/2,width,CV_8U,left_img_data,width*sizeof(uint8_t)); 
   //  imshow("grey",gr);
   //  char key2 = cv::waitKey(1);
    //  cout<<"here"<<endl;
    //  cout<<"hdfhaf"<<endl;
     // continue;

      // compute visual odometry
      int32_t dims[] = {width,height/2,width};
//	cout<<"success"<<endl;
//	continue;
      ros::spinOnce();
      double relative_yaw = 0;
      if (first_run){
	prev_yaw = imu_yaw;
	first_yaw_reading = imu_yaw;
      }
      else{
	relative_yaw = imu_yaw - first_yaw_reading;
//	prev_yaw = imu_yaw;
      }
    //  cout<<relative_yaw<<endl;

      if (viso.process(left_img_data,right_img_data,dims,relative_yaw,viso_yaw)) {
	cout<<"success"<<endl;
//	continue;
      //  first_frame = true;
	// on success, update current pose
	if(got_lost){
	  got_lost = false;
	  continue;
	}
	if(!first_run){
//	  cout<<"!first_run"<<endl;
	  pose = pose * Matrix::inv(viso.getMotion());
	}
	first_run = false;

	current_rotation.setValue(pose.val[0][0],pose.val[0][1],pose.val[0][2],pose.val[1][0],pose.val[1][1],pose.val[1][2],pose.val[2][0],pose.val[2][1],pose.val[2][2]);
	tf::Transform viso_transform_current(current_rotation,tf::Vector3(pose.val[0][3],pose.val[1][3],pose.val[2][3]));
	tf::Transform imu_transform_current = (imu_transform_viso)*viso_transform_current*(imu_transform_viso.inverse());
//	tf::Transform imu_transform_current = viso_transform_current*(imu_transform_viso.inverse());
//	tf::Transform imu_transform_current = (imu_transform_viso.inverse())*viso_transform_current*imu_transform_viso;
//	tf::Transform imu_transform_current = viso_transform_current;
	viso_odom.header.stamp = ros::Time::now();
	viso_odom.header.frame_id = "/odom";
	viso_odom.child_frame_id = "/base_link";
	tf::poseTFToMsg(imu_transform_current, viso_odom.pose.pose);
	tf::Quaternion qw(viso_odom.pose.pose.orientation.x,viso_odom.pose.pose.orientation.y,viso_odom.pose.pose.orientation.z,viso_odom.pose.pose.orientation.w);
	tf::Matrix3x3 rpy(qw);
	double rll,pt,yw;
	rpy.getRPY(rll,pt,yw);
	viso_yaw = yw;
	cout<<"viso "<<viso_yaw<<endl;
	cout<<"imu "<<imu_yaw<<endl;
	cout<<"imu2 "<<first_yaw_reading<<endl;
	cout<<"relative "<<relative_yaw<<endl;
/*	tf::Vector3 origin = imu_transform_current.getOrigin();
	tf::Quaternion rotation2 = imu_transform_current.getRotation();
	tf::Quaternion rotation = rotation2.normalized();
	tf::Vector3 axis = rotation2.getAxis();
	double w = rotation2.getW();
	float length = sqrt(pow(axis.x(),2) + pow(axis.y(),2) + pow(axis.z(),2)+ pow(w,2));


	viso_odom.pose.pose.position.x = origin.x();
	viso_odom.pose.pose.position.y = origin.y();
	viso_odom.pose.pose.position.z = origin.z();

	viso_odom.pose.pose.orientation.x = axis.x()/length;
	viso_odom.pose.pose.orientation.y = axis.y()/length;
	viso_odom.pose.pose.orientation.z = axis.z()/length;
	viso_odom.pose.pose.orientation.w = w/length;*/
	for(int s = 0 ; s<36; s++)
	  viso_odom.pose.covariance[s] = STANDARD_POSE_COVARIANCE[s];
	// output some statistics
//	double num_matches = viso.getNumberOfMatches();
//	double num_inliers = viso.getNumberOfInliers();
//	cout << ", Matches: " << num_matches;
//	cout << ", Inliers: " << 100.0*num_inliers/num_matches << " %" << ", Current pose: " << endl;
//	cout << pose.val[0][3] <<" "<<pose.val[2][3]<<endl;;
	odom_pub.publish(viso_odom);
  //    height = height*2;
      }
      else{
	std::cout<<"fail"<<std::endl;
	got_lost = true;
      }
      newFrame = false;
  //    free(left_img_data);
  //    free(right_img_data);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    // release uint8_t buffers

    // catch image read errors here
  }

  // output

  // exit
  return 0;
}

