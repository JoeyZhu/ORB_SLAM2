/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

//test with
/*
 * rosbag play --pause V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
 *
 * pub base to camera with:
 *
 * static_transform_publisher x y z yaw pitch roll frame_id child_frame_id
 * <launch>
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_base" args="0 0 0 -3.1415926/2 3.1415926/2 0 /base_link /camera_base" />
</launch>
 */
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static const boost::array<double, 36> STANDARD_POSE_COVARIANCE =
{ { 0.1, 0, 0, 0, 0, 0,
    0, 0.1, 0, 0, 0, 0,
    0, 0, 0.1, 0, 0, 0,
    0, 0, 0, 0.17, 0, 0,
    0, 0, 0, 0, 0.17, 0,
    0, 0, 0, 0, 0, 0.17 } };
static const boost::array<double, 36> STANDARD_TWIST_COVARIANCE =
{ { 0.002, 0, 0, 0, 0, 0,
    0, 0.002, 0, 0, 0, 0,
    0, 0, 0.05, 0, 0, 0,
    0, 0, 0, 0.09, 0, 0,
    0, 0, 0, 0, 0.09, 0,
    0, 0, 0, 0, 0, 0.09 } };
static const boost::array<double, 36> BAD_COVARIANCE =
{ { 9999, 0, 0, 0, 0, 0,
    0, 9999, 0, 0, 0, 0,
    0, 0, 9999, 0, 0, 0,
    0, 0, 0, 9999, 0, 0,
    0, 0, 0, 0, 9999, 0,
    0, 0, 0, 0, 0, 9999 } };


using namespace std;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

ros::Publisher *odom_pub_ptr;
tf::TransformBroadcaster *odom_broadcaster_ptr;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ORBSLAM_STEREO");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM2 Stereo path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

    ros::NodeHandle nh;

    odom_pub_ptr = new ros::Publisher(nh.advertise<nav_msgs::Odometry>("odom", 50));
    odom_broadcaster_ptr = new tf::TransformBroadcaster;

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));

    ROS_INFO("INIT SUCCESS");

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void publish_odom(geometry_msgs::Vector3 xyz_p,
        geometry_msgs::Vector3 xyz_v_l,
        geometry_msgs::Vector3 xyz_v_a,
        geometry_msgs::Quaternion odom_quat){

    ROS_INFO_THROTTLE(1.0, "pub odom");

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "/odom";
    odom_trans.child_frame_id = "/base_link";

    odom_trans.transform.translation.x = xyz_p.x;
    odom_trans.transform.translation.y = xyz_p.y;
    odom_trans.transform.translation.z = xyz_p.z;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster_ptr->sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = xyz_p.x;
    odom.pose.pose.position.y = xyz_p.y;
    odom.pose.pose.position.z = xyz_p.z;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "/base_link";
    odom.twist.twist.linear = xyz_v_l;
    odom.twist.twist.angular = xyz_v_a;
    odom_pub_ptr->publish(odom);
}

void integrateAndPublish(const tf::Transform& sensor_transform, const ros::Time& timestamp)
{
	//TODO: transform sensor_tf to base_link_tf
/*
  if (sensor_frame_id_.empty())
  {
    ROS_ERROR("[odometer] update called with unknown sensor frame id!");
    return;
  }
  if (timestamp < last_update_time_)
  {
    ROS_WARN("[odometer] saw negative time change in incoming sensor data, resetting pose.");
    integrated_pose_.setIdentity();
    tf_listener_.clear();
  }
  integrated_pose_ *= delta_transform;

  // transform integrated pose to base frame
  tf::StampedTransform base_to_sensor;
  std::string error_msg;
  if (tf_listener_.canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, &error_msg))
  {
    tf_listener_.lookupTransform(
        base_link_frame_id_,
        sensor_frame_id_,
        timestamp, base_to_sensor);
  }
  else
  {
    ROS_WARN_THROTTLE(10.0, "The tf from '%s' to '%s' does not seem to be available, "
                            "will assume it as identity!",
                            base_link_frame_id_.c_str(),
                            sensor_frame_id_.c_str());
    ROS_DEBUG("Transform error: %s", error_msg.c_str());
    base_to_sensor.setIdentity();
  }

  tf::Transform base_transform = base_to_sensor * integrated_pose_ * base_to_sensor.inverse();
*/

	static ros::Time last_update_time_;
	static tf::Transform sensor_transform_pre;

  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.stamp = timestamp;
  odometry_msg.header.frame_id = "/odom";
  odometry_msg.child_frame_id = "/base_link";
  tf::poseTFToMsg(sensor_transform, odometry_msg.pose.pose);

  // calculate twist (not possible for first run as no delta_t can be computed)
  if (!last_update_time_.isZero())
  {
    double delta_t = (timestamp - last_update_time_).toSec();
    if (delta_t)
    {
      odometry_msg.twist.twist.linear.x = (sensor_transform * sensor_transform_pre.inverse()).getOrigin().getX() / delta_t;
      odometry_msg.twist.twist.linear.y = (sensor_transform * sensor_transform_pre.inverse()).getOrigin().getY() / delta_t;
      odometry_msg.twist.twist.linear.z = (sensor_transform * sensor_transform_pre.inverse()).getOrigin().getZ()  / delta_t;
      tf::Quaternion delta_rot = (sensor_transform * sensor_transform_pre.inverse()).getRotation();
      tfScalar angle = delta_rot.getAngle();
      tf::Vector3 axis = delta_rot.getAxis();
      tf::Vector3 angular_twist = axis * angle / delta_t;
      odometry_msg.twist.twist.angular.x = angular_twist.x();
      odometry_msg.twist.twist.angular.y = angular_twist.y();
      odometry_msg.twist.twist.angular.z = angular_twist.z();
    }
  }

  odometry_msg.pose.covariance = STANDARD_POSE_COVARIANCE;
  odometry_msg.twist.covariance = STANDARD_TWIST_COVARIANCE;
  odom_pub_ptr->publish(odometry_msg);

  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp = odometry_msg.header.stamp;
  pose_msg.header.frame_id = odometry_msg.header.frame_id;
  pose_msg.pose = odometry_msg.pose.pose;

  odom_broadcaster_ptr->sendTransform(
          tf::StampedTransform(sensor_transform, timestamp,
          "/odom", "/base_link"));

  last_update_time_ = timestamp;
  sensor_transform_pre = sensor_transform;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat T_C_W_opencv;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
        T_C_W_opencv = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        T_C_W_opencv = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
    ROS_INFO("ORBSLAM");
    // If tracking successfull
    if (!T_C_W_opencv.empty()) {
    	//T_C_W_opencv = T_C_W_opencv.inv();
        static int counter = 0;
        counter++;
        geometry_msgs::Vector3 xyz_p, xyz_v_l, xyz_v_a;
        geometry_msgs::Quaternion odom_quat;

        tf::Matrix3x3 rot_mat(
        		T_C_W_opencv.at<float>(0,0), T_C_W_opencv.at<float>(0,1), T_C_W_opencv.at<float>(0,2),
				T_C_W_opencv.at<float>(1,0), T_C_W_opencv.at<float>(1,1), T_C_W_opencv.at<float>(1,2),
				T_C_W_opencv.at<float>(2,0), T_C_W_opencv.at<float>(2,1), T_C_W_opencv.at<float>(2,2));
        tf::Quaternion q;
        rot_mat.getRotation(q);
        odom_quat.x = q.getX();
        odom_quat.y = q.getY();
        odom_quat.z = q.getZ();
        odom_quat.w = q.getW();

        std::cout << "T_C_W_opencv" << std::endl;
        std::cout << T_C_W_opencv << std::endl;

        std::cout << "quaternion:" << std::endl;
        std::cout << odom_quat << std::endl;

        xyz_p.x = T_C_W_opencv.at<float>(0,3);
        xyz_p.y = T_C_W_opencv.at<float>(1,3);
        xyz_p.z = T_C_W_opencv.at<float>(2,3);

        std::cout << "xyz position: " << std::endl << xyz_p << std::endl;

        tf::Vector3 t(T_C_W_opencv.at<float>(0,3), T_C_W_opencv.at<float>(1,3), T_C_W_opencv.at<float>(2,3));
        tf::Transform camera_transform(rot_mat, t);

        integrateAndPublish(camera_transform, msgLeft->header.stamp);
        publish_odom(xyz_p, xyz_v_l, xyz_v_a,odom_quat);

    }
}


