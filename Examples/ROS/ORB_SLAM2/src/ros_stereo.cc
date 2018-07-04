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
 * Note: copy FrameTrajectory_TUM_Format.txt to /home/joey/codes/ORB_SLAM2/Examples/ROS/ORB_SLAM2
 * rosbag play --pause -l V1_02_medium.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
 *
 * pub base to camera with:
 *
 * static_transform_publisher x y z yaw pitch roll frame_id child_frame_id    -1.74 for install bias
 * rosrun tf2_ros static_transform_publisher 0 0 0 -1.74 0 -1.57 /base_link /camera_base
 * rosrun tf static_transform_publisher 0 0 0 -1.74 0 -1.57 /odom /odom_camera_base 10
 * rosrun tf static_transform_publisher 0 0 0 -1.74 0 -1.57 /base_link /camera_base 10
 * <launch>
<node pkg="tf2_ros" type="static_transform_publisher" name="camera_base" args="0 0 0 -3.1415926/2 3.1415926/2 0 /base_link /camera_base" />
</launch>

in config.yaml file add:
Follow.enable: 1
Follow.file: /media/joey/data/dataset/1004/FrameTrajectory_TUM_Format.txt

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
#include <tf/transform_listener.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <math.h>

#include <termios.h>

int get_path_from_tum_trajectory(const string &filename);
float pose_distance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2);

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
tf::TransformListener *tf_listener_ptr;

std::vector<geometry_msgs::PoseStamped> plan_pose;
ros::Publisher *plan_pub_ptr;
ros::Publisher *current_pose_pub_ptr;
ros::Publisher *local_target_pose_pub_ptr;
ros::Publisher *cmd_vel_pub_ptr;

int follow_enable = 0;

char getch()
{
    fd_set set;
    struct timeval timeout;
    int rv;
    char buff = 0;
    int len = 1;
    int filedesc = 0;
    FD_ZERO(&set);
    FD_SET(filedesc, &set);

    timeout.tv_sec = 0;
    timeout.tv_usec = 1000;

    rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

    struct termios old;
    if (tcgetattr(filedesc, &old) < 0)
        ROS_ERROR("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(filedesc, TCSANOW, &old) < 0)
        ROS_ERROR("tcsetattr ICANON");

    if(rv == -1)
        ROS_ERROR("select");
    else if(rv == 0)
        ROS_DEBUG("no_key_pressed");
    else
        read(filedesc, &buff, len );

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
        ROS_ERROR ("tcsetattr ~ICANON");
    return (buff);
}

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
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true, true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
	ss >> boolalpha >> igb.do_rectify;

    // Load settings related to stereo calibration
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);

    if(igb.do_rectify)
    {      
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

    //get path from tum trajectory.txt
    fsSettings["Follow.enable"] >> follow_enable;
    if(follow_enable){
        string path_file;
        path_file = (string)fsSettings["Follow.file"];
        int i = get_path_from_tum_trajectory(path_file);
        if(i < 0){
            printf("please check FrameTrajectory_TUM_Format.txt\n");
            exit(-1);
        }
    }

    odom_pub_ptr = new ros::Publisher(nh.advertise<nav_msgs::Odometry>("odom", 50));
    odom_broadcaster_ptr = new tf::TransformBroadcaster;

    plan_pub_ptr = new ros::Publisher(nh.advertise<nav_msgs::Path>("plan", 1));
    cmd_vel_pub_ptr = new ros::Publisher(nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1));
    current_pose_pub_ptr = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("current_pose", 1));
    local_target_pose_pub_ptr = new ros::Publisher(nh.advertise<visualization_msgs::Marker>("local_target", 1));

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
    tf_listener_ptr = new tf::TransformListener;


    ROS_INFO("INIT SUCCESS");

    ros::spin();
//    while(ros::ok()){
//      ros::spinOnce();
//      int c = getch();
//      if(c == 'q'){
//        break;
//      }
//    }

    // Stop all threads
    ROS_INFO("shuting down orbslam");
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    if(!follow_enable){
        SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    }
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

int get_path_from_tum_trajectory(const string &filename){
    ifstream f;
    f.open(filename.c_str());
    if(!f.good()){
        printf("trajectory file name not correct!\n");
        return -1;
    }
    double time, t[3], q[4];

    plan_pose.clear();
    while(f.good()){
        int i = 0;
        f >> time;
        f >> t[i++];
        f >> t[i++];
        f >> t[i++];
        i = 0;
        f >> q[i++];
        f >> q[i++];
        f >> q[i++];
        f >> q[i++];
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "odom_camera_base";
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = t[0];
        pose.pose.position.y = t[1];
        pose.pose.position.z = t[2];
        pose.pose.orientation.x = q[0];
        pose.pose.orientation.y = q[1];
        pose.pose.orientation.z = q[2];
        pose.pose.orientation.w = q[3];
        plan_pose.push_back(pose);
        //printf("time: %f, %f,%f, %f,%f, %f,%f, %f\n", time, t[0], t[1], t[2], q[0], q[1], q[2], q[3]);
    }

    return 0;
}

void integrateAndPublish(const tf::Transform& sensor_transform, const ros::Time& timestamp)
{
	static ros::Time last_update_time_;
	static tf::Transform base_transform_pre;
    std::string base_link_frame_id_ = "/base_link";
    std::string sensor_frame_id_ = "/camera_base";

    ROS_INFO("integrate and publish");
	if (sensor_frame_id_.empty()) {
		ROS_ERROR("[odometer] update called with unknown sensor frame id!");
		return;
	}
	if (timestamp < last_update_time_) {
		ROS_WARN(
				"[odometer] saw negative time change in incoming sensor data, resetting pose.");
	}

    tf::StampedTransform base_to_sensor;
    std::string error_msg;

    if (tf_listener_ptr->canTransform(base_link_frame_id_, sensor_frame_id_, timestamp, &error_msg))
    {
    	tf_listener_ptr->lookupTransform(
          base_link_frame_id_,
          sensor_frame_id_,
		  timestamp, base_to_sensor);
    }
    else
    {
      ROS_WARN_THROTTLE(1.0, "The tf from '%s' to '%s' does not seem to be available, "
                              "will assume it as identity!",
                              base_link_frame_id_.c_str(),
                              sensor_frame_id_.c_str());
      ROS_DEBUG("Transform error: %s", error_msg.c_str());
      base_to_sensor.setIdentity();
    }
    tf::Transform base_transform = base_to_sensor * sensor_transform * base_to_sensor.inverse();

    double bs_yaw = 1.0, bs_pitch, bs_roll;
    base_to_sensor.getBasis().getEulerYPR(bs_yaw, bs_pitch, bs_roll);
//    std::cout << "base_to_sensor" << bs_yaw << ", " << bs_pitch << ", " << bs_roll << std::endl;
    std::cout << "sensor_transform: " << sensor_transform.getOrigin().getX()
    		<<", " << sensor_transform.getOrigin().getY()
			<<", " << sensor_transform.getOrigin().getZ() << std::endl;
//    std::cout << "sensor_transform: " << base_transform.getOrigin().getX()
//    		<<", " << base_transform.getOrigin().getY()
//			<<", " << base_transform.getOrigin().getZ() << std::endl;


  nav_msgs::Odometry odometry_msg;
  odometry_msg.header.stamp = timestamp;
  odometry_msg.header.frame_id = "/odom";
  odometry_msg.child_frame_id = "/base_link";
  tf::poseTFToMsg(base_transform, odometry_msg.pose.pose);

  // calculate twist (not possible for first run as no delta_t can be computed)
  if (!last_update_time_.isZero())
  {
    double delta_t = (timestamp - last_update_time_).toSec();
    if (delta_t)
    {
      odometry_msg.twist.twist.linear.x = (base_transform * base_transform_pre.inverse()).getOrigin().getX() / delta_t;
      odometry_msg.twist.twist.linear.y = (base_transform * base_transform_pre.inverse()).getOrigin().getY() / delta_t;
      odometry_msg.twist.twist.linear.z = (base_transform * base_transform_pre.inverse()).getOrigin().getZ()  / delta_t;
      tf::Quaternion delta_rot = (base_transform * base_transform_pre.inverse()).getRotation();
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

  geometry_msgs::PoseStamped odom_pose;
  odom_pose.header.stamp = odometry_msg.header.stamp;
  odom_pose.header.frame_id = odometry_msg.header.frame_id;
  odom_pose.pose = odometry_msg.pose.pose;

  //todo: use timestamp rather than ros::Time::now()
  odom_broadcaster_ptr->sendTransform(
          tf::StampedTransform(base_transform, ros::Time::now(),
          "/odom", "/base_link"));

  last_update_time_ = timestamp;
  base_transform_pre = base_transform;

  // convert plan pose in odom_camera_base to odom
  // get current and leading path points in base_link coordinate
  std::vector<geometry_msgs::PoseStamped> poses_odom;
  float min_dist = 9999;
  int min_dist_idx = 0;
  for(unsigned int i = 0; i < plan_pose.size(); i++){
      geometry_msgs::PoseStamped pose_odom;
      tf::Pose pose_cam_tf, pose_odom_tf;
      tf::poseMsgToTF(plan_pose[i].pose, pose_cam_tf);
      pose_odom_tf = base_to_sensor * pose_cam_tf;   // pose in camera coordinate convert to pose in odom coordinate
      poseTFToMsg(pose_odom_tf,pose_odom.pose);
      poses_odom.push_back(pose_odom);

	  // get local point in poses_odom
	  float dist = pose_distance(pose_odom, odom_pose);
	  if(dist < min_dist){
		  min_dist = dist;
		  min_dist_idx = i;
	  }
  }
  // publish local pose and local target for visuallization
  visualization_msgs::Marker current_points;
  current_points.header.frame_id = "odom";
  current_points.header.stamp = ros::Time::now();
  current_points.ns = "current_pose";
  current_points.action = visualization_msgs::Marker::ADD;
  current_points.id = 0;
  current_points.type = visualization_msgs::Marker::POINTS;
  current_points.scale.x = 0.1;
  current_points.scale.y = 0.1;
  current_points.color.r = 1.0f;
  current_points.color.a = 0.8f;
  geometry_msgs::Point p;
  p.x = poses_odom[min_dist_idx].pose.position.x;
  p.y = poses_odom[min_dist_idx].pose.position.y;
  p.z = poses_odom[min_dist_idx].pose.position.z;
  current_points.points.push_back(p);
  //test-debug
//  int base_idx = min_dist_idx;
  //get 1 meters far away
  float dist = 0, dist1;
  int target_idx = 0, target_idx1 = 0;
  int found_count = 0;
  while((dist < 1)&&(found_count < 10000)){
      found_count++;
      printf("found...\n");
	  target_idx = (min_dist_idx++)%poses_odom.size();

      //check pose is in front of the robot
      tf::Pose target_pose_odom, target_pose_base;
      tf::poseMsgToTF(poses_odom[target_idx].pose, target_pose_odom);
      target_pose_base = base_transform.inverse() * target_pose_odom;   // P_in_b = T_b_b0 * P_in_b0, I published T_b0_b
      tf::Vector3 pose = target_pose_base.getOrigin();
      float heading_angle = atan2(pose.getY(), pose.getX());
      if((heading_angle > 2.0)||(pose.getX() < -1.0)){
          continue;
      }
	  dist = pose_distance(poses_odom[target_idx], odom_pose);
  }
  p.x = poses_odom[target_idx].pose.position.x;
  p.y = poses_odom[target_idx].pose.position.y;
  p.z = poses_odom[target_idx].pose.position.z;
  current_points.points.push_back(p);
  current_pose_pub_ptr->publish(current_points);

  //test-debug
//  printf("indexes: %d, %d; point1: %f, %f, point2: %f, %f\n", base_idx, target_idx,
//          current_points.points[0].x, current_points.points[0].y,
//          current_points.points[1].x, current_points.points[1].y);
//  printf("dist, dist1: %f, %f\n", dist, dist1);

  // for visualization
  nav_msgs::Path gui_path;
  gui_path.poses.resize(poses_odom.size());

  gui_path.header.frame_id = "odom";
  gui_path.header.stamp = ros::Time::now();
  gui_path.poses = poses_odom;

  //printf("path size: %ld\n", gui_path.poses.size());
  plan_pub_ptr->publish(gui_path);

  // get cmd_vel from base_link
  //std::cout << "trjectory poses min_dist_idx: " << target_idx << std::endl;
  //std::cout << "target point in odom: " << p.x << " " << p.y << " " << p.z << std::endl;
  geometry_msgs::PoseStamped leading_pose_in_base;
  tf::Pose target_pose_odom, target_pose_base;
  tf::poseMsgToTF(poses_odom[target_idx].pose, target_pose_odom);
  target_pose_base = base_transform.inverse() * target_pose_odom;   // P_in_b = T_b_b0 * P_in_b0, I published T_b0_b
  poseTFToMsg(target_pose_base,leading_pose_in_base.pose);
  leading_pose_in_base.header.frame_id = "base_link";
  leading_pose_in_base.header.stamp = poses_odom[target_idx].header.stamp;
  float yaw_base, heading_yaw, turn_yaw;
  yaw_base = tf::getYaw(odometry_msg.pose.pose.orientation);
  heading_yaw = atan2(current_points.points[1].y - current_points.points[0].y, current_points.points[1].x - current_points.points[0].x);
  turn_yaw = (heading_yaw - yaw_base);
  if(turn_yaw > M_PI){
      turn_yaw -= 2 * M_PI;
  }else if(turn_yaw < -M_PI){
      turn_yaw += 2*M_PI;
  }else{}
  ROS_INFO("leading_pose_in_base x, y: %f, %f, %f, %f, %f\n", leading_pose_in_base.pose.position.x,
          leading_pose_in_base.pose.position.y,
          yaw_base,
          heading_yaw,
          turn_yaw);
  // alturnative2: use leading pose y to turn
  if(leading_pose_in_base.pose.position.y > 0){
	  turn_yaw = 1.0;
  }else{
	  turn_yaw = -1.0;
  }
  // alturnative: use current pose to turn . tested. not work both in theory and pratice
/*  geometry_msgs::PoseStamped current_pose_in_base;
  tf::Pose current_pose_odom, current_pose_base;
  tf::poseMsgToTF(poses_odom[min_dist_idx].pose, current_pose_base);
  current_pose_base = base_transform.inverse() * current_pose_odom;
  poseTFToMsg(current_pose_base, current_pose_in_base.pose);
  current_pose_in_base.header.frame_id = "base_link";
  current_pose_in_base.header.stamp = poses_odom[min_dist_idx].header.stamp;
  if(current_pose_in_base.pose.position.y > 0 ){
	  turn_yaw = 1;
  }else{
	  turn_yaw = -1;
  }*/

  //test-debug
//  static int test = 0;
//  if(turn_yaw > 1.3){
//      test++;
//      if(test > 6){
//          exit(-2);
//      }
//  }
  //publish cmd_vel with position.y
  geometry_msgs::Twist msg;
  msg.linear.x = 1.0;
  msg.angular.z = turn_yaw;
  ROS_INFO("OUTPUT CMD_VEL: %f, %f\n", msg.linear.x, msg.angular.z);
  //todo: add stategy

  cmd_vel_pub_ptr->publish(msg);
}

float pose_distance(const geometry_msgs::PoseStamped &pose1, const geometry_msgs::PoseStamped &pose2){
	float dx, dy, dz;
	dx = pose1.pose.position.x - pose2.pose.position.x;
	dy = pose1.pose.position.y - pose2.pose.position.y;
	dz = pose1.pose.position.z - pose2.pose.position.z;
	return hypot(hypot(dx, dy), dz);
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    ros::Time now = ros::Time::now();
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

    cv::Mat T_C_C0_opencv, T_C0_C_opencv;
    if(do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
//        printf("before orbslam track\n");
        T_C0_C_opencv = mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
//        printf("after orbslam track\n");
    }
    else
    {
    	T_C0_C_opencv = mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
    }
//    ROS_INFO("ORBSLAM");
    // If tracking successfull
    if (!T_C0_C_opencv.empty()) {
    	T_C_C0_opencv = T_C0_C_opencv.inv();
        static int counter = 0;
        counter++;
        geometry_msgs::Vector3 xyz_p, xyz_v_l, xyz_v_a;
        geometry_msgs::Quaternion odom_quat;

        tf::Matrix3x3 rot_mat(
        		T_C_C0_opencv.at<float>(0,0), T_C_C0_opencv.at<float>(0,1), T_C_C0_opencv.at<float>(0,2),
				T_C_C0_opencv.at<float>(1,0), T_C_C0_opencv.at<float>(1,1), T_C_C0_opencv.at<float>(1,2),
				T_C_C0_opencv.at<float>(2,0), T_C_C0_opencv.at<float>(2,1), T_C_C0_opencv.at<float>(2,2));
        tf::Quaternion q;
        rot_mat.getRotation(q);
        odom_quat.x = q.getX();
        odom_quat.y = q.getY();
        odom_quat.z = q.getZ();
        odom_quat.w = q.getW();

        xyz_p.x = T_C_C0_opencv.at<float>(0,3);
        xyz_p.y = T_C_C0_opencv.at<float>(1,3);
        xyz_p.z = T_C_C0_opencv.at<float>(2,3);

#if 0
        std::cout << "T_C_C0_opencv" << std::endl;
        std::cout << T_C_C0_opencv << std::endl;

        std::cout << "quaternion:" << std::endl;
        std::cout << odom_quat << std::endl;
        std::cout << "xyz position: " << std::endl << xyz_p << std::endl;
#endif

        tf::Vector3 t(T_C_C0_opencv.at<float>(0,3), T_C_C0_opencv.at<float>(1,3), T_C_C0_opencv.at<float>(2,3));
        tf::Transform camera_transform(rot_mat, t);

        //printf("time: %d, %d\n", cv_ptrLeft->header.stamp.toSec(), msgLeft->header.stamp.sec);
        if(follow_enable){
            integrateAndPublish(camera_transform, now);
        }
    }
}


