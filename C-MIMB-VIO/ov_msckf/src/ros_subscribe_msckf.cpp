/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2019 Patrick Geneva
 * Copyright (C) 2019 Kevin Eckenhoff
 * Copyright (C) 2019 Guoquan Huang
 * Copyright (C) 2019 OpenVINS Contributors
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CompressedImage.h>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "core/RosVisualizer.h"
#include "utils/dataset_reader.h"
#include "utils/parse_ros.h"

#include "std_msgs/Bool.h"


using namespace ov_msckf;


VioManager* sys;
RosVisualizer* viz_;


// Buffer data
double time_buffer = -1;
cv::Mat img0_buffer, img1_buffer,img2_buffer, img3_buffer;

// Callback functions
void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg);
void callback_control(const std_msgs::Bool::ConstPtr& msg);
void callback_monocular(const sensor_msgs::ImageConstPtr& msg0);
void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1);
void callback_four(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1, \
                    const sensor_msgs::ImageConstPtr& msg2, const sensor_msgs::ImageConstPtr& msg3);

void callback_cam(const sensor_msgs::ImageConstPtr& msg);

// Main function
int main(int argc, char** argv) {

    // Launch our ros node
    ros::init(argc, argv, "run_subscribe_msckf");
    ros::NodeHandle nh("~");

    // Create our VIO system
    VioManagerOptions params = parse_ros_nodehandler(nh);
    sys = new VioManager(params);
    viz_ = new RosVisualizer(nh, sys);

    //load posegraph
   if(params.use_prior_map)
   {
    if (!params.use_robust_match)
       sys->loadPoseGraph_multimap_visualization(params.map_save_path,params.keyframe_pose_filenames);
    else
        sys->loadPoseGraph_multimap(params.map_save_path,params.pose_graph_filenames,params.keyframe_pose_filenames,params.query_pose_filenames);
   }
   cout<<"finish load Posegraph "<<endl;


    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Our camera topics (left and right stereo)
    std::string topic_imu;
    std::string topic_camera0, topic_camera1, topic_camera2, topic_camera3;
    std::string topic_control;
    nh.param<std::string>("topic_imu", topic_imu, "/imu0");
    nh.param<std::string>("topic_camera0", topic_camera0, "/cam0/image_raw");
    nh.param<std::string>("topic_camera1", topic_camera1, "/cam1/image_raw");
    nh.param<std::string>("topic_camera2", topic_camera2, "/cam2/image_raw");
    nh.param<std::string>("topic_camera3", topic_camera3, "/cam3/image_raw");

    nh.param<std::string>("topic_control", topic_control, "/control_signal");

    // Logic for sync stereo subscriber
    // https://answers.ros.org/question/96346/subscribe-to-two-image_raws-with-one-function/?answer=96491#post-id-96491
    message_filters::Subscriber<sensor_msgs::Image> image_sub0(nh,topic_camera0.c_str(),1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub1(nh,topic_camera1.c_str(),1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub2(nh,topic_camera2.c_str(),1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub3(nh,topic_camera3.c_str(),1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
        message_filters::Synchronizer<sync_pol> sync(sync_pol(5), image_sub0,image_sub1, image_sub2,image_sub3);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol_stereo;
        message_filters::Synchronizer<sync_pol_stereo> sync_stereo(sync_pol_stereo(5), image_sub0,image_sub1);
    ROS_WARN("params.state_options.num_cameras is %d",params.state_options.num_cameras);

    // Create subscribers
    ros::Subscriber subimu = nh.subscribe(topic_imu.c_str(), 9999, callback_inertial);
    ros::Subscriber subcontrol = nh.subscribe(topic_control.c_str(),1,callback_control);
    ros::Subscriber subcam;
    if(params.state_options.num_cameras == 1) {
        ROS_INFO("subscribing to: %s", topic_camera0.c_str());
        subcam = nh.subscribe(topic_camera0.c_str(), 1, callback_monocular);
    } else if(params.state_options.num_cameras == 2) {
        ROS_INFO("subscribing to: %s", topic_camera0.c_str());
        ROS_INFO("subscribing to: %s", topic_camera1.c_str());
        sync_stereo.registerCallback(boost::bind(&callback_stereo, _1, _2));

    }else if(params.state_options.num_cameras == 4)  {
       
        ROS_INFO("subscribing to: %s", topic_camera0.c_str());
        ROS_INFO("subscribing to: %s", topic_camera1.c_str());
        ROS_INFO("subscribing to: %s", topic_camera2.c_str());
        ROS_INFO("subscribing to: %s", topic_camera3.c_str());
        sync.registerCallback(boost::bind(&callback_four, _1, _2, _3, _4));
    } 
    else {
        ROS_ERROR("INVALID MAX CAMERAS SELECTED!!!");
        std::exit(EXIT_FAILURE);
    }

    //===================================================================================
    //===================================================================================
    //===================================================================================

    // Spin off to ROS
    ROS_INFO("done...spinning to ros");
    ros::spin();

    // Final visualization
    viz_->visualize_final();

    // Finally delete our system
    delete sys;
    delete viz_;


    // Done!
    return EXIT_SUCCESS;


}

void callback_cam(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
     try {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cout.precision(15);
    cout<<"time : "<<cv_ptr->header.stamp.toSec()<<endl;
}


void callback_inertial(const sensor_msgs::Imu::ConstPtr& msg) {

    // convert into correct format
    double timem = msg->header.stamp.toSec();
    Eigen::Vector3d wm, am;
    wm << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
    am << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;

    // send it to our VIO system
    sys->feed_measurement_imu(timem, wm, am);
    viz_->visualize_odometry(timem);

}

void callback_control(const std_msgs::Bool::ConstPtr& msg){
    sys->feed_measurement_control(msg->data);
}



void callback_monocular(const sensor_msgs::ImageConstPtr& msg0) {

    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Fill our buffer if we have not
    if(img0_buffer.rows == 0) {
        time_buffer = cv_ptr->header.stamp.toSec();
        img0_buffer = cv_ptr->image.clone();
        return;
    }

    // send it to our VIO system
    sys->feed_measurement_monocular(time_buffer, img0_buffer, 0);
    viz_->visualize();

    // move buffer forward
    time_buffer = cv_ptr->header.stamp.toSec();
    img0_buffer = cv_ptr->image.clone();

    cout.precision(15);
    cout<<"time : "<<cv_ptr->header.stamp.toSec()<<endl;

}



void callback_stereo(const sensor_msgs::ImageConstPtr& msg0, const sensor_msgs::ImageConstPtr& msg1) {

    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr0;
    try {
        cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get the image
    cv_bridge::CvImageConstPtr cv_ptr1;
    try {
        cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }


    // Fill our buffer if we have not
    if(img0_buffer.rows == 0 || img1_buffer.rows == 0) {
        time_buffer = cv_ptr0->header.stamp.toSec();
        img0_buffer = cv_ptr0->image.clone();
        time_buffer = cv_ptr1->header.stamp.toSec();
        img1_buffer = cv_ptr1->image.clone();
        return;
    }

    // send it to our VIO system
    cout<<"before feed_measurement_stereo"<<endl;
    sys->feed_measurement_stereo(time_buffer, img0_buffer, img1_buffer, 0, 1);
    cout<<"after feed_measurement_stere"<<endl;
    viz_->visualize();

    // move buffer forward
    time_buffer = cv_ptr0->header.stamp.toSec();
    img0_buffer = cv_ptr0->image.clone();
    time_buffer = cv_ptr1->header.stamp.toSec();
    img1_buffer = cv_ptr1->image.clone();

}

void callback_four(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1, const sensor_msgs::ImageConstPtr &msg2, \
                   const sensor_msgs::ImageConstPtr &msg3) {

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr0;
  try {
    cv_ptr0 = cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr1;
  try {
    cv_ptr1 = cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr2;
  try {
    cv_ptr2 = cv_bridge::toCvShare(msg2, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Get the image
  cv_bridge::CvImageConstPtr cv_ptr3;
  try {
    cv_ptr3 = cv_bridge::toCvShare(msg3, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Fill our buffer if we have not
  if(img0_buffer.rows == 0 || img1_buffer.rows == 0 || img2_buffer.rows == 0 || img3_buffer.rows == 0) {
    time_buffer = cv_ptr0->header.stamp.toSec();
    img0_buffer = cv_ptr0->image.clone();
    time_buffer = cv_ptr1->header.stamp.toSec();
    img1_buffer = cv_ptr1->image.clone();
    time_buffer = cv_ptr2->header.stamp.toSec();
    img2_buffer = cv_ptr2->image.clone();
    time_buffer = cv_ptr3->header.stamp.toSec();
    img3_buffer = cv_ptr3->image.clone();
    return;
   }


  // send it to our VIO system
  sys->feed_measurement_four(time_buffer, img0_buffer, img1_buffer,img2_buffer, img3_buffer, 0, 1, 2, 3);
  viz_->visualize();
  
  // move buffer forward
  time_buffer = cv_ptr0->header.stamp.toSec();
  img0_buffer = cv_ptr0->image.clone();
  time_buffer = cv_ptr1->header.stamp.toSec();
  img1_buffer = cv_ptr1->image.clone();
  time_buffer = cv_ptr2->header.stamp.toSec();
  img2_buffer = cv_ptr2->image.clone();
  time_buffer = cv_ptr3->header.stamp.toSec();
  img3_buffer = cv_ptr3->image.clone();
}

















