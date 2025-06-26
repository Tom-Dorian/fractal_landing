/*
 * This file is part of the aruco_fractal_tracker distribution (https://github.com/dimianx/aruco_fractal_tracker).
 * Copyright (c) 2024-2025 Dmitry Anikin <dmitry.anikin@proton.me>.
 *
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "aruco_fractal_tracker/aruco_fractal_tracker_node.hpp"

#include <stdexcept>
#include <string>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

namespace fractal_tracker
{
ArucoFractalTracker::ArucoFractalTracker(const rclcpp::NodeOptions & options)
:Node("aruco_fractal_tracker", options)
{
  
  detector_.setConfiguration("FRACTAL_3L_6");

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10, std::bind(&ArucoFractalTracker::imageCallback, this, std::placeholders::_1));

  image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("Aruco_image", 10);

  marker_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/arucoPose", 10);
  cv::Size image_size(1920, 1080);
  cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
    1000.0, 0,  960.0,
    0,  1000.0, 540.0,
    0,  0,  1.0);
  cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << 
    0.0, 0.0, 0.0, 0.0, 0.0); // Default distortion coefficients
  
  // cv::Size image_size(640, 320);
  // cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) <<
  //   554.3827, 0,  320.0,
  //   0,  554.3827, 160.0,
  //   0,  0,  1.0);
  // cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << 
  //   0.0, 0.0, 0.0, 0.0, 0.0); // Default distortion coefficients
  aruco::CameraParameters cam_params;
  cam_params.setParams(camera_matrix, dist_coeffs, image_size);
  detector_.setParams(cam_params, .75); // Set marker size to 0.75 meters
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
}


void ArucoFractalTracker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat gray;

  try 
  {
    cv_ptr = cv_bridge::toCvCopy(msg);
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }

  cv::cvtColor(cv_ptr->image, gray, CV_BGR2GRAY);

  if (detector_.detect(gray))
  {
    detector_.drawMarkers(cv_ptr->image);
    
    std::vector<aruco::Marker> markers = detector_.getMarkers();
    for (auto&& marker : markers)
      marker.draw(cv_ptr->image, cv::Scalar(255, 255, 255), 2);

    detector_.draw2d(cv_ptr->image);

    if (detector_.poseEstimation())
    {
      cv::Mat tvec = detector_.getTvec();
      cv::Mat rvec = detector_.getRvec();
      detector_.draw3d(cv_ptr->image);
      
      cv::Mat rmatrix;
      cv::Rodrigues(rvec, rmatrix);
      tf2::Matrix3x3 tf2_rot(rmatrix.at<double>(0, 0), rmatrix.at<double>(0, 1), rmatrix.at<double>(0, 2),
                             rmatrix.at<double>(1, 0), rmatrix.at<double>(1, 1), rmatrix.at<double>(1, 2),
                             rmatrix.at<double>(2, 0), rmatrix.at<double>(2, 1), rmatrix.at<double>(2, 2));
        
      tf2::Vector3 tf2_translation(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0));
      tf2::Transform tf2_transform(tf2_rot, tf2_translation);
      tf2::Quaternion quat;
      tf2_rot.getRotation(quat);

      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = msg->header.frame_id;
      pose.header.stamp = this->get_clock()->now();
      pose.pose.position.x = tf2_translation.x();
      pose.pose.position.y = tf2_translation.y();
      pose.pose.position.z = tf2_translation.z();
      pose.pose.orientation.x = quat.getX();
      pose.pose.orientation.y = quat.getY();
      pose.pose.orientation.z = quat.getZ();
      pose.pose.orientation.w = quat.getW();

      RCLCPP_INFO_STREAM(this->get_logger(), "Marker detected at position: "
        << pose.pose.position.x << ", " << pose.pose.position.y << ", " << pose.pose.position.z);
      marker_pose_pub_->publish(pose);

      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = this->get_clock()->now();
      transform.header.frame_id = "simple_drone/bottom_cam_link";
      transform.child_frame_id = "marker_frame";
      transform.transform.translation.x = tf2_translation.x();
      transform.transform.translation.y = tf2_translation.y();
      transform.transform.translation.z = tf2_translation.z();
      transform.transform.rotation.x = quat.getX();
      transform.transform.rotation.y = quat.getY();
      transform.transform.rotation.z = quat.getZ();
      transform.transform.rotation.w = quat.getW();

      tf_broadcaster_->sendTransform(transform);
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(this->get_logger(), "No markers detected in the image.");
  }

  try 
  {
    image_pub_->publish(*cv_ptr->toImageMsg());
  } 
  catch (cv_bridge::Exception& e) 
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "cv_bridge exception: " << e.what());
    return;
  }
}

} // namespace fractal_tracker

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fractal_tracker::ArucoFractalTracker)
