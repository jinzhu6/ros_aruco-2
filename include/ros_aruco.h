//
// Created by askat on 5/15/19.
//

#ifndef ROS_ARUCO_ROS_ARUCO_H
#define ROS_ARUCO_ROS_ARUCO_H

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include <iostream>


class ArUcoNode{
private:
    /*camera calibration parameters */
    cv::Mat cameraMatrix; // camera matrix
    cv::Mat distCoeffs; // distortion coefficients
    int width, height, fps; // camera parameters

    /*aruco module parameters*/
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> parameters; // detector parameters
    double aruco_len; //size of the marker (meter)
    float axis_len; //drawn axis length

    // ROS
    ros::NodeHandle n_h_; // ROS NodeHandle
    ros::Publisher pose_pub; // ROS topic publisher
    int32_t image_seq;


public:
    ArUcoNode();        // constructor
    int arucoDetect();

};

#endif //ROS_ARUCO_ROS_ARUCO_H
