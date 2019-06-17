//
// Created by askat on 5/15/19.
//

#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <opencv2/aruco.hpp>
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <chrono>
#include <iostream>
#include "ros_aruco.h"

int ArUcoNode::arucoDetect() {
    cv::VideoCapture cap(0); // open the default camera
    if(!cap.isOpened()) {
        ROS_WARN("%s", "Camera is not reachable");
        return -1;
    }
    // set camera parameters
    cap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,height);
    cap.set(CV_CAP_PROP_FPS,fps);
    cap.set(CV_CAP_PROP_FOURCC ,CV_FOURCC('M', 'J', 'P', 'G') );
    cv::namedWindow("window",1);

    while(cap.grab()){
        //std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
        //std::cout<<"1 "<<ros::Time::now()<<std::endl;
        cv::Mat frame, image;
        cap >> image; // get a new frame from camera
        // sharpen the frame
        //cv::GaussianBlur(frame, image, cv::Size(0, 0), 5);
        //cv::addWeighted(frame, 1.5, image, -0.5, 0, image);

        fiducial_msgs::FiducialTransformArray fta;
        fta.header.stamp = ros::Time::now();
        image_seq = image_seq + 1;
        fta.image_seq = image_seq;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        // detect markers
        cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters);
        // if at least one marker detected
        if (!ids.empty()) {
            // draw detected markers
            cv::aruco::drawDetectedMarkers(image, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            // estimate pose of each marker
            cv::aruco::estimatePoseSingleMarkers(corners, aruco_len, cameraMatrix, distCoeffs, rvecs, tvecs);
            // draw axis for each marker
            for(int i=0; i<ids.size(); ++i){
                double angle = norm(rvecs[i]);
                cv::Vec3d axis = rvecs[i] / angle;

                fiducial_msgs::FiducialTransform ft;
                ft.fiducial_id = ids[i];

                ft.transform.translation.x = tvecs[i][0];
                ft.transform.translation.y = tvecs[i][1];
                ft.transform.translation.z = tvecs[i][2];

                tf2::Quaternion q;
                q.setRotation(tf2::Vector3(axis[0], axis[1], axis[2]), angle);

                ft.transform.rotation.w = q.w();
                ft.transform.rotation.x = q.x();
                ft.transform.rotation.y = q.y();
                ft.transform.rotation.z = q.z();

                fta.transforms.emplace_back(ft);
                cv::aruco::drawAxis(image, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], axis_len);
            }
            pose_pub.publish(fta);
        }
        //std::cout<<"2 "<<ros::Time::now()<<std::endl;
        cv::imshow("window", image);
        cv::waitKey(1);
        /*std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(t2-t1);
        std::cout<<time_span.count() <<std::endl;*/
    }
}

ArUcoNode::ArUcoNode():n_h_("") {
    // set camera parameters
    cameraMatrix = (cv::Mat_<float>(3,3) << 1161.662745, 0.000000, 779.138273, 0.000000, 1163.671965, 447.123263, 0.000000, 0.000000, 1.000000);
    distCoeffs = (cv::Mat_<float>(1,5) << 0.109948, -0.176981, -0.000347, -0.000009, 0.000000);
    width=1600, height=896, fps=30;

    // set marker parameters
    dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
    aruco_len = 0.07; axis_len = 0.05;

    // set detector parameters
    parameters = new cv::aruco::DetectorParameters();
    n_h_.param<double>("adaptiveThreshConstant", parameters->adaptiveThreshConstant, 7);
    n_h_.param<int>("adaptiveThreshWinSizeMax", parameters->adaptiveThreshWinSizeMax, 21);
    n_h_.param<int>("adaptiveThreshWinSizeMin", parameters->adaptiveThreshWinSizeMin, 3);
    n_h_.param<int>("adaptiveThreshWinSizeStep", parameters->adaptiveThreshWinSizeStep, 4);
    n_h_.param<int>("cornerRefinementMaxIterations", parameters->cornerRefinementMaxIterations, 30);
    n_h_.param<double>("cornerRefinementMinAccuracy", parameters->cornerRefinementMinAccuracy, 0.01);
    n_h_.param<int>("cornerRefinementWinSize", parameters->cornerRefinementWinSize, 3);
#if OPENCV_MINOR_VERSION==2
    nh.param<bool>("doCornerRefinement",detectorParams->doCornerRefinement, true);
#else
    bool doCornerRefinement = true;
    n_h_.param<bool>("doCornerRefinement", doCornerRefinement, true);
    if (doCornerRefinement) {
        bool cornerRefinementSubPix = true;
        n_h_.param<bool>("cornerRefinementSubPix", cornerRefinementSubPix, true);
        if (cornerRefinementSubPix) {
            parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
        }
        else {
            parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
        }
    }
    else {
        parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_NONE;
    }
#endif
    n_h_.param<double>("errorCorrectionRate", parameters->errorCorrectionRate , 0.6);
    n_h_.param<double>("minCornerDistanceRate", parameters->minCornerDistanceRate , 0.05);
    n_h_.param<int>("markerBorderBits", parameters->markerBorderBits, 1);
    n_h_.param<double>("maxErroneousBitsInBorderRate", parameters->maxErroneousBitsInBorderRate, 0.04);

    n_h_.param<int>("minDistanceToBorder", parameters->minDistanceToBorder, 1);
    n_h_.param<double>("minMarkerDistanceRate", parameters->minMarkerDistanceRate, 0.05);
    n_h_.param<double>("minMarkerPerimeterRate", parameters->minMarkerPerimeterRate, 0.03); // default 0.3
    n_h_.param<double>("maxMarkerPerimeterRate", parameters->maxMarkerPerimeterRate, 4.0);
    n_h_.param<double>("minOtsuStdDev", parameters->minOtsuStdDev, 5.0);
    n_h_.param<double>("perspectiveRemoveIgnoredMarginPerCell", parameters->perspectiveRemoveIgnoredMarginPerCell, 0.13);
    n_h_.param<int>("perspectiveRemovePixelPerCell", parameters->perspectiveRemovePixelPerCell, 4);
    n_h_.param<double>("polygonalApproxAccuracyRate", parameters->polygonalApproxAccuracyRate, 0.04);

    // ROS publsiher parameters
    pose_pub = n_h_.advertise<fiducial_msgs::FiducialTransformArray>("/aruco_poses", 1);
    image_seq = 0;

    arucoDetect();
}
