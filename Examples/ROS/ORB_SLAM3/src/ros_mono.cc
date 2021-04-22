/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include<opencv2/core/core.hpp>

#include"../../../include/System.h"

#include "../../../include/Converter.h"

using namespace std;

static ros::Publisher voPub;
static ros::Publisher odomPub;

/* Global path vector */
nav_msgs::Path voPath;
nav_msgs::Odometry lastOdom;
geometry_msgs::Pose lastPose;

/* Projection matrix */
cv::Mat H = cv::Mat::eye(4, 4, CV_32F);

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System* pSLAM):mpSLAM(pSLAM){}

    void GrabImage(const sensor_msgs::ImageConstPtr& msg);

    ORB_SLAM3::System* mpSLAM;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "Mono");
    ros::start();

    if(argc != 4)
    {
        cerr << endl << "Usage: rosrun ORB_SLAM3 Mono path_to_vocabulary path_to_settings output_name" << endl;        
        ros::shutdown();
        return 1;
    }

    // Initialize last pose to (0,0,0)
    lastPose.position.x = 0.0;
    lastPose.position.y = 0.0;
    lastPose.position.z = 0.0;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR,true);

    ImageGrabber igb(&SLAM);

    ros::NodeHandle nodeHandler;

    ros::Subscriber sub = nodeHandler.subscribe("/camera_array/cam2/image_raw", 1, &ImageGrabber::GrabImage,&igb);

    voPub = nodeHandler.advertise<nav_msgs::Path>("/vo/path", 1000);

    odomPub = nodeHandler.advertise<nav_msgs::Odometry>("/odometry/vo", 1000);

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    // SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    SLAM.SaveTrajectoryTUM(f_file);
    SLAM.SaveKeyFrameTrajectoryTUM(kf_file);

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabImage(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat camPose;
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat mTcw = mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec());

    cout << "camPose = " << mTcw << endl;

    if (!mTcw.empty())
    {

        cv::Mat mRcw = mTcw.rowRange(0,3).colRange(0,3);

        cv::Mat mtcw = mTcw.rowRange(0,3).col(3);

        nav_msgs::Odometry odom;
        geometry_msgs::PoseStamped poseStamped;
        geometry_msgs::Pose pose;
        
        odom.header = msg->header;
        poseStamped.header = msg->header;

        lastPose.position.x = mtcw.at<float>(0, 0);
        lastPose.position.y = mtcw.at<float>(1, 0);
        lastPose.position.z = mtcw.at<float>(2, 0);
        pose.position = lastPose.position;

        vector<float> q = ORB_SLAM3::Converter::toQuaternion(mRcw);

        pose.orientation.x = q[0];
        pose.orientation.y = q[1];
        pose.orientation.z = q[2];
        pose.orientation.w = q[3];

        // cout << "R = " << camPose.rowRange(0,3).colRange(0,3) << ", T = " << trans << endl;

        poseStamped.pose = pose;

        odom.pose.pose = pose;

        voPath.poses.push_back(poseStamped);
        voPub.publish(voPath);
        odomPub.publish(odom);
        lastOdom = odom;
    } else {
        // Tracking lost, so continue to publish previous message
        voPub.publish(voPath);
        odomPub.publish(lastOdom);
    }
}