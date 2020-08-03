/*
 * ROS Stereo Image Synchroniser  
 * stereo_sync_node.cpp
 * 
 *
 *  __ _  _   ___ ______ ____                    _                   
 * /_ | || | / _ \____  / __ \                  | |                  
 *  | | || || (_) |  / / |  | |_   _  __ _ _ __ | |_ _   _ _ __ ___  
 *  | |__   _> _ <  / /| |  | | | | |/ _` | '_ \| __| | | | '_ ` _ \ 
 *  | |  | || (_) |/ / | |__| | |_| | (_| | | | | |_| |_| | | | | | |
 *  |_|  |_| \___//_/   \___\_\\__,_|\__,_|_| |_|\__|\__,_|_| |_| |_|
 *
 * Copyright (C) 2020 1487Quantum
 * 
 * 
 * Licensed under the MIT License.
 * 
 */

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

#include <opencv2/ximgproc.hpp>
#include <opencv2/features2d.hpp>

class stereo_sync_node {
public:
    stereo_sync_node(const ros::NodeHandle& nh_);
    bool init(); //Init
    // === CALLBACK & PUBLISHER ===
    void imgL_callback(const sensor_msgs::ImageConstPtr& imgp, const sensor_msgs::CameraInfoConstPtr& cam_info);
    void imgR_callback(const sensor_msgs::ImageConstPtr& imgp, const sensor_msgs::CameraInfoConstPtr& cam_info);

private:
    std::string topic_out; //Output topic name
    ros::NodeHandle nh; //Node handle
    // Pub/Sub
    image_transport::Publisher imgPub_L;
    image_transport::Publisher imgPub_R;
    ros::Publisher pub_info_camera_L;
    ros::Publisher pub_info_camera_R;

    image_transport::CameraSubscriber imgSubL;
    image_transport::CameraSubscriber imgSubR;

    cv::Mat leftImg;
    cv::Mat rightImg;

    sensor_msgs::CameraInfo leftTime;
    sensor_msgs::CameraInfo rightTime;

    void compareTime(const sensor_msgs::CameraInfo& left, const sensor_msgs::CameraInfo& right);
    int getMilli(const sensor_msgs::CameraInfo& cam_info, const int& accuracy, bool getFull);
    sensor_msgs::CameraInfo extractCamInfo(const sensor_msgs::CameraInfoConstPtr& cptr);
};

stereo_sync_node::stereo_sync_node(const ros::NodeHandle& nh_)
{
    this->nh = nh_;
};

bool stereo_sync_node::init()
{
    // Pub-Sub
    image_transport::ImageTransport it_L(this->nh);
    image_transport::ImageTransport it_R(this->nh);
    imgSubL = it_L.subscribeCamera("/left/image_raw", 1, &stereo_sync_node::imgL_callback, this);
    imgSubR = it_R.subscribeCamera("/right/image_raw", 1, &stereo_sync_node::imgR_callback, this);

    imgPub_L = it_L.advertise("/left_sync/image_raw", 1);
    imgPub_R = it_R.advertise("/right_sync/image_raw", 1);
    pub_info_camera_L = this->nh.advertise<sensor_msgs::CameraInfo>("/left_sync/camera_info", 1);
    pub_info_camera_R = this->nh.advertise<sensor_msgs::CameraInfo>("/right_sync/camera_info", 1);

    return true;
}

//larger accuracy int->higher framerate, default 1e4
int stereo_sync_node::getMilli(const sensor_msgs::CameraInfo& cam_info, const int& accuracy, bool getFull)
{
    boost::posix_time::ptime tmp_posix_time = cam_info.header.stamp.toBoost();
    //std::string iso_time_str = boost::posix_time::to_iso_extended_string(tmp_posix_time);
    //ROS_WARN_STREAM("R: "+iso_time_str+ "\tSC: "+std::to_string(t_seconds));
    return tmp_posix_time.time_of_day().fractional_seconds() / (getFull ? 1 : accuracy); //Get full would not divide for accuracy comparision
}

sensor_msgs::CameraInfo stereo_sync_node::extractCamInfo(const sensor_msgs::CameraInfoConstPtr& cptr)
{
    sensor_msgs::CameraInfo tmpInfo;
    tmpInfo.header = cptr->header;
    tmpInfo.height = cptr->height;
    tmpInfo.width = cptr->width;
    tmpInfo.distortion_model = cptr->distortion_model;
    tmpInfo.D = cptr->D;
    tmpInfo.K = cptr->K;
    tmpInfo.R = cptr->R;
    tmpInfo.P = cptr->P;
    tmpInfo.binning_x = cptr->binning_x;
    tmpInfo.binning_y = cptr->binning_y;
    tmpInfo.roi = cptr->roi;
    return tmpInfo;
}

void stereo_sync_node::compareTime(const sensor_msgs::CameraInfo& left, const sensor_msgs::CameraInfo& right)
{
    int accuracy = 1e4;
    int maxDiff{ 2000 };
    int leftSec{ getMilli(left, accuracy, 1) };
    int rightSec{ getMilli(right, accuracy, 1) };
    int offset{ leftSec - rightSec };
    if (offset < 0)
        offset *= -1;
    if ((leftSec / accuracy == rightSec / accuracy) && offset <= maxDiff) {
        ROS_WARN_STREAM("Time sync: " + std::to_string(leftSec / accuracy) + "\tOffset: " + std::to_string(offset));

        cv::Mat iL = this->leftImg;
        cv::Mat iR = this->rightImg;

        //Publish result
        imgPub_L.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", iL).toImageMsg());
        imgPub_R.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", iR).toImageMsg());

        //Set header stamp
        ros::Time timeNow = ros::Time::now();
        this->leftTime.header.stamp = timeNow;
        this->rightTime.header.stamp = timeNow;

        pub_info_camera_L.publish(this->leftTime);
        pub_info_camera_R.publish(this->rightTime);

        /*
        cv::Size sizeL(640, 480); //the dst image size,e.g.100x100
        cv::Size sizeR(640, 480); //the dst image size,e.g.100x100
        cv::Mat dstL; //dst image
        cv::Mat dstR; //dst image
        cv::Mat mf;
        cv::resize(iL, iL, sizeL, 0, 0, cv::INTER_AREA); //resize image
        cv::resize(iR, iR, sizeR, 0, 0, cv::INTER_AREA); //resize image
        // cv::hconcat(dstL,dstR,mf);
        cv::namedWindow("22", cv::WINDOW_NORMAL);
        cv::imshow("22", iL);

        cv::waitKey(1);
*/
    }
}

// === CALLBACK & PUBLISHER ===
void stereo_sync_node::imgL_callback(const sensor_msgs::ImageConstPtr& imgp, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    try {
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR8) };

        this->leftImg = imagePtrRaw->image;
        this->leftTime = extractCamInfo(cam_info);
        compareTime(this->leftTime, this->rightTime);

        //ROS_WARN_STREAM("L: "+iso_time_str+ "\tSC: "+std::to_string(t_seconds));

        /*
       
        //Publish result
        imgPub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());

        sensor_msgs::CameraInfo info_camera;
        info_camera.header.stamp = ros::Time::now();
        info_camera.width = 2 * (cirRad + borderOffset.x);
        info_camera.height = frame.rows + 2 * borderOffset.y;
        pub_info_camera.publish(info_camera);

*/
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgp->encoding.c_str());
    }
}

void stereo_sync_node::imgR_callback(const sensor_msgs::ImageConstPtr& imgp, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    try {
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR8) };

        this->rightImg = imagePtrRaw->image;
        this->rightTime = extractCamInfo(cam_info);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imgp->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stereo_sync_node");
    ros::NodeHandle nh;

    stereo_sync_node st_sync(nh);
    ROS_WARN("Starting...");
    if (!st_sync.init()) {
        ROS_WARN("Exiting...");
        ros::shutdown();
        return -1;
    }

    ros::spin();

    return 0;
}
