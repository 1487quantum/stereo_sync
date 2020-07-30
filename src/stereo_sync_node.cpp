//#include "masker_util/stereo_sync_node.hpp"

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
    image_transport::Publisher imgPub;
    ros::Publisher pub_info_camera;

    image_transport::CameraSubscriber imgSubL;
    image_transport::CameraSubscriber imgSubR;

   cv::Mat leftImg;
    cv::Mat rightImg;

    sensor_msgs::CameraInfoConstPtr leftTime;
    sensor_msgs::CameraInfoConstPtr rightTime;

    void compareTime(const sensor_msgs::CameraInfoConstPtr& left, const sensor_msgs::CameraInfoConstPtr& right);
    int getMilli(const sensor_msgs::CameraInfoConstPtr& cam_info, const int& accuracy, bool getFull);
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

    // imgPub = it.advertise(this->topic_out + "/image_raw", 1);
    // pub_info_camera = this->nh.advertise<sensor_msgs::CameraInfo>(this->topic_out + "/camera_info", 1);

    return true;
}

//larger accuracy int->higher framerate, default 1e4
int stereo_sync_node::getMilli(const sensor_msgs::CameraInfoConstPtr& cam_info, const int& accuracy, bool getFull)
{
    boost::posix_time::ptime tmp_posix_time = cam_info->header.stamp.toBoost();
    //std::string iso_time_str = boost::posix_time::to_iso_extended_string(tmp_posix_time);
    //ROS_WARN_STREAM("R: "+iso_time_str+ "\tSC: "+std::to_string(t_seconds));
    return tmp_posix_time.time_of_day().fractional_seconds() / (getFull ? 1 : accuracy); //Get full would not divide for accuracy comparision
}

void stereo_sync_node::compareTime(const sensor_msgs::CameraInfoConstPtr& left, const sensor_msgs::CameraInfoConstPtr& right)
{
    int accuracy = 1e4;
    int maxDiff{ 2500 };
    int leftSec{ getMilli(left, accuracy, 1) };
    int rightSec{ getMilli(right, accuracy, 1) };
    int offset{ leftSec - rightSec };
    if (offset < 0)
        offset *= -1;
    if ((leftSec / accuracy == rightSec / accuracy) && offset <= maxDiff){
        ROS_WARN_STREAM("Time sync: " + std::to_string(leftSec / accuracy) + "\tOffset: " + std::to_string(offset));
 cv::namedWindow("Original", cv::WINDOW_NORMAL);
cv::Mat iL = this->leftImg;
cv::Mat iR = this->rightImg;
 //cv::hconcat(iL,iR,iL);
        cv::imshow("Original", iL);
        cv::waitKey(1);
}
}

// === CALLBACK & PUBLISHER ===
void stereo_sync_node::imgL_callback(const sensor_msgs::ImageConstPtr& imgp, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
    try {
        cv_bridge::CvImagePtr imagePtrRaw{ cv_bridge::toCvCopy(imgp, sensor_msgs::image_encodings::BGR8) };

	this->leftImg = imagePtrRaw->image;
        this->leftTime = cam_info;
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
        this->rightTime = cam_info;

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

