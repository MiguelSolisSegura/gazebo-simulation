#include "ros/publisher.h"
#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h"
#include "ros/subscriber.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>


class RotateService
{
public:
    RotateService(){
        // Initialize the service server
        serviceServer_ = nh_.advertiseService("/rotate_robot",
                                              &RotateService::handleRequest,
                                              this);
        // Initialize the subscriber
        sub_ = nh_.subscribe("/odom", 1000, &RotateService::odomCallback, this);

        // Initialize the publisher
        pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        ROS_INFO("Service server /rotate_robot started.");
    }

    bool handleRequest(my_rb1_ros::Rotate::Request &req,
                       my_rb1_ros::Rotate::Response &res) {
        current_angle_ = quaternionZToDegrees(z_, w_);
        while (current_angle_ != req.degrees) {
            ROS_DEBUG("Current angle: %d", current_angle_);
            cmd_.angular.z = (req.degrees > 0) ? 0.25 : -0.25;        
            pub_.publish(cmd_);
            ros::spinOnce();
            current_angle_ = quaternionZToDegrees(z_, w_);
        }

        res.result = "SUCCESSFUL";
        cmd_.angular.z = 0.0;
        pub_.publish(cmd_);

        return true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
        w_ = msg->pose.pose.orientation.w;
        z_ = msg->pose.pose.orientation.z;
    }

    int quaternionZToDegrees(double z, double w) {
        // Calculate the rotation angle in radians
        double theta = 2 * std::atan2(z, w);
        // Convert the angle to degrees
        int angle = theta * (180.0 / M_PI);
        return angle;
    }


private:
    ros::NodeHandle nh_;
    ros::ServiceServer serviceServer_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    float z_;
    float w_;
    int current_angle_;
    geometry_msgs::Twist cmd_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_service_server");
    RotateService server;

    ros::spin();

    return 0;
}
