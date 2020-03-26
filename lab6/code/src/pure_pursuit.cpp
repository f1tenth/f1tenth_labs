#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
// TODO: include ROS msg type headers and libraries you need

class PurePursuit {
private:
    ros::NodeHandle n;
    // TODO: create ROS subscribers and publishers

public:
    Safety() {
        n = ros::NodeHandle();

        // TODO: create ROS subscribers and publishers
        
    }

    void pose_callback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg) {
        // TODO: find the current waypoint to track using methods mentioned in lecture

        // TODO: transform goal point to vehicle frame of reference

        // TODO: calculate curvature/steering angle

        // TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "pure_pursuit_node");
    PurePursuit pp;
    ros::spin();
    return 0;
}