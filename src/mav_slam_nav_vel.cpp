#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

# define M_PI           3.14159265358979323846

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

geometry_msgs::PoseStamped pose;
nav_msgs::Path path;
void path_cb(const nav_msgs::Path::ConstPtr& msg){
    path = *msg;
}

geometry_msgs::Twist target_vel;
void vel_cb(const geometry_msgs::Twist::ConstPtr& msg){
    // target_vel.angular.z = 0;
    // target_vel.linear.x = msg->linear.x;
    // target_vel.linear.y = msg->linear.y;
    // target_vel.linear.z = 0;

    target_vel.angular.z = msg->angular.z;
    target_vel.linear.x = msg->linear.x * cos(current_pose.pose.orientation.z * M_PI) + msg->linear.y * sin(current_pose.pose.orientation.z * M_PI);
    target_vel.linear.y = -msg->linear.x * sin(current_pose.pose.orientation.z * M_PI) + msg->linear.y * cos(current_pose.pose.orientation.z * M_PI);
    target_vel.linear.z = 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_slam_nav_vel");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>
            ("move_base/DWAPlannerROS/local_plan", 10, path_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::Twist>
            ("/cmd_vel", 10, vel_cb);
    ros::Publisher target_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        geometry_msgs::TwistStamped vel;
        vel.twist = target_vel;
        
        if(current_pose.pose.position.z < 2){
            vel.twist.linear.z = 0.1;
        }else{
            vel.twist.linear.z = -0.1;
        }

        //local_pos_pub.publish(pose);
        target_vel_pub.publish(vel);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}