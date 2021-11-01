#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Path.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

bool path_published = false;
geometry_msgs::PoseStamped pose;
nav_msgs::Path path;
void path_cb(const nav_msgs::Path::ConstPtr& msg){
    path = *msg;

    if(path.poses.size() < 5){
        pose.pose.position.x = path.poses[path.poses.size()].pose.position.x;
        pose.pose.position.y = path.poses[path.poses.size()].pose.position.y;
        pose.pose.orientation = path.poses[path.poses.size()].pose.orientation;
    }else{
        pose.pose.position.x = path.poses[5].pose.position.x;
        pose.pose.position.y = path.poses[5].pose.position.y;
        pose.pose.orientation = path.poses[5].pose.orientation;
    }

    path_published = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mav_slam_nav");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>
            ("move_base/DWAPlannerROS/local_plan", 10, path_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
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
        if(path_published != true){
            pose = current_pose;            
        }        

        pose.pose.position.z = 2;

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}