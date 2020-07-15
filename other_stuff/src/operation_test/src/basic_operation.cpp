#include <ros/ros.h>

int main(int argc,char **argv){

    ros::init(argc,argv,"operation_test");
    ros::NodeHandle n;

    ros::Rate rate(10);

    while(ros::ok()){

        ROS_INFO("Test");
        rate.sleep();
    }

}
