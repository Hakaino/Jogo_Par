#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"

//this node sends the messages to the boost
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "P_2");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<std_msgs::String>("p2_topic", 1);
    std_msgs::String choice;
    std::string a, b, c, d, e;
    a = "tic";
    b = "puk";
    c = "bla";
    d = "zuw";
    e = "ret";
    while (ros::ok && choice.data != "quit") {
      ros::Duration(0.5).sleep();
	    ros::spinOnce();
      ROS_INFO("chose one >> tic\tpuk\tbla\tzuw\tret");
      std::cin >> choice.data;
      if (choice.data == a || choice.data == b || choice.data == c ||
         choice.data == d || choice.data == e){
           publisher.publish(choice);
           ROS_INFO("Your play was sent.");
      };
    };
}
