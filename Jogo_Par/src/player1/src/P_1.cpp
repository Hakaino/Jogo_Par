#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"

//this node sends the messages to the boost
int main(int argc, char *argv[]) {
    ros::init(argc, argv, "P_1");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<std_msgs::String>("p1_topic", 1);
    std_msgs::String choice;
    std::string a, b, c, d, e;
    a = "tic";
    b = "puk";
    c = "bla";
    d = "zuw";
    e = "ret";
    while (ros::ok && choice.data != "quit") {
      ros::Duration(0.5).sleep();
      ROS_INFO("\nchose one >>\ttic\tpuk\tbla\tzuw\tret\ttype 'quit' to quit");
      std::cin >> choice.data;
      if (choice.data == a || choice.data == b || choice.data == c ||
         choice.data == d || choice.data == e || choice.data == "quit"){
           publisher.publish(choice);
           ROS_INFO("Your play was sent.");
      };
	    ros::spinOnce();
    };
}
