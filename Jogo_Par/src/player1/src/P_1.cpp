#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"

std_msgs::String choice;
bool running = true;

//Makes user input available by opening a cin >> in the terminal
void input_card(){
    std::cin >> choice.data;
}


//Callback function to read current scoreboard
//Data contains the string, while c_str is used for character strings
//After scoreboard is printed, the player is informed about options again and
//user input is made available again by input_card();
void read_message(const std_msgs::String::ConstPtr& val){
  ROS_INFO("%s\n", val->data.c_str());
  ROS_INFO("chose one >>\ttic\tpuk\tbla\tzuw\tret\ttype 'quit' to quit");
  input_card();
}

//Callback function to quit, by turning the global bool "running" false
void quit(const std_msgs::String::ConstPtr& val){
  running = false;
}

//Parameters for int main can vary, but argc and argv can be widely used to allow command line arguments
//Argc(argument counter) keeps track of how many arguments are passed to the main, itself being the first
//Argv(argument vector) keeps track of the exact arguments passed to the main, the name of the program being the first
int main(int argc, char *argv[]) {
    //ros::init needs to see argc and argv so that it can perform
    //any ROS arguments and name remapping that were provided at the command line
    ros::init(argc, argv, "P_1");
    //NodeHandle is the main access point to communications with the ROS system.
    //The first NodeHandle constructed will fully initialize this node, and the last
    //NodeHandle destructed will close down the node.
    ros::NodeHandle nh;
    //Publisher for player input named p1_topic, which is read by referee
    //Notice queue size is 1 to only allow players to send 1 message at a time
    ros::Publisher publisher = nh.advertise<std_msgs::String>("p1_topic", 1);
    //Subscriber to scoreboard, again queue size is only 1 to only allow a single
    //scoreboard to exist at a time
    ros::Subscriber feedback = nh.subscribe("feed_topic", 1, read_message);
    //Subscriber to quit game, not used
    ros::Subscriber q = nh.subscribe("quit_topic", 1, quit);
    //5 strings, one for each card available
    std::string a, b, c, d, e;
    a = "tic";
    b = "puk";
    c = "bla";
    d = "zuw";
    e = "ret";
    //ROS version of cout - here the player makes the first move, again input_card
    //prompt a cin >> from the player to select a card
    ROS_INFO("[PLAYER 1]");
    ROS_INFO("chose one >>\ttic\tpuk\tbla\tzuw\tret\ttype 'quit' to quit");
    input_card();
    //A while loop makes the node continuously publish the players choice
    //ros::ok makes it run as long as there is a roscore running
    //The logical operator ||(or) is used
    while (ros::ok && choice.data != "quit") {
      if (choice.data == a || choice.data == b || choice.data == c ||
         choice.data == d || choice.data == e || choice.data == "quit"){
           //This is where the players entry is published in order for the referee to read it
           publisher.publish(choice);
           ROS_INFO("Your play was sent.");
           //After a play has been sent, the players entry is changed back to nothing
           choice.data = " ";
      //If the player has entered a blank response, he is told so and prompted to make a reentry
      } else if(choice.data != " ") {
        ROS_INFO("wrong option, try again...");
        input_card();
      }
      ros::Duration(0.5).sleep();
      //spinOnce is used to clear the queue for all the subscribers and publishers
	    ros::spinOnce();
    };
}
