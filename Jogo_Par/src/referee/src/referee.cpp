//Gives access to fundamental ros operations
#include "ros/ros.h"
//Allows strings to be passed around
#include <iostream>
//Allows movement of the turtlesim/turtlebot
#include "geometry_msgs/Twist.h"
//Allows publishing and subscribing to strings
#include "std_msgs/String.h"
//#include <experimental/random>
//#include <math.h>
//Gives access to images from turtlebot camera
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
//#includes typically gives access to headers from different packages

//An array of empty strings is created, one for each player
std::string players[] = {"", "", "", ""};
//An array of integers is created, again one for each player
int points[] = {0, 0, 0, 0};
//A single string used to publish scoreboard is created
std_msgs::String strea_message;

//4 callback functions are created to allow the referee to read their input cards
void p1(const std_msgs::String::ConstPtr& val){
    players[0] = val->data;
}

void p2(const std_msgs::String::ConstPtr& val){
    players[1] = val->data;
}

void p3(const std_msgs::String::ConstPtr& val){
    players[2] = val->data;
}

void p4(const std_msgs::String::ConstPtr& val){
    players[3] = val->data;
}

/*
//A global bool is used to declare when there is a winner
bool winner = false;

//An example of object oriented programming. Since there are multiple steps in saving
//an image, these are all carried out by a single object, the "ImageSaver"
class ImageSaver{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    int i = 0;
    char filename[80];

public:
    ImageSaver()
            : it_(nh_)
    {
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageSaver::imageCb, this);
    }


    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        if(winner){
            sprintf(filename,"Winner_Round:%d.png", i++);
            cv::imwrite(filename,cv_ptr->image);
            winner = false;
        }
    }
};*/

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "referee");
    ros::NodeHandle nh;
    //Subscriptions are made to each of the player nodes
    ros::Subscriber player_1 = nh.subscribe("p1_topic", 1, p1);
    ros::Subscriber player_2 = nh.subscribe("p2_topic", 1, p2);
    ros::Subscriber player_3 = nh.subscribe("p3_topic", 1, p3);
    ros::Subscriber player_4 = nh.subscribe("p4_topic", 1, p4);
    //Publisher to make the turtlesim move - movement has to be published to
    //turtle1/cmd_vel for turtlesim and /cmd_vel_mux/input/navi for real turtlebot
    ros::Publisher velocity = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
    //Publisher for showing scoreboard to individual players
    ros::Publisher feedback = nh.advertise<std_msgs::String>("feed_topic", 1);
    ros::Duration(1.0).sleep();
    //A handle for the movement of the turtlesim is created
    geometry_msgs::Twist msg;
    //A handle for a stringstream is created, this stream can be accessed as a string object using .str
    std::stringstream ss;
    int round = 0;
    int highest = 0;
    int untie = 0;
    ROS_INFO("\tLet's play a game...");
    //A while loop with ros::ok is used again to make the node run continuously
    //and only stops when round is at 4.
    while (ros::ok && round + untie <= 4){
        //spinOnce is again used to clear messages to make the referee ready for current inputs
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        //An if statement is used to determine if all players have made entries
        //Again a logical operator is used &&(and) to take multiple condition into account
        if (players[0] != "" && players[1] != "" && players[2] != "" && players[3] != "") {
            //If all the players have made an input, round increments by 1
            round++;
            //Nested for loops are used to iterate through the players entries and determine if there are any matches
            //The first for loop will run through 4 times with px going from 0-3
            //Inside this for loop another for loop is made to check for matches
            //I.e. when px = 0, py will run from 0-3 and check the if statement for each py,
            //and so on for when px = 1
            //If the px is not (!=) py, the referee will know that it's two different players and
            //then a logical operators (&&) is used again to check if the entries are the same
            //using the array of strings, "players", declared in the beginning
            //If these conditions are true, both players points will be incremented by 1
            //Since this condition will check true that player 1's entry is the same as player 3's
            //and that player 3's is the same as player 1's, they both get two points in total
            for (int px = 0; px < 4; px++) for (int py = 0; py < 4; py++)
                    if (px != py && players[px] == players[py]){
                        points[px]++;
                        points[py]++;
                    //If the player instead write "quit", the while loop will break and stop the game
                    } else if (players[px] == "quit") break;
            //The stringstream is accessed as a string object using .str
            //Afterwards it can be changed following the rules for input and output of any stream
            ss.str(std::string());
            ss << "ROUND "<< round << "\tP1: " << points[0] << "\tP2: "
               << points[1] << "\tP3: " << points[2] << "\tP4: " << points[3];
            //As the object handle within the std_msgs/String is called data, the content of the ss stringstream
            //can be transferred to the single string defined right in the beginning
            strea_message.data = ss.str();
            //The content of the scoreboard is also printed to the referee window using ROS_INFO
            ROS_INFO("%s", strea_message.data.c_str());
            //Scoreboard is then published to the player nodes
            feedback.publish(strea_message);
            //All players entries are changed back to being empty
            for (int i = 0; i < 4; i++) players[i] = "";
        };
        //As the game will progress past round 5 if there is a tie, a check for this must be implemented
        //If round is 5 or above, the value of "untie" will become 1 (0-5(round)+6)
        if (round > 4){
            untie = -round + 6;
            //First all players point are checked, and if any of their points are higher that the current
            //highest, they become the new highest. Afterwards it check how many players points =
            //highest. If there is only 1, untie changes back to 0 and the game is over. Are there however
            //2 players both with points matching highest, untie goes down to -1 and allows the while loop
            //from earlier to run again.
            for (int j = 0; j < 4; j++) if (points[j] > highest) highest = points[j];
            for (int j = 0; j < 4; j++) if (points[j] == highest) untie--;
        };
    };
    //Now to point out the specific winner, player points are iterated through again, and the player with
    //the most points are selected. If the players points = to highest, the string stream from earlier is
    //updated and published same way as the scoreboard, this time announcing which specific player is the winner
    for (int player = 0; player < 4; player++) if (points[player] == highest) {
            ss.str(std::string());
            ss << "\n\n\t#####################################" <<
               "\n\t###   The winner is player "<< player + 1 <<"!!!   ###" <<
               "\n\t#####################################\n";
            strea_message.data = ss.str();
            ROS_INFO("s%", strea_message.data.c_str());
            feedback.publish(strea_message);
            //Movement is then published to the turtlesim. As there are 2 radians for a whole circle, the
            //turtlesim will turn with half a radian to make an exact 90 degree turn.
            msg.angular.z = player * M_PI / 2;
            velocity.publish(msg);
            ros::Duration(1.0).sleep();
            /*winner = true;
            ImageSaver is;*/
            ros::Duration(1.0).sleep();
            ros::spinOnce();
        };
}
