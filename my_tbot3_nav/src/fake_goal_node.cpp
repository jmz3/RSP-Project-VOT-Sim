#include <ros/ros.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <geometry_msgs/Point.h>
#include <stdlib.h>
#include <time.h>

int sequence = 0;
ros::Publisher pub;
geometry_msgs::Point msg;
int if_continue = 1;

double pos_x[10] = {-0.814,-0.431,0.2894,1.188,1.831,1.720,1.15,0.0464,-0.98,-2.1609};
double pos_y[10] = {-0.5782,-1.904,-1.937,-1.798,-1.092,-0.0169,0.54,0.6396,0.6802,-0.078};

void resultCB(const move_base_msgs::MoveBaseActionResult result_msg){


    int result = result_msg.status.status;
    if (result == 3){
        srand(time(NULL));
        int rand_num = rand()%10+1;
        std::cout<<rand_num<<std::endl;
        if ((rand_num != 1) && (if_continue)){
            if (sequence < 10){
                std::cout<<"Traveling to waypoint No."<<sequence<<std::endl;
                msg.x = pos_x[sequence];
                msg.y = pos_y[sequence];
                msg.z = 0;
                sequence++;
                if (sequence == 10){
                    sequence = 0;
                }
                pub.publish(msg);
            }
        } else if (if_continue) {
            std::cout<<"Going for the ball!"<<std::endl;
            msg.x = 1.614;
            msg.y = 0.498;
            msg.z = 0.5;

            pub.publish(msg);
            if_continue = 0;
        }
    } 

}

int main(int argc, char** argv){

    ros::init(argc, argv, "robot_nav");
    ros::NodeHandle nh;

    pub = nh.advertise<geometry_msgs::Point>("/fake_goal",5);
    ros::Subscriber sub = nh.subscribe("move_base/result",1000,resultCB);

    std::cout<<"Preparing to set waypoints..."<<std::endl;
    ros::Duration(2).sleep();
    
    std::cout<<"Traveling to waypoint No."<<sequence<<std::endl;

    msg.x = pos_x[sequence];
    msg.y = pos_y[sequence];
    msg.z = 0;
    sequence++;
    pub.publish(msg);

    std::cout<<"good so far"<<std::endl;
    ros::spin();
    
    return 0;
}

