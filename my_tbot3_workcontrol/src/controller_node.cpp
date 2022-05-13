#include <my_tbot3_workcontrol/controller.hpp>

int main(int argc, char** argv){

    ros::init(argc,argv,"my_server");

    ros::NodeHandle nh;

    controller my_controller(nh);

    while(nh.ok()){

        ros::spinOnce();

    }

    return 0;
}