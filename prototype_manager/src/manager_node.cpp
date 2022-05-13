#include <prototype_manager/manager.hpp>

int main( int argc , char** argv){

    ros::init(argc,argv,"controller_server");

    ros::NodeHandle nodeHandle_;

    manager my_manager(nodeHandle_);

    while(nodeHandle_.ok()){
        ros::spinOnce();
    }
    return 0;
}