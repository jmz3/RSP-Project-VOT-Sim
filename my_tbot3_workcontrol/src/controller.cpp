#include <my_tbot3_workcontrol/controller.hpp>


controller::controller(ros::NodeHandle& nh):
        nh(nh),
        rate(0.1){

        stopMSG.angular.x = 0.0;
        stopMSG.linear.z = 0.0;

        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
        callback = boost::bind(&controller::controller_config,this,_1,_2);
        my_controller.setCallback(callback);

}



void controller::controller_config(my_tbot3_workcontrol::ControllerConfig& config, uint32_t level){
    if ((config.Switch_to_Navigation == 1) && (level==0)){
        std::cout<<"###################################################################"<<std::endl;
        std::cout<<"Switch to Navigation Mode"<<std::endl;
        std::cout<<"Saving map..."<<std::endl;
        std::cout<<"Please launch start_nav_with_gz.launch for navigation"<<std::endl;
        std::cout<<"SLAM nodes shutting down......"<<std::endl;
        std::cout<<"###################################################################"<<std::endl;

        system("rosnode kill /auto_move");
        system("rosrun map_server map_saver -f /tmp/map");
        cmd_vel_pub.publish(stopMSG);

        system("rosnode kill /turtlebot3_slam_gmapping");
        system("rosnode kill /teleop_twist_keyboard");
        system("rosnode kill /rqt_reconfigure");
        system("rosnode kill /robot_state_publisher");
        system("rosnode kill /gazebo_gui");
        system("rosnode kill /controller_node");


        //system("rosnode kill /rviz");

    } 
    
    //std::cout<<config.Operation_Mode<<" "<<level<<std::endl;
}
