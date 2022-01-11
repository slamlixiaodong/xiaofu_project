#include <xiaofu_task_server/xiaofu_task_server.h>
int main(int argc,char **argv)
{
    ros::init(argc,argv,"xiaofu_task_server_node");
    xiaofu_robot xiaofu_robot_node;
    // xiaofu_robot_node.open_battery("open_battery");
    ROS_INFO("SYSTEM STARTING!!!");
    xiaofu_robot_node.thread_fun();
    ros::spin();
}
