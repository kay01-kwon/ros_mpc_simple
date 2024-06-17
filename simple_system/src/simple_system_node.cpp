#include <simple_system/double_integral_system.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "simple_system_node");

    DoubleIntegralSystem dis;

    ros::Rate loop_rate(200);

    while(ros::ok())
    {
        dis.publish_state();
        ros::spinOnce();
        loop_rate.sleep();
    }

}