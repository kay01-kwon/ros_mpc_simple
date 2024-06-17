#ifndef DOUBLE_INTEGRAL_SYSTEM_HPP_
#define DOUBLE_INTEGRAL_SYSTEM_HPP_

#include <ros/ros.h>
#include <simple_system_msg/simple_system_state.h>
#include <simple_system_msg/simple_system_control_input.h>
#include "type_definitions.hpp"
#include <boost/lambda/lambda.hpp>
#include <boost/numeric/odeint.hpp>
#include <boost/numeric/odeint/external/eigen/eigen.hpp>

using boost::numeric::odeint::runge_kutta4;
using simple_system_msg::simple_system_state;
using simple_system_msg::simple_system_control_inputConstPtr;

class DoubleIntegralSystem{

    public:

    DoubleIntegralSystem();

    DoubleIntegralSystem(double m);

    private:

    ros::NodeHandle nh_;

    ros::Subscriber control_input_subscriber;
    ros::Publisher state_publisher;

    runge_kutta4<mat41_t> rk4;

    // Mass
    double m_;
    
    // State
    mat41_t s_, dsdt_;

    mat44_t A;

    mat42_t B;

    // Control input
    mat21_t u_;

    double curr_t_, prev_t_, dt_;

    bool time_init;

    void ROS_setup();

    void callback_control_input(const simple_system_control_inputConstPtr& u);

    void publish_state();

    void state_initializer();

    void sytem_dynamics(
        const mat41_t& s,
        mat41_t& dsdt,
        double t
    );

    void solve();

};


#endif