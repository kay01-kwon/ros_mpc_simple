#include "double_integral_system.hpp"

DoubleIntegralSystem::DoubleIntegralSystem()
{
    state_initializer();
    nh_.getParam("mass",m_);

    ROS_setup();
}

DoubleIntegralSystem::DoubleIntegralSystem(double m):
m_(m)
{
    state_initializer();
    ROS_setup();
}

void DoubleIntegralSystem::publish_state()
{
    curr_t_ = ros::Time::now().toSec() + ros::Time::now().toNSec()*10e-9;
    
    if(time_init == false)
    {
        time_init = true;
        prev_t_ = curr_t_;
        return;
    }

    solve();

    simple_system_state published_state_msg;

    published_state_msg.stamp = ros::Time::now();

    for(int i = 0; i < 4; i++)
        published_state_msg.s[i] = s_(i);

    state_publisher.publish(published_state_msg);
}

void DoubleIntegralSystem::ROS_setup()
{
    control_input_subscriber = nh_.subscribe("/input", 1, &DoubleIntegralSystem::callback_control_input, this);
    state_publisher = nh_.advertise<simple_system_state>("/state",1);
}

void DoubleIntegralSystem::callback_control_input(const simple_system_control_inputConstPtr &u)
{
    for(int i = 0; i < 2; i++)
        u_(i) = u->u[i];
}

void DoubleIntegralSystem::state_initializer()
{
    s_.setZero();
    u_.setZero();

    A << 0, 0, 1, 0,
        0, 0, 0, 1,
        0, 0, 0, 0,
        0, 0, 0, 0;
    
    B << 0, 0,
        0, 0,
        1, 0,
        0, 1;

    time_init = true;
}

void DoubleIntegralSystem::sytem_dynamics(const mat41_t &s, mat41_t &dsdt, double t)
{

    dsdt = A*s + 1/m_*B*u_;

}

void DoubleIntegralSystem::solve()
{
    dt_ = curr_t_ - prev_t_;

    rk4.do_step(
        std::bind(
        &DoubleIntegralSystem::sytem_dynamics,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3
        ),
        s_, prev_t_, dt_
    );

    prev_t_ = curr_t_;

}
