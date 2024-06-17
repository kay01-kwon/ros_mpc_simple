#include "double_integral_system.hpp"

DoubleIntegralSystem::DoubleIntegralSystem()
{
    m_ = 1;
    state_initializer();
}

DoubleIntegralSystem::DoubleIntegralSystem(double m):
m_(m)
{
    state_initializer();
}

void DoubleIntegralSystem::state_initializer()
{
    s_.setZero();
    u_.setZero();

    A << 0, 1, 0, 0,
        0, 0, 0, 0,
        0, 0, 0, 1,
        0, 0, 0, 0;
    
    B << 0, 0,
        1, 0,
        0, 0,
        0, 1,
        0, 0;
}

void DoubleIntegralSystem::sytem_dynamics(const mat41_t &s, mat41_t &dsdt, double t)
{

    dsdt = A*s + B*u_;

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
