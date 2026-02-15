#include "imu_serial_reader/filter.hpp"

Filter::Filter(float alpha) : alpha_(alpha), initialized_(false), initialized4_(false)
{
    filtered_value_[0] = 0.0f;
    filtered_value_[1] = 0.0f;
    filtered_value_[2] = 0.0f;
    filtered_value4_[0] = 0.0f;
    filtered_value4_[1] = 0.0f;
    filtered_value4_[2] = 0.0f;
    filtered_value4_[3] = 0.0f;
}

void Filter::lowPassFilterUpdate(const float input[3])
{
    if (!initialized_)
    {
        filtered_value_[0] = input[0];
        filtered_value_[1] = input[1];
        filtered_value_[2] = input[2];
        initialized_ = true;
    }
    else
    {
        filtered_value_[0] = alpha_ * input[0] + (1.0f - alpha_) * filtered_value_[0];
        filtered_value_[1] = alpha_ * input[1] + (1.0f - alpha_) * filtered_value_[1];
        filtered_value_[2] = alpha_ * input[2] + (1.0f - alpha_) * filtered_value_[2];
    }
}

void Filter::getFiltered(float output[3]) const
{
    output[0] = filtered_value_[0];
    output[1] = filtered_value_[1];
    output[2] = filtered_value_[2];
}

void Filter::setAlpha(float alpha)
{
    alpha_ = alpha;
}

void Filter::lowPassFilterUpdate4(const float input[4])
{
    if (!initialized4_)
    {
        filtered_value4_[0] = input[0];
        filtered_value4_[1] = input[1];
        filtered_value4_[2] = input[2];
        filtered_value4_[3] = input[3];
        initialized4_ = true;
    }
    else
    {
        filtered_value4_[0] = alpha_ * input[0] + (1.0f - alpha_) * filtered_value4_[0];
        filtered_value4_[1] = alpha_ * input[1] + (1.0f - alpha_) * filtered_value4_[1];
        filtered_value4_[2] = alpha_ * input[2] + (1.0f - alpha_) * filtered_value4_[2];
        filtered_value4_[3] = alpha_ * input[3] + (1.0f - alpha_) * filtered_value4_[3];
    }
}

void Filter::getFiltered4(float output[4]) const
{
    output[0] = filtered_value4_[0];
    output[1] = filtered_value4_[1];
    output[2] = filtered_value4_[2];
    output[3] = filtered_value4_[3];
}
