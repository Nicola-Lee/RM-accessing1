#include "LowPassFilter.h"

LowPassFilter::LowPassFilter(float alpha)
    : alpha_(alpha), value_(0.0f), initialized_(false) {}

float LowPassFilter::update(float input)
{
    if (!initialized_) {
        value_ = input;  // 首次采样直接采用实测值
        initialized_ = true;
    } else {
        value_ += alpha_ * (input - value_);
    }
    return value_;
}

void LowPassFilter::reset()
{
    value_ = 0.0f;
    initialized_ = false;
}

float LowPassFilter::value() const
{
    return value_;
}
