#include "FrictionFeedforward.h"

FrictionFeedforward::FrictionFeedforward(
    float current_command, float blend_rpm)
    : current_command_(current_command),
      blend_rpm_(blend_rpm) {}

float FrictionFeedforward::calculate(float target_speed_rpm) const
{
    if (current_command_ <= 0.0f || blend_rpm_ <= 0.0f) {
        return 0.0f;
    }

    float ratio = target_speed_rpm / blend_rpm_;

    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    return current_command_ * ratio;
}
