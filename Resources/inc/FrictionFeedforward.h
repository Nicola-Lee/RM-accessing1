#ifndef FRICTION_FEEDFORWARD_H
#define FRICTION_FEEDFORWARD_H

class FrictionFeedforward {
public:
    // current_command：最大补偿值，单位为电流指令原始值
    // blend_rpm：从零平滑过渡到最大补偿所需的目标速度
    FrictionFeedforward(float current_command, float blend_rpm);

    float calculate(float target_speed_rpm) const;

private:
    float current_command_;
    float blend_rpm_;
};

#endif
