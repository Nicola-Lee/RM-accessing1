#include "ControlTask.h"
#include "LowPassFilter.h"
#include "FrictionFeedforward.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"
#include "Gm6020.h"
#include <cmath>
#include "Pid.h"

namespace {

volatile float g_feedback_angle_rad = 0.0f;
volatile float g_debug_target_position_rad = 0.0f;
volatile float g_debug_actual_position_rad = 0.0f;

volatile uint32_t g_debug_control_count = 0;
volatile uint32_t g_debug_hal_ms = 0;

volatile float g_debug_angle_rad = 0.0f;

constexpr float kCurrentLimit = 2000.0f;
FrictionFeedforward g_friction_ff(270.0f, 3.0f);// 190

volatile float g_debug_friction_ff = 0.0f;
volatile float g_debug_current_cmd = 0.0f;

volatile int16_t g_debug_speed_rpm = 0;   // 实际速度
volatile int16_t g_debug_target_rpm = 0;  // 目标速度

Gm6020 g_gm6020(1);

constexpr float kControlPeriodS = 0.001f;// 控制周期
constexpr float kTargetAmplitudeRpm = 100.0f;// 幅值 100rmp
constexpr float kTargetFrequencyHz = 0.1f;// 10s
constexpr float kPi = 3.14159265358979323846f;

Pid g_speed_pid(26.5f, 0.5f, 0.0f, 1800.0f, -1800.0f);

constexpr float kPositionAmplitudeRad = 10.0f;  // 幅值 ±10.0 rad
constexpr float kPositionFrequencyHz = 0.05f;  // 20 s

volatile bool g_feedback_received = false;

Pid g_position_pid(120.0f, 0.0f, 0.0f, 100.0f, -100.0f); // PID27.0 5.3 0.0 friction 180

volatile uint32_t g_last_feedback_ms = 0;

// 低通滤波器
LowPassFilter g_speed_filter(0.05f); // 确定α值
volatile float g_debug_filtered_speed_rpm = 0.0f;

CAN_TxHeaderTypeDef g_tx_header = {
    .StdId = 0x1FE,
    .ExtId = 0,
    .IDE = CAN_ID_STD,
    .RTR = CAN_RTR_DATA,
    .DLC = 8,
    .TransmitGlobalTime = DISABLE,
};

uint8_t g_tx_data[8] = {0};

void SendMotorCommand()
{
    uint32_t mailbox = 0;

    // 当前阶段始终发送 0，确保不会意外转动。
    // 后续 PID 输出时再写：g_gm6020.setVoltage(...);
    // g_gm6020.setVoltage(0);

    for (uint8_t i = 0; i < 8; ++i) {
        g_tx_data[i] = 0;
    }

    g_gm6020.encode(g_tx_data);

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &g_tx_header, g_tx_data, &mailbox);
    }
}

} // namespace

extern "C" void ControlTaskInit(void)
{
    CAN_FilterTypeDef filter = {0};

    // 仅接收标准帧 0x205（GM6020, ID=1）的反馈
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = static_cast<uint16_t>(0x205U << 5);
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = static_cast<uint16_t>(0x7FFU << 5);
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
        Error_Handler();
    }
    // 启用CAN1
    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }
    // 当FIFO0收到一帧合法信息时，中断
    if (HAL_CAN_ActivateNotification(
            &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    // TIM6 当前配置即为 1 kHz
    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) {
        Error_Handler();
    }
}

extern "C" void MainTask(void)
{
    ++g_debug_control_count;
    g_debug_hal_ms = HAL_GetTick();

    // 记录运行周期次数
    static uint32_t control_tick = 0;
    static bool position_initialized = false;
    static float zero_position_rad = 0.0f;

    // 连续 10 ms 未收到电机反馈时，停止输出，避免失联后继续驱动。
    if (!g_feedback_received || (HAL_GetTick() - g_last_feedback_ms) > 10U) {
        position_initialized = false;
        control_tick = 0;
        
        g_speed_filter.reset();
        g_debug_filtered_speed_rpm = 0.0f;
        g_debug_target_rpm = 0;
        g_position_pid.reset();
        g_speed_pid.reset();
        g_gm6020.setCurrent(0);
        SendMotorCommand();
        return;
    }

    float actual_position_rad = g_feedback_angle_rad;
    
    // 避免上电后向编码器零点移动
    if (!position_initialized) {
        zero_position_rad = actual_position_rad;
        position_initialized = true;
        g_position_pid.reset();
        g_speed_pid.reset();
        return;
    }

    // 速度控制下0.1Hz，10000个周期；位置控制下0.05Hz，20000个周期
    float t = static_cast<float>(control_tick++ % 10000U) * kControlPeriodS;

    // 做0.5rad正弦摆动,先调速度
    
    // float target_position_rad = 
    //     zero_position_rad +
    //     kPositionAmplitudeRad *
    //     std::sin(2.0f * kPi * kPositionFrequencyHz * t);
    
    // g_debug_target_position_rad = target_position_rad - zero_position_rad;
    // g_debug_actual_position_rad = actual_position_rad - zero_position_rad;
    
    // float target_speed_rpm = g_position_pid.calc(target_position_rad,actual_position_rad);
    float target_speed_rpm = kTargetAmplitudeRpm * std::sin(2.0f * kPi * kTargetFrequencyHz * t);
        // 
        // 用位置控制时不用计算目标速度了

    float raw_speed_rpm = static_cast<float>(g_gm6020.velRpm());
    // float filtered_speed_rpm = g_speed_filter.update(raw_speed_rpm);

    g_debug_target_rpm = static_cast<int16_t>(target_speed_rpm); // 目标速度
    g_debug_speed_rpm = static_cast<int16_t>(raw_speed_rpm); // 实际速度
    // g_debug_filtered_speed_rpm = filtered_speed_rpm; // 滤波速度

    float pid_output =g_speed_pid.calc(target_speed_rpm, raw_speed_rpm);
    // float pid_output =g_speed_pid.calc(target_speed_rpm, filtered_speed_rpm);
    float friction_ff = g_friction_ff.calculate(target_speed_rpm);
    
    float current_cmd = pid_output + friction_ff;
    if (current_cmd > kCurrentLimit) current_cmd = kCurrentLimit;
    if (current_cmd < -kCurrentLimit) current_cmd = -kCurrentLimit;

    //g_debug_friction_ff = friction_ff;
    //g_debug_current_cmd = current_cmd;

    g_gm6020.setCurrent(static_cast<int16_t>(current_cmd));
    SendMotorCommand();
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
    if (std_id == g_gm6020.rxId()) {
        g_gm6020.decode(data);

        g_feedback_angle_rad = g_gm6020.angle();
        g_debug_angle_rad = g_feedback_angle_rad;

        g_last_feedback_ms = HAL_GetTick();
        g_feedback_received = true;
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header = {0};
    uint8_t rx_data[8] = {0};

    if (hcan != &hcan1) {
        return;
    }

    if (HAL_CAN_GetRxMessage(
            hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK) {
        return;
    }

    if (rx_header.IDE == CAN_ID_STD &&
        rx_header.RTR == CAN_RTR_DATA &&
        rx_header.DLC == 8) {
        CanFeedbackCallback(rx_header.StdId, rx_data);
    }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim6) {
        HAL_IWDG_Refresh(&hiwdg);
        MainTask();
    }
}