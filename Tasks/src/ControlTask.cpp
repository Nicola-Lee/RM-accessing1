#include "ControlTask.h"

#include "can.h"
#include "tim.h"
#include "Gm6020.h"

namespace {

// 0x205 = 0x204 + 1，因此电机拨码 ID 为 1
Gm6020 g_gm6020(1);

CAN_TxHeaderTypeDef g_tx_header = {
    .StdId = 0x1FF,
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
    g_gm6020.setVoltage(0);

    for (uint8_t i = 0; i < 8; ++i) {
        g_tx_data[i] = 0;
    }

    // ID=1 对应 data[0..1]
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

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Error_Handler();
    }

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
    SendMotorCommand();
}

extern "C" void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
    if (std_id == g_gm6020.rxId()) {
        g_gm6020.decode(data);
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
        MainTask();
    }
}