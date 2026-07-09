#include "Gimbal_CAN.h"

__IO Gimbal_CAN_t gimbal_can = {0};
float gimbal_angle_Real = 0;
static uint32_t angle_Real_INT = 0;

/* ---- CAN 滤波器 + 启动 ---- */
void Gimbal_CAN_Init(void)
{
    CAN_FilterTypeDef FilterConfig;
    FilterConfig.FilterBank = 0;
    FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    FilterConfig.FilterIdHigh = 0x0000;
    FilterConfig.FilterIdLow = CAN_ID_EXT;
    FilterConfig.FilterMaskIdHigh = 0x0000;
    FilterConfig.FilterMaskIdLow = CAN_ID_EXT;
    FilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    FilterConfig.FilterActivation = ENABLE;

    if (HAL_CAN_ConfigFilter(&hcan1, &FilterConfig) != HAL_OK)
    {
        while(1);
    }

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

/* ---- CAN 发送 (多包拆分) ---- */
#define CAN_TX_WAIT_TIMEOUT_MS 10
#define MAX_PACKETS 10
#define MAX_CMD_LEN 64

static bool can_WaitMailboxFree(uint32_t timeoutMs)
{
    uint32_t startTick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0)
    {
        if ((HAL_GetTick() - startTick) >= timeoutMs)
            return false;
    }
    return true;
}

void gimbal_can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    if ((len < 2) || (len > MAX_CMD_LEN))
        return;

    uint8_t remaining = len - 2;
    uint8_t dataOffset = 0;
    uint8_t packetNum = 0;

    while ((remaining > 0) && (packetNum < MAX_PACKETS))
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint8_t TxData[8];
        uint32_t TxMailbox;

        memset(TxData, 0, 8);
        TxHeader.ExtId = ((uint32_t)cmd[0] << 8) | packetNum;
        TxHeader.IDE = CAN_ID_EXT;
        TxHeader.RTR = CAN_RTR_DATA;

        uint8_t payloadLen = (remaining < 8) ? remaining : 7;
        TxHeader.DLC = payloadLen + 1;

        TxData[0] = cmd[1];
        for (uint8_t i = 0; i < payloadLen; i++)
            TxData[i + 1] = cmd[dataOffset + 2 + i];

        if (!can_WaitMailboxFree(CAN_TX_WAIT_TIMEOUT_MS))
            return;

        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
            return;

        remaining -= payloadLen;
        dataOffset += payloadLen;
        packetNum++;
    }
}

/* ---- CAN 接收中断处理 ---- */
static void Angle_Read(void)
{
    if (gimbal_can.pack[0] == 0x36)
    {
        angle_Real_INT = (uint32_t)gimbal_can.pack[2] << 24 |
                         (uint32_t)gimbal_can.pack[3] << 16 |
                         (uint32_t)gimbal_can.pack[4] << 8  |
                         (uint32_t)gimbal_can.pack[5];
        gimbal_angle_Real = (float)angle_Real_INT * 360.0f / 65536.0f;
        if (gimbal_can.pack[1] == 1)
            gimbal_angle_Real = -gimbal_angle_Real;
    }
}

void Gimbal_CAN_RX_IRQHandler(void)
{
    if (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
    {
        CAN_RxHeaderTypeDef rxHeader;
        uint8_t rxData[8];
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
        {
            uint8_t packIdx = (uint8_t)(rxHeader.ExtId & 0xFF);
            memcpy((void *)&gimbal_can.pack[packIdx * 8], rxData, 8);
            Angle_Read();
            gimbal_can.rxFrameFlag = true;
        }
    }
}

/* ---- 等待响应 ---- */
bool Gimbal_CAN_WaitResponse(void)
{
    if (gimbal_can.rxFrameFlag)
    {
        gimbal_can.rxFrameFlag = false;
        return true;
    }
    return false;
}

void Gimbal_CAN_WaitBlocking(void)
{
    uint32_t start = HAL_GetTick();
    while (!gimbal_can.rxFrameFlag)
    {
        if ((HAL_GetTick() - start) >= 5)
            break;
    }
    gimbal_can.rxFrameFlag = false;
}
