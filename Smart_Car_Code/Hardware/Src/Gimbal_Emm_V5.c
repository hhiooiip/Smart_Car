#include "Gimbal_Emm_V5.h"

void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr;
    cmd[1] = 0x0A;
    cmd[2] = 0x6D;
    cmd[3] = 0x6B;
    gimbal_can_SendCmd(cmd, 4);
}

void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr;
    cmd[1] = 0x0E;
    cmd[2] = 0x52;
    cmd[3] = 0x6B;
    gimbal_can_SendCmd(cmd, 4);
}

void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
    uint8_t i = 0;
    uint8_t cmd[16] = {0};
    cmd[i] = addr; ++i;

    switch(s)
    {
        case S_VER  : cmd[i] = 0x1F; ++i; break;
        case S_RL   : cmd[i] = 0x20; ++i; break;
        case S_PID  : cmd[i] = 0x21; ++i; break;
        case S_VBUS : cmd[i] = 0x24; ++i; break;
        case S_CPHA : cmd[i] = 0x27; ++i; break;
        case S_ENCL : cmd[i] = 0x31; ++i; break;
        case S_TPOS : cmd[i] = 0x33; ++i; break;
        case S_VEL  : cmd[i] = 0x35; ++i; break;
        case S_CPOS : cmd[i] = 0x36; ++i; break;
        case S_PERR : cmd[i] = 0x37; ++i; break;
        case S_FLAG : cmd[i] = 0x3A; ++i; break;
        case S_ORG  : cmd[i] = 0x3B; ++i; break;
        case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
        case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
        default: break;
    }
    cmd[i] = 0x6B; ++i;
    gimbal_can_SendCmd(cmd, i);
}

void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr;
    cmd[1] = 0x46;
    cmd[2] = 0x69;
    cmd[3] = svF;
    cmd[4] = ctrl_mode;
    cmd[5] = 0x6B;
    gimbal_can_SendCmd(cmd, 6);
}

void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state;
    cmd[4] = snF;
    cmd[5] = 0x6B;
    gimbal_can_SendCmd(cmd, 6);
}

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc;
    cmd[6] = snF;
    cmd[7] = 0x6B;
    gimbal_can_SendCmd(cmd, 8);
}

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};
    cmd[0]  = addr;
    cmd[1]  = 0xFD;
    cmd[2]  = dir;
    cmd[3]  = (uint8_t)(vel >> 8);
    cmd[4]  = (uint8_t)(vel >> 0);
    cmd[5]  = acc;
    cmd[6]  = (uint8_t)(clk >> 24);
    cmd[7]  = (uint8_t)(clk >> 16);
    cmd[8]  = (uint8_t)(clk >> 8);
    cmd[9]  = (uint8_t)(clk >> 0);
    cmd[10] = raF;
    cmd[11] = snF;
    cmd[12] = 0x6B;
    gimbal_can_SendCmd(cmd, 13);
}

void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF;
    cmd[4] = 0x6B;
    gimbal_can_SendCmd(cmd, 5);
}

void Emm_V5_Synchronous_motion(uint8_t addr)
{
    uint8_t cmd[16] = {0};
    cmd[0] = addr;
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = 0x6B;
    gimbal_can_SendCmd(cmd, 4);
}
