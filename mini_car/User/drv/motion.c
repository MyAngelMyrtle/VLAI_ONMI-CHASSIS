/********************************************************************************

	* @file
  * @author  luys
  * @version V3.0.0
  * @date    06-20-2017
  * @brief

*********************************************************************************/
/********************************************************************************

	* @
  * @
  * @
  * @
  * @

*********************************************************************************/

#include "can.h"
#include "string.h"
#include "usart.h"
 //canopen区域
 
 #define PP_Mode  1
 #define PV_Mode  3 
 #define PT_Mode  4 
 
 
 //SDO CMD
 #define  SDO_W1   0x2F
 #define  SDO_W2   0x2B
 #define  SDO_W4   0x23
 #define  SDO_RD   0x40
 
 
 //CAN ID
 
 #define  Left_Wheel_ID                 0x0001
 #define  Right_Wheel_ID                0x0002
 
 //Object dictionary of CANopen
 
 #define  Control_word                  0x6040
 #define  Status_word                   0x6041
 #define  Modes_of_operation            0x6060
 #define  Modes_0f_operation_display    0x6061
 #define  Position_actual_value         0x6063
 #define  Velocity_sensor_actual_value  0x6069
 #define  Velocity_actual_value         0x606C
 #define  Target_torque                 0x6071
 #define  Target_position               0x607A
 #define  Profile_velocity              0x6081
 #define  Profile_accleration           0x6083
 #define  Profile_deceleration          0x6084
 #define  Torque_slope                  0x6087
 #define  Position_factor               0x6093
 #define  Target_velocity               0x60FF
// 速度模式PV：
u32 PV_spd;
u32 PP_spd;
u32 PT_spd;

/*模式设置*/
u8 Contol_Mode_SET(u8 CANopen_ID, u8 CANopen_mode)
{
	CAN1Sedbuf[0] = SDO_W1;
	CAN1Sedbuf[1] = 0x60;
	CAN1Sedbuf[2] = 0x60;
	CAN1Sedbuf[3] = 0x00;
	CAN1Sedbuf[4] = CANopen_mode;
	CAN1Sedbuf[5] = 0x00;
	CAN1Sedbuf[6] = 0x00;
	CAN1Sedbuf[7] = 0x00;
	CAN1_Send(0x600 + CANopen_ID, 8);
	delay_ms(5);
	return (1);
}
/*激活节点*/
u8 CANopen_Activate(u8 CANopen_ID)
{
	CAN1Sedbuf[0] = 0x01;
	CAN1Sedbuf[1] = CANopen_ID;
	CAN1_Send(0x000, 2);
	delay_ms(5);
	return (1);
}

u8 SDO_Write_OD(u8 CANopen_ID, u8 CMD, u16 Index, u8 SubIndex, u32 DATA)
{
	CAN1Sedbuf[0] = CMD;
	CAN1Sedbuf[1] = (u8)(Index & 0xFF);
	CAN1Sedbuf[2] = (u8)(Index >> 8 & 0xFF);
	CAN1Sedbuf[3] = SubIndex;
	CAN1Sedbuf[4] = (u8)(DATA & 0xFF);
	CAN1Sedbuf[5] = (u8)(DATA >> 8 & 0xFF);
	CAN1Sedbuf[6] = (u8)(DATA >> 16 & 0xFF);
	CAN1Sedbuf[7] = (u8)(DATA >> 24 & 0xFF);
	CAN1_Send(0x600 + CANopen_ID, 8);
	delay_ms(5);
	return (1);
}

u32 SDO_Read_OD(u8 CANopen_ID, u8 CMD, u16 Index, u8 SubIndex)
{
	return (1);
}
// PV canopen设置

void CANopen_PV_Init(void)
{
	// STEP1:激活节点1、节点2
	CANopen_Activate(Left_Wheel_ID);
	CANopen_Activate(Right_Wheel_ID);

	// STEP2:设置速度模式	6060H写为3
	Contol_Mode_SET(Left_Wheel_ID, PV_Mode);
	Contol_Mode_SET(Right_Wheel_ID, PV_Mode);

	// STEP3:设置加减速	写6083H和6084H
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6083, 0x00, 0x000003e8);
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6084, 0x00, 0x000003e8);

	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6083, 0x00, 0x000003e8);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6084, 0x00, 0x000003e8);

	// STEP4:设置目标转速为0	写60FFH

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x60FF, 0x00, 0x00000000);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x60FF, 0x00, 0x00000000);
}

void CANopen_PV_SET(u32 Acc, u32 Dec, s32 TargetVelocity)
{
	// STEP1:设置加减速	写6083H和6084H
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6083, 0x00, Acc);
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6084, 0x00, Dec);

	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6083, 0x00, Acc);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6084, 0x00, Dec);

	// STEP2:设置目标转速 	写60FFH
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x60FF, 0x00, TargetVelocity);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x60FF, 0x00, TargetVelocity);
}

void Motor_PV_Zero(void)
{
	// 写0x60FF速度值
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x60FF, 0x00, 0);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x60FF, 0x00, 0);
}

void Motor_PV_Go(void)
{
	// 写0x60FF速度值
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x60FF, 0x00, PV_spd);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x60FF, 0x00, PV_spd);
}

void Motor_PV_Left(void)
{
	// 写0x60FF速度值
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x60FF, 0x00, ~PV_spd + 1);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x60FF, 0x00, PV_spd);
}

void Motor_PV_Right(void)
{
	// 写0x60FF速度值
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x60FF, 0x00, PV_spd);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x60FF, 0x00, ~PV_spd + 1);
}

void Motor_PV_Back(void)
{
	// 写0x60FF速度值
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x60FF, 0x00, ~PV_spd + 1);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x60FF, 0x00, ~PV_spd + 1);
}

// PP canopen设置
void CANopen_PP_Init(void)
{
	// STEP1:激活节点1、节点2
	CANopen_Activate(1);
	CANopen_Activate(2);

	// STEP2:设置位置模式	6060H写为1
	Contol_Mode_SET(Left_Wheel_ID, PP_Mode);
	Contol_Mode_SET(Right_Wheel_ID, PP_Mode);

	// STEP3:设置目标脉冲	写607AH	10000,常规电机转1圈

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x607A, 0x00, 0);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x607A, 0x00, 0);

	// STEP4:设置目标转速为10rpm	写6081H

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6081, 0x00, 0);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6081, 0x00, 0);

	// STEP5:设置加减速	写6083H和6084H
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6083, 0x00, 0x03E8);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6083, 0x00, 0x03E8);

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6084, 0x00, 0x03E8);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6084, 0x00, 0x03E8);

	// STEP6:设置电子齿轮比 分子分母 	写6093H的sub1和sub2
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6093, 0x01, 1);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6093, 0x01, 1);

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6093, 0x02, 1);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6093, 0x02, 1);
}

void CANopen_PP_Set(s32 TargetPosition, u32 ProfileVelocity)
{
	// STEP1:设置目标脉冲	写607AH	TargetPosition
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x607A, 0x00, TargetPosition);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x607A, 0x00, TargetPosition);

	// STEP2:设置目标转速为	写6081H	 ProfileVelocity
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6081, 0x00, ProfileVelocity);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6081, 0x00, ProfileVelocity);
}

void Motor_PP_Trigger(void)
{
	// STEP1:6040 bits4清0
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x0F);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x0F);

	// STEP2:置bits4为1，电机运动
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x5F);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x5F);

	// STEP3:6040 bits4清0
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x4F);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x4F);
}

// PT canopen设置
void CANopen_PT_Init(void)
{
	// STEP1:激活节点1、节点2
	CANopen_Activate(1);
	CANopen_Activate(2);

	// STEP2:设置转矩模式	6060H写为4
	Contol_Mode_SET(Left_Wheel_ID, PT_Mode);
	Contol_Mode_SET(Right_Wheel_ID, PT_Mode);

	// STEP3:设置加减速	写6087H
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6087, 0x00, 0x03e8);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6087, 0x00, 0x03e8);

	// STEP4:设置目标转矩为0	写6071H
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6071, 0x00, 0);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6071, 0x00, 0);
}

void CANopen_PT_Set(s16 TargetTorque)
{
	// STEP1:设置目标转矩为0	写6071H
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6071, 0x00, TargetTorque);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6071, 0x00, TargetTorque);
}

void Motor_enable(void)
{
	// 使能写0x6040分别为6、7、F

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x06);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x06);

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x07);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x07);

	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x0F);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x0F);
}

void Motor_Disable(void)
{
	// 失能写0x6040分别为7
	SDO_Write_OD(Left_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x07);
	SDO_Write_OD(Right_Wheel_ID, SDO_W4, 0x6040, 0x00, 0x07);
}

void Motor_read(void) // 主循环函数
{
	CAN1_Read();
	CAN2_Read();
	Motor_ERRtest();
}

void Motor_ERRtest(void)
{
}
