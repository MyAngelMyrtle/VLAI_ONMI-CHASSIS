#include "bsp_fdcan.h"
#include "drv_8030.h"
#include "chassis_task.h"


s8030_moto_t moto[2];

/**
  * @brief  配置节点2为可操作状态（NMT命令）
  * @param  hfdcan CAN句柄指针
  * @retval None
  */
void MotoSetOperational(hcan_t *hfdcan)
{
    uint32_t can_id = 0x000;  // NMT命令帧ID
    static uint8_t txdata[8];
    
    txdata[0] = 0x01;  // 启动节点命令
    txdata[1] = 0x02;  // 节点2
    txdata[2] = 0x00;
    txdata[3] = 0x00;
    txdata[4] = 0x00;
    txdata[5] = 0x00;
    txdata[6] = 0x00;
    txdata[7] = 0x00;
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  设置电机运行模式为速度模式
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @retval None
  */
void MotoSetSpeedMode(hcan_t *hfdcan, uint8_t node_id)
{
    uint32_t can_id = 0x600 + node_id;  // SDO发送给节点2的ID
    static uint8_t txdata[8];
    
    // 设置6060h为3（速度控制模式）
    txdata[0] = 0x2F;  // 写入1字节
    txdata[1] = 0x60;
    txdata[2] = 0x60;
    txdata[3] = 0x00;
    txdata[4] = 0x03;  // 速度模式
    txdata[5] = 0x00;
    txdata[6] = 0x00;
    txdata[7] = 0x00;
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  设置加速度时间
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @param  accel_time 加速度时间（ms，从0到3000rpm的时间）
  * @retval None
  */
void MotoSetAcceleration(hcan_t *hfdcan, uint8_t node_id, uint32_t accel_time)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    // 设置6083h（Profile acceleration）
    txdata[0] = 0x23;  // 写入4字节
    txdata[1] = 0x83;
    txdata[2] = 0x60;
    txdata[3] = 0x00;
    txdata[4] = (uint8_t)(accel_time & 0xFF);
    txdata[5] = (uint8_t)((accel_time >> 8) & 0xFF);
    txdata[6] = (uint8_t)((accel_time >> 16) & 0xFF);
    txdata[7] = (uint8_t)((accel_time >> 24) & 0xFF);
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  设置减速度时间
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @param  decel_time 减速度时间（ms，从3000rpm到0的时间）
  * @retval None
  */
void MotoSetDeceleration(hcan_t *hfdcan, uint8_t node_id, uint32_t decel_time)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    // 设置6084h（Profile deceleration）
    txdata[0] = 0x23;  // 写入4字节
    txdata[1] = 0x84;
    txdata[2] = 0x60;
    txdata[3] = 0x00;
    txdata[4] = (uint8_t)(decel_time & 0xFF);
    txdata[5] = (uint8_t)((decel_time >> 8) & 0xFF);
    txdata[6] = (uint8_t)((decel_time >> 16) & 0xFF);
    txdata[7] = (uint8_t)((decel_time >> 24) & 0xFF);
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  设置目标速度
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @param  target_speed 目标速度（0.1rpm单位）
  * @retval None
  */
void MotoSetTargetSpeed(hcan_t *hfdcan, uint8_t node_id, int32_t target_speed)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    // 设置60FFh（Target velocity）
    txdata[0] = 0x23;  // 写入4字节
    txdata[1] = 0xFF;
    txdata[2] = 0x60;
    txdata[3] = 0x00;
    txdata[4] = (uint8_t)(target_speed & 0xFF);
    txdata[5] = (uint8_t)((target_speed >> 8) & 0xFF);
    txdata[6] = (uint8_t)((target_speed >> 16) & 0xFF);
    txdata[7] = (uint8_t)((target_speed >> 24) & 0xFF);
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  设置控制字使能电机（三步使能过程）
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @retval None
  */
void MotoControlWordFirstReady(hcan_t *hfdcan, uint8_t node_id)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    // 第一步：写0x06
    txdata[0] = 0x2B;  // 写入2字节
    txdata[1] = 0x40;
    txdata[2] = 0x60;
    txdata[3] = 0x00;
    txdata[4] = 0x06;  // 0x06
    txdata[5] = 0x00;
    txdata[6] = 0x00;
    txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);  // 适当延时
    
    // 第二步：写0x07
    txdata[4] = 0x07;  // 0x07
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 第三步：写0x0F
    txdata[4] = 0x0F;  // 0x0F
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
}

/**
  * @brief  设置控制字禁能电机
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @retval None
  */
void MotoControlWordDisable(hcan_t *hfdcan, uint8_t node_id)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    txdata[0] = 0x2B;  // 写入2字节
    txdata[1] = 0x40;
    txdata[2] = 0x60;
    txdata[3] = 0x00;
    txdata[4] = 0x07;  // 0x07 - 失能
    txdata[5] = 0x00;
    txdata[6] = 0x00;
    txdata[7] = 0x00;
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  设置控制字禁能电机
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @retval None
  */
void MotoControlWordEnable(hcan_t *hfdcan, uint8_t node_id)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    txdata[0] = 0x2B;  // 写入2字节
    txdata[1] = 0x40;
    txdata[2] = 0x60;
    txdata[3] = 0x00;
    txdata[4] = 0x0F;  // 0x07 - 失能
    txdata[5] = 0x00;
    txdata[6] = 0x00;
    txdata[7] = 0x00;
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  配置TPDO1
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @retval None
  */
void MotoConfigureTPDO1(hcan_t *hfdcan, uint8_t node_id)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    // 禁用TPDO1映射
    txdata[0] = 0x2F; txdata[1] = 0x00; txdata[2] = 0x1A; txdata[3] = 0x00;
    txdata[4] = 0x00; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置第一个映射：6069h（velocity_sensor_actual_value）
    txdata[0] = 0x23; txdata[1] = 0x00; txdata[2] = 0x1A; txdata[3] = 0x01;
    txdata[4] = 0x20; txdata[5] = 0x00; txdata[6] = 0x69; txdata[7] = 0x60;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置第二个映射：6063h（Position actual value）
    txdata[0] = 0x23; txdata[1] = 0x00; txdata[2] = 0x1A; txdata[3] = 0x02;
    txdata[4] = 0x20; txdata[5] = 0x00; txdata[6] = 0x63; txdata[7] = 0x60;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 启用2个映射
    txdata[0] = 0x2F; txdata[1] = 0x00; txdata[2] = 0x1A; txdata[3] = 0x00;
    txdata[4] = 0x02; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置传输类型为同步传输
    txdata[0] = 0x2F; txdata[1] = 0x00; txdata[2] = 0x18; txdata[3] = 0x02;
    txdata[4] = 0x01; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置禁止时间
    txdata[0] = 0x2F; txdata[1] = 0x00; txdata[2] = 0x18; txdata[3] = 0x03;
    txdata[4] = 0x02; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置事件定时器
    txdata[0] = 0x2F; txdata[1] = 0x00; txdata[2] = 0x18; txdata[3] = 0x05;
    txdata[4] = 0x02; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  配置TPDO2
  * @param  hfdcan CAN句柄指针
  * @param  node_id 节点ID
  * @retval None
  */
void MotoConfigureTPDO2(hcan_t *hfdcan, uint8_t node_id)
{
    uint32_t can_id = 0x600 + node_id;
    static uint8_t txdata[8];
    
    // 禁用TPDO2映射
    txdata[0] = 0x2F; txdata[1] = 0x01; txdata[2] = 0x1A; txdata[3] = 0x00;
    txdata[4] = 0x00; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置第一个映射：6041h（Statusword）
    txdata[0] = 0x23; txdata[1] = 0x01; txdata[2] = 0x1A; txdata[3] = 0x01;
    txdata[4] = 0x10; txdata[5] = 0x00; txdata[6] = 0x41; txdata[7] = 0x60;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 启用1个映射
    txdata[0] = 0x2F; txdata[1] = 0x01; txdata[2] = 0x1A; txdata[3] = 0x00;
    txdata[4] = 0x01; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置传输类型为同步传输
    txdata[0] = 0x2F; txdata[1] = 0x01; txdata[2] = 0x18; txdata[3] = 0x02;
    txdata[4] = 0x01; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置禁止时间
    txdata[0] = 0x2F; txdata[1] = 0x01; txdata[2] = 0x18; txdata[3] = 0x03;
    txdata[4] = 0x02; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
    HAL_Delay(10);
    
    // 设置事件定时器
    txdata[0] = 0x2F; txdata[1] = 0x01; txdata[2] = 0x18; txdata[3] = 0x05;
    txdata[4] = 0x02; txdata[5] = 0x00; txdata[6] = 0x00; txdata[7] = 0x00;
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  发送同步帧
  * @param  hfdcan CAN句柄指针
  * @retval None
  */
void MotoSendSyncFrame(hcan_t *hfdcan)
{
    uint32_t can_id = 0x80;  // 同步帧ID
    static uint8_t txdata[8];
    
    txdata[0] = 0x00;
    txdata[1] = 0x00;
    txdata[2] = 0x00;
    txdata[3] = 0x00;
    txdata[4] = 0x00;
    txdata[5] = 0x00;
    txdata[6] = 0x00;
    txdata[7] = 0x00;
    
    fdcanx_send_data(&hfdcan1, can_id, txdata, 8);
}

/**
  * @brief  完整的速度模式配置示例（节点2）
  * @param  hfdcan CAN句柄指针
  * @retval None
  */
void MotoSpeedConfigExample(hcan_t *hfdcan)
{
    uint8_t node_id = 0x02;  // 节点2
    
    // STEP1: 节点置为可操作状态
    MotoSetOperational(hfdcan);
    HAL_Delay(10);
    
    // STEP2: 设置运行模式为速度模式
    MotoSetSpeedMode(hfdcan, node_id);
    HAL_Delay(10);
    
    // STEP3: 设置加速度和减速度时间为1秒（1000ms）
    MotoSetAcceleration(hfdcan, node_id, 1000);  // 1秒
    HAL_Delay(10);
    MotoSetDeceleration(hfdcan, node_id, 1000);  // 1秒
    HAL_Delay(10);
    
    // STEP4: 设置目标速度为10rpm（10 * 10 = 100，因为单位是0.1rpm）
    MotoSetTargetSpeed(hfdcan, node_id, 100);  // 10rpm
    HAL_Delay(10);
    
    // STEP5: 控制字使能电机
    MotoControlWordEnable(hfdcan, node_id);
    HAL_Delay(10);
    
    // STEP6: 配置TPDO（可选）
    MotoConfigureTPDO1(hfdcan, node_id);
    HAL_Delay(10);
    MotoConfigureTPDO2(hfdcan, node_id);
    HAL_Delay(10);
    
    // STEP7: 发送同步帧触发TPDO上传
    MotoSendSyncFrame(hfdcan);
}
