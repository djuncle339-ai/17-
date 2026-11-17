#include "mpu6050_i2c.h"
#include "ti_msp_dl_config.h"
// static void I2C_Delay(void)
// {
//     volatile uint32_t i = 15; 
//     while (i--);
// }

// // static uint8_t getstatus(GPIO_Regs *gpio_port, uint32_t pin)
// // {
// //     return (DL_GPIO_readPins(gpio_port, pin) & pin) ? 1 : 0;
// // }
// #define SCL_H DL_GPIO_setPins(MPU6050_PORT, MPU6050_SCL_PIN)
// #define SCL_L DL_GPIO_clearPins(MPU6050_PORT, MPU6050_SCL_PIN)
// #define SDA_H DL_GPIO_setPins(MPU6050_PORT, MPU6050_SDA_PIN)
// #define SDA_L DL_GPIO_clearPins(MPU6050_PORT, MPU6050_SDA_PIN)
// // 建议直接改写 SDA_READ 宏：
// #define SDA_READ (DL_GPIO_readPins(MPU6050_PORT, MPU6050_SDA_PIN) != 0)

// void I2C_Start(void)
// {
//     SDA_H;
//     SCL_H;
//     I2C_Delay();
//     SDA_L;
//     I2C_Delay();
//     SCL_L;
//     I2C_Delay();
// }

// void I2C_Stop(void)
// {
//     SCL_L;
//     SDA_L;
//     I2C_Delay();
//     SCL_H;
//     I2C_Delay();
//     SDA_H;
//     I2C_Delay();
// }

// I2C_STATUS I2C_Wait_Ack(uint8_t retry_time)
// {
//     uint8_t retry = 0;
//     SDA_H; 
//     I2C_Delay();
//     SCL_H;
//     I2C_Delay();

//     while (SDA_READ)
//     {
//         retry++;
//         if (retry > retry_time)
//         {
//             I2C_Stop();
//             return I2C_ERR;
//         }
//         I2C_Delay();
//     }
//     SCL_L;
//     I2C_Delay();
//     return I2C_OK;
// }

// void I2C_Ack(void)
// {
//     SCL_L;
//     SDA_L;
//     I2C_Delay();
//     SCL_H;
//     I2C_Delay();
//     SCL_L;
//     I2C_Delay();
//     SDA_H;
// }

// void I2C_NAck(void)
// {
//     SCL_L;
//     SDA_H;
//     I2C_Delay();
//     SCL_H;
//     I2C_Delay();
//     SCL_L;
//     I2C_Delay();
// }

// void I2C_Send_Byte(uint8_t txd)
// {
//     for (int i = 0; i < 8; i++)
//     {
//         if ((txd & 0x80) >> 7)
//             SDA_H;
//         else
//             SDA_L;
//         txd <<= 1;
//         I2C_Delay();
//         SCL_H;
//         I2C_Delay();
//         SCL_L;
//     }
// }

// uint8_t I2C_Read_Byte(unsigned char ack)
// {
//     uint8_t receive = 0;
//     SDA_H; 
//     for (int i = 0; i < 8; i++)
//     {
//         SCL_L;
//         I2C_Delay();
//         SCL_H;
//         I2C_Delay();
//         receive <<= 1;
//         if (SDA_READ)
//             receive++;
//     }
//     SCL_L;
//     I2C_Delay();

//     if (!ack)
//         I2C_NAck();
//     else
//         I2C_Ack();

//     return receive;
// }

// I2C_STATUS I2C_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t *data)
// {
//     I2C_Start();

//     I2C_Send_Byte(dev_addr << 1); // 写模式
//     if (I2C_Wait_Ack(200) == I2C_ERR) return I2C_ERR;

//     I2C_Send_Byte(reg_addr);
//     if (I2C_Wait_Ack(200) == I2C_ERR) return I2C_ERR;

//     for (int i = 0; i < len; i++)
//     {
//         I2C_Send_Byte(*data++);
//         if (I2C_Wait_Ack(200) == I2C_ERR) return I2C_ERR;
//     }

//     I2C_Stop();
//     return I2C_OK;
// }

// I2C_STATUS I2C_Read_Reg(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t *data)
// {
//     I2C_Start();

//     I2C_Send_Byte(dev_addr << 1); 
//     if (I2C_Wait_Ack(200) == I2C_ERR) return I2C_ERR;

//     I2C_Send_Byte(reg_addr);
//     if (I2C_Wait_Ack(200) == I2C_ERR) return I2C_ERR;

//     // 重启总线 (Repeat Start)
//     I2C_Start();
//     I2C_Send_Byte((dev_addr << 1) | 1); // 读模式
//     if (I2C_Wait_Ack(200) == I2C_ERR) return I2C_ERR;

//     for (int i = 0; i < len - 1; i++)
//     {
//         *data++ = I2C_Read_Byte(1); // 读取并发送 ACK
//     }
//     *data = I2C_Read_Byte(0); // 最后一个字节发送 NACK

//     I2C_Stop();
//     return I2C_OK;
// }



// 对应 SysConfig 配置生成的 MPU6050 I2C 实例名
#ifndef I2C_MPU6050_INST
#define I2C_MPU6050_INST I2C_MPU6050_INST
#endif

#define I2C_TIMEOUT_MS  (10)

extern int mspm0_get_clock_ms(unsigned long *count);

/**
 * @brief  独立的 I2C 总线复位/解锁函数（不依赖 OLED）
 */
static void mpu6050_i2c_bus_reset(void)
{
    // 复位并重新初始化 MPU6050 的 I2C 外设
    DL_I2C_reset(I2C_MPU6050_INST);
    DL_I2C_enablePower(I2C_MPU6050_INST);
    
    // 调用 SysConfig 自动生成的初始化函数恢复硬件
    SYSCFG_DL_I2C_MPU6050_init();
}

/**
 * @brief  向 MPU6050 写寄存器数据
 */
I2C_STATUS I2C_Write_Reg(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t* data)
{
    unsigned long start, cur;
    uint16_t tx_bytes_sent = 0;
    uint16_t total_len = len + 1; // 1字节寄存器地址 + 数据长度

    mspm0_get_clock_ms(&start);

    // 1. 等待总线空闲
    while (!(DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            mpu6050_i2c_bus_reset();
            return I2C_ERR;
        }
    }

    // 2. 填充寄存器地址到 TX FIFO
    DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, &reg_addr, 1);
    tx_bytes_sent = 1;

    // 3. 填充初始数据到 TX FIFO
    while (tx_bytes_sent < total_len && !DL_I2C_isControllerTXFIFOFull(I2C_MPU6050_INST))
    {
        DL_I2C_transmitControllerData(I2C_MPU6050_INST, data[tx_bytes_sent - 1]);
        tx_bytes_sent++;
    }

    DL_I2C_clearInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    // 4. 启动 Controller 发送
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, total_len);

    // 5. 动态补充 TX FIFO (针对大于 8 字节的长数据传输)
    while (tx_bytes_sent < total_len)
    {
        if (!DL_I2C_isControllerTXFIFOFull(I2C_MPU6050_INST))
        {
            DL_I2C_transmitControllerData(I2C_MPU6050_INST, data[tx_bytes_sent - 1]);
            tx_bytes_sent++;
        }

        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            mpu6050_i2c_bus_reset();
            return I2C_ERR;
        }
    }

    // 6. 等待传输完成
    while (!DL_I2C_getRawInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            mpu6050_i2c_bus_reset();
            return I2C_ERR;
        }
    }

    return I2C_OK;
}

/**
 * @brief  从 MPU6050 读取寄存器数据
 */
I2C_STATUS I2C_Read_Reg(uint8_t dev_addr, uint8_t reg_addr, uint16_t len, uint8_t* data)
{
    unsigned long start, cur;
    uint16_t rx_cnt = 0;

    if (data == NULL || len == 0)
    {
        return I2C_ERR;
    }

    // ================= 第一阶段：写目标寄存器地址 =================
    mspm0_get_clock_ms(&start);

    while (!(DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            mpu6050_i2c_bus_reset();
            return I2C_ERR;
        }
    }

    DL_I2C_fillControllerTXFIFO(I2C_MPU6050_INST, &reg_addr, 1);
    DL_I2C_clearInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (!DL_I2C_getRawInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            mpu6050_i2c_bus_reset();
            return I2C_ERR;
        }
    }

    // ================= 第二阶段：切换接收方向，读取数据 =================
    mspm0_get_clock_ms(&start);

    while (!(DL_I2C_getControllerStatus(I2C_MPU6050_INST) & DL_I2C_CONTROLLER_STATUS_IDLE))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            mpu6050_i2c_bus_reset();
            return I2C_ERR;
        }
    }

    DL_I2C_clearInterruptStatus(I2C_MPU6050_INST, DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);

    // 启动 Read 传输
    DL_I2C_startControllerTransfer(I2C_MPU6050_INST, dev_addr, DL_I2C_CONTROLLER_DIRECTION_RX, len);

    // 轮询提取数据
    while (rx_cnt < len)
    {
        while (!DL_I2C_isControllerRXFIFOEmpty(I2C_MPU6050_INST))
        {
            data[rx_cnt++] = DL_I2C_receiveControllerData(I2C_MPU6050_INST);
        }

        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS))
        {
            mpu6050_i2c_bus_reset();
            return I2C_ERR;
        }
    }

    return I2C_OK;
}

