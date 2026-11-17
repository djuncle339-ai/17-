#include "mpu6050.h"

#include <math.h>

#include "mpu6050_i2c.h"
double Accel_Z_corrector = 14418.0;

#include "inv_mpu_dmp_motion_driver.h"
#include "inv_mpu.h"

#define q30  1073741824.0f //用于归一化四元数

#define DEFAULT_MPU_HZ  (100) //DMP采样频率100Hz，跟主循环10ms读取一次的节奏对齐，避免FIFO里产生比消费快的积压

extern int mspm0_delay_ms(unsigned long num_ms);
short gyro[3], accel[3], sensors; //用于读取DMP_FIFO

static signed char gyro_orientation[9] = { 1, 0, 0,  //方向矩阵
                                           0, 1, 0,
                                           0, 0, 1};
static  unsigned short inv_row_2_scale(const signed char *row) //用于初始化DMP
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;

}


static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx) //用于初始化DMP
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;

}

static uint8_t run_self_test(void) //用于初始化DMP
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
        return 0;
    }
    return 1;

}
uint8_t MPU6050_GetID(void) {
    uint8_t id = 0;
    I2C_Read_Reg(MPU6050_ADDR,WHO_AM_I_REG,1,&id);
    return id;
}
// DMP（数字运动处理器）初始化函数
void DMP_Init(void)
{
    uint8_t temp[1]={0};
    temp[0] = MPU6050_GetID();          // 读取MPU6050设备ID
    if(temp[0]!=0x68) return;; // 校验设备ID是否为0x68（正常值），错误则系统复位

    // 主初始化流程
    if(!mpu_init()) // 成功返回0，初始化MPU6050底层,与步骤7相呼应
    {
        // 配置传感器使能状态
        if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {} // 启用陀螺仪和加速度计的三轴数据

        // FIFO配置
        if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {} // 配置FIFO存储陀螺仪和加速度计数据

        // 采样率设置
        if(!mpu_set_sample_rate(DEFAULT_MPU_HZ)) {} // 设置采样率为默认值（典型值如100Hz）

        // DMP固件加载
        if(!dmp_load_motion_driver_firmware()) {} // 加载DMP运动驱动固件

        // 方向矩阵配置
        if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {} // 设置传感器方向校准矩阵

        // 启用DMP特性
        if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |  // 启用6轴低功耗四元数 | 敲击检测
                DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL |   // Android方向识别 | 原始加速度数据
                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL)) {}       // 校准后的陀螺仪数据 | 陀螺仪校准

        // FIFO速率设置
        if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ)) {} // 设置DMP输出速率与采样率同步

        run_self_test(); // 运行自检程序（校准传感器）

        // 启用DMP
        if(!mpu_set_dmp_state(1)) {} // 激活DMP功能（1=启用，0=关闭）
    }

}

uint8_t calc_lpf(void);

/**
 * @brief mpu6050初始化
 * @return 初始化状态
 */
MPU6050_STATUS MPU6050_Init(void) {
    uint8_t id = 0;
    I2C_Read_Reg(MPU6050_ADDR,WHO_AM_I_REG,1,&id);
    if (id != 0x68) {
        return MPU6050_INIT_ERROR; // Initialization failed
    }
    //软件复位
    uint8_t cmd = 0x80;
    I2C_STATUS status = I2C_Write_Reg(MPU6050_ADDR,PWR_MGMT_1_REG, 1, &cmd);
    if (status == I2C_ERR) {
        return MPU6050_INIT_ERROR;
    }
    mspm0_delay_ms(200);
    cmd = 0x00;
    status = I2C_Write_Reg(MPU6050_ADDR,PWR_MGMT_1_REG,1,&cmd);
    if (status == I2C_ERR) {
        return MPU6050_INIT_ERROR;
    }
    cmd = GYRO_FSR << 3;
    status = I2C_Write_Reg(MPU6050_ADDR,GYRO_CONFIG_REG,1,&cmd);
    if (status == I2C_ERR) {
        return MPU6050_INIT_ERROR;
    }
    cmd = ACCEL_FSR << 3;
    status = I2C_Write_Reg(MPU6050_ADDR,ACCEL_CONFIG_REG,1,&cmd);
    if (status == I2C_ERR) {
        return MPU6050_INIT_ERROR;
    }
    cmd = 1000 / SAMPLE_RATE - 1;
    status = I2C_Write_Reg(MPU6050_ADDR,SMPLRT_DIV_REG,1,&cmd);
    if (status == I2C_ERR) {
        return MPU6050_INIT_ERROR;
    }
    cmd = calc_lpf();
    status = I2C_Write_Reg(MPU6050_ADDR,CFG_REG,1,&cmd);
    if (status == I2C_ERR) {
        return MPU6050_INIT_ERROR;
    }
    cmd = 0x01;
    I2C_Write_Reg(MPU6050_ADDR, MPU_PWR_MGMT1_REG, 1,&cmd);    /* 设置CLKSEL，PLL X轴为参考 */
    cmd = 0x00;
    I2C_Write_Reg(MPU6050_ADDR, MPU_PWR_MGMT2_REG, 1,&cmd);    /* 加速度与陀螺仪都工作 */
    return MPU6050_INIT_OK;
}
/**
 * @brief 读取加速度数据
 * @param DataStruct 数据结构体指针
 * @return 读取状态
 */
MPU6050_STATUS MPU6050_Read_Accel(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register
    I2C_STATUS status = I2C_Read_Reg(MPU6050_ADDR, ACCEL_XOUT_H_REG, 6, Rec_Data);
    if (status == I2C_ERR) {
        return MPU6050_I2C_ERROR;
    }
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / ACCEL_LSB;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / ACCEL_LSB;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    return MPU6050_READ_OK;
}
/**
 * @brief 读取陀螺仪数据
 * @param DataStruct 数据结构体指针
 * @return 读取状态
 */
MPU6050_STATUS MPU6050_Read_Gyro(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    I2C_STATUS status = I2C_Read_Reg(MPU6050_ADDR, GYRO_XOUT_H_REG, 6, Rec_Data);
    if (status == I2C_ERR) {
        return MPU6050_I2C_ERROR;
    }
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Gx = DataStruct->Gyro_X_RAW / GYRO_LSB;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / GYRO_LSB;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / GYRO_LSB;
    return MPU6050_READ_OK;
}
/**
 * @brief 读取温度数据
 * @param DataStruct 数据结构体指针
 * @return 读取状态
 */
MPU6050_STATUS MPU6050_Read_Temp(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    I2C_STATUS status = I2C_Read_Reg(MPU6050_ADDR, TEMP_OUT_H_REG, 2, Rec_Data);
    if (status == I2C_ERR) {
        return MPU6050_I2C_ERROR;
    }
    int16_t temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    return MPU6050_READ_OK;
}
/**
 * @brief 读取所有传感器数据
 * @param DataStruct 数据结构体指针
 * @return 读取状态
 */
MPU6050_STATUS MPU6050_Read_All(MPU6050_t *DataStruct) {
    MPU6050_STATUS status;
    status = MPU6050_Read_Accel(DataStruct);
    if (status != MPU6050_READ_OK) {
        return status;
    }
    status = MPU6050_Read_Gyro(DataStruct);
    if (status != MPU6050_READ_OK) {
        return status;
    }
    status = MPU6050_Read_Temp(DataStruct);
    if (status != MPU6050_READ_OK) {
        return status;
    }
    return MPU6050_READ_OK;
}
/**
 * @brief 计算低通滤波器配置值
 * @return 低通滤波器配置值
 */
uint8_t calc_lpf(void) {
    const uint8_t lpf = SAMPLE_RATE >> 1;
    uint8_t dat;
    if (lpf >= 188)
    {
        dat = 1;
    }
    else if (lpf >= 98)
    {
        dat = 2;
    }
    else if (lpf >= 42)
    {
        dat = 3;
    }
    else if (lpf >= 20)
    {
        dat = 4;
    }
    else if (lpf >= 10)
    {
        dat = 5;
    }
    else
    {
        dat = 6;
    }
    return dat;
}
int MPU6050_Read(uint8_t addr , uint8_t reg, uint8_t len,uint8_t* buf) {
    return I2C_Read_Reg(addr, reg, len, buf);
}

int MPU6050_Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data) {
    return I2C_Write_Reg(addr, reg, len, data);
}

/**
 * @brief 从DMP获取姿态数据（欧拉角：俯仰角、横滚角、偏航角）
 * @param pitch 指向存储俯仰角的变量指针
 * @param roll 指向存储横滚角的变量指针
 * @param yaw 指向存储偏航角的变量指针
 * @return 获取状态（0：成功，1：读取失败，2：无有效四元数数据）
 */
uint8_t MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw)
{
        unsigned long sensor_timestamp;
        unsigned char more = 1;
        long quat[4];

        // 循环读空FIFO：dmp_read_fifo每次只取一包，more>0说明FIFO里还有
        // 没取完的旧数据。之前只读一包就返回，DMP输出速率(200Hz/5ms一包)
        // 比调用速率(10ms一次)快，导致FIFO里数据越积越多，硬件FIFO满了
        // 就会自动清空重置，中间那部分转动量就丢了——这就是转180°只读到
        // 150°左右的根本原因。这里改成循环读，每次调用都把当前所有积压
        // 的包全部取完，只用最后一包的四元数来算欧拉角，避免丢数据。
        do {
            if (dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more))
                return 1;
        } while (more);

    // 四元数数据解析（q30格式转浮点）
    if (sensors & INV_WXYZ_QUAT)
    {
        float q0 = quat[0] / q30;
        float q1 = quat[1] / q30;
        float q2 = quat[2] / q30;
        float q3 = quat[3] / q30;

        // 欧拉角计算（单位：度）
        *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3;  // 俯仰角

        *roll  = atan2(2 * q2 * q3 + 2 * q0 * q1,
                      -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3;  // 横滚角
        *yaw   = atan2(2 * (q1 * q2 + q0 * q3),
                       q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.3;  // 偏航角
    }
    else {
        return 2;
    }

    return 0;
}
