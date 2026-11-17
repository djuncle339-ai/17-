#ifndef MPU6050_H
#define MPU6050_H
#include <stdint.h>

typedef enum {
    MPU6050_INIT_OK = 0,
    MPU6050_INIT_ERROR,
    MPU6050_I2C_ERROR,
    MPU6050_READ_OK,
    MPU6050_READ_ERROR,
}MPU6050_STATUS;
#define MPU6050_ADDR 0x68
#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43
#define CFG_REG         0X1A
#define MPU_PWR_MGMT1_REG 0X6B
#define MPU_PWR_MGMT2_REG 0X6C

/* 设置陀螺仪传感器量程范围
 * gyro_frs: 0 --> ±250dps  lsb=131.0
 *           1 --> ±500dps  lsb=65.5
 *           2 --> ±1000dps lsb=32.8
 *           3 --> ±2000dps lsb=16.4
*/
#define GYRO_FSR              (3)
#define GYRO_LSB              (16.4f)

/*
 * 设置加速度传感器量程范围
 * accel_frs: 0 --> ±2g  lsb=16384.0
 *            1 --> ±4g  lsb=8192.0
 *            2 --> ±8g  lsb=4096.0
 *            3 --> ±16g lsb=2048.0
 */
#define ACCEL_FSR             (0)
#define ACCEL_LSB             (16384.0f)

/*
 * 设置采样率
 * rate: 采样率（4~1000Hz）
 */
#define SAMPLE_RATE           (200)

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;
} MPU6050_t;

MPU6050_STATUS MPU6050_Init(void);
MPU6050_STATUS MPU6050_Read_Accel(MPU6050_t *DataStruct);
MPU6050_STATUS MPU6050_Read_Gyro(MPU6050_t *DataStruct);
MPU6050_STATUS MPU6050_Read_Temp(MPU6050_t *DataStruct);
MPU6050_STATUS MPU6050_Read_All(MPU6050_t *DataStruct);
void DMP_Init(void);
uint8_t MPU6050_DMP_Get_Data(float *pitch, float *roll, float *yaw);
#endif