#include <math.h>
#include "mpu6050.h"

// ------------------- Mahony 算法参数 -------------------
// Kp: 比例增益，决定加速度计修正陀螺仪姿态的速度（Kp越大，收敛越快，但对加速度噪声越敏感）
// Ki: 积分增益，用于消除陀螺仪的零点漂移
#define MAHONY_KP 2.0f
#define MAHONY_KI 0.005f

// 全局/静态变量定义
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 代表姿态的四元数
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;   // 误差积分累加量

/**
 * @brief  快速平方根倒数算法 (Fast Inverse Square Root)
 * @note   用于代替 1.0f / sqrtf(x)，显著节省无 FPU 主控的算力
 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/**
 * @brief  Mahony 姿态解算核心函数 (适用于 MPU6050 6轴数据)
 * @param  gx, gy, gz : 陀螺仪三轴角速度（单位：弧度/秒 rad/s）
 * @param  ax, ay, az : 加速度计三轴数据（单位：任意，已归一化，通常直接传入 g）
 * @param  dt         : 两次调用的时间间隔（单位：秒 s，例如 100Hz 采样则 dt = 0.01f）
 * @param  pitch      : 输出俯仰角指针（单位：度 °）
 * @param  roll       : 输出横滚角指针（单位：度 °）
 * @param  yaw        : 输出偏航角指针（单位：度 °，注：6轴算法的Yaw会随时间漂移）
 */
void Mahony_AHRS_Update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float dt,
                        float *pitch, float *roll, float *yaw)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    // 1. 如果加速度计处于自由落体或极高加速度下，忽略加速度计修正
    if (!(ax == 0.0f && ay == 0.0f && az == 0.0f))
    {
        // 归一化加速度计测量值
        norm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // 根据当前四元数估计的重力方向向量 (vx, vy, vz)
        vx = 2.0f * (q1q3 - q0q2);
        vy = 2.0f * (q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;

        // 计算测量重力向量与估计重力向量的向量叉积（即误差向量 ex, ey, ez）
        ex = (ay * vz - az * vy);
        ey = (az * vx - ax * vz);
        ez = (ax * vy - ay * vx);

        // 积分误差补偿
        if (MAHONY_KI > 0.0f)
        {
            exInt += ex * MAHONY_KI * dt;
            eyInt += ey * MAHONY_KI * dt;
            ezInt += ez * MAHONY_KI * dt;
            gx += exInt;
            gy += eyInt;
            gz += ezInt;
        }

        // 比例补偿
        gx += MAHONY_KP * ex;
        gy += MAHONY_KP * ey;
        gz += MAHONY_KP * ez;
    }

    // 2. 使用微分方程更新四元数
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    float qa = q0, qb = q1, qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;

    float asin_arg = -2.0f * q1 * q3 + 2.0f * q0 * q2;
    if (asin_arg > 1.0f)
        asin_arg = 1.0f;
    if (asin_arg < -1.0f)
        asin_arg = -1.0f;

    *pitch = asinf(asin_arg) * 57.29578f;
    *roll = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.29578f;
    *yaw = atan2f(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29578f;
}