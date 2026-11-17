#ifndef __MAHONY_H__
#define __MAHONY_H__

#define DEG_TO_RAD 0.0174532925f
void Mahony_AHRS_Update(float gx, float gy, float gz, 
                        float ax, float ay, float az, 
                        float dt, 
                        float *pitch, float *roll, float *yaw);



#endif /* __MAHONY_H__ */