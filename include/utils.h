#include "MPU6050_6Axis_MotionApps20.h"

void SetOffsets(MPU6050 mpu, int TheOffsets[6]);
void Initialize(MPU6050 mpu);
float tapear(float x, float in_min, float in_max, float out_min, float out_max);