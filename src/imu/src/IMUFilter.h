#ifndef IMU_FILTER_MADWICK_IMU_FILTER_NEW_H
#define IMU_FILTER_MADWICK_IMU_FILTER_NEW_H
#include "dancer_io/Common/Utility/Utility.h"

#include <iostream>
#include <cmath>
#include <Eigen/Core>

class ImuFilter
{
  public:

    ImuFilter();
    ~ImuFilter();
    void getOrientation(float& q0, float& q1, float& q2, float& q3)
    {
        q0 = this->q0;
        q1 = this->q1;
        q2 = this->q2;
        q3 = this->q3;
    }

    void getRPY(float& _roll, float& _pitch, float& _yaw)
    {
        _roll = roll;
        _pitch = pitch;
        _yaw = yaw;
    }

    void getAccWog(float& _ax_wog, float& _ay_wog, float& _az_wog)
    {
        _ax_wog = ax_wog;
        _ay_wog = ay_wog;
        _az_wog = az_wog;
    }

    void Fusing(
        float wx, float wy, float wz,
        float ax, float ay, float az);

    void Lowpass_Filter(float ax_new, float ay_new, float az_new,
                        float &ax_old, float &ay_old, float &az_old);

    void iniIMU(float wx, float wy, float wz,
                float ax, float ay, float az,
                int n);

    void iniGravity();

    void iniQuaternion();

    void iniAcclast(float ax, float ay, float az);

    void getCleanData(float &wx, float &wy, float &wz,
                      float ax, float ay, float az);

    void calAccWog();

    void clearData();

    void UpdateBias(float wx, float wy, float wz);

    float ax_wog, ay_wog, az_wog;

    float wx_b = 0;
    float wy_b = 0;
    float wz_b = 0;

    float g = 9.8;

  private:

    // **** state variables
    float q0, q1, q2, q3;  // quaternion
    float roll = 0;
    float pitch= 0;
    float yaw = 0;

    float gx_ini = 0;
    float gy_ini = 0;
    float gz_ini = 0;

    float ax_last = 0;
    float ay_last = 0;
    float az_last = 0;

    float wx_old = 0.0;
    float wy_old = 0.0;
    float wz_old = 0.0;
    float exI = 0.0;
    float eyI = 0.0;
    float ezI = 0.0;

};


#endif
