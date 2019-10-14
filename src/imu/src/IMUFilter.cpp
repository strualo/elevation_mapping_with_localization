#include "dancer_io/State/IMUFilter.h"
#include "dancer_io/Common/Parameters.h"
#define FILTER_FREQUENCE 0.01 // 5ms //change for test
#define LPF_FREQ 50
#define Ki 0.02
#define Kp 0.7

using namespace std;
using namespace Eigen;

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots#Reciprocal_of_the_square_root
static float invSqrt(float x)
{
  float xhalf = 0.5f * x;
  union
  {
    float x;
    int i;
  } u;
  u.x = x;
  u.i = 0x5f3759df - (u.i >> 1);
  /* The next line can be repeated any number of times to increase accuracy */
  u.x = u.x * (1.5f - xhalf * u.x * u.x);
  return u.x;
}


template<typename T>
static inline void normalizeVector(T& vx, T& vy, T& vz)
{
  T recipNorm = invSqrt (vx * vx + vy * vy + vz * vz);
  vx *= recipNorm;
  vy *= recipNorm;
  vz *= recipNorm;
}

template<typename T>
static inline void normalizeQuaternion(T& q0, T& q1, T& q2, T& q3)
{
  T recipNorm = invSqrt (q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

template<typename T>
static inline void cross(
  T x1, T x2, T x3,
  T y1, T y2, T y3,
  T& z1, T& z2, T& z3)
{
  z1 = x2 * y3 - x3 * y2;
  z2 = x3 * y1 - x1 * y3;
  z3 = x1 * y2 - x2 * y1;
}

static inline void QuaternionToEulerAngles(float qw, float qx, float qy, float qz,
                                           float& roll, float& pitch, float& yaw)
{
    roll = atan2f(2.f * (qz*qy + qw*qx), 1-2*(qx*qx+qy*qy)); //x
    pitch =  asinf(2.f * (qw*qy - qx*qz)); //y
    yaw = atan2f(2.f * (qx*qy + qw*qz), 1-2*(qy*qy+qz*qz));//z

    if (abs(pitch * 180 / M_PI) > 75.0) // 奇异姿态,俯仰角为±90°
    {
       if(pitch > 0)
          yaw = yaw - roll;
       else
          yaw = yaw + roll;

        if(yaw > M_PI)
           yaw = yaw - 2 * M_PI;
        else if(yaw < -M_PI)
           yaw = yaw + 2 * M_PI;

        roll = 0;
    }

}

void qUtoV(
  float v1_x, float v1_y, float v1_z,
  float v2_x, float v2_y, float v2_z,
  float& q0, float& q1, float& q2, float& q3)//possible bug
{
  normalizeVector(v1_x, v1_y, v1_z);
  normalizeVector(v2_x, v2_y, v2_z);
  float sum_x = v1_x + v2_x;
  float sum_y = v1_y + v2_y;
  float sum_z = v1_z + v2_z;
  normalizeVector(sum_x, sum_y, sum_z);
  q0 = v1_x * sum_x + v1_y * sum_y + v1_z * sum_z;
  cross(v1_x, v1_y, v1_z, sum_x, sum_y, sum_z, q1, q2, q3);
  normalizeQuaternion(q0, q1, q2, q3);
}

// void Loawpass_test(float &wx, float &wy, float &wz){
//   static float coe_test = 1.0 / (1.0 + 2 * M_PI * LPF_FREQ * FILTER_FREQUENCE);
//   if(wx_old == 0.0 || wy_old == 0.0 || wz_old == 0.0)
//   {
//     wx_old = wx;
//     wy_old = wy;
//     wz_old = wz;
//   }
//   else
//   {
//     wx = coe_test * wx + (1 - coe_test) * wx_old;
//     wy = coe_test * wy + (1 - coe_test) * wy_old;
//     wz = coe_test * wz + (1 - coe_test) * wz_old;
//   }
// }

ImuFilter::ImuFilter()
{
}

ImuFilter::~ImuFilter()
{
}

void ImuFilter::Lowpass_Filter(
  float ax_new, float ay_new, float az_new,
  float &ax_old, float &ay_old, float &az_old)
{
  static float coe = 1.0 / (1.0 + 2 * M_PI * LPF_FREQ * FILTER_FREQUENCE);
  coe = 1.0;
  if(ax_old == 0.0 || ay_old == 0.0 || az_old == 0.0)
  {
    ax_old = ax_new;
    ay_old = ay_new;
    az_old = az_new;
  }
  else
  {
    ax_old = coe * ax_new + (1 - coe) * ax_old;
    ay_old = coe * ay_new + (1 - coe) * ay_old;
    az_old = coe * az_new + (1 - coe) * az_old;
  }

}

void ImuFilter::iniIMU(
    float wx, float wy, float wz,
    float ax, float ay, float az,
    int n)
{
  Lowpass_Filter(ax, ay, az, ax_last, ay_last, az_last);
  if(abs(wx) < 0.05 && abs(wy) < 0.05 && abs(wz) < 0.05)
  {
    wx_b = 1.0 * (n - 1) / n * wx_b + 1.0 / n * wx;
    wy_b = 1.0 * (n - 1) / n * wy_b + 1.0 / n * wy;
    wz_b = 1.0 * (n - 1) / n * wz_b + 1.0 / n * wz;
    double temp_gravity = sqrt(ax_last * ax_last + ay_last * ay_last + az_last * az_last);
    if (temp_gravity < 10.5 && temp_gravity > 9.5)
    {
      //ROS_INFO_STREAM("G:"<< temp_gravity);
      if (gx_ini == 0 && gy_ini == 0 && gz_ini == 0)
      {
        gx_ini = ax_last;
        gy_ini = ay_last;
        gz_ini = az_last;
      }
      else
      {
        gx_ini = 1.0 * (n - 1) / n * gx_ini + 1.0 / n * ax_last;
        gy_ini = 1.0 * (n - 1) / n * gy_ini + 1.0 / n * ay_last;
        gz_ini = 1.0 * (n - 1) / n * gz_ini + 1.0 / n * az_last;
      }
    }
  }
}

void ImuFilter::iniGravity()
{
  if(gx_ini != 0 && gy_ini != 0 && gz_ini != 0)
      g = sqrt(gx_ini * gx_ini + gy_ini * gy_ini + gz_ini * gz_ini);
}

void ImuFilter::iniQuaternion()
{
  float gx_ini_temp = gx_ini;
  float gy_ini_temp = gy_ini;
  float gz_ini_temp = gz_ini;
  normalizeVector(gx_ini_temp, gy_ini_temp, gz_ini_temp);
  qUtoV(gx_ini_temp, gy_ini_temp, gz_ini_temp, 0.0, 0.0, -1.0, q0, q1, q2, q3);
}

void ImuFilter::iniAcclast(float ax, float ay, float az)
{
  ax_last = ax;
  ay_last = ay;
  az_last = az;
}

void ImuFilter::getCleanData(
    float &wx, float &wy, float &wz,
    float ax, float ay, float az)
{
    Lowpass_Filter(ax, ay, az, ax_last, ay_last, az_last);
    //Loawpass_test(wx, wy, wz);
    UpdateBias(wx, wy, wz);
    wx = wx - wx_b;
    wy = wy - wy_b;
    wz = wz - wz_b;
}

void ImuFilter::Fusing(
    float wx, float wy, float wz,
    float ax, float ay, float az)
{
    getCleanData(wx, wy, wz, ax, ay, az);
    float ax_temp = ax_last;
    float ay_temp = ay_last;
    float az_temp = az_last;

    normalizeVector(ax_temp, ay_temp, az_temp);
    float vgx = -2.0 * q1 * q3 + 2.0 * q0 * q2;
    float vgy = -2.0 * q2 * q3 - 2.0 * q0 * q1;
    float vgz = -1.0 + 2.0 * q1 * q1 + 2.0 * q2 * q2;
    float ex,ey,ez;
    cross(ax_temp, ay_temp, az_temp, vgx, vgy, vgz, ex, ey, ez);

    float exP,eyP,ezP;
    if(parameters.state.imu_Kp == 0)
    {
      exI = exI + ex * Ki * FILTER_FREQUENCE;
      eyI = eyI + ey * Ki * FILTER_FREQUENCE;
      ezI = ezI + ez * Ki * FILTER_FREQUENCE;
      exP = ex * Kp;
      eyP = ey * Kp;
      ezP = ez * Kp;
    }
    else
    {
      exI = exI + ex * parameters.state.imu_Ki * FILTER_FREQUENCE;
      eyI = eyI + ey * parameters.state.imu_Ki * FILTER_FREQUENCE;
      ezI = ezI + ez * parameters.state.imu_Ki * FILTER_FREQUENCE;
      exP = ex * parameters.state.imu_Kp;
      eyP = ey * parameters.state.imu_Kp;
      ezP = ez * parameters.state.imu_Kp;
    }

    vgx = FILTER_FREQUENCE / 2 * (wx + exI + exP);
    vgy = FILTER_FREQUENCE / 2 * (wy + eyI + eyP);
    vgz = FILTER_FREQUENCE / 2 * (wz + ezI + ezP);

    float qa, qb, qc;
    qa = q0;
    qb = q1;
    qc = q2;

    q0 = q0 + (-qb * vgx - qc * vgy - q3 * vgz);
    q1 = q1 + (qa * vgx + qc * vgz - q3 * vgy);
    q2 = q2 + (qa * vgy - qb * vgz + q3 * vgx);
    q3 = q3 + (qa * vgz + qb * vgy - qc * vgx);
    normalizeQuaternion(q0, q1, q2, q3);
    QuaternionToEulerAngles(q0, q1, q2, q3, roll, pitch, yaw);
}

//TODO 格式转换问题
void ImuFilter::calAccWog()
{
    Quaterniond Q(q0, q1, q2, q3);
    Matrix3d R;
    R = Q.matrix();
    Vector3d  v2(0, 0, 0), v3(ax_last, ay_last, az_last);
    // v2 = R.inverse() * v1 * g;
    // v3 = v3 - v2;
    // v3 = R * v3;
    // ax_wog = v3(0);
    // ay_wog = v3(1);
    // az_wog = v3(2);
    //
    v2 = R * v3 ;
    ax_wog = -v2(0);
    ay_wog = -v2(1);
    az_wog = g - v2(2);
}

void ImuFilter::clearData()
{
  wx_old = 0.0;
  wy_old = 0.0;
  wz_old = 0.0;
  exI = 0.0;
  eyI = 0.0;
  ezI = 0.0;
  ax_last = 0;
  ay_last = 0;
  az_last = 0;
  roll = 0;
  pitch = 0;
  yaw = 0; 
}

void ImuFilter::UpdateBias(float wx, float wy, float wz)
{
  static int small_ticks = 0;


  if(abs(wx) < 0.05 && abs(wy) < 0.05 && abs(wz) < 0.05)
  {
    if(small_ticks < 10)
          small_ticks++;
    else
    {
      wx_b = 0.99 * wx_b + 0.01 * wx;
      wy_b = 0.99 * wy_b + 0.01 * wy;
      wz_b = 0.99 * wz_b + 0.01 * wz;
      double temp_gravity = sqrt(ax_last * ax_last + ay_last * ay_last + az_last * az_last);
      if (temp_gravity < 10.5 && temp_gravity > 9.5)
      {
        gx_ini = 0.95 * gx_ini + 0.05 * ax_last;
        gy_ini = 0.95 * gy_ini + 0.05 * ay_last;
        gz_ini = 0.95 * gz_ini + 0.05 * az_last;
      }
      iniGravity();
    }
  }
  else
      small_ticks = 0;
}


