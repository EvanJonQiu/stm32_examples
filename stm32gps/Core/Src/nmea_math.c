#include "nmea_math.h"

double nmea_math_deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

double nmea_math_rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}

// 将NMEA格式坐标转换为十进制度数（标准经纬度）
double nmea_math_nmea2decimal(double nmea_coord) {
  // NMEA格式：度分.分分 -> 十进制度数
  // 例如：49.17236379 -> 49.28737965°
  int degrees = (int)nmea_coord;  // 整数部分为度
  double minutes = (nmea_coord - degrees) * 100.0;  // 小数部分*100为分
  return degrees + minutes / 60.0;  // 度 + 分/60
}

// 将十进制度数转换为NMEA格式
double nmea_math_decimal2nmea(double decimal_deg) {
  // 十进制度数 -> NMEA格式：度分.分分
  // 例如：49.28737965° -> 49.17236379
  int degrees = (int)decimal_deg;
  double minutes = (decimal_deg - degrees) * 60.0;
  return degrees + minutes / 100.0;
}

