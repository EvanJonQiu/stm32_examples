#ifndef NMEA_MATH_H
#define NMEA_MATH_H

#include <math.h>

double nmea_math_deg2rad(double deg);
double nmea_math_rad2deg(double rad);

// NMEA坐标转换为经纬度
double nmea_math_nmea2decimal(double nmea_coord);
double nmea_math_decimal2nmea(double decimal_deg);



#endif