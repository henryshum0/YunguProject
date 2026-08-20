#pragma once

#include <cmath>

namespace offboard::frame
{

// ----------------------------------------------------------------------
//  ENU <-> NED conventions used across the offboard package
// ----------------------------------------------------------------------
//  ENU (world): x = East, y = North, z = up,  yaw 0 = East, CCW+
//  NED (PX4):   x = North, y = East,  z = down, yaw 0 = North, CW+
//
//  A vector swaps as  (x_ned, y_ned, z_ned) = (y_enu, x_enu, -z_enu)  and
//  the inverse is the same mapping, so position/velocity/acceleration all
//  share one transform. Yaw needs an additional pi/2 offset and a sign flip.

/// Unit-circle constants shared by the yaw conversion.
constexpr double kPiHalf = 1.57079632679489661923;   ///< pi / 2
constexpr double kTwoPi  = 6.28318530717958647692;   ///< 2 * pi

/// Generic ENU -> NED mapping of a 3-vector (position, velocity or
/// acceleration — they all use the same swap + z-negate).
inline void enuToNed(double ex, double ey, double ez,
                     float &nx, float &ny, float &nz)
{
    nx = static_cast<float>(ey);
    ny = static_cast<float>(ex);
    nz = static_cast<float>(-ez);
}

/// ENU yaw (unwrapped, 0 = East, CCW+) -> NED yaw wrapped into [-pi, pi]
/// (0 = North, CW+), so PX4 always receives a heading in range and tracks the
/// shortest path.
inline double enuYawToNed(double yaw_enu)
{
    return std::remainder(kPiHalf - yaw_enu, kTwoPi);
}

/// ENU yaw rate -> NED yaw rate (sign flip only; a rate has no wrap).
inline double enuYawRateToNed(double yaw_rate_enu)
{
    return -yaw_rate_enu;
}

/// Rotate vector v by the unit quaternion q (Hamilton convention, w-first):
///   v' = q * v * q^-1
inline void rotateByQuat(double qw, double qx, double qy, double qz,
                         double vx, double vy, double vz,
                         double &ox, double &oy, double &oz)
{
    const double tx = 2.0 * (qy * vz - qz * vy);
    const double ty = 2.0 * (qz * vx - qx * vz);
    const double tz = 2.0 * (qx * vy - qy * vx);
    const double cx = qy * tz - qz * ty;
    const double cy = qz * tx - qx * tz;
    const double cz = qx * ty - qy * tx;
    ox = vx + qw * tx + cx;
    oy = vy + qw * ty + cy;
    oz = vz + qw * tz + cz;
}

/// Hamilton quaternion product p * q.
inline void quatMul(double pw, double px, double py, double pz,
                    double qw, double qx, double qy, double qz,
                    double &w, double &x, double &y, double &z)
{
    w = pw * qw - px * qx - py * qy - pz * qz;
    x = pw * qx + px * qw + py * qz - pz * qy;
    y = pw * qy - px * qz + py * qw + pz * qx;
    z = pw * qz + px * qy - py * qx + pz * qw;
}

/// Convert a NED pose (position + quaternion) into ENU.
///   p_enu = (y_ned, x_ned, -z_ned)
///   q_enu = qE * q_ned * qD
///     qE = (0, 1/sqrt(2), 1/sqrt(2), 0): NED -> ENU reference-frame change
///     qD = (0, 1, 0, 0):                 PX4-NED body <-> gz-ENU body change
///          (x fwd, y right, z down  vs  x fwd, y left, z up = 180 deg about x)
inline void nedToEnu(double pxn, double pyn, double pzn,
                     double qnw, double qnx, double qny, double qnz,
                     double &pxe, double &pye, double &pze,
                     double &qew, double &qex, double &qey, double &qez)
{
    pxe = pyn;
    pye = pxn;
    pze = -pzn;

    const double a = 1.0 / std::sqrt(2.0);
    double t1w, t1x, t1y, t1z;
    quatMul(0.0, a, a, 0.0, qnw, qnx, qny, qnz, t1w, t1x, t1y, t1z);   // qE * q_ned
    quatMul(t1w, t1x, t1y, t1z, 0.0, 1.0, 0.0, 0.0, qew, qex, qey, qez); // * qD
}

}  // namespace offboard::frame
