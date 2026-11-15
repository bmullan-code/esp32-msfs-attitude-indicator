// QMI8658 6-axis IMU: only the accelerometer is used, to sense which way is down.
#pragma once

struct Vec3 { float x, y, z; };

namespace imu {
bool begin();                 // false if the sensor does not answer
bool readAccel(Vec3& g);      // acceleration in g, sensor axes
}
