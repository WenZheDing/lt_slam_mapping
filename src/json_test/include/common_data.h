#pragma once
#include <iostream>
#include <string>
#include <vector>

namespace MapManagerDataType {
class Time {
    public:
    uint32_t sec;
    uint32_t nsec;
};

class Header {
 public:
  std::string frame_id;
  Time stamp;
};

class Bool {
 public:
  bool flag;
};

class Position {
 public:
    float x;
    float y;
    float heading;
};

class Point {
 public:
  float x;
  float y;
  float z;
  float intensity;
  uint32_t ring;
};

class PointCloud2 {
 public:
  std::vector<Point> points;
};

class Imu {
 public:
  float Time;
  float z;
  float y;
  float x;
  float AngleRoll;
  float AnglePitch;
  float AngleHeading;
};

class Lidar {
 public:
  Header header;
  uint32_t height;
  uint32_t width;
  bool is_bigendian;
  bool is_dense;
  uint32_t point_step;
  uint32_t row_step;
  PointCloud2 data;
};

class OccupancyGrid {
 public:
  Header header;
  Position origin;
  float width;
  float height;
  float length;
  float resolution;
  std::vector<uint8_t> data;
};

}  // namespace MapManagerDataType