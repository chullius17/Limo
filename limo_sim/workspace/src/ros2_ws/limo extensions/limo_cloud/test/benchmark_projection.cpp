#include "limo_cloud/projector.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iostream>
#include <vector>

int main()
{
  for (auto shape : {std::array<uint32_t, 2>{320, 120}, {640, 480}}) {
    sensor_msgs::msg::Image depth, labels;
    depth.width = labels.width = shape[0];
    depth.height = labels.height = shape[1];
    depth.encoding = "32FC1";
    labels.encoding = "mono8";
    depth.step = depth.width * 4;
    labels.step = labels.width;
    depth.data.resize(depth.step * depth.height);
    labels.data.resize(labels.step * labels.height);
    for (size_t i = 0; i < labels.data.size(); ++i) {
      const float d = 0.2F + static_cast<float>(i % 200) * 0.01F;
      std::memcpy(depth.data.data() + i * 4, &d, 4);
      labels.data[i] = 1 + i % 255;
    }
    sensor_msgs::msg::CameraInfo info;
    info.width = 640;
    info.height = 480;
    info.k = {513.3, 0, 319.4, 0, 514.2, 241.2, 0, 0, 1};
    geometry_msgs::msg::Transform transform;
    transform.rotation.w = 1;
    limo_cloud::Projector projector;
    limo_cloud::ProjectionOptions options;
    sensor_msgs::msg::PointCloud2 output;
    std::vector<double> samples;
    for (int run = 0; run < 1050; ++run) {
      const auto start = std::chrono::steady_clock::now();
      projector.project(depth, labels, info, transform, options, output);
      if (run >= 50) {
        samples.push_back(std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - start).count());
      }
    }
    std::sort(samples.begin(), samples.end());
    std::cout << shape[0] << 'x' << shape[1] << ": " << output.width
              << " points; median=" << samples[500] << " us; p95=" << samples[950]
              << " us (projection + packing only, 1000 frames)\n";
  }
}
