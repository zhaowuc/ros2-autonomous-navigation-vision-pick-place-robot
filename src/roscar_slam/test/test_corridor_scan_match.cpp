// Standalone diagnostic against the installed, unmodified Karto matcher.
// Build with C++17, karto_sdk/ROS/Eigen include paths, and -lkartoSlamToolbox.
// No ROS node, robot connection, sensor data, or motion is required.
#include <karto_sdk/Mapper.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

struct Result
{
  double response;
  karto::Pose2 pose;
  karto::Matrix3 covariance;
};

Result match_corridor(int stride)
{
  constexpr int beams = 540;
  const double pi = std::acos(-1.0);
  const int bins = beams * stride;
  const karto::Name name("corridor_" + std::to_string(bins));
  std::unique_ptr<karto::LaserRangeFinder> laser(
    karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, name));
  laser->SetMinimumRange(0.18);
  laser->SetMaximumRange(10.0);
  laser->SetRangeThreshold(10.0);
  laser->SetIs360Laser(true);
  laser->SetMinimumAngle(-pi);
  laser->SetMaximumAngle(pi);
  laser->SetAngularResolution(2.0 * pi / bins);
  karto::SensorManager::GetInstance()->RegisterSensor(laser.get());

  // The exact same 540 measured rays in both encodings. A finite corridor
  // [-6, 8] x [-1.1, 1.4] provides end-wall features for longitudinal alignment.
  karto::RangeReadingsVector ranges(bins, std::numeric_limits<double>::infinity());
  for (int i = 0; i < beams; ++i) {
    const double angle = -pi + i * 2.0 * pi / beams;
    const double x = std::cos(angle);
    const double y = std::sin(angle);
    const double along = std::abs(x) < 1e-12 ? 10.0 : (x > 0.0 ? 8.0 : -6.0) / x;
    const double across = std::abs(y) < 1e-12 ? 10.0 : (y > 0.0 ? 1.4 : -1.1) / y;
    ranges[i * stride] = std::min(along, across);
  }

  karto::Mapper mapper;
  mapper.setParamCoarseSearchAngleOffset(0.349);
  mapper.setParamCoarseAngleResolution(0.0349);
  mapper.setParamFineSearchAngleOffset(0.00349);
  mapper.setParamUseResponseExpansion(false);
  std::unique_ptr<karto::ScanMatcher> matcher(
    karto::ScanMatcher::Create(&mapper, 0.5, 0.01, 0.1, 10.0));
  karto::LocalizedRangeScan reference(name, ranges);
  reference.SetOdometricPose(karto::Pose2(0.0, 0.0, 0.0));
  reference.SetCorrectedPose(karto::Pose2(0.0, 0.0, 0.0));
  karto::LocalizedRangeScan current(name, ranges);
  current.SetOdometricPose(karto::Pose2(0.12, -0.08, 0.06));
  current.SetCorrectedPose(karto::Pose2(0.12, -0.08, 0.06));
  const karto::LocalizedRangeScanVector references{&reference};
  Result result;
  result.response = matcher->MatchScan(
    &current, references, result.pose, result.covariance, false, true);
  std::cout << "bins=" << bins << " response=" << result.response
            << " recovered_pose=" << result.pose.GetX() << ',' << result.pose.GetY()
            << ',' << result.pose.GetHeading()
            << " variance_xy=" << result.covariance(0, 0)
            << ',' << result.covariance(1, 1) << '\n';
  karto::SensorManager::GetInstance()->UnregisterSensor(laser.get());
  return result;
}

int main()
{
  const Result compact = match_corridor(1);
  const Result sparse = match_corridor(10);
  if (compact.response < 0.65 || sparse.response > 0.100001 || sparse.response <= 0.0) {
    throw std::runtime_error("Compact scan must clear 0.65; sparse encoding caps score at 0.1");
  }
  if (std::hypot(compact.pose.GetX(), compact.pose.GetY()) > 0.05 ||
    std::abs(compact.pose.GetHeading()) > 0.015)
  {
    throw std::runtime_error("Compact scan did not recover the known corridor pose");
  }
  std::cout << "PASS: compact scan restores usable confidence and known-pose convergence\n";
}
