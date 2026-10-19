/*
 * Copyright (C) Photon Vision.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace photon::gpu {

/** Detector settings, mirroring WPILib's AprilTagDetector Config and
 * QuadThresholdParameters. */
struct GpuAprilTagConfig {
  int width = 0;
  int height = 0;
  /** Integer decimation; must divide width and height. */
  int decimation = 2;
  bool refineEdges = true;
  double decodeSharpening = 0.25;
  /** CPU threads for quad fitting and decoding. 0 = automatic. */
  int numThreads = 0;
  double criticalAngleRad = 0.785398;
  double maxLineFitMSE = 10.0;
  int minWhiteBlackDiff = 5;
  /** AprilTag family name: "tag36h11" or "tag16h5". */
  std::string family = "tag36h11";
};

struct GpuDetection {
  std::string family;
  int id = 0;
  int hamming = 0;
  float decisionMargin = 0;
  double homography[9] = {};
  double center[2] = {};
  /** Corners x0, y0 .. x3, y3, in the same order as the CPU detector. */
  double corners[8] = {};
};

struct GpuTimings {
  double gpuMs = 0;
  double quadFitMs = 0;
  double decodeMs = 0;
  double totalMs = 0;
};

struct GpuDeviceInfo {
  bool available = false;
  bool software = false;
  /** Device description, or why the GPU path is unavailable. */
  std::string description;
};

/**
 * Probes for a usable Vulkan device (creating the shared device context on
 * success). Software (CPU) Vulkan implementations are refused unless
 * allowSoftware is set or APRILTAG_VK_ALLOW_CPU=1.
 */
GpuDeviceInfo QueryDevice(bool allowSoftware);

/** Empty if the config can run on the GPU, otherwise the reason it can't. */
std::string UnsupportedReason(const GpuAprilTagConfig& config);

/**
 * One camera's GPU AprilTag detector. All detectors share one Vulkan device;
 * each has its own submission lane, so different cameras may call Detect
 * concurrently. A single detector is not thread-safe.
 */
class GpuAprilTagDetector {
 public:
  /** Throws std::runtime_error if the GPU or config is unusable. */
  GpuAprilTagDetector(const GpuAprilTagConfig& config, bool allowSoftware);
  ~GpuAprilTagDetector();

  GpuAprilTagDetector(const GpuAprilTagDetector&) = delete;
  GpuAprilTagDetector& operator=(const GpuAprilTagDetector&) = delete;

  /** Detects tags in an 8-bit grayscale image of the configured size. Throws on
   * GPU errors. */
  std::vector<GpuDetection> Detect(const uint8_t* gray, int strideBytes);

  const GpuTimings& LastTimings() const { return timings_; }
  const GpuAprilTagConfig& config() const { return config_; }

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
  GpuAprilTagConfig config_;
  GpuTimings timings_;
};

}  // namespace photon::gpu
