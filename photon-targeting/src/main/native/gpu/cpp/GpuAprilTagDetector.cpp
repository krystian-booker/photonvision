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

#include "photon/gpu/GpuAprilTagDetector.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "photon/gpu/VulkanLoader.h"
#include "vkapriltag/TagDecoder.h"
#include "vkapriltag/gpu/GpuDetector.h"
#include "vkapriltag/gpu/QuadDecode.h"
#include "vkapriltag/vk/Context.h"

extern "C" {
#include "apriltag.h"
#include "tag16h5.h"
#include "tag36h11.h"
}  // extern "C"

namespace photon::gpu {
namespace {

using apriltag_vulkan::vk::Context;
using apriltag_vulkan::vk::ContextOptions;

// The Vulkan device is shared by every camera. Created on first use and kept
// for the life of the process; detectors hold a shared_ptr so it outlives them.
std::mutex g_contextMutex;
std::shared_ptr<Context> g_context;
std::string g_contextError;
bool g_contextSoftwareAllowed = false;

bool EnvAllowsSoftware() {
  const char* v = std::getenv("APRILTAG_VK_ALLOW_CPU");
  return v != nullptr && v[0] != '\0' && v[0] != '0';
}

// Returns the shared context, creating it if needed. Throws with the reason on
// failure.
std::shared_ptr<Context> SharedContext(bool allowSoftware) {
  allowSoftware = allowSoftware || EnvAllowsSoftware();
  std::lock_guard lock(g_contextMutex);
  if (g_context) {
    if (g_context->caps().is_cpu_device() && !allowSoftware) {
      throw std::runtime_error(
          "Only a software (CPU) Vulkan device is available");
    }
    return g_context;
  }
  // Retry a previous failure only if the caller is now more permissive
  if (!g_contextError.empty() && (g_contextSoftwareAllowed || !allowSoftware)) {
    throw std::runtime_error(g_contextError);
  }
  if (!VulkanLoaderAvailable()) {
    g_contextError = "Vulkan loader (libvulkan.so.1) not found";
    g_contextSoftwareAllowed = allowSoftware;
    throw std::runtime_error(g_contextError);
  }
  ContextOptions options;
  options.allow_cpu_device = allowSoftware;
  // Up to 4 cameras: lets the context spread cameras across compute queues when
  // it can
  options.expected_streams = 4;
  options.verbose = false;
  try {
    g_context = std::make_shared<Context>(options);
    g_contextError.clear();
    return g_context;
  } catch (const std::exception& e) {
    g_contextError = e.what();
    g_contextSoftwareAllowed = allowSoftware;
    throw;
  }
}

apriltag_family_t* CreateFamily(const std::string& name) {
  if (name == "tag36h11") return tag36h11_create();
  if (name == "tag16h5") return tag16h5_create();
  return nullptr;
}

void DestroyFamily(const std::string& name, apriltag_family_t* tf) {
  if (tf == nullptr) return;
  if (name == "tag36h11") tag36h11_destroy(tf);
  if (name == "tag16h5") tag16h5_destroy(tf);
}

double MsSince(std::chrono::steady_clock::time_point t0) {
  return std::chrono::duration<double, std::milli>(
             std::chrono::steady_clock::now() - t0)
      .count();
}

}  // namespace

GpuDeviceInfo QueryDevice(bool allowSoftware) {
  GpuDeviceInfo info;
  try {
    auto ctx = SharedContext(allowSoftware);
    info.available = true;
    info.software = ctx->caps().is_cpu_device();
    info.description = Context::DescribeDevice(ctx->caps());
  } catch (const std::exception& e) {
    info.available = false;
    info.description = e.what();
  }
  return info;
}

std::string UnsupportedReason(const GpuAprilTagConfig& config) {
  if (config.width <= 0 || config.height <= 0) return "invalid image size";
  if (config.decimation < 1) return "decimation must be at least 1";
  if (config.width % config.decimation != 0 ||
      config.height % config.decimation != 0) {
    return "decimation " + std::to_string(config.decimation) +
           " does not divide " + std::to_string(config.width) + "x" +
           std::to_string(config.height);
  }
  if (config.family != "tag36h11" && config.family != "tag16h5") {
    return "unsupported tag family " + config.family;
  }
  return "";
}

struct GpuAprilTagDetector::Impl {
  std::shared_ptr<Context> ctx;
  std::unique_ptr<apriltag_vulkan::vk::Lane> lane;
  std::unique_ptr<apriltag_vulkan::GpuDetector> gpu;
  std::unique_ptr<apriltag_vulkan::QuadDecode> quadDecode;
  std::unique_ptr<apriltag_vulkan::TagDecoder> tagDecoder;
  apriltag_detector_t* td = nullptr;
  apriltag_family_t* family = nullptr;
  std::string familyName;
  bool reversedBorder = false;
  std::vector<uint8_t> packed;

  ~Impl() {
    // Destruction order: decoder and GPU objects before the detector state and
    // the device
    tagDecoder.reset();
    quadDecode.reset();
    gpu.reset();
    lane.reset();
    if (td != nullptr) apriltag_detector_destroy(td);
    DestroyFamily(familyName, family);
  }
};

namespace {
// Gives each detector a different queue slot when the device has several
// compute queues
std::mutex g_streamMutex;
size_t g_nextStream = 0;
}  // namespace

GpuAprilTagDetector::GpuAprilTagDetector(const GpuAprilTagConfig& config,
                                         bool allowSoftware)
    : impl_(std::make_unique<Impl>()), config_(config) {
  if (auto reason = UnsupportedReason(config); !reason.empty()) {
    throw std::runtime_error(reason);
  }
  impl_->ctx = SharedContext(allowSoftware);

  impl_->familyName = config.family;
  impl_->family = CreateFamily(config.family);
  if (impl_->family == nullptr)
    throw std::runtime_error("could not create tag family");
  impl_->reversedBorder = impl_->family->reversed_border;

  impl_->td = apriltag_detector_create();
  apriltag_detector_add_family(impl_->td, impl_->family);
  impl_->td->quad_decimate = static_cast<float>(config.decimation);
  impl_->td->refine_edges = config.refineEdges;
  impl_->td->decode_sharpening = config.decodeSharpening;
  impl_->td->nthreads = 1;  // the TagDecoder runs its own pool
  impl_->td->qtp.cos_critical_rad = std::cos(config.criticalAngleRad);
  impl_->td->qtp.max_line_fit_mse = static_cast<float>(config.maxLineFitMSE);
  impl_->td->qtp.min_white_black_diff = config.minWhiteBlackDiff;

  apriltag_vulkan::DetectorConfig dc;
  dc.width = static_cast<uint32_t>(config.width);
  dc.height = static_cast<uint32_t>(config.height);
  dc.decimation = static_cast<uint32_t>(config.decimation);
  dc.min_white_black_diff = static_cast<uint32_t>(config.minWhiteBlackDiff);
  dc.tag_width = static_cast<uint32_t>(impl_->family->width_at_border);
  dc.reversed_border = impl_->reversedBorder;
  dc.normal_border = !impl_->reversedBorder;
  dc.max_line_fit_mse = static_cast<float>(config.maxLineFitMSE);
  dc.cos_critical_rad = std::cos(config.criticalAngleRad);
  dc.cpu_threads =
      static_cast<uint32_t>(config.numThreads > 0 ? config.numThreads : 0);

  size_t stream;
  {
    std::lock_guard lock(g_streamMutex);
    stream = g_nextStream++;
  }
  impl_->lane = impl_->ctx->CreateLane(impl_->ctx->StreamSlot(stream));
  impl_->gpu = std::make_unique<apriltag_vulkan::GpuDetector>(*impl_->ctx,
                                                              *impl_->lane, dc);
  impl_->quadDecode = std::make_unique<apriltag_vulkan::QuadDecode>(dc);
  impl_->tagDecoder = std::make_unique<apriltag_vulkan::TagDecoder>(
      impl_->td, dc.decimation, dc.cpu_threads);
}

GpuAprilTagDetector::~GpuAprilTagDetector() = default;

std::vector<GpuDetection> GpuAprilTagDetector::Detect(const uint8_t* gray,
                                                      int strideBytes) {
  const auto t0 = std::chrono::steady_clock::now();
  const int w = config_.width;
  const int h = config_.height;
  const uint8_t* frame = gray;
  if (strideBytes != w) {
    impl_->packed.resize(static_cast<size_t>(w) * h);
    for (int y = 0; y < h; ++y) {
      std::memcpy(impl_->packed.data() + static_cast<size_t>(y) * w,
                  gray + static_cast<size_t>(y) * strideBytes, w);
    }
    frame = impl_->packed.data();
  }

  impl_->gpu->Detect(frame);
  timings_.gpuMs = MsSince(t0);

  const auto t1 = std::chrono::steady_clock::now();
  const auto quads =
      impl_->quadDecode->Decode(impl_->gpu->last_line_fit_points);
  timings_.quadFitMs = MsSince(t1);

  const auto t2 = std::chrono::steady_clock::now();
  zarray_t* detections = impl_->tagDecoder->Decode(
      quads, frame, static_cast<uint32_t>(w), static_cast<uint32_t>(h),
      impl_->reversedBorder);
  timings_.decodeMs = MsSince(t2);

  std::vector<GpuDetection> out;
  out.reserve(zarray_size(detections));
  for (int i = 0; i < zarray_size(detections); ++i) {
    apriltag_detection_t* det;
    zarray_get(detections, i, &det);
    GpuDetection d;
    d.family = impl_->familyName;
    d.id = det->id;
    d.hamming = det->hamming;
    d.decisionMargin = det->decision_margin;
    for (int k = 0; k < 9; ++k) d.homography[k] = det->H->data[k];
    d.center[0] = det->c[0];
    d.center[1] = det->c[1];
    for (int k = 0; k < 4; ++k) {
      d.corners[2 * k] = det->p[k][0];
      d.corners[2 * k + 1] = det->p[k][1];
    }
    out.push_back(std::move(d));
  }
  timings_.totalMs = MsSince(t0);
  return out;
}

}  // namespace photon::gpu
