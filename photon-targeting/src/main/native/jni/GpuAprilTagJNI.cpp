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

#include <exception>
#include <string>
#include <vector>

#include <jni.h>
#include <opencv2/core/mat.hpp>

#include "org_photonvision_jni_GpuAprilTagJNI.h"

#ifdef __linux__
#include "photon/gpu/GpuAprilTagDetector.h"
#endif

namespace {

void ThrowRuntime(JNIEnv* env, const std::string& msg) {
  jclass cls = env->FindClass("java/lang/RuntimeException");
  if (cls != nullptr) env->ThrowNew(cls, msg.c_str());
}

std::string ToString(JNIEnv* env, jstring s) {
  if (s == nullptr) return "";
  const char* chars = env->GetStringUTFChars(s, nullptr);
  std::string out(chars);
  env->ReleaseStringUTFChars(s, chars);
  return out;
}

jobjectArray StringArray(JNIEnv* env, const std::vector<std::string>& values) {
  jclass stringClass = env->FindClass("java/lang/String");
  jobjectArray arr = env->NewObjectArray(values.size(), stringClass, nullptr);
  for (size_t i = 0; i < values.size(); ++i) {
    jstring s = env->NewStringUTF(values[i].c_str());
    env->SetObjectArrayElement(arr, i, s);
    env->DeleteLocalRef(s);
  }
  return arr;
}

}  // namespace

extern "C" {

/*
 * Class:     org_photonvision_jni_GpuAprilTagJNI
 * Method:    isSupportedPlatform
 * Signature: ()Z
 */
JNIEXPORT jboolean JNICALL
Java_org_photonvision_jni_GpuAprilTagJNI_isSupportedPlatform
  (JNIEnv*, jclass)
{
#ifdef __linux__
  return JNI_TRUE;
#else
  return JNI_FALSE;
#endif
}

/*
 * Class:     org_photonvision_jni_GpuAprilTagJNI
 * Method:    queryDevice
 * Signature: (Z)[Ljava/lang/Object;
 */
JNIEXPORT jobjectArray JNICALL
Java_org_photonvision_jni_GpuAprilTagJNI_queryDevice
  (JNIEnv* env, jclass, jboolean allowSoftware)
{
#ifdef __linux__
  auto info = photon::gpu::QueryDevice(allowSoftware);
  return StringArray(env, {info.available ? "true" : "false",
                           info.software ? "true" : "false", info.description});
#else
  return StringArray(env, {"false", "false",
                           "GPU AprilTag detection is only built for Linux"});
#endif
}

/*
 * Class:     org_photonvision_jni_GpuAprilTagJNI
 * Method:    unsupportedReason
 * Signature: (IIILjava/lang/String;)Ljava/lang/String;
 */
JNIEXPORT jstring JNICALL
Java_org_photonvision_jni_GpuAprilTagJNI_unsupportedReason
  (JNIEnv* env, jclass, jint width, jint height, jint decimation,
   jstring family)
{
#ifdef __linux__
  photon::gpu::GpuAprilTagConfig config;
  config.width = width;
  config.height = height;
  config.decimation = decimation;
  config.family = ToString(env, family);
  auto reason = photon::gpu::UnsupportedReason(config);
  return reason.empty() ? nullptr : env->NewStringUTF(reason.c_str());
#else
  return env->NewStringUTF("GPU AprilTag detection is only built for Linux");
#endif
}

/*
 * Class:     org_photonvision_jni_GpuAprilTagJNI
 * Method:    create
 * Signature: (IIIZDIDDILjava/lang/String;Z)J
 */
JNIEXPORT jlong JNICALL
Java_org_photonvision_jni_GpuAprilTagJNI_create
  (JNIEnv* env, jclass, jint width, jint height, jint decimation,
   jboolean refineEdges, jdouble decodeSharpening, jint numThreads,
   jdouble criticalAngleRad, jdouble maxLineFitMSE, jint minWhiteBlackDiff,
   jstring family, jboolean allowSoftware)
{
#ifdef __linux__
  photon::gpu::GpuAprilTagConfig config;
  config.width = width;
  config.height = height;
  config.decimation = decimation;
  config.refineEdges = refineEdges;
  config.decodeSharpening = decodeSharpening;
  config.numThreads = numThreads;
  config.criticalAngleRad = criticalAngleRad;
  config.maxLineFitMSE = maxLineFitMSE;
  config.minWhiteBlackDiff = minWhiteBlackDiff;
  config.family = ToString(env, family);
  try {
    return reinterpret_cast<jlong>(
        new photon::gpu::GpuAprilTagDetector(config, allowSoftware));
  } catch (const std::exception& e) {
    ThrowRuntime(env,
                 std::string("GPU AprilTag detector unavailable: ") + e.what());
    return 0;
  }
#else
  ThrowRuntime(env, "GPU AprilTag detection is only built for Linux");
  return 0;
#endif
}

/*
 * Class:     org_photonvision_jni_GpuAprilTagJNI
 * Method:    destroy
 * Signature: (J)V
 */
JNIEXPORT void JNICALL
Java_org_photonvision_jni_GpuAprilTagJNI_destroy
  (JNIEnv*, jclass, jlong handle)
{
#ifdef __linux__
  delete reinterpret_cast<photon::gpu::GpuAprilTagDetector*>(handle);
#endif
}

/*
 * Class:     org_photonvision_jni_GpuAprilTagJNI
 * Method:    detect
 * Signature: (JJ)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_org_photonvision_jni_GpuAprilTagJNI_detect
  (JNIEnv* env, jclass, jlong handle, jlong matAddr)
{
#ifdef __linux__
  auto* detector = reinterpret_cast<photon::gpu::GpuAprilTagDetector*>(handle);
  auto* mat = reinterpret_cast<cv::Mat*>(matAddr);
  if (detector == nullptr || mat == nullptr) {
    ThrowRuntime(env, "null detector or image");
    return nullptr;
  }
  if (mat->type() != CV_8UC1 || mat->cols != detector->config().width ||
      mat->rows != detector->config().height) {
    ThrowRuntime(env,
                 "GPU AprilTag detector needs an 8-bit grayscale image of the "
                 "configured size");
    return nullptr;
  }
  try {
    auto detections =
        detector->Detect(mat->data, static_cast<int>(mat->step[0]));
    constexpr int kStride = 22;
    std::vector<double> packed(detections.size() * kStride);
    for (size_t i = 0; i < detections.size(); ++i) {
      const auto& d = detections[i];
      double* out = packed.data() + i * kStride;
      out[0] = d.id;
      out[1] = d.hamming;
      out[2] = d.decisionMargin;
      for (int k = 0; k < 9; ++k) out[3 + k] = d.homography[k];
      out[12] = d.center[0];
      out[13] = d.center[1];
      for (int k = 0; k < 8; ++k) out[14 + k] = d.corners[k];
    }
    jdoubleArray arr = env->NewDoubleArray(packed.size());
    env->SetDoubleArrayRegion(arr, 0, packed.size(), packed.data());
    return arr;
  } catch (const std::exception& e) {
    ThrowRuntime(env,
                 std::string("GPU AprilTag detection failed: ") + e.what());
    return nullptr;
  }
#else
  ThrowRuntime(env, "GPU AprilTag detection is only built for Linux");
  return nullptr;
#endif
}

/*
 * Class:     org_photonvision_jni_GpuAprilTagJNI
 * Method:    lastTimings
 * Signature: (J)[D
 */
JNIEXPORT jdoubleArray JNICALL
Java_org_photonvision_jni_GpuAprilTagJNI_lastTimings
  (JNIEnv* env, jclass, jlong handle)
{
  double values[4] = {0, 0, 0, 0};
#ifdef __linux__
  if (auto* detector =
          reinterpret_cast<photon::gpu::GpuAprilTagDetector*>(handle)) {
    const auto& t = detector->LastTimings();
    values[0] = t.gpuMs;
    values[1] = t.quadFitMs;
    values[2] = t.decodeMs;
    values[3] = t.totalMs;
  }
#endif
  jdoubleArray arr = env->NewDoubleArray(4);
  env->SetDoubleArrayRegion(arr, 0, 4, values);
  return arr;
}

}  // extern "C"
