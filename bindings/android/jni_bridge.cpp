#include "cloud_bridge.hpp"

#include <jni.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <new>
#include <stop_token>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unistd.h>
#include <vector>

namespace {

using kpt::android::DecodeErrorCode;
using kpt::android::DecodeResult;

struct DecodeRequest {
  std::stop_source stop;
};

std::atomic<jlong> next_handle{1};
std::mutex request_mutex;
std::unordered_map<jlong, std::shared_ptr<DecodeRequest>> requests;
std::mutex cloud_mutex;
std::unordered_map<jlong, std::shared_ptr<DecodeResult>> clouds;

jlong newHandle() noexcept {
  const auto handle = next_handle.fetch_add(1, std::memory_order_relaxed);
  return handle > 0 ? handle : next_handle.fetch_add(1, std::memory_order_relaxed);
}

std::shared_ptr<DecodeRequest> requestFor(jlong handle) {
  std::lock_guard lock(request_mutex);
  const auto found = requests.find(handle);
  return found == requests.end() ? nullptr : found->second;
}

std::shared_ptr<DecodeResult> cloudFor(jlong handle) {
  std::lock_guard lock(cloud_mutex);
  const auto found = clouds.find(handle);
  return found == clouds.end() ? nullptr : found->second;
}

jlong registerCloud(DecodeResult result) {
  const auto handle = newHandle();
  auto cloud = std::make_shared<DecodeResult>(std::move(result));
  std::lock_guard lock(cloud_mutex);
  clouds.emplace(handle, std::move(cloud));
  return handle;
}

DecodeResult failure(DecodeErrorCode code, std::string message) {
  DecodeResult result;
  result.error_code = code;
  result.error_message = std::move(message);
  return result;
}

class ScopedFd final {
public:
  explicit ScopedFd(int descriptor) : descriptor_(descriptor) {}
  ~ScopedFd() {
    if (descriptor_ >= 0)
      static_cast<void>(::close(descriptor_));
  }
  ScopedFd(const ScopedFd &) = delete;
  ScopedFd &operator=(const ScopedFd &) = delete;
  [[nodiscard]] int get() const noexcept { return descriptor_; }

private:
  int descriptor_ = -1;
};

struct ReadResult {
  DecodeErrorCode error_code = DecodeErrorCode::None;
  std::string error_message;
  std::vector<std::byte> bytes;
};

ReadResult readDescriptor(int descriptor, std::stop_token stop) {
  const int duplicate = ::dup(descriptor);
  if (duplicate < 0) {
    return {DecodeErrorCode::Io,
            "cannot duplicate input descriptor: " +
                std::error_code(errno, std::generic_category()).message(),
            {}};
  }
  ScopedFd input(duplicate);
  ReadResult result;
  constexpr std::size_t chunk_size = 64U * 1024U;
  std::array<std::byte, chunk_size> chunk{};
  while (true) {
    if (stop.stop_requested()) {
      return {DecodeErrorCode::Cancelled, "operation cancelled", {}};
    }
    const auto remaining = kpt::android::kMaximumInputBytes - result.bytes.size();
    const auto requested = std::min(chunk.size(), remaining + 1U);
    const auto count = ::read(input.get(), chunk.data(), requested);
    if (count < 0) {
      if (errno == EINTR)
        continue;
      return {DecodeErrorCode::Io,
              "cannot read input descriptor: " +
                  std::error_code(errno, std::generic_category()).message(),
              {}};
    }
    if (count == 0)
      return result;
    const auto size = static_cast<std::size_t>(count);
    if (size > remaining) {
      return {DecodeErrorCode::ResourceLimit,
              "input exceeds Android 64 MiB limit", {}};
    }
    result.bytes.insert(result.bytes.end(), chunk.begin(),
                        chunk.begin() + static_cast<std::ptrdiff_t>(size));
  }
}

std::string stringFromJava(JNIEnv *environment, jstring value) {
  if (value == nullptr)
    return {};
  const char *characters = environment->GetStringUTFChars(value, nullptr);
  if (characters == nullptr)
    return {};
  const auto length = environment->GetStringUTFLength(value);
  std::string result(characters, static_cast<std::size_t>(length));
  environment->ReleaseStringUTFChars(value, characters);
  return result;
}

} // namespace

extern "C" JNIEXPORT jlong JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_createDecodeRequest(
    JNIEnv *, jobject) {
  try {
    const auto handle = newHandle();
    auto request = std::make_shared<DecodeRequest>();
    std::lock_guard lock(request_mutex);
    requests.emplace(handle, std::move(request));
    return handle;
  } catch (...) {
    return 0;
  }
}

extern "C" JNIEXPORT jlong JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_decodeFd(
    JNIEnv *environment, jobject, jlong request_handle, jint descriptor,
    jstring source_name) {
  try {
    const auto request = requestFor(request_handle);
    if (!request) {
      return registerCloud(
          failure(DecodeErrorCode::Internal, "decode request is unavailable"));
    }
    auto input = readDescriptor(descriptor, request->stop.get_token());
    if (input.error_code != DecodeErrorCode::None) {
      return registerCloud(
          failure(input.error_code, std::move(input.error_message)));
    }
    auto name = stringFromJava(environment, source_name);
    return registerCloud(kpt::android::decodePointCloud(
        input.bytes, name, request->stop.get_token()));
  } catch (const std::bad_alloc &) {
    try {
      return registerCloud(
          failure(DecodeErrorCode::OutOfMemory, "decoder out of memory"));
    } catch (...) {
      return 0;
    }
  } catch (const std::exception &exception) {
    try {
      return registerCloud(
          failure(DecodeErrorCode::Internal, exception.what()));
    } catch (...) {
      return 0;
    }
  } catch (...) {
    return 0;
  }
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_cancelDecodeRequest(
    JNIEnv *, jobject, jlong request_handle) {
  if (const auto request = requestFor(request_handle))
    request->stop.request_stop();
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_releaseDecodeRequest(
    JNIEnv *, jobject, jlong request_handle) {
  std::lock_guard lock(request_mutex);
  requests.erase(request_handle);
}

extern "C" JNIEXPORT jint JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_cloudErrorCode(
    JNIEnv *, jobject, jlong cloud_handle) {
  const auto cloud = cloudFor(cloud_handle);
  return cloud ? static_cast<jint>(cloud->error_code)
               : static_cast<jint>(DecodeErrorCode::Internal);
}

extern "C" JNIEXPORT jstring JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_cloudErrorMessage(
    JNIEnv *environment, jobject, jlong cloud_handle) {
  const auto cloud = cloudFor(cloud_handle);
  const auto &message = cloud ? cloud->error_message : "cloud is unavailable";
  return environment->NewStringUTF(message.c_str());
}

extern "C" JNIEXPORT jlong JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_cloudPointCount(
    JNIEnv *, jobject, jlong cloud_handle) {
  const auto cloud = cloudFor(cloud_handle);
  return cloud ? static_cast<jlong>(cloud->vertices.size()) : 0;
}

extern "C" JNIEXPORT jint JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_cloudFlags(
    JNIEnv *, jobject, jlong cloud_handle) {
  const auto cloud = cloudFor(cloud_handle);
  if (!cloud)
    return 0;
  return (cloud->metadata.has_color ? 1 : 0) |
         (cloud->metadata.has_intensity ? 2 : 0) |
         (cloud->metadata.has_noise ? 4 : 0);
}

extern "C" JNIEXPORT jfloatArray JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_cloudMetadata(
    JNIEnv *environment, jobject, jlong cloud_handle) {
  const auto cloud = cloudFor(cloud_handle);
  if (!cloud)
    return nullptr;
  std::array<jfloat, 10> values{};
  std::copy(cloud->metadata.bounds.begin(), cloud->metadata.bounds.end(),
            values.begin());
  values[6] = cloud->metadata.intensity_min;
  values[7] = cloud->metadata.intensity_max;
  values[8] = cloud->metadata.intensity_p05;
  values[9] = cloud->metadata.intensity_p90;
  auto result = environment->NewFloatArray(static_cast<jsize>(values.size()));
  if (result != nullptr) {
    environment->SetFloatArrayRegion(result, 0,
                                     static_cast<jsize>(values.size()),
                                     values.data());
  }
  return result;
}

extern "C" JNIEXPORT jobject JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_cloudVertexBuffer(
    JNIEnv *environment, jobject, jlong cloud_handle) {
  const auto cloud = cloudFor(cloud_handle);
  if (!cloud || cloud->vertices.empty())
    return nullptr;
  return environment->NewDirectByteBuffer(
      cloud->vertices.data(),
      static_cast<jlong>(cloud->vertices.size() *
                         sizeof(kpt::android::PackedVertex)));
}

extern "C" JNIEXPORT void JNICALL
Java_io_github_nerdneilsfield_kpt_viewer_NativeBindings_releaseCloud(
    JNIEnv *, jobject, jlong cloud_handle) {
  std::lock_guard lock(cloud_mutex);
  clouds.erase(cloud_handle);
}
