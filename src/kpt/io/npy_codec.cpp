#include "kpt/io/npy_codec.hpp"
#include "kpt/cancellation.hpp"
#include "platform/utf8_path.hpp"

#include <spdlog/spdlog.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace kpt::io_detail {
namespace {

std::string displayPath(const std::filesystem::path &path) {
  auto converted = platform::pathToUtf8(path);
  return converted ? std::move(converted).value() : "<invalid-native-path>";
}

constexpr char kMagic[] = {(char)0x93, 'N', 'U', 'M', 'P', 'Y'};

struct NpyHeader {
  int version_major = 0;
  int version_minor = 0;
  std::string dtype;
  bool fortran_order = false;
  std::vector<std::size_t> shape;
  std::size_t data_offset = 0;
};

int endiannessFromDtype(std::string_view dt) {
  if (dt.empty())
    return 0;
  if (dt[0] == '<')
    return 1; // little-endian
  if (dt[0] == '>')
    return -1; // big-endian
  if (dt[0] == '|' || dt[0] == '=')
    return 0; // not applicable / native
  return 0;
}

int itemSizeFromDtype(std::string_view dt) {
  if (dt.size() < 3)
    return 0;
  std::size_t start = (dt[0] == '<' || dt[0] == '>' || dt[0] == '|' ||
                       dt[0] == '=')
                          ? 1
                          : 0;
  if (start + 1 >= dt.size())
    return 0;
  // NumPy dtype: endianness + type_char + item_size
  // e.g. '<f4' = little-endian float32, '|u1' = uint8
  std::string num;
  for (std::size_t i = start + 1; i < dt.size(); ++i) {
    if (dt[i] >= '0' && dt[i] <= '9')
      num += dt[i];
    else
      break;
  }
  if (num.empty())
    return 0;
  return std::stoi(num);
}

char typeCharFromDtype(std::string_view dt) {
  if (dt.empty())
    return '\0';
  std::size_t start = (dt[0] == '<' || dt[0] == '>' || dt[0] == '|' ||
                       dt[0] == '=')
                          ? 1
                          : 0;
  if (start < dt.size())
    return dt[start];
  return '\0';
}

NpyHeader parseHeader(std::istream &input, const std::filesystem::path &path) {
  char magic[6];
  input.read(magic, 6);
  if (!input || std::memcmp(magic, kMagic, 6) != 0)
    throw std::runtime_error("NPY parse error: bad magic: " +
                             displayPath(path));

  NpyHeader h;
  input.read(reinterpret_cast<char *>(&h.version_major), 1);
  input.read(reinterpret_cast<char *>(&h.version_minor), 1);
  if (!input)
    throw std::runtime_error("NPY parse error: truncated version: " +
                             displayPath(path));

  std::uint32_t header_len = 0;
  if (h.version_major == 1) {
    std::uint16_t len16;
    input.read(reinterpret_cast<char *>(&len16), 2);
    header_len = len16;
  } else if (h.version_major == 2) {
    std::uint32_t len32;
    input.read(reinterpret_cast<char *>(&len32), 4);
    header_len = len32;
  } else {
    throw std::runtime_error("NPY parse error: unsupported version " +
                             std::to_string(h.version_major) + ": " +
                             displayPath(path));
  }

  if (!input)
    throw std::runtime_error("NPY parse error: truncated header length: " +
                             displayPath(path));

  std::string header(header_len, '\0');
  input.read(header.data(), header_len);
  if (!input)
    throw std::runtime_error("NPY parse error: truncated header: " +
                             displayPath(path));
  h.data_offset = 6 + 2 + (h.version_major == 2 ? 4 : 2) + header_len;

  std::string clean;
  for (char c : header) {
    if (c != ' ' && c != '\n' && c != '\r' && c != '\t')
      clean += c;
  }

  auto pos = clean.find("'descr':");
  if (pos != std::string::npos) {
    auto q1 = clean.find('\'', pos + 8);
    auto q2 = clean.find('\'', q1 + 1);
    if (q1 != std::string::npos && q2 != std::string::npos)
      h.dtype = clean.substr(q1 + 1, q2 - q1 - 1);
  }

  pos = clean.find("'fortran_order':");
  if (pos != std::string::npos) {
    if (clean.find("True", pos) != std::string::npos)
      h.fortran_order = true;
  }

  pos = clean.find("'shape':");
  if (pos != std::string::npos) {
    auto lp = clean.find('(', pos);
    auto rp = clean.find(')', pos);
    if (lp != std::string::npos && rp != std::string::npos) {
      std::string shape_str = clean.substr(lp + 1, rp - lp - 1);
      std::istringstream ss(shape_str);
      std::string tok;
      while (std::getline(ss, tok, ',')) {
        if (!tok.empty()) {
          h.shape.push_back(static_cast<std::size_t>(std::stoull(tok)));
        }
      }
    }
  }

  return h;
}

void loadFloat32Rows(const NpyHeader &h, std::istream &input,
                     PointCloudIRGB &cloud, bool &has_color,
                     bool &has_intensity, std::stop_token stop,
                     const std::filesystem::path &path) {
  const int item_size = itemSizeFromDtype(h.dtype);
  const char type_char = typeCharFromDtype(h.dtype);
  const int endian = endiannessFromDtype(h.dtype);

  if (item_size == 0)
    throw std::runtime_error("NPY parse error: bad dtype '" + h.dtype +
                             "': " + displayPath(path));

  if (h.shape.size() != 2)
    throw std::runtime_error("NPY parse error: expected 2D array, got " +
                             std::to_string(h.shape.size()) + "D: " +
                             displayPath(path));

  const std::size_t n = h.shape[0];
  const std::size_t cols = h.shape[1];
  if (cols != 3 && cols != 4 && cols != 6 && cols != 7)
    throw std::runtime_error("NPY parse error: unsupported column count " +
                             std::to_string(cols) + ": " + displayPath(path));

  has_intensity = (cols == 4 || cols == 7);
  has_color = (cols == 6 || cols == 7);

  cloud.clear();
  cloud.reserve(n);
  cloud.has_noise = false;

  const bool need_swap =
      (endian == 1 && std::endian::native != std::endian::little) ||
      (endian == -1 && std::endian::native == std::endian::little);

  auto read_value = [&](float &out) {
    if (type_char == 'f' && item_size == 4) {
      float v;
      input.read(reinterpret_cast<char *>(&v), 4);
      if (need_swap) {
        char *p = reinterpret_cast<char *>(&v);
        std::swap(p[0], p[3]);
        std::swap(p[1], p[2]);
      }
      out = v;
    } else if (type_char == 'f' && item_size == 8) {
      double v;
      input.read(reinterpret_cast<char *>(&v), 8);
      if (need_swap) {
        char *p = reinterpret_cast<char *>(&v);
        std::swap(p[0], p[7]);
        std::swap(p[1], p[6]);
        std::swap(p[2], p[5]);
        std::swap(p[3], p[4]);
      }
      out = static_cast<float>(v);
    } else if (type_char == 'i' && item_size == 4) {
      std::int32_t v;
      input.read(reinterpret_cast<char *>(&v), 4);
      if (need_swap) {
        char *p = reinterpret_cast<char *>(&v);
        std::swap(p[0], p[3]);
        std::swap(p[1], p[2]);
      }
      out = static_cast<float>(v);
    } else if (type_char == 'u' && item_size == 1) {
      std::uint8_t v;
      input.read(reinterpret_cast<char *>(&v), 1);
      out = static_cast<float>(v);
    } else {
      throw std::runtime_error("NPY parse error: unsupported dtype: " +
                               displayPath(path));
    }
  };

  for (std::size_t i = 0; i < n; ++i) {
    if (i % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();

    PointT pt;
    read_value(pt.x);
    read_value(pt.y);
    read_value(pt.z);
    if (cols == 4) {
      read_value(pt.intensity);
    } else if (cols == 6) {
      float r, g, b;
      read_value(r);
      read_value(g);
      read_value(b);
      pt.r = static_cast<std::uint8_t>(std::clamp(r, 0.0F, 255.0F));
      pt.g = static_cast<std::uint8_t>(std::clamp(g, 0.0F, 255.0F));
      pt.b = static_cast<std::uint8_t>(std::clamp(b, 0.0F, 255.0F));
    } else if (cols == 7) {
      float r, g, b;
      read_value(r);
      read_value(g);
      read_value(b);
      read_value(pt.intensity);
      pt.r = static_cast<std::uint8_t>(std::clamp(r, 0.0F, 255.0F));
      pt.g = static_cast<std::uint8_t>(std::clamp(g, 0.0F, 255.0F));
      pt.b = static_cast<std::uint8_t>(std::clamp(b, 0.0F, 255.0F));
    }
    cloud.points.push_back(pt);
  }
  cloud.width = cloud.points.size();
  cloud.height = 1;
}

} // namespace

void loadNpy(std::istream &input, const std::filesystem::path &path,
             PointCloudIRGB &cloud, bool &has_color, bool &has_intensity,
             std::stop_token stop) {
  const auto h = parseHeader(input, path);
  loadFloat32Rows(h, input, cloud, has_color, has_intensity, stop, path);
}

void saveNpy(std::ostream &output, const std::filesystem::path &path,
             const PointCloudIRGB &cloud, std::stop_token stop) {
  const std::size_t cols = 7;
  const std::size_t n = cloud.size();

  std::ostringstream header_ss;
  header_ss << "{'descr': '<f4', 'fortran_order': False, 'shape': ("
            << n << ", " << cols << "), }";

  std::string header_str = header_ss.str();
  std::size_t header_len = header_str.size() + 1;
  constexpr std::size_t align = 64;
  while ((header_len + 1) % align != 0)
    ++header_len;
  header_str.resize(header_len - 1, ' ');
  header_str += '\n';

  output.write(kMagic, 6);
  const std::uint8_t v_major = 1;
  const std::uint8_t v_minor = 0;
  output.write(reinterpret_cast<const char *>(&v_major), 1);
  output.write(reinterpret_cast<const char *>(&v_minor), 1);
  const std::uint16_t hlen = static_cast<std::uint16_t>(header_str.size());
  output.write(reinterpret_cast<const char *>(&hlen), 2);
  output.write(header_str.data(), header_str.size());
  if (!output)
    throw std::runtime_error("NPY write error: header: " + displayPath(path));

  for (std::size_t i = 0; i < n; ++i) {
    if (i % 10000 == 0 && stop.stop_requested())
      throw OperationCancelled();
    const auto &p = cloud.points[i];
    float row[7] = {p.x, p.y, p.z,
                    static_cast<float>(p.r),
                    static_cast<float>(p.g),
                    static_cast<float>(p.b),
                    p.intensity};
    output.write(reinterpret_cast<const char *>(row), sizeof(row));
  }
  if (!output)
    throw std::runtime_error("NPY write error: data: " + displayPath(path));
}

} // namespace kpt::io_detail
