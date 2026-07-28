#include "platform/services.hpp"

#include "platform/utf8_path.hpp"

#import <CoreText/CoreText.h>
#import <Foundation/Foundation.h>

#include <ft2build.h>
#include FT_FREETYPE_H

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace kpt::platform {
namespace {

template <class T> class CfOwner {
public:
  explicit CfOwner(T value = nullptr) : value_(value) {}
  ~CfOwner() {
    if (value_ != nullptr)
      CFRelease(value_);
  }
  CfOwner(const CfOwner &) = delete;
  CfOwner &operator=(const CfOwner &) = delete;
  CfOwner(CfOwner &&other) noexcept
      : value_(std::exchange(other.value_, nullptr)) {}
  CfOwner &operator=(CfOwner &&other) noexcept {
    if (this != &other) {
      if (value_ != nullptr)
        CFRelease(value_);
      value_ = std::exchange(other.value_, nullptr);
    }
    return *this;
  }
  [[nodiscard]] T get() const { return value_; }

private:
  T value_;
};

class FreeTypeLibrary {
public:
  FreeTypeLibrary() {
    if (FT_Init_FreeType(&library_) != 0)
      library_ = nullptr;
  }
  ~FreeTypeLibrary() {
    if (library_ != nullptr)
      FT_Done_FreeType(library_);
  }
  FreeTypeLibrary(const FreeTypeLibrary &) = delete;
  FreeTypeLibrary &operator=(const FreeTypeLibrary &) = delete;
  [[nodiscard]] FT_Library get() const { return library_; }

private:
  FT_Library library_ = nullptr;
};

class FreeTypeFace {
public:
  FreeTypeFace(FT_Library library, const std::filesystem::path &file,
               FT_Long index) {
    if (FT_New_Face(library, file.c_str(), index, &face_) != 0)
      face_ = nullptr;
  }
  ~FreeTypeFace() {
    if (face_ != nullptr)
      FT_Done_Face(face_);
  }
  FreeTypeFace(const FreeTypeFace &) = delete;
  FreeTypeFace &operator=(const FreeTypeFace &) = delete;
  [[nodiscard]] FT_Face get() const { return face_; }

private:
  FT_Face face_ = nullptr;
};

PlatformError fontError(std::string message,
                        std::error_code system_error = {}) {
  return {PlatformErrorCode::FontFileUnavailable, std::move(message),
          system_error};
}

bool hasRequiredGlyphs(FT_Face face, std::u32string_view required_characters) {
  for (const char32_t character : required_characters) {
    if (FT_Get_Char_Index(face, static_cast<FT_ULong>(character)) == 0)
      return false;
  }
  return true;
}

std::optional<std::string> utf8String(CFStringRef value) {
  if (value == nullptr)
    return std::nullopt;
  const CFIndex maximum = CFStringGetMaximumSizeForEncoding(
                              CFStringGetLength(value), kCFStringEncodingUTF8) +
                          1;
  if (maximum <= 0)
    return std::nullopt;
  std::string result(static_cast<std::size_t>(maximum), '\0');
  if (!CFStringGetCString(value, result.data(), maximum,
                          kCFStringEncodingUTF8)) {
    return std::nullopt;
  }
  result.resize(std::char_traits<char>::length(result.c_str()));
  return result;
}

PlatformResult<std::filesystem::path> filePath(CFURLRef url) {
  if (url == nullptr)
    return fontError("matched font has no file URL");

  CfOwner<CFStringRef> path(CFURLCopyFileSystemPath(url, kCFURLPOSIXPathStyle));
  if (path.get() == nullptr)
    return fontError("cannot inspect matched font path");
  const CFIndex maximum =
      CFStringGetMaximumSizeOfFileSystemRepresentation(path.get());
  if (maximum <= 0)
    return fontError("matched font path is empty");

  std::vector<UInt8> bytes(static_cast<std::size_t>(maximum));
  if (!CFURLGetFileSystemRepresentation(url, true, bytes.data(),
                                        static_cast<CFIndex>(bytes.size()))) {
    return fontError("cannot convert matched font URL to a native path");
  }
  return std::filesystem::path(reinterpret_cast<const char *>(bytes.data()));
}

PlatformResult<std::optional<FontFace>>
matchOverride(std::string_view override_utf8,
              std::u32string_view required_characters) {
  auto file = pathFromUtf8(override_utf8);
  if (!file) {
    auto error = std::move(file).error();
    error.code = PlatformErrorCode::EnvironmentDecodeFailed;
    error.message = "KPT_CJK_FONT is not valid UTF-8";
    return error;
  }

  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(file.value(), filesystem_error) ||
      filesystem_error) {
    return fontError("KPT_CJK_FONT is not a readable regular file",
                     filesystem_error);
  }

  FreeTypeLibrary library;
  if (library.get() == nullptr)
    return fontError("cannot initialize FreeType");
  FreeTypeFace collection(library.get(), file.value(), -1);
  if (collection.get() == nullptr)
    return fontError("KPT_CJK_FONT is not a supported font file");

  const FT_Long face_count = collection.get()->num_faces;
  for (FT_Long index = 0; index < face_count; ++index) {
    FreeTypeFace face(library.get(), file.value(), index);
    if (face.get() != nullptr &&
        hasRequiredGlyphs(face.get(), required_characters)) {
      return std::optional<FontFace>{
          FontFace{std::move(file).value(), static_cast<int>(index)}};
    }
  }
  return fontError("KPT_CJK_FONT has no face containing required glyphs");
}

bool coreTextHasRequiredCharacters(CTFontRef font,
                                   std::u32string_view required_characters) {
  CfOwner<CFCharacterSetRef> characters(CTFontCopyCharacterSet(font));
  if (characters.get() == nullptr)
    return false;
  for (const char32_t character : required_characters) {
    if (!CFCharacterSetIsLongCharacterMember(
            characters.get(), static_cast<UTF32Char>(character))) {
      return false;
    }
  }
  return true;
}

PlatformResult<std::optional<FontFace>>
faceForDescriptor(CTFontDescriptorRef descriptor,
                  std::u32string_view required_characters) {
  CfOwner<CTFontRef> font(
      CTFontCreateWithFontDescriptor(descriptor, 0.0, nullptr));
  if (font.get() == nullptr ||
      !coreTextHasRequiredCharacters(font.get(), required_characters)) {
    return std::optional<FontFace>{};
  }

  CfOwner<CFTypeRef> raw_url(
      CTFontDescriptorCopyAttribute(descriptor, kCTFontURLAttribute));
  if (raw_url.get() == nullptr ||
      CFGetTypeID(raw_url.get()) != CFURLGetTypeID()) {
    return std::optional<FontFace>{};
  }
  auto path = filePath(static_cast<CFURLRef>(raw_url.get()));
  if (!path)
    return std::optional<FontFace>{};

  CfOwner<CFStringRef> raw_name(CTFontCopyPostScriptName(font.get()));
  const auto postscript_name = utf8String(raw_name.get());
  if (!postscript_name)
    return std::optional<FontFace>{};

  FreeTypeLibrary library;
  if (library.get() == nullptr)
    return fontError("cannot initialize FreeType");
  FreeTypeFace collection(library.get(), path.value(), -1);
  if (collection.get() == nullptr)
    return std::optional<FontFace>{};

  std::optional<int> matched_index;
  for (FT_Long index = 0; index < collection.get()->num_faces; ++index) {
    FreeTypeFace face(library.get(), path.value(), index);
    if (face.get() == nullptr ||
        !hasRequiredGlyphs(face.get(), required_characters) ||
        face.get()->postscript_name == nullptr ||
        *postscript_name != face.get()->postscript_name) {
      continue;
    }
    if (matched_index)
      return std::optional<FontFace>{};
    matched_index = static_cast<int>(index);
  }
  if (!matched_index)
    return std::optional<FontFace>{};
  return std::optional<FontFace>{
      FontFace{std::move(path).value(), *matched_index}};
}

PlatformResult<std::optional<FontFace>>
matchSystem(std::u32string_view required_characters) {
  CfOwner<CFMutableCharacterSetRef> required(
      CFCharacterSetCreateMutable(kCFAllocatorDefault));
  if (required.get() == nullptr)
    return fontError("cannot allocate Core Text character query");
  for (const char32_t character : required_characters) {
    CFCharacterSetAddCharactersInRange(
        required.get(),
        CFRangeMake(static_cast<CFIndex>(character), static_cast<CFIndex>(1)));
  }

  const void *keys[] = {kCTFontCharacterSetAttribute};
  const void *values[] = {required.get()};
  CfOwner<CFDictionaryRef> attributes(CFDictionaryCreate(
      kCFAllocatorDefault, keys, values, 1, &kCFTypeDictionaryKeyCallBacks,
      &kCFTypeDictionaryValueCallBacks));
  CfOwner<CTFontDescriptorRef> query(
      CTFontDescriptorCreateWithAttributes(attributes.get()));
  if (query.get() == nullptr)
    return fontError("cannot create Core Text font query");

  // Mandatory attributes must remain null: making the character-set key
  // mandatory can exclude valid fallback descriptors. Every candidate is
  // verified through both Core Text and FreeType below.
  CfOwner<CFArrayRef> candidates(
      CTFontDescriptorCreateMatchingFontDescriptors(query.get(), nullptr));
  if (candidates.get() == nullptr)
    return std::optional<FontFace>{};

  const CFIndex count = CFArrayGetCount(candidates.get());
  for (CFIndex index = 0; index < count; ++index) {
    const auto descriptor = static_cast<CTFontDescriptorRef>(
        const_cast<void *>(CFArrayGetValueAtIndex(candidates.get(), index)));
    auto matched = faceForDescriptor(descriptor, required_characters);
    if (!matched)
      continue;
    if (matched.value())
      return matched;
  }
  return std::optional<FontFace>{};
}

class MacFonts final : public Fonts {
public:
  PlatformResult<std::optional<FontFace>>
  matchUiFont(std::u32string_view required_characters) const override {
    @autoreleasepool {
      if (const char *override_font = std::getenv("KPT_CJK_FONT");
          override_font != nullptr && *override_font != '\0') {
        return matchOverride(override_font, required_characters);
      }
      return matchSystem(required_characters);
    }
  }
};

} // namespace

std::unique_ptr<Fonts> createMacFonts() { return std::make_unique<MacFonts>(); }

} // namespace kpt::platform
