#include "platform/services.hpp"

#include "platform/utf8_path.hpp"

#include <fontconfig/fontconfig.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <utility>

namespace kpt::platform {
namespace {

using PatternPtr = std::unique_ptr<FcPattern, decltype(&FcPatternDestroy)>;
using CharSetPtr = std::unique_ptr<FcCharSet, decltype(&FcCharSetDestroy)>;
using FontSetPtr = std::unique_ptr<FcFontSet, decltype(&FcFontSetDestroy)>;

PlatformError fontError(std::string message,
                        std::error_code system_error = {}) {
  return {PlatformErrorCode::FontFileUnavailable, std::move(message),
          system_error};
}

CharSetPtr makeRequiredCharSet(std::u32string_view required_characters) {
  CharSetPtr charset(FcCharSetCreate(), &FcCharSetDestroy);
  if (!charset)
    return charset;
  for (const char32_t character : required_characters)
    FcCharSetAddChar(charset.get(), static_cast<FcChar32>(character));
  return charset;
}

bool hasRequiredCharacters(FcPattern *pattern,
                           std::u32string_view required_characters) {
  FcCharSet *charset = nullptr;
  if (FcPatternGetCharSet(pattern, FC_CHARSET, 0, &charset) != FcResultMatch ||
      charset == nullptr) {
    return false;
  }
  for (const char32_t character : required_characters) {
    if (FcCharSetHasChar(charset, static_cast<FcChar32>(character)) == FcFalse)
      return false;
  }
  return true;
}

PlatformResult<std::optional<FontFace>>
fontFaceFromPattern(FcPattern *pattern,
                    std::u32string_view required_characters) {
  if (!hasRequiredCharacters(pattern, required_characters))
    return std::optional<FontFace>{};

  FcChar8 *filename = nullptr;
  if (FcPatternGetString(pattern, FC_FILE, 0, &filename) != FcResultMatch ||
      filename == nullptr) {
    return fontError("matched font has no local file");
  }

  auto file = pathFromUtf8(reinterpret_cast<const char *>(filename));
  if (!file)
    return fontError("matched font path is not valid UTF-8",
                     file.error().system_error);

  int encoded_index = 0;
  if (FcPatternGetInteger(pattern, FC_INDEX, 0, &encoded_index) !=
      FcResultMatch) {
    encoded_index = 0;
  }

  std::error_code error;
  if (!std::filesystem::is_regular_file(file.value(), error) || error)
    return fontError("matched font file is not readable", error);
  std::ifstream readable(file.value(), std::ios::binary);
  if (!readable)
    return fontError("matched font file cannot be opened");

  // Fontconfig stores the face number in the low 16 bits. High bits identify
  // named variable-font instances and are not ImGui's FontNo.
  return std::optional<FontFace>{
      FontFace{std::move(file).value(), encoded_index & 0xFFFF}};
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

  auto filename = pathToUtf8(file.value());
  if (!filename)
    return fontError("KPT_CJK_FONT cannot be represented as UTF-8");

  FontSetPtr faces(FcFontSetCreate(), &FcFontSetDestroy);
  if (!faces)
    return fontError("cannot allocate Fontconfig font set");
  int count = 0;
  if (FcFreeTypeQueryAll(
          reinterpret_cast<const FcChar8 *>(filename.value().c_str()), 0,
          nullptr, &count, faces.get()) == FcFalse ||
      count == 0) {
    return fontError("KPT_CJK_FONT is not a supported font file");
  }

  for (int index = 0; index < faces->nfont; ++index) {
    auto matched =
        fontFaceFromPattern(faces->fonts[index], required_characters);
    if (!matched)
      return std::move(matched).error();
    if (matched.value())
      return matched;
  }
  return fontError("KPT_CJK_FONT has no face containing required glyphs");
}

class LinuxFonts final : public Fonts {
public:
  PlatformResult<std::optional<FontFace>>
  matchUiFont(std::u32string_view required_characters) const override {
    if (FcInit() == FcFalse)
      return fontError("Fontconfig initialization failed");

    if (const char *override_font = std::getenv("KPT_CJK_FONT");
        override_font != nullptr && *override_font != '\0') {
      return matchOverride(override_font, required_characters);
    }

    auto required = makeRequiredCharSet(required_characters);
    if (!required)
      return fontError("cannot allocate Fontconfig character set");

    PatternPtr query(FcPatternCreate(), &FcPatternDestroy);
    if (!query)
      return fontError("cannot allocate Fontconfig query");
    FcPatternAddCharSet(query.get(), FC_CHARSET, required.get());
    FcPatternAddBool(query.get(), FC_SCALABLE, FcTrue);
    FcConfigSubstitute(nullptr, query.get(), FcMatchPattern);
    FcDefaultSubstitute(query.get());

    FcResult result = FcResultNoMatch;
    FontSetPtr candidates(
        FcFontSort(nullptr, query.get(), FcFalse, nullptr, &result),
        &FcFontSetDestroy);
    if (!candidates)
      return fontError("Fontconfig match failed");

    for (int index = 0; index < candidates->nfont; ++index) {
      auto matched =
          fontFaceFromPattern(candidates->fonts[index], required_characters);
      if (!matched)
        continue;
      if (matched.value())
        return matched;
    }
    return std::optional<FontFace>{};
  }
};

} // namespace

std::unique_ptr<Fonts> createLinuxFonts() {
  return std::make_unique<LinuxFonts>();
}

} // namespace kpt::platform
