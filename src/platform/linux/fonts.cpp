#include "platform/services.hpp"

#include "platform/utf8_path.hpp"

#include <fontconfig/fontconfig.h>
#include <ft2build.h>
#include FT_FREETYPE_H

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

  FT_Library library = nullptr;
  if (FT_Init_FreeType(&library) != 0)
    return fontError("cannot initialize FreeType");
  const auto release_library =
      std::unique_ptr<std::remove_pointer_t<FT_Library>,
                      decltype(&FT_Done_FreeType)>(library, &FT_Done_FreeType);

  FT_Face collection = nullptr;
  if (FT_New_Face(library, file.value().c_str(), -1, &collection) != 0)
    return fontError("KPT_CJK_FONT is not a supported font file");
  const FT_Long face_count = collection->num_faces;
  FT_Done_Face(collection);

  for (FT_Long index = 0; index < face_count; ++index) {
    FT_Face face = nullptr;
    if (FT_New_Face(library, file.value().c_str(), index, &face) != 0)
      continue;
    bool matched = true;
    for (const char32_t character : required_characters) {
      if (FT_Get_Char_Index(face, static_cast<FT_ULong>(character)) == 0) {
        matched = false;
        break;
      }
    }
    FT_Done_Face(face);
    if (matched) {
      return std::optional<FontFace>{
          FontFace{std::move(file).value(), static_cast<int>(index)}};
    }
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

    std::optional<PlatformError> first_candidate_error;
    bool inspected_candidate = false;
    for (int index = 0; index < candidates->nfont; ++index) {
      auto matched =
          fontFaceFromPattern(candidates->fonts[index], required_characters);
      if (!matched) {
        if (!first_candidate_error)
          first_candidate_error = std::move(matched).error();
        continue;
      }
      inspected_candidate = true;
      if (matched.value())
        return matched;
    }
    if (!inspected_candidate && first_candidate_error)
      return std::move(*first_candidate_error);
    return std::optional<FontFace>{};
  }
};

} // namespace

std::unique_ptr<Fonts> createLinuxFonts() {
  return std::make_unique<LinuxFonts>();
}

} // namespace kpt::platform
