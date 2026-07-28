#include "platform/services.hpp"

#include "platform/utf8_path.hpp"

#define WIN32_LEAN_AND_MEAN
#include <dwrite.h>
#include <windows.h>
#include <wrl/client.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace kpt::platform {
namespace {

using Microsoft::WRL::ComPtr;

PlatformError fontError(std::string message, HRESULT result = S_OK) {
  std::error_code error;
  if (FAILED(result))
    error = {static_cast<int>(result), std::system_category()};
  return {PlatformErrorCode::FontFileUnavailable, std::move(message), error};
}

bool hasRequiredGlyphs(IDWriteFontFace *face,
                       std::u32string_view required_characters) {
  if (required_characters.empty())
    return true;

  std::vector<UINT32> code_points;
  code_points.reserve(required_characters.size());
  for (const char32_t character : required_characters)
    code_points.push_back(static_cast<UINT32>(character));

  std::vector<UINT16> glyphs(code_points.size());
  if (FAILED(face->GetGlyphIndices(code_points.data(),
                                   static_cast<UINT32>(code_points.size()),
                                   glyphs.data()))) {
    return false;
  }
  for (const UINT16 glyph : glyphs) {
    if (glyph == 0)
      return false;
  }
  return true;
}

PlatformResult<std::optional<FontFace>>
localFontFace(IDWriteFontFace *face, std::u32string_view required_characters) {
  if (!hasRequiredGlyphs(face, required_characters))
    return std::optional<FontFace>{};

  UINT32 file_count = 0;
  HRESULT result = face->GetFiles(&file_count, nullptr);
  if (FAILED(result))
    return fontError("cannot inspect matched font files", result);
  if (file_count != 1)
    return std::optional<FontFace>{};

  IDWriteFontFile *raw_file = nullptr;
  file_count = 1;
  result = face->GetFiles(&file_count, &raw_file);
  if (FAILED(result))
    return fontError("cannot obtain matched font file", result);
  ComPtr<IDWriteFontFile> file;
  file.Attach(raw_file);

  const void *reference_key = nullptr;
  UINT32 reference_key_size = 0;
  result = file->GetReferenceKey(&reference_key, &reference_key_size);
  if (FAILED(result))
    return fontError("cannot obtain matched font reference key", result);

  ComPtr<IDWriteFontFileLoader> loader;
  result = file->GetLoader(&loader);
  if (FAILED(result))
    return fontError("cannot obtain matched font loader", result);

  ComPtr<IDWriteLocalFontFileLoader> local_loader;
  result = loader.As(&local_loader);
  if (FAILED(result))
    return std::optional<FontFace>{};

  UINT32 path_length = 0;
  result = local_loader->GetFilePathLengthFromKey(
      reference_key, reference_key_size, &path_length);
  if (FAILED(result))
    return fontError("cannot obtain matched font path length", result);

  std::wstring path(static_cast<std::size_t>(path_length) + 1U, L'\0');
  result = local_loader->GetFilePathFromKey(reference_key, reference_key_size,
                                            path.data(),
                                            static_cast<UINT32>(path.size()));
  if (FAILED(result))
    return fontError("cannot obtain matched font path", result);
  path.resize(path_length);

  return std::optional<FontFace>{
      FontFace{std::filesystem::path(std::move(path)),
               static_cast<int>(face->GetIndex())}};
}

PlatformResult<ComPtr<IDWriteFactory>> createFactory() {
  ComPtr<IDWriteFactory> factory;
  const HRESULT result = DWriteCreateFactory(
      DWRITE_FACTORY_TYPE_SHARED, __uuidof(IDWriteFactory),
      reinterpret_cast<IUnknown **>(factory.GetAddressOf()));
  if (FAILED(result))
    return fontError("cannot create DirectWrite factory", result);
  return factory;
}

PlatformResult<std::optional<FontFace>>
matchOverride(IDWriteFactory *factory, const std::filesystem::path &file_path,
              std::u32string_view required_characters) {
  std::error_code filesystem_error;
  if (!std::filesystem::is_regular_file(file_path, filesystem_error) ||
      filesystem_error) {
    return fontError("KPT_CJK_FONT is not a readable regular file");
  }
  std::ifstream readable(file_path, std::ios::binary);
  if (!readable)
    return fontError("KPT_CJK_FONT cannot be opened");

  ComPtr<IDWriteFontFile> file;
  HRESULT result =
      factory->CreateFontFileReference(file_path.c_str(), nullptr, &file);
  if (FAILED(result))
    return fontError("KPT_CJK_FONT is not a supported local font", result);

  BOOL supported = FALSE;
  DWRITE_FONT_FILE_TYPE file_type = DWRITE_FONT_FILE_TYPE_UNKNOWN;
  DWRITE_FONT_FACE_TYPE face_type = DWRITE_FONT_FACE_TYPE_UNKNOWN;
  UINT32 face_count = 0;
  result = file->Analyze(&supported, &file_type, &face_type, &face_count);
  if (FAILED(result) || supported == FALSE || face_count == 0)
    return fontError("KPT_CJK_FONT is not a supported font file", result);

  IDWriteFontFile *files[] = {file.Get()};
  for (UINT32 face_index = 0; face_index < face_count; ++face_index) {
    ComPtr<IDWriteFontFace> face;
    result = factory->CreateFontFace(face_type, 1, files, face_index,
                                     DWRITE_FONT_SIMULATIONS_NONE, &face);
    if (FAILED(result))
      continue;
    if (hasRequiredGlyphs(face.Get(), required_characters)) {
      return std::optional<FontFace>{
          FontFace{file_path, static_cast<int>(face_index)}};
    }
  }
  return fontError("KPT_CJK_FONT has no face containing required glyphs");
}

PlatformResult<std::optional<FontFace>>
matchSystem(IDWriteFactory *factory, std::u32string_view required_characters) {
  ComPtr<IDWriteFontCollection> collection;
  HRESULT result = factory->GetSystemFontCollection(&collection, FALSE);
  if (FAILED(result))
    return fontError("cannot enumerate the DirectWrite font collection",
                     result);

  const UINT32 family_count = collection->GetFontFamilyCount();
  std::optional<PlatformError> first_candidate_error;
  bool inspected_candidate = false;
  for (UINT32 family_index = 0; family_index < family_count; ++family_index) {
    ComPtr<IDWriteFontFamily> family;
    if (FAILED(collection->GetFontFamily(family_index, &family)))
      continue;
    const UINT32 font_count = family->GetFontCount();
    for (UINT32 font_index = 0; font_index < font_count; ++font_index) {
      ComPtr<IDWriteFont> font;
      if (FAILED(family->GetFont(font_index, &font)))
        continue;
      ComPtr<IDWriteFontFace> face;
      if (FAILED(font->CreateFontFace(&face)))
        continue;

      auto matched = localFontFace(face.Get(), required_characters);
      if (!matched) {
        if (!first_candidate_error)
          first_candidate_error = std::move(matched).error();
        continue;
      }
      inspected_candidate = true;
      if (matched.value())
        return matched;
    }
  }
  if (!inspected_candidate && first_candidate_error)
    return std::move(*first_candidate_error);
  return std::optional<FontFace>{};
}

class WindowsFonts final : public Fonts {
public:
  PlatformResult<std::optional<FontFace>>
  matchUiFont(std::u32string_view required_characters) const override {
    auto factory = createFactory();
    if (!factory)
      return std::move(factory).error();

    if (const wchar_t *override_font = _wgetenv(L"KPT_CJK_FONT");
        override_font != nullptr && *override_font != L'\0') {
      const std::filesystem::path file_path(override_font);
      auto utf8_validation = pathToUtf8(file_path);
      if (!utf8_validation) {
        auto error = std::move(utf8_validation).error();
        error.code = PlatformErrorCode::EnvironmentDecodeFailed;
        error.message = "KPT_CJK_FONT is not valid UTF-16";
        return error;
      }
      return matchOverride(factory.value().Get(), file_path,
                           required_characters);
    }
    return matchSystem(factory.value().Get(), required_characters);
  }
};

} // namespace

std::unique_ptr<Fonts> createWindowsFonts() {
  return std::make_unique<WindowsFonts>();
}

} // namespace kpt::platform
