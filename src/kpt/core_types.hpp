#pragma once

namespace kpt {

enum class Format { Bin, PCD, PLY, XYZ, XYZI, XYZRGB, XYZRGBI };

enum class ColorBy { Intensity, RGB, Z, Label, None };

enum class View {
  Front,
  Right,
  Back,
  Left,
  Top,
  Bottom,
  TopRightFront,
  TopLeftFront,
  BotRightFront,
  BotLeftFront
};

} // namespace kpt
