#include <metal_stdlib>
using namespace metal;

struct GpuVertex {
  float4 position;
  float4 color;
  float4 scalar;
};

struct Uniforms {
  float4x4 view_projection;
  float4 background;
  float4 parameters; // point size, color mode, scalar min, scalar max
};

struct VertexOut {
  float4 position [[position]];
  float point_size [[point_size]];
  float3 color;
  float intensity;
  float z;
};

vertex VertexOut point_vertex(const device GpuVertex *vertices [[buffer(0)]],
                              constant Uniforms &uniforms [[buffer(1)]],
                              uint index [[vertex_id]]) {
  const GpuVertex value = vertices[index];
  VertexOut output;
  output.position = uniforms.view_projection * value.position;
  output.point_size = uniforms.parameters.x;
  output.color = value.color.xyz;
  output.intensity = value.scalar.x;
  output.z = value.position.z;
  return output;
}

float3 turbo(float value) {
  const float x = clamp(value, 0.0f, 1.0f);
  const float4 k_red =
      float4(0.13572138f, 4.61539260f, -42.66032258f, 132.13108234f);
  const float4 k_green =
      float4(0.09140261f, 2.19418839f, 4.84296658f, -14.18503333f);
  const float4 k_blue =
      float4(0.10667330f, 12.64194608f, -60.58204836f, 110.36276771f);
  const float2 k_red2 = float2(-152.94239396f, 59.28637943f);
  const float2 k_green2 = float2(4.27729857f, 2.82956604f);
  const float2 k_blue2 = float2(-89.90310912f, 27.34824973f);
  const float4 v4 = float4(1.0f, x, x * x, x * x * x);
  const float2 v2 = v4.zw * v4.z;
  return float3(dot(v4, k_red) + dot(v2, k_red2),
                dot(v4, k_green) + dot(v2, k_green2),
                dot(v4, k_blue) + dot(v2, k_blue2));
}

fragment float4 point_fragment(VertexOut input [[stage_in]],
                               float2 point_coord [[point_coord]],
                               constant Uniforms &uniforms [[buffer(1)]]) {
  if (distance(point_coord, float2(0.5f)) > 0.5f)
    discard_fragment();

  const int color_mode = int(uniforms.parameters.y);
  if (color_mode == 0)
    return float4(input.color, 1.0f);
  if (color_mode == 4)
    return float4(1.0f);

  const float value = color_mode == 1 ? input.intensity : input.z;
  const float span =
      max(uniforms.parameters.w - uniforms.parameters.z, 1.0e-12f);
  return float4(turbo((value - uniforms.parameters.z) / span), 1.0f);
}
