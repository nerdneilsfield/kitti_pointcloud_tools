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
  float4 transform;  // world origin xyz, world scale
  float4 fixed_color;
  float4 noise_color; // rgb, enabled
};

struct VertexOut {
  float4 position [[position]];
  float point_size [[point_size]];
  float3 color;
  float intensity;
  float z;
  float noise;
};

vertex VertexOut point_vertex(const device GpuVertex *vertices [[buffer(0)]],
                              constant Uniforms &uniforms [[buffer(1)]],
                              uint index [[vertex_id]]) {
  const GpuVertex value = vertices[index];
  VertexOut output;
  const float3 local_position =
      (value.position.xyz - uniforms.transform.xyz) * uniforms.transform.w;
  output.position =
      uniforms.view_projection * float4(local_position, 1.0f);
  output.point_size = uniforms.parameters.x;
  output.color = value.color.xyz;
  output.intensity = value.scalar.x;
  output.noise = value.scalar.y;
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

float3 palette5(float value, float3 c0, float3 c1, float3 c2, float3 c3,
                float3 c4) {
  const float position = clamp(value, 0.0f, 1.0f) * 4.0f;
  if (position < 1.0f)
    return mix(c0, c1, position);
  if (position < 2.0f)
    return mix(c1, c2, position - 1.0f);
  if (position < 3.0f)
    return mix(c2, c3, position - 2.0f);
  return mix(c3, c4, position - 3.0f);
}

float3 scalar_color(float value, int color_map) {
  if (color_map == 1)
    return palette5(value, float3(0.267004f, 0.004874f, 0.329415f),
                    float3(0.229739f, 0.322361f, 0.545706f),
                    float3(0.127568f, 0.566949f, 0.550556f),
                    float3(0.369214f, 0.788888f, 0.382914f),
                    float3(0.993248f, 0.906157f, 0.143936f));
  if (color_map == 2)
    return palette5(value, float3(0.050383f, 0.029803f, 0.527975f),
                    float3(0.494877f, 0.011990f, 0.657865f),
                    float3(0.798216f, 0.280197f, 0.469538f),
                    float3(0.973416f, 0.585761f, 0.251540f),
                    float3(0.940015f, 0.975158f, 0.131326f));
  if (color_map == 3)
    return palette5(value, float3(0.001462f, 0.000466f, 0.013866f),
                    float3(0.341500f, 0.062300f, 0.429400f),
                    float3(0.735700f, 0.215900f, 0.330200f),
                    float3(0.978400f, 0.557900f, 0.034900f),
                    float3(0.988362f, 0.998364f, 0.644924f));
  if (color_map == 4)
    return palette5(value, float3(0.001462f, 0.000466f, 0.013866f),
                    float3(0.316654f, 0.071690f, 0.485380f),
                    float3(0.716387f, 0.214982f, 0.475290f),
                    float3(0.986700f, 0.535582f, 0.382210f),
                    float3(0.987053f, 0.991438f, 0.749504f));
  if (color_map == 5)
    return float3(clamp(value, 0.0f, 1.0f));
  return turbo(value);
}

fragment float4 point_fragment(VertexOut input [[stage_in]],
                               float2 point_coord [[point_coord]],
                               constant Uniforms &uniforms [[buffer(1)]]) {
  if (distance(point_coord, float2(0.5f)) > 0.5f)
    discard_fragment();

  const int color_mode = int(uniforms.parameters.y);
  float3 base_color;
  if (color_mode == 0) {
    base_color = input.color;
  } else if (color_mode == 4) {
    base_color = uniforms.fixed_color.xyz;
  } else {
    const float value = color_mode == 1 ? input.intensity : input.z;
    const float span =
        max(uniforms.parameters.w - uniforms.parameters.z, 1.0e-12f);
    const float normalized = (value - uniforms.parameters.z) / span;
    base_color = color_mode == 1
                     ? scalar_color(normalized, int(uniforms.fixed_color.w))
                     : turbo(normalized);
  }
  if (uniforms.noise_color.w > 0.5f && input.noise > 0.5f)
    base_color = uniforms.noise_color.xyz;
  return float4(base_color, 1.0f);
}

fragment float4 guide_fragment(VertexOut input [[stage_in]]) {
  return float4(input.color, 1.0f);
}
