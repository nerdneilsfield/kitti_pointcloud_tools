# Native Point-Cloud Types and I/O

Date: 2026-07-28

Status: Implemented in source; clean post-migration acceptance pending

## 1. Decision

KPT no longer uses PCL as its point-cloud container, file-codec provider, or
viewer. `kpt_core` owns a small plain-C++ cloud type and native codecs for all
seven public formats. The legacy PCLVisualizer executables `pc_viewer` and
`pc_player` are retired; their interactive workflows live in `pc_gui`.

This decision removes PCL and VTK from CMake discovery, the vcpkg manifest,
public headers and link graphs. OpenCV remains scoped to `kpt_render` for PNG
output. Dear ImGui, GLFW and graphics backends remain GUI-only concerns.

## 2. In-memory contract

`kpt::PointXYZRGBI` is a value type with:

```cpp
float x, y, z;
std::uint8_t r, g, b;
float intensity;
```

Every member defaults to zero. `PointCloudIRGB` owns
`std::vector<PointXYZRGBI> points` and supplies the container operations used
by KPT: iteration, `size`, `empty`, `clear`, `reserve`, `push_back`, and
append through `operator+=`. Shared and shared-const aliases preserve the
existing ownership model without importing third-party types.

The model deliberately omits organized-cloud dimensions, sensor pose, frame
IDs and density metadata. No KPT consumer used those PCL fields. PCD
`WIDTH`/`HEIGHT` are validated for point count and then flattened; PCD
`VIEWPOINT` and PLY camera elements are not retained.

## 3. Public dispatch

`kpt::load(path)` and `kpt::save(path, cloud, optional_format)` remain the
public API. Extension detection is ASCII-case-insensitive:

| Extension | Format |
|---|---|
| `.bin` | KITTI binary |
| `.pcd` | Point Cloud Data |
| `.ply` | Polygon File Format |
| `.xyz` | XYZ text |
| `.xyzi` | XYZI text |
| `.xyzrgb` | XYZRGB text |
| `.xyzrgbi` | XYZRGBI text |

Unknown extensions fail. An explicit save format may override the output
extension, chiefly for CLI `--ascii-flavor`.

Filesystem calls take `std::filesystem::path` directly. Native codecs do not
convert a Windows path through a third-party narrow-string ABI.

## 4. Codec matrix

| Codec | Read | Write | Missing attributes |
|---|---|---|---|
| BIN | 4 little-endian IEEE-754 float32 values per point | same | RGB becomes zero |
| XYZ | exactly 3 text columns | 3 columns | RGB/intensity become zero |
| XYZI | exactly 4 text columns | 4 columns | RGB becomes zero |
| XYZRGB | exactly 6 text columns | 6 columns | intensity becomes zero |
| XYZRGBI | exactly 7 text columns | 7 columns | none |
| PCD | `ascii`, little-endian `binary`, LZF `binary_compressed` | PCD 0.7 little-endian binary | absent RGB/intensity become zero |
| PLY | PLY 1.0 ASCII, binary little-endian, binary big-endian | PLY 1.0 binary little-endian | absent RGB/intensity become zero |

Text parsing uses the classic locale. Blank lines and lines beginning with
`#` are ignored. A row with the wrong column count, or a non-integral RGB
value outside `[0,255]`, is skipped. At most 50 individual warnings are
logged, followed by one count summary. Writers use enough float digits for a
float32 text round trip instead of a display-oriented fixed precision.

BIN records are always little-endian, including on a big-endian host. BIN
intentionally drops RGB.

## 5. PCD dialect

The reader consumes schema rather than assuming a C++ struct layout:

- `FIELDS` and legacy `FIELD`, `SIZE`, `TYPE`, optional `COUNT`;
- `WIDTH`/`HEIGHT`, `POINTS`, or both when consistent;
- signed and unsigned 1/2/4/8-byte integers, float32 and float64;
- arbitrary field order and unknown fields with `COUNT > 1`;
- required scalar `x`, `y`, and `z`;
- `intensity` or `reflectance`;
- packed `rgb`/`rgba` encoded as float32 bits or uint32;
- separate `r`/`red`, `g`/`green`, and `b`/`blue` channels.

Mapped aliases may not be duplicated and mapped fields must have `COUNT 1`.
Separate channels override the corresponding channel from packed RGB.
Binary-compressed input uses the PCD field-major layout after validated LZF
decompression.

The writer emits PCD 0.7, `WIDTH = points`, `HEIGHT = 1`, and `DATA binary`.
Each record is little-endian `x`, `y`, `z`, packed RGB bits and `intensity`,
all four bytes wide.

## 6. PLY dialect

The reader builds an element/property schema before consuming payload:

- scalar aliases `char/int8`, `uchar/uint8`, `short/int16`,
  `ushort/uint16`, `int/int32`, `uint/uint32`, `float/float32`, and
  `double/float64`;
- arbitrary element and property order;
- scalar and list properties;
- required scalar `x`, `y`, and `z` on the unique `vertex` element;
- optional `intensity`, `red/r`, `green/g`, and `blue/b`;
- unknown elements/properties consumed without changing the cloud.

List count types must be integral. Binary endianness follows the header.
The writer emits only a vertex element with little-endian float32 XYZ and
intensity plus uint8 RGB. It does not invent faces or camera metadata.

## 7. Failure and resource policy

Structured codecs are strict and transactional: they parse into a temporary
cloud and publish it only after the complete header and payload validate.
Truncated input, trailing records beyond declared PCD `POINTS`, arithmetic
overflow, duplicate mapped fields, invalid scalar ranges and malformed LZF
references fail with a path-bearing `std::runtime_error`.

Current fixed guards include:

- PCD header: at most 1 MiB;
- PCD fields: at most 4096;
- PCD points: at most 100,000,000;
- every byte-count addition and multiplication checked before allocation;
- PLY counts checked against `size_t` and container capacity;
- PLY eager vertex reserve capped at 1,000,000;
- PLY list byte-count arithmetic checked, and every declared item consumed.

PLY does not yet impose a smaller fixed total-record or header-byte ceiling
than process/container limits. PCD compressed input has a 32-bit compressed
size prefix. Those facts must be considered before treating either parser as
a hardened service for arbitrary hostile uploads; tighter global byte/work
budgets are a follow-up hardening item.

Writers verify open, header and payload stream state. Workflow conversion
continues to write a temporary sibling and atomically replace the destination,
so a codec failure does not publish a partial conversion result.

## 8. Build and target boundaries

```text
kpt_core
  types + BIN/text/PCD/PLY + labels + workflow

kpt_render
  kpt_core + OpenCV image output

kpt_gui_app
  kpt_core + cloud-to-viewport adapter + portable GUI composition
```

`kpt_viewport_cloud_adapter` converts the core cloud to immutable viewport
vertices. Its former `pcl_adapter` name is retired because no PCL type crosses
that boundary.

## 9. Compatibility and non-goals

Compatibility means the documented dialects and independent golden fixtures,
not every behavior accepted by every historical PCL release. In particular:

- arbitrary PCL metadata is not retained;
- PLY meshes are consumed for alignment but not represented or written;
- writers choose one portable binary dialect rather than preserving source
  encoding or property order;
- loss of attributes in BIN/XYZ/XYZI/XYZRGB is intentional and documented.

## 10. Verification

Codec tests cover independent ASCII/binary/compressed PCD fixtures, ASCII and
both-endian PLY fixtures, reordered/extra fields, scalar aliases, lists,
malformed schemas, truncation, Unicode paths, writer byte layout and the
seven-format conversion matrix.

Linux, Windows and macOS acceptance remains distinct from source coverage.
This design does not claim Windows or macOS execution. A clean Linux
configure/build/test run after the dependency migration must be recorded in
`docs/cross-platform-build.md`; target-native runs remain required before
their verification rows can change.
