# Point Cloud Tools

VS Code extension for
[kitti_pointcloud_tools](https://github.com/nerdneilsfield/kitti_pointcloud_tools).
It uses the repository's native WebAssembly codecs and a Three.js webview to
view and play KITTI-style point clouds.

## Features

- Opens KITTI BIN, PCD, PLY, XYZ, XYZI, XYZRGB, and XYZRGBI files without
  taking over generic `.bin` files by default.
- Decodes in a Web Worker with the same schema-aware codecs as KPT.
- Displays RGB, intensity, height, fixed, and semantic-label colors.
- Highlights non-zero PCD `noise`, `is_noise`, or `noise_class` values with a
  configurable color (red by default).
- Plays naturally sorted sequences with optional labels and two pose tracks.
- Converts and exports between all supported point-cloud formats.
- Runs in desktop VS Code and browser-hosted VS Code environments.
- Uses octree-derived ordering and distant-cloud LOD for large clouds.
- Keeps files local; no point-cloud data is uploaded.

## Usage

Run **KPT: Open Point Cloud**, **KPT: Open Point Cloud Sequence**, or
**KPT: Convert Point Cloud** from the Command Palette. The same open and
convert commands are available from supported files in Explorer.

For build instructions, runtime limits, verification, and architecture, see
the
[VS Code extension documentation](https://github.com/nerdneilsfield/kitti_pointcloud_tools/blob/master/docs/vscode-extension.md).

## License

[BSD 3-Clause](https://github.com/nerdneilsfield/kitti_pointcloud_tools/blob/master/LICENSE)

Bundled dependency licenses are listed in
[Third-Party Notices](THIRD_PARTY_NOTICES.md).
