# KITTI Point Cloud Tools

VS Code extension for
[kitti_pointcloud_tools](https://github.com/nerdneilsfield/kitti_pointcloud_tools).
It uses the repository's native WebAssembly codecs and a Three.js webview to
view and play KITTI-style point clouds.

## Features

- Opens KITTI BIN, PCD, PLY, XYZ, XYZI, XYZRGB, and XYZRGBI files.
- Decodes in a Web Worker with the same schema-aware codecs as KPT.
- Displays RGB, intensity, height, fixed, and semantic-label colors.
- Highlights non-zero PCD `noise`, `is_noise`, or `noise_class` values with a
  configurable color (red by default).
- Plays naturally sorted sequences with optional labels and two pose tracks.
- Uses octree-derived ordering and distant-cloud LOD for large clouds.
- Keeps files local; no point-cloud data is uploaded.

## Usage

Open a supported point-cloud file with **KITTI Point Cloud Tools**, or run
**KPT: Open Point Cloud Sequence** from the Command Palette.

For build instructions, runtime limits, verification, and architecture, see
the
[VS Code extension documentation](https://github.com/nerdneilsfield/kitti_pointcloud_tools/blob/master/docs/vscode-extension.md).

## License

[BSD 3-Clause](https://github.com/nerdneilsfield/kitti_pointcloud_tools/blob/master/LICENSE)
