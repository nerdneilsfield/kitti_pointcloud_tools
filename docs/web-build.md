# WebAssembly / WebGL2 build

The browser workbench is a static Emscripten application. It contains Viewer
and Player only; Convert, Batch, Render, snapshots, native paths, and globs
remain native-only.

## Prerequisites

The verified SDK is Emscripten 6.0.5:

```bash
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk
./emsdk install 6.0.5
./emsdk activate 6.0.5
source ./emsdk_env.sh
```

CMake 3.21+, Ninja, Git, and a C++20-capable host compiler are also required.
The SDK is intentionally not vendored.

## Build and run

From the repository root:

```bash
cmake --preset web-release
cmake --build --preset web-release
python3 tools/serve_web.py --port 8765
```

Open `http://127.0.0.1:8765/`. Build products are written to
`build/web-release/site/`: HTML, JavaScript, WASM, worker code, and the
preloaded Noto Sans SC font.

Do not open `index.html` with a `file:` URL. WebAssembly pthreads require
`SharedArrayBuffer`, hence a cross-origin-isolated response. The supplied
server sends:

```text
Cross-Origin-Opener-Policy: same-origin
Cross-Origin-Embedder-Policy: require-corp
Cross-Origin-Resource-Policy: same-origin
Content-Type: application/wasm
```

Production hosting must send equivalent headers for HTML, JavaScript, WASM,
worker, data, and font responses.

## Browser file model

- Viewer selects one of `bin`, `pcd`, `ply`, `xyz`, `xyzi`, `xyzrgb`, or
  `xyzrgbi`.
- Player selects point-cloud frames, optional `.label` files, and up to two
  pose CSV/text files separately.
- Frames are sorted by filename. Labels pair by `<stem>.label`; duplicates,
  unsupported formats, missing labels, and orphan labels are rejected before
  loading.
- Browser `File` objects remain in an in-memory registry. Only assets needed
  by the current request are copied into MEMFS, parsed by the existing KPT
  loaders, then removed. Imported files are never uploaded or persisted.
- ImGui layout is the only browser-persisted data and is stored in
  `localStorage`.

The WASM build uses four pthread workers, 256 MiB initial memory, growth up to
2 GiB, and the existing three-frame playback cache. Emscripten warns that
combining pthreads with memory growth can slow JavaScript heap access. KPT
intentionally retains growth instead of reserving 2 GiB at startup, and
suppresses that advisory warning. Deployment without cross-origin isolation,
WebGL2, or sufficient memory produces a visible error.

## Browser smoke tests

Install Playwright once:

```bash
cd web
npm install
npx playwright install chromium firefox webkit
cd ..
```

Run the static server in one terminal, then:

```bash
cd web
KPT_WEB_URL=http://127.0.0.1:8765 npm test
```

On Linux CI without a display, run headed browsers through `xvfb-run`.
The Chromium project selects ANGLE SwiftShader:

```bash
KPT_WEB_HEADED=1 xvfb-run -a npm test
```

Current desktop Chrome, Firefox, and Safari are targeted. Mobile browsers,
ZIP/directory import, online deployment, and containers are outside the first
release.
