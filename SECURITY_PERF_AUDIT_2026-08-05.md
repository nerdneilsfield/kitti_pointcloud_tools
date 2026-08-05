# Kitti Pointcloud Tools — 对抗性安全与性能审计报告

- 日期: 2026-08-05
- 范围: 五路并行 subagent 深度审计（IO codec / GUI render / CLI workflow / platform+wasm / VSCode+build），关键发现逐一回源核实
- 方法: 敌对视角审阅，逐条引用 `file:line`，标注严重度与修复方案
- 结论: 无远程代码执行 / 无当前威胁模型下任意文件写。存在 **1 个可触发内存破坏的 Critical（PLY 解析 UAF）**，以及若干 High 级安全/性能问题。

---

## 一、CRITICAL

### C1. PLY header 解析器悬垂指针 / use-after-free（内存破坏，可被恶意 PLY 触发）
- **位置**: `src/kpt/io/ply_codec.cpp:181, 214-215, 221-233`
- **证据**:
  ```cpp
  181:  Element *current = nullptr;
  ...
  214:      header.elements.push_back({std::string(tokens[1]), count, {}});
  215:      current = &header.elements.back();
  ...
  221:      if (current->properties.size() == maximum_properties_per_element)  // UAF 读
  224:        current->properties.push_back({...});                            // UAF 写
  ```
  `current` 是 `std::vector<Element>` 的裸指针。第二个 `element` 行触发 vector 扩容移动后旧缓冲被释放，`current` 悬空；后续 `property` 行解引用已释放内存。含 ≥2 个 `element` 且其后跟 `property` 的任意 PLY 即触发。
- **掩盖**: 仓库自带 fixture `tests/ply_io_test.cpp:80-94`（`element face 1` + `element vertex 2` + properties）恰好命中此路径。ASan 下必报 `heap-use-after-free`；无 ASan 时若 malloc 已复用该块，则向**其它对象的活跃内存**写入 property 数据 → 堆破坏 / 崩溃 / 潜在代码执行。独立复现已验证为干净的 ASan UAF（`vector<Property>::size()` 处读）。
- **修复**: 改用 `std::size_t current_index` 索引，或循环前 `header.elements.reserve(maximum_elements)`（上限 1024，保证不重分配），或 `property` 行重新取 `&header.elements.back()`。

---

## 二、HIGH（安全）

### H1. PCD `binary_compressed` 解压内存放大 ~50–100×（资源耗尽 / 分配炸弹）
- **位置**: `src/kpt/io/pcd_codec.cpp:35, 729-775, 817-841`
- **证据**: `output_size` 上限 512 MiB 于解析时先全额分配（:735）；LZF 约 3 字节输入膨胀 264 字节输出。约 5–17 MB 构造文件（通过 `maximum_lzf_size` 检查）即可逼迫 ~512 MiB 解压缓冲 + `cloud.reserve(20'000'000)` ~400 MB + 压缩副本 ≈ **1 GB 峰值**。`decode()` 内存路径经 web bridge 可达。
- **修复**: 压缩路径收紧 `kMaxBodyBytes` 或点数上限；流式解压进 cloud 而非物化 512 MiB 中间缓冲；延迟 `reserve`（仿 PLY `maximum_eager_reserve`）。

### H2. POSIX no-replace `publish()` 的 `link()` 硬链路径名 —— symlink 换入后目标指向任意受害者路径
- **位置**: `src/platform/native_file_posix.cpp:104-114`（free 函数回退）, `:263-297`（`PosixOutputFile::publish` 回退，`::link` @:290，macOS `RENAME_EXCL`→link @:283）
- **证据**: `link(2)` 不解析 symlink。攻击者可写临时目录时，删掉刚建临时文件、原地植 symlink → `link` 把 `destination` 硬链到 symlink inode → 后续对 `destination` 的普通 `open`/`ifstream`（无 `O_NOFOLLOW`）跟随其指向任意受害者文件 → 任意文件读/覆盖。可达于 Linux 无 O_TMPFILE/硬链回退路径（:229-230, :251-252）与 macOS 回退。
- **修复**: 敌意目录内绝不按路径名 link；Linux 用 fd 绑定 `linkat(AT_EMPTY_PATH)`/`renameat2(RENAME_NOREPLACE)`，失败即闭合；消费者 open 全部加 `O_NOFOLLOW`。

### H3. 设置配置目录权限 + `loadIni` check-then-open TOCTOU；`ifstream` 跟随 symlink
- **位置**: `src/platform/linux/services.cpp:61`（`create_directories` 默认 0755/0777），`src/platform/common/settings_store.cpp:174-190`（`exists`→`is_regular_file`→再 `ifstream` 打开）
- **冲击**: 宽松 umask 的共享主机上，他用户可 (a) 竞态替换检查与打开之间，逼 `loadIni` 读任意文件（信息泄露 / INI 注入 ImGui）；(b) 以 FIFO 阻塞 GUI 线程。写入侧较安全（`O_EXCL` + CSPRNG nonce），但父目录权限削弱之。
- **修复**: 配置目录 `0700`；INI 用 `O_NOFOLLOW|O_NONBLOCK` + `fstat` 校验常规文件。

### H4. CI actions 全部按可变 tag 引用，未 pin SHA（供应链）
- **位置**: `.github/workflows/package-release.yml:24,27,70,181,201-202`（含第三方 `mymindstorm/setup-emsdk@v14`），`.github/actions/setup-vcpkg/action.yml:8`
- **冲击**: tag 移动或上游仓库被接管 → 替换代码进入产出 VSIX/DEB/DMG 的 release 管道；`release` job 有 `permissions: contents: write`，被攻陷的 action 可推送任意 release 资产。
- **修复**: 全部 `uses:` pin 40 字符 commit SHA + 注释版本；启用 dependabot `update-type: pin`。

### H5. Web bridge `boundedString()` 无堆范围知识地扫描 NUL → wasm trap，C++ catch 捕不到
- **位置**: `src/gui/web/bridge.cpp:54-63`（:57-59 `while (length <= kMaximumSelectionPayloadBytes && value[length] != '\0') ++length;`），经 `kpt_web_selection_changed`/`kpt_web_stage_complete`（:197, :253）暴露给任意 JS。
- **冲击**: 传入线性内存尾部附近 1 MiB 内无 NUL 的指针 → typed-array OOB 读 → wasm trap。Trap 非 C++ 异常，`:202-238` 的 `catch(...)` 捕不住 → 整个模块中止。当前 shipped JS 恒 NUL 终止，属嵌入健壮性风险非现行漏洞。
- **修复**: JS 侧显式传长度，或扫描范围钳制到 `HEAP8.length - value`，拒绝堆外指针。

---

## 三、HIGH（性能）

### H6. OpenGL：每帧全新 VAO/VBO + 全量 CPU 重打包，跑在 GL/UI 线程
- **位置**: `src/gui/backend/opengl/point_renderer.cpp:641-745`（:673-675 `glBufferData` 全量），`:653` RenderState 快照
- **冲击**: 播放时每帧（revision 变化即触发）：O(n) reserve+过滤+转格式 → 全新 GPU 分配全量重传（数百万点 × 32B ≈ 上百 MB/帧）→ 无 `glBufferSubData`/持久映射/双缓冲；`GL_STATIC_DRAW` 用于每帧更换数据，阻断 stream 优化。UI 线程同时被 O(n) CPU 拷贝阻塞。
- **修复**: 固定/倍增缓冲 + `glBufferStorage`(immutable) + orphaning；CPU 打包移 worker 线程（upload 只 memcpy）；持久 mapped buffer + 环形双缓冲。

### H7. Metal：跨 command buffer 的 blit→render 无同步，GPU 读未初始化缓冲
- **位置**: `src/gui/backend/metal/point_renderer.mm:213-241`（upload 建独立 command buffer，:236 commit，:239 立即换 `vertex_buffer`），`:364-370`（render 用同帧 frame command buffer 引用该缓冲）
- **冲击**: 同 queue 保证提交顺序但不保证前一个完成后下一个才开始——执行可重叠。render pass 可能先于 blit 完成而读 private 缓冲（未初始化/垃圾数据）。无 `MTLEvent`/fence。每帧播放皆中招，GPU 端 data race。
- **修复**: ① 同一 command buffer 内先 blit 再 render；② `MTLSharedEvent` 串接 blit→frame buffer；③ 改用 shared 缓冲 + `didModifyRange` 免 private+blit。

### H8. WebGL（wasm32）LOD 索引 32-bit 溢出 → 越界索引
- **位置**: `src/gui/backend/opengl/point_renderer.cpp:691-706`（:694 `lod_indices[index] = static_cast<std::uint32_t>(index * copied.size() / lod_indices.size())`）
- **冲击**: wasm32 下 `size_t`=32-bit。`copied.size() > 2^32/index` 即溢出（copied>500k 时约 index≥7k 起）。桌面 64-bit 安全，web 端所有 >50 万点云交互 LOD 路径受影响 → `drawElements` 越界索引 / OOB GPU 读。
- **修复**: `static_cast<std::uint32_t>(index * (copied.size() / lod_indices.size()))` 或全程 `uint64_t`。

### H9. VSCode 转换在扩展宿主主线程同步跑 wasm（阻塞所有扩展）
- **位置**: `vscode/src/converter.ts:14-45`（`module.ccall` 同步），调用点 `vscode/src/extension.ts:376-401`
- **冲击**: 大文件转换（上限 512 MiB）时扩展宿主（所有扩展/命令/文件监听）卡秒到分钟级；内存 = 输入 512 MiB + wasm 堆副本 + 解码云（~20M 点 ≈ 1 GB）+ 编码输出，数 GB 逼近 2 GiB wasm 上限。
- **修复**: 放独立 `Worker`/子进程；调低转换上限；流式 decode→encode。

### H10. Frame cache 命中时 UI 线程全量重建 snapshot + 3 帧全云层常驻
- **位置**: `src/gui/app.cpp:1273-1281`（`makeViewportCloudSnapshot` 全量深拷 40B/点）, `:1371-1380`（cache 常驻 ≤3 原始云，500 万点 ≈ 300 MB）
- **冲击**: 回刷已载入帧时 UI 冻结数百 ms；内存双重打击。
- **修复**: snapshot 构建移 worker；revision 未变时覆用既有 snapshot。

### H11. 快速 scrub 时 `apply=true` 绕过 pending 去重，同帧重复并发载入
- **位置**: `src/gui/app.cpp:1283-1328`（:1283 `if (!pending_frames_.insert(index).second && !apply) return;`）
- **冲击**: 快速 Next/Slider 对同一帧重复 I/O、重复 snapshot、重复 upload，工作白做。
- **修复**: `apply=true` 且已在 pending 时仅更新 `desired_frame_` 等待既有 job。

### H12. RenderState 每次 upload/render/resize 快照+还原 ~40–50 次 GL 调用
- **位置**: `src/gui/backend/opengl/point_renderer.cpp:373-403`（`glGetIntegerv`×11 + 布尔×2 + `glIsEnabled`×4），`:405-451` 还原，构造/upload/resize/每帧 render 各建一次
- **冲击**: 交互拖拽时每帧纯开销。非致命。
- **修复**: 只存/还原实际改动的状态。

---

## 四、MEDIUM

### 安全
| # | 发现 | 位置 |
|---|------|------|
| M1 | `readPoses` 无行长/点数上限 + 无 reserve → OOM（唯一未受 64 KiB 行上限约束的读取器） | `src/kpt/workflow/workflow.cpp:196-224` |
| M2 | `loadLabel` 无上限读 `.label` → OOM；count-mismatch 检查在整档载入后才触发 | `src/kpt/label/label.cpp:21-28` |
| M3 | Inf/NaN 坐标穿透过滤（`isfinite(value) && out-of-range` 中 NaN 令 `isfinite` 为假 → 检查被跳过）污染下游渲染/统计 | `pcd_codec.cpp:446-453`, `ply_codec.cpp:457-465`, `io.cpp:309-311` |
| M4 | Windows `publish()` 先 `CloseHandle` 再 rename → 临时路径空窗 TOCTOU | `src/platform/native_file_windows.cpp:129-140` |
| M5 | webview 接收任意 `message`，无 origin/source/shape 校验即转发 worker → DoS + 驱动 2 GiB wasm 分配 | `vscode/webview/main.ts:288-339` |
| M6 | decoder worker 信任全部请求字段与原生 size/指针；decode queue 无界 | `vscode/webview/decoder.worker.ts:15-21,119,141-160` |
| M7 | wasm 导出未用的 `kpt_decode_file`/`kpt_decode_labeled_file` + `-sFILESYSTEM=1`（扩展只用 `*_memory` 变体） | `bindings/wasm/kpt_wasm_decoder.cpp:310-341`, `CMakeLists.txt:271-277` |
| M8 | 单次 `settings_enabled_ = flushSettings()` 失败即永久禁用保存，且失败仍清 `WantSaveIniSettings` 丢数据 | `glfw_webgl_runtime.cpp:141,285-292` |
| M9 | 扩展 `readPoses`（pose CSV）无 stat/读上限 → OOM（云/标签文件有 `readBounded` 512 MiB 上限，poses 无） | `vscode/src/extension.ts:572-598` |
| M10 | CSP 缺 `object-src`/`base-uri`/`form-action`；`style-src 'unsafe-inline'` | `vscode/src/extension.ts:168-170` |

### 性能
| # | 发现 | 位置 |
|---|------|------|
| P1 | ASCII 浮点每 token 一次 `std::string` 分配 + `istringstream` 构造 + locale imbue（KITTI 量级 4–6M tokens） | `pcd_codec.cpp:112-138,499-505`, `ply_codec.cpp:355-406` |
| P2 | 二进制热路径每标量 `input.read()` 虚拟调用（`loadBin` 4 次/点 × 20M） | `io.cpp:170-179`, `pcd_codec.cpp:683`, `ply_codec.cpp:324` |
| P3 | `encode()` 双缓冲全量输出：`ostringstream` 整串 + 再 memcpy 进 `vector<std::byte>`，峰值 ~3× 输出 | `io.cpp:547-551` |
| P4 | `applyLabel` 每点两次 `std::map` 查找 O(N·log M)（RangeNet id ≤259，应用 array O(1)） | `src/kpt/label/label.cpp:66-85` |
| P5 | wasm 解码无总字节预算，单文件可把 worker 堆推向 2 GiB → worker/tab OOM（解码 SoA ≈ 580 MB + 512 MiB 输入） | `bindings/wasm/kpt_wasm_decoder.cpp:139-141,252-268`, `CMakeLists.txt:272-274` |
| P6 | decoder 边界全量复制：JS→wasm 输入 + wasm→JS 全部 SoA `.slice()`（20M 点 ≈ 240+340 MB 堆流量） | `decoder.worker.ts:80-84,141-156` |
| P7 | `setRotationCenterFromScreen` UI 线程 O(n) 全顶点投影 → 中键选 pivot 冻数百 ms | `src/gui/viewport/model.cpp:317-344` |
| P8 | `needsContinuousRedraw` 每帧 `snapshots()` 锁 mutex + 复制全部 job 名字符串 | `app.cpp:262-265`, `job_system.cpp:105-116` |
| P9 | Metal 顶点 48B 浪费 1/3 带宽（`simd_float4`×3 实仅 8 float 用） | `metal/point_renderer.mm:22-30` |
| P10 | 上传双重深拷贝：snapshot 拷贝 + renderer 再过滤拷贝，均每 revision 新分配 | `cloud_adapter.cpp:85-117`, `opengl/point_renderer.cpp:641-651` |
| P11 | 原生后端无 LOD/视锥剔除/点数上限，全量上传+全量绘制 | `opengl/point_renderer.cpp:932`, `metal/.mm:368-370` |
| P12 | 扩展 per-load 多重全尺寸副本（readFile→slice→postMessage 结构化克隆→HEAPU8.set→输出 slice）≈3× 文件 | `extension.ts:83-107`, `decoder.worker.ts:84,141-156` |
| P13 | 序列面板隐藏仍解码（`retainContextWhenHidden: true`）+ 每帧串行读 | `extension.ts:481,505-534` |
| P14 | worker 每次 error/timeout/reload 重实例化 + wasm 重新编译 + 重 slice | `main.ts:136-193,330-337` |
| P15 | viewer 每帧 O(n) `finiteRange` 全扫 + LOD `gatherGeometry` 重拷 | `vscode/webview/viewer.ts:131-134,204-209` |
| P16 | `buildSpatialIndex` 递归 `.slice()` 每节点分配（退化云 8 层深度反复 churn） | `decoder.worker.ts:246-249,279-283` |
| P17 | `analyzeCloud` 三轴全排序 + 240 MB 暂存（应 `nth_element`） | `src/kpt/render/render.cpp:294-347` |

---

## 五、LOW

- 安全：popl 解析异常未捕获 → SIGABRT（`pc_convert.cc:13`, `pc_batch_convert.cc:29`, `pc_render.cc:85`）；`enumerate`/`is_regular_file` 跟随 symlink 读取 input_dir 外文件（`workflow.cpp:240-242,186`）；并发批次 `overwrite=true` 静默互相覆盖（`pc_batch_convert.cc:96`）；单档工具无 `--overwrite` 旗标硬编码覆盖（`pc_convert.cc:37`, `pc_render.cc:158`）；临时文件 0666 世界可读（`native_file_posix.cpp:332,348`）；临时名用 `mt19937_64` 可预测（`io.cpp:103-116`, `render.cpp:100-117`）；崩溃遗留 `.tmp` 无清扫（`io.cpp:106` 等）；`loadBin`/`decodeBinaryStream` 失败时部分填充调用方云（`io.cpp:148-180`）；`savePly` 上限（14.3M）与 `loadPly`（20M）不对称（`ply_codec.cpp:620-626`）；`takeJob` 重排队 `priority_queue::push` 可抛异常穿出 jthread → `terminate`（`job_system.cpp:140-146`）；`scriptUri` 未转义插入 HTML（`extension.ts:275`）；`load` handler 未校验 `bytes` 类型（`main.ts:333-336`）；bridge 无路径包含校验，`kpt_decode_file` 可读任意虚拟 FS 路径（`bridge.cpp:82-101`）；`maxFileSizeMiB=512` 超出 decoder 20M 点原生上限（`.bin` 最大合法档必失败）；字段名注入异常/日志（`pcd_codec.cpp:433,451` 等）；`VIEWPOINT` 超 float 范围静默 Inf（`pcd_codec.cpp:273-276`）；Windows 主路径 `ReplaceFileW` 无目录持久性保证；web localStorage 未版本化/未校验（`services.cpp:19-41`）；`rapidcsv.h` 死代码（1647 行，无使用处）若有朝启用则无上限。
- 性能：`loadBin` 无 bulk read（P2 附带）；`loadAsciiBody` 无 `reserve`（`pcd_codec.cpp:561-615`）；`io.cpp` ASCII 每行 `istringstream`+每行 `vector<float>`；PCD header 每 token `std::string` 分配（`pcd_codec.cpp:75-83`）；批次转换纯串行（`pc_batch_convert.cc:110-121`）；每档重复 `create_directories`（`workflow.cpp:307-309`）；Docker 无 `.dockerignore`（16 GB build/ 全量上传 daemon）；`glDrawArrays(..., int)` 强转 >2^31 点包裹（`point_renderer.cpp:932`）；`writeImageAtomic` exists 预检 TOCTOU 冗余（`render.cpp:596-597`）；`classifyError` 把 unknown format 误报为 internal-error。

---

## 六、已核实无漏洞（防御到位处）

- 所有尺寸/偏移/计数数学使用 `checkedMultiply`/`checkedAdd`（含 `COUNT 2^64-1` 边界）；PCD/PLY header 扫描全部长度有界、无死循环。
- LZF 解压器输入/输出双边界检查、重叠安全、精确输出长度验证。
- PLY/PCD 加载事务性（失败不动调用方云）；字节序可移植处理。
- 输出原子写：Linux O_TMPFILE→linkat→rename 绑定 file identity；`writeImageAtomic` 全量写循环；`MemoryStreamBuf` seek/overflow 正确。
- 无 shell-out（src/ 零 `system`/`popen`/`exec`）；无 printf 风格格式化（全 spdlog `{}`）；UTF-8 校验器正确；`O_EXCL`/`CREATE_NEW` 临时文件独占。
- 无 host data race：snapshot 不可变 `shared_ptr<const>` + `ui_.post` 单向队列；app 析构序正确（jobs 先于 ui 销毁）；`glFinish`/`glReadPixels` 仅在 test_support。
- 无命令注入/路径穿越：扩展仅 `spawnSync(argv)` 无 shell；webview 资源 `joinPath`+`localResourceRoots`；CI 无明文 secret，Mesa 下载 SHA256 pin。
- Settings 写入用 CSPRNG nonce + `O_EXCL`；GPU 上传对分配失败有错误处理（OpenGL）。

---

## 七、建议修复优先级

1. **C1**（PLY UAF）—— 立即可被恶意 PLY 触发，改索引即可修复，最高优先。
2. **H2/H3/H4**（POSIX link 回退 / 配置目录权限 / CI pin SHA）—— 供应链与本地提权面。
3. **H6/H7/H8**（GL 全量重传 / Metal blit 同步 / WebGL 32-bit 溢出）—— 渲染性能三巨头 + 一个平台特定索引破坏。
4. **H1**（压缩放大）与 **M1/M2**（unbounded readers）—— 资源耗尽面，收口上限即可。
5. **P1/P2**（`from_chars` + bulk read）—— KITTI 规模数据加载最大常数因子收益。

---

## 八、复核与修复状态（追加）

本节不覆盖上文原始证据；记录回源复核后的结论与当前代码状态。

### 原审计结论修订

- **C1：原始 Critical 断言不成立。** 当前解析器在每条 `element` 后都会重新取得
  `header.elements.back()`；后续 `property` 行不会使用前一个元素的旧地址，故未能在
  当前源码复现所称 UAF。仓库双 `element` fixture 覆盖该路径；稳定索引改动仍保留，
  以消除未来重构重新引入地址失效的可能。ASan/UBSan 测试未报 UAF。
- **H7：原始 Metal data-race 断言不成立。** 同一 `MTLCommandQueue` 的 command buffer
  按提交顺序执行；原实现缺少 queue 归属校验且使用独立上传设计，仍作防御性重构：
  persistent shared/ring buffer、同 queue 校验，不再使用独立 blit→render 路径。
- **H8：算术缺陷成立，影响表述收窄。** wasm32 的 LOD 乘法会溢出并产生错误采样，
  但当前取模后的结果未证明为 GPU 越界读；已改为全程 `uint64_t` 并做 draw-count 边界校验。
- **P15：原始“每帧”表述不成立。** `finiteRange` 与 `gatherGeometry` 在单次 cloud
  load 时执行，不在 render loop 每帧执行；保留输入/输出预算校验，未作无必要重构。

### 已完成

- **IO/资源耗尽**：PLY 使用稳定索引；PCD compressed 增加 512 MiB body、768 MiB
  working-set、精确解压预算与延迟 reserve；ASCII 使用有界 token 与 `from_chars`；BIN
  bulk read；NaN/Inf 与 float 范围拒绝；解析失败事务性提交；pose/label/批次输入有文件、
  行数、行长、点数上限。
- **文件与配置**：POSIX no-replace 无安全 descriptor 绑定时 fail-closed，移除路径名
  `link()` 回退；临时文件 `0600` + CSPRNG；Linux 配置目录 `0700`；配置读取使用
  `O_NOFOLLOW|O_NONBLOCK|O_CLOEXEC` 与 `fstat`；Windows 在仍持有句柄时发布；PNG 与批次
  输出移除 exists check-then-use；默认不覆盖，`--overwrite` 显式启用。
- **CLI/workflow**：CLI parser 异常分类；输入目录拒绝 symlink/FIFO；重复输出拒绝；
  `takeJob` reserved-worker 路径不再错误重排队；`apply=true` 重复 frame 请求先去重，
  不再推进 generation 使既有 job 结果失效。
- **GPU**：OpenGL VAO/VBO、guide buffer、LOD index buffer 复用并按容量增长；GL/WebGL
  LOD index 使用 `uint64_t`，OpenGL/Metal interactive draw 限制 500k；Metal 使用 32-byte
  packed vertex、persistent shared/ring buffer 与 device/queue 校验；frame snapshot
  在 worker 构造，pending request 合并，job UI 查询不再复制完整名称快照；trim 改用
  `nth_element`。
- **WASM/web/VSCode**：ABI 升至 5；移除文件路径 decoder exports 与 filesystem runtime
  依赖；字符串 ABI 显式长度；heap range、文件名、输入/label/输出/working-set、queue、
  virtual-root 均有上限；web message 校验 origin/source/schema/type；localStorage 有
  version/schema；settings flush 失败不丢 dirty state；VSCode converter 移入 Node/browser
  worker；webview 增加 `object-src/base-uri/form-action` CSP；GitHub Actions 固定 40-char SHA，
  加入 pin 检查。

### 部分缓解或明确残留

- **H6/H10/P6/P10**：已消除新 GPU 对象、二次 encode buffer、UI 全量 snapshot
  构造及主要热路径的无界分配；decoder/renderer 边界仍保留必要的数据复制，尚未改为
  零拷贝协议。
- **P7**：中键 pivot 选择仍需扫描当前 cloud；这是低频交互路径的 O(n)，未引入额外空间
  索引。
- **P11**：interactive render 已有 500k LOD；非 interactive native render 仍绘制完整
  cloud，未改变默认视觉精度。
- **P12/P14**：VSCode API 的 webview `postMessage` 仍产生一次结构化 clone；输入/输出与
  queue 已限额。worker 崩溃/超时后的重启仍需重新实例化 WASM，重启路径已有超时和预算保护。
- **Low**：遗留 `.kpt-tmp-*`/settings `.tmp.*` 的崩溃清理、Docker build context 优化、
  少数 native 输入 path check/open 竞态与 Windows 设置目录持久性仍属后续工程项；不影响
  本轮已封堵的任意写、symlink publish、解析 OOM 与 wasm trap 面。

### 收尾复核补充

- **H12**：OpenGL `RenderState` 已按 create/upload/resize/render scope 分层捕获，仅查询并
  恢复调用路径实际改变的状态；完整 scope 仍保留原恢复契约。
- **P1/P2**：PCD/PLY/通用 ASCII 使用固定 token buffer 与 `from_chars`；PCD/PLY/BIN
  二进制输入改为批量读取，避免逐 scalar `istream::read` 热路径。

### 验证记录

- Native RelWithDebInfo build：通过。
- Native CTest：9/9 通过；OpenGL 使用独立 Xvfb + software GL。
- ASan/UBSan：`kpt_tests` 通过（12973 assertions / 91 cases；关闭系统 fontconfig
  leak noise 后无项目 UAF/UB）。
- VSCode：`tsc --noEmit`、esbuild bundle、CI SHA pin check 通过；初次 v0.2.0
  CI 的真实 WASM 构建成功，但 prepublish 因 browser bundle 仍要求已迁移到 worker 的
  `kpt_convert_memory` marker 失败。已移除过时 browser marker，并把 Node/browser
  converter worker 加入 VSIX 必备文件校验；本地环境仍缺 `/upstream/emscripten` 与 Ninja，
  完整 VSIX 重跑待 CI 验证。
- Browser smoke 未作成功判定：现有 checkout artifact 仍为 ABI 4，源码 worker 已要求 ABI 5；
  待 Emscripten 工具链可用后须重新生成 `kpt_decoder.js/.wasm` 再运行 WebGL/VSCode smoke。

### 发布流复核补充

- 全量复核 `.github/workflows/package-release.yml`、`.github/actions/setup-vcpkg/action.yml`
  与 `tools/check-action-pins.sh`：权限边界、版本门禁、矩阵、artifact 汇聚、checksum、
  release 更新与并发组契约一致。
- 所有第三方 `uses:` 均为 40 字符 SHA，且逐一与对应远端 `v7`/`v8`/`v6`/`v14` tag
  解析结果一致；本地 pin 检查通过。
- v0.2.0 CI 暴露的 AppleClang 浮点 `from_chars` overload 缺失，已加入 Apple 专用的
  `std::locale::classic()` 流解析回退；Linux/Emscripten/MSVC 继续走四参 `from_chars`
  fast path。同时修复 VSIX stale marker；修复后需以强制更新的 `v0.2.0` 完整重跑发布流。
- 重跑时 Ubuntu 22.04 公共 apt mirror 出现索引/包短暂不同步（`linux-libc-dev` 404），
  Docker build 已加入清单刷新、下载重试与失败后清理，降低镜像瞬态失败导致的发布阻断。
