# Cross-Platform Design Adversarial Review Audit

Date: 2026-07-28

Reviewed document: `2026-07-28-cross-platform-design.md`
Scope: verification of three independent review reports against repository
state, vendored APIs, and upstream documentation.

Verdicts:

- **True**: claim and consequence are supported.
- **Partial**: underlying risk is real, but wording, mechanism, or severity is
  inaccurate.
- **False**: cited code/API does not support the claim.
- **Out of scope**: valid possible work, but expressly excluded by the product
  decision.

Every true or partial finding is addressed in the revised design. False
findings are recorded so that the design does not acquire workarounds for
nonexistent behavior.

## 1. Evidence Baseline

Repository facts used repeatedly:

- `src/kpt/types.hpp` includes PCL and aliases `PointCloudIRGB` to
  `pcl::PointCloud`.
- Current `kpt` is monolithic and publicly links PCL/OpenCV; viewer/player code
  is part of that target.
- `src/gui/app.cpp` posts completed worker results through `ui_.post`, reads
  `DisplayFramebufferScale`, and calls the renderer's GL-specific
  `centerPixelVisible`.
- `PointRenderer::centerPixelVisible` calls `glReadPixels`.
- GUI and GUI tests link `OpenGL::GL`.
- CMake globally mutates OpenMP flags, although `src/` and `tests/` contain no
  OpenMP pragma or runtime call.
- Vendored Dear ImGui is 1.92.8: `ImGui::Image` consumes `ImTextureRef`,
  `ImFontConfig::FontNo` selects a collection face, and manual ini persistence
  is supported.
- Contrary to one report, vendored `ImFileOpen` converts UTF-8 to UTF-16 and
  calls `_wfopen` on MSVC Windows.
- Vendored `examples/example_glfw_metal/main.mm` already uses
  `GLFW_NO_API`, `glfwGetCocoaWindow`, `CAMetalLayer`, one command buffer, and
  Dear ImGui's Metal backend.

Upstream facts:

- CMake preset schema 3 is available in CMake 3.21 and adds conditions and
  `toolchainFile`; build/test presets arrived in schema 2. Workflow presets
  require schema 6/CMake 3.25.
- Apple documents that Metal point primitives need vertex output
  `[[point_size]]`; otherwise point size is undefined.
- DirectWrite can enumerate the files of a font face, but a font supplied by a
  non-local/custom loader need not resolve to a normal filesystem path.

## 2. Model 1 Audit

| Item | Verdict | Verification and disposition |
|---|---|---|
| 1. Explicit CI deferral sentence is absent | **True** | The old risk-row mention concerned binary caching, not CI. §3 now states independently that CI automation is deferred and the design covers local presets/tests only. |
| 2. `dx11` is advertised before implementation | **True** | Old `STRINGS` accepted it while Phase 5 was conditional. §7.1 now exposes only `auto/opengl/metal`, validates membership, and adds `dx11` only with an implementation. |
| 3. `Paths::configDirectory()` has no failure channel | **True** | A raw path cannot represent `ConfigurationDirectoryUnavailable`. §6.3 now returns `PlatformResult<path>` and §6.4 defines non-fatal persistence failure. |
| 4. Renderer readback adapter has no implementable shape | **True** | Pixel assertions require backend-private access. §8 defines `RendererTestAccess`, `RendererTestFixture`, GL/Metal readback behavior, and test-only target isolation. |
| Minor: `Services` could become a god interface | **Partial** | Aggregation alone is not a god interface, but ownership/evolution rules were absent. §6.3 makes it a composition-root bundle, fixes lifetime, permits fake injection, and forbids adding methods to the bundle. |

## 3. Model 2 Audit

### 3.1 Blockers

| Item | Verdict | Verification and disposition |
|---|---|---|
| B1. “Platform-neutral core” hides PCL | **True** | `types.hpp` proves the dependency. §§1, 4, 6.1, and 7.2 now distinguish OS-neutral from dependency-free, show PCL/OpenCV/spdlog, retain PCL common/I/O and its provider-declared dependencies in core, and split direct PCLVisualizer use. |
| B2. Thread affinity contract is absent | **True** | Current worker results are marshalled through `ui_.post`; GL calls run on UI thread. §§6.5–6.7 now specify UI-thread ownership, immutable snapshots, revision coalescing, explicit frame context, and main-before-trajectory order. |
| B3. DPI/Retina semantics are absent | **True** | Current code reads `DisplayFramebufferScale`; old interfaces erased that fact. §6.7 now separates logical size, physical extent, and scale; projection/render targets use physical pixels and Metal configures `contentsScale`/`drawableSize`. |

### 3.2 Major Findings

| Item | Verdict | Verification and disposition |
|---|---|---|
| M1. ImGui 1.92 uses narrow `fopen` for ini on Windows | **False as stated; boundary risk true** | Vendored `ImFileOpen` performs UTF-8→wide conversion and `_wfopen`. The real bug is producing its UTF-8 argument via `path.string()`. §6.4 bans that boundary, defines conversions, and adopts native-path `SettingsStore` plus ImGui memory APIs. |
| M2. `std::getenv` cannot carry a Chinese Windows font path | **True** | Narrow CRT environment decoding is code-page dependent. §6.3 requires `_wgetenv` and native `path` construction. |
| M3. Two strong `createGuiRuntime` definitions may silently select one | **False** | Ordinary duplicate strong definitions are a link error, not silent selection. §6.7 nevertheless adds configure-time exactly-one-backend validation, a generated active-backend constant, and runtime context compatibility checks. |
| M4. CMake 3.25 excludes Ubuntu 22.04's packaged CMake | **True** | Workflow presets caused the unnecessary floor. §§5.1, 7.4, 10, and 13 now use CMake 3.21/schema 3 and separate configure/build/test commands. |
| M5. OpenMP portability is ignored | **True, with stronger finding** | No project source uses OpenMP. Rather than design three unused integrations, Phase 0 removes global discovery/flag mutation and permits later target-scoped reintroduction only for a real caller. |
| M6. Existing GUI tests force OpenGL in a Metal build | **True** | Current CMake links `OpenGL::GL`. §8 splits model, OpenGL, and Metal test targets and forbids an OpenGL dependency in the Metal preset. |
| M7. Readback contract is unimplementable | **True** | Same disposition as Model 1 item 4. |
| M8. Cross-backend pixel tolerance is undefined | **True** | §8 now defines behavioral checks, exact extent/layout, channel tolerance, center neighborhood, statistical color separation, and explicitly rejects bit identity. |

### 3.3 Minor Findings

| Item | Verdict | Verification and disposition |
|---|---|---|
| m1. spdlog omitted from dependency account | **True** | Added to current-state text and layering. |
| m2. `PointRenderer` is not “compact” | **True as editorial criticism** | Replaced with an accurate statement that roughly 25 methods mix four concerns. |
| m3. `Services` lifetime is absent | **True** | §6.3 defines owner, destruction order, and fake injection. |
| m4. Initial Metal upload may stall | **True** | §6.9 states UI-thread copy/coalescing and defers staging/ring buffers until measurement. |
| m5. Phase 0 Windows core build is blocked by monolithic PCL/VTK | **True, with scope correction** | Phase 0 now starts with core/viewer target split and validates with viewers disabled. `pcl_io` legitimately retains its imported-target dependency graph, which some providers conservatively overlink; the enforceable exclusion is direct PCLVisualizer use. |
| m6. Obsolete policy-version override is unaddressed | **True** | Phase 0 removes `CMAKE_POLICY_VERSION_MINIMUM 3.5`. |
| m7. Runtime/model target graph may cycle | **True as ambiguity risk** | §7.2 now defines contracts, model, jobs, app, and backend targets with one-way dependencies. |
| m8. GLFW+Metal may require a GLFW patch | **False for attachment; runtime edge cases remain** | Vendored official example proves Cocoa handle and `CAMetalLayer` attachment. §14 narrows the spike to Retina/sleep-wake behavior. |
| m9. Catch2 POSIX signal define is meaningless on Windows | **True** | Phase 0 conditions it on POSIX. The hypothetical Catch2 v3 migration is not designed without a version-upgrade request. |
| m10. `cloudRevision()` semantics are absent | **True** | §6.5 defines monotonic replacement-only revisions, no camera/style bump, and newest-result coalescing. |

### 3.4 Nits

| Item | Verdict | Verification and disposition |
|---|---|---|
| §5.5 lacks a bridge from offscreen texture to ownership | **True as readability issue** | Added an explicit “therefore” paragraph connecting pass ordering to runtime ownership. |
| TTC face parameter is not verified | **True** | Vendored API uses `ImFontConfig::FontNo`; §6.3 now names that exact field. |
| Windows viewer question is a product decision | **True** | Moved from implementation spikes to §15. |

## 4. Model 3 Audit

### 4.1 Critical Findings

| Item | Verdict | Verification and disposition |
|---|---|---|
| C1. Metal renderer receives no command buffer/context | **True** | §6.6 adds opaque `FrameContext`; runtime owns the active command buffer and backend-private renderer obtains encoders through it. Globals/thread-locals are forbidden. |
| C2. `App` model ownership and frame order are absent | **True** | §6.7 adds two `ViewportSession` objects and complete mutate/upload/resize/render/Image ordering. |
| C3. Phase 0 Windows acceptance conflicts with monolithic `kpt` | **True** | Same disposition as Model 2 m5. |
| C4. Metal point primitive/shader packaging is undesigned | **True** | §6.9 specifies `MTLPrimitiveTypePoint`, `[[point_size]]`, square-point parity, clamping, `.metallib` compilation/resource lookup, and diagnostics. |
| C5. `bool initialize` conflicts with structured error enum | **True** | §6.7 uses `Result<void, GuiError>`; §9 declares one error model and removes the duplicate bool/enum channel. |

### 4.2 Important Findings

| Item | Verdict | Verification and disposition |
|---|---|---|
| I1. ImGui 1.92 `ImTextureRef` and lifetime are absent | **True** | §6.6 returns `ViewportTexture{ImTextureRef, uv}` and defines resize/frame lifetime. |
| I2. UTF-8 rules do not reach dialogs/call sites | **True** | §6.4 adds the complete boundary table and ancestor/sibling plus Chinese-path tests. |
| I3. vcpkg toolchain example is incomplete | **Partial** | Chainload, triplet ABI fields, early compiler check, and local MSVC prerequisite were missing and are added in §7.3. A CI recipe is intentionally out of scope. |
| I4. GLFW X11 option leaks across hosts | **True** | §7.1 scopes X11/Wayland settings to Linux; Win32/Cocoa use host defaults. |
| I5. DirectWrite/Core Text local font path is underestimated | **True** | §6.3 and §14 add local-file resolution, no-match fallback, and an implementation spike. |
| I6. Frame concurrency is absent | **True** | Same disposition as Model 2 B2/m10. |
| I7. Metal smoke/readback acceptance is ambiguous | **True** | §8 separates mandatory offscreen contracts from spike-dependent automatic lifecycle smoke and explicit manual checks. |
| I8. Global CMake hygiene is absent | **True** | Phase 0 now mandates target-scoped dependencies/options and removes inactive globals. |
| I9. Jobs do not belong in `kpt_gui_model` | **True** | §7.2 creates `kpt_jobs`. |
| I10. `dx11` is advertised without implementation | **True** | Same disposition as Model 1 item 2. |

### 4.3 Minor Findings

| Item | Verdict | Verification and disposition |
|---|---|---|
| M1. macOS x64 lacks a debug preset | **True** | §7.4 adds it and also makes Linux vcpkg symmetric. |
| M2. Minimum CMake support needs a clear floor | **True** | CMake 3.21 is now explicit, with the feature reason. |
| M3. MSVC/clang-cl warnings are absent | **True** | Phase 0 adds compiler-family warning policy. |
| M4. No CI matrix appears in DoD | **Out of scope** | User explicitly deferred CI. §3 records that decision without implementation-level matrix/runner material. |
| M5. Retina mapping is underspecified | **True** | Same disposition as Model 2 B3. |
| M6. COM initialization is omitted | **True for Known Folders; overstated for DirectWrite** | Microsoft’s Known Folders guidance explicitly requires `CoInitializeEx` before `SHGetKnownFolderPath`; the earlier audit incorrectly generalized from the DirectWrite factory example. The design now gives a UI-thread RAII apartment to the Windows platform bootstrap and balances initialization on the same thread. |
| M7. Font face index application is absent | **True** | §6.3 specifies `ImFontConfig::FontNo`. |
| M8. Texture-origin deadline is absent | **True** | §6.6 normalizes UV in `ViewportTexture`; `App` has no flip. |
| M9. Platform/runtime factories lack test injection | **Partial** | Platform fake injection was absent; added in §6.3. `App` receives runtime-created renderer sessions, so unit tests inject fake renderers without replacing the global factory. |

## 5. Review of Review Quality

The reports correctly found the most dangerous omissions: hidden PCL/VTK
coupling, command-buffer access, App composition, thread affinity, DPI,
readback, error channels, and Phase 0 ordering. Their overlap materially raises
confidence in those findings.

They also show three recurring adversarial-review failure modes:

1. **Version drift**: assuming old Dear ImGui file behavior despite vendored
   1.92.8 code.
2. **Linker folklore**: claiming duplicate strong definitions silently choose
   an implementation.
3. **Speculative platform requirements**: treating a possible GLFW patch as
   established fact.

Accordingly, the revised design follows a stricter rule: repository version and
primary API evidence outrank generic platform memory.

Correction after a later plan review: the original audit’s COM verdict was
wrong for Known Folders. Microsoft’s current primary guidance explicitly
requires `CoInitializeEx` before `SHGetKnownFolderPath`; §4.3 M6 and the design
now reflect that evidence.

## 6. Second-Pass “Sabine Lens” Review

“Sabine lens” was not found as an established software-design review method.
For this audit it means a fresh red-team pass that does not reuse the three
models' issue taxonomy. The pass used nine independent lenses: lifecycle,
ownership, failure reachability, concurrency/versioning, dimensions/DPI,
dependency DAG, migration ordering, test observability, and product scope.

| Lens | New finding | Repair | Closure check |
|---|---|---|---|
| Failure reachability | `initialize` was fixed, but `createViewportRenderer`, `upload`, and `resize` could still allocate resources without an error return. | All now return `Result`; failure preserves valid state where possible and propagates. | No resource-creating renderer/runtime method in the contract has a bare success-only signature. |
| Platform failure reachability | Font lookup and UTF-8 conversion still used `optional`/bare values, conflating no match with decode/I/O failure; settings persistence had behavior but no interface. | `Fonts` and both path conversions now return `PlatformResult`; `SettingsStore` defines missing-file, atomic-save, and I/O-error semantics. | Absence, invalid UTF-8, and platform failure are distinguishable without sentinel paths/strings. |
| Lifecycle | `FrameContext` lifetime and nested-frame behavior were implicit. | Added `Created → Initialized → FrameActive → Initialized → Shutdown`, one-active-frame rule, context validity, partial-init cleanup, and idempotent shutdown. | Every context use lies between `beginFrame` and `renderAndPresent`. |
| Concurrency/versioning | Worker jobs could finish out of order; “monotonic revision” did not say who allocated it. | UI assigns generation before dispatch, starts at 1, reserves 0 for empty, and discards stale completions. | Camera/style do not bump revision; at most newest snapshot uploads. |
| Borrowed memory | `upload(span)` did not define whether the backend retained the span. | Contract requires copy/encoded copy before the span expires. | Snapshot/vector lifetime cannot become an undocumented backend dependency. |
| Dimensions | A collapsed panel can request a zero-sized target, invalid for GPU texture creation. | Zero extent suspends rendering, skips `ImGui::Image`, and recreates on positive size. | No zero-sized GPU resource is required. |
| DPI dynamics | Metrics existed but could become stale after moving the window between monitors. | Refresh each frame/callback; first milestone disables ImGui platform multi-viewports and names the single GLFW window as authority. | Logical/physical conversion has one current source. |
| Backend parity | “Clamp to device range” assumed a query/portable range not specified by contract. | Shared initial point-size range is `[1,64]`, lowered only if the minimum-Mac spike requires it. | Both backends test one declared range. |
| Target DAG | Target names were split but dependency directions remained partly implicit. | `contracts` owns POD/interfaces; model depends on contracts; app has no backend dependency; binary selects one backend. | No runtime↔renderer library cycle is required. |
| Migration order | Phase 2 required `FrameContext`, while Phase 3 claimed to introduce it. Phase 4 redundantly introduced services from Phase 1. | Phase 2 introduces the contract/minimal GL adapter; Phase 3 transfers ownership to runtime; Phase 4 integrates existing macOS services. | Each phase consumes artifacts from an earlier or the same phase only. |
| Dependency resolution | A preset could enable CMake viewers without the matching vcpkg manifest feature. | Each committed preset explicitly sets both `KPT_BUILD_PCL_VIEWERS` and `VCPKG_MANIFEST_FEATURES`. | Target graph and dependency graph cannot diverge by inherited default. |
| Test oracle | Empty/non-finite inputs were listed but expected behavior was not stated. | Empty means background-only; non-finite vertices are ignored and cannot corrupt later draws. | Contract now has a binary pass/fail oracle. |

The second pass found no remaining blocker after these repairs. Residual
uncertainty is deliberately isolated in §14 implementation spikes: pinned
vcpkg graph, imported PCL targets, Retina/sleep-wake behavior, Metal lifecycle
smoke reliability, and local font-file resolution. Those are evidence-gathering
tasks, not missing interface contracts.
