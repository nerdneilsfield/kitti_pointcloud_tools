# Semantic CI Cache Plan

## Purpose

Make GitHub Actions run only stages whose product behavior changed. A change
to formatting, comments, or local variable names must not rebuild a product.
A test-only change must run only its affected test stage and must never cause
a release package rebuild. A tagged release runs no tests.

This plan separates two concerns:

1. local, incremental semantic analysis produces committed stage fingerprints;
2. GitHub Actions compares those fingerprints with reusable caches and skips
   expensive setup/build on a hit.

GitHub does not parse ASTs. The commit hook does. This keeps runner startup
time and hosted minutes out of semantic analysis.

## Invariants

- Cache-key syntax never directly contains a commit SHA, run ID, tag name, or
  version number. In Legacy mode, `VERSION` remains an intentional semantic
  input, so its digest changes the exact artifact key.
- Every release fingerprint excludes `tests/**`.
- `test.*` fingerprints may include test source and fixtures; `release.*`
  fingerprints may not.
- Each selected stage begins with a same-platform exact cache restore. A hit skips its
  expensive build, dependency setup, Emscripten/vcpkg bootstrap, and ccache
  work. The small runner allocation is accepted because only this same-platform
  restore can prove the cache archive is usable; a cache hit cannot eliminate
  that stage's runner allocation without an equally expensive same-platform
  probe job.
- Release runs no native, browser, Electron, sanitizer, or package tests.
- A cache miss builds exactly that stage and no unrelated platform; release
  still runs zero test commands.
- Cache contents contain no credentials, signing material, tokens, or full
  working trees.
- A build payload cache is not proof that tests passed. A test runtime cache
  is not proof that tests passed. Master records test results independently;
  release deliberately does not gate publishing on tests.
- **Activation gate:** cache-only cross-version payload reuse remains disabled
  until a reviewed version-finalizer refactor exists. Before then, `VERSION`
  intentionally invalidates release payloads.

## Fingerprint contract

`.github/semantic-fingerprints/` contains one committed JSON fragment per
stage, avoiding a single all-branches-conflict file. It has two independent
namespaces:

```json
{
  "schema": 1,
  "release": {
    "deb_22": "...",
    "deb_24": "...",
    "deb_26": "...",
    "windows": "...",
    "macos": "...",
    "vsix": "...",
    "publish": "..."
  },
  "test": {
    "native": "...",
    "asan_ubsan": "...",
    "tsan": "...",
    "browser": "...",
    "electron": "..."
  }
}
```

### Release fingerprints

| Fingerprint | Semantic inputs |
|---|---|
| `release.deb_22` | Native product AST; common Linux package inputs; Ubuntu 22.04 Docker/overlay inputs |
| `release.deb_24` | Native product AST; common Linux package inputs; Ubuntu 24.04 Docker/overlay inputs |
| `release.deb_26` | Native product AST; common Linux package inputs; Ubuntu 26.04 Docker/overlay inputs |
| `release.windows` | Native product AST; Windows packaging/install resources; Windows icon resources |
| `release.macos` | Native product AST; macOS packaging/install resources; macOS icon resources |
| `release.vsix` | WASM decoder AST; extension-host and webview TypeScript AST; VSIX manifest/resources |
| `release.publish` | Release YAML publish section; Open VSX derivation/publish scripts; release metadata templates |

The release namespace excludes test sources, test fixtures, browser smoke
pages, test configuration, and test-only dependencies.

### Test fingerprints

| Fingerprint | Semantic inputs |
|---|---|
| `test.native` | Native product AST reachable by native tests; native test AST; fixtures |
| `test.asan_ubsan` | Same product AST; sanitizer-specific test/configuration AST |
| `test.tsan` | Same product AST; threading test/configuration AST |
| `test.browser` | Local extension-review input: browser test AST and fixtures |
| `test.electron` | Local extension-review input: Electron integration test AST and fixtures |

Changing only `tests/**` therefore alters only the `test` namespace. It does
not invalidate a DEB, ZIP, DMG, VSIX, ccache release key, or release stage.

## Local semantic analysis

### Canonical AST

The analyzer runs once on Linux and considers only repository source-language
ASTs: C/C++ source/header syntax and TypeScript syntax. It does **not** model
transitive includes/imports, SDK headers, compiler flags, sysroots, generated
dependency graphs, macro expansion, link order, or target-specific conditional
compilation. Thus one Linux parser result is intentionally shared by Windows,
macOS, and WASM source fingerprints.

This is a speed/cost policy, not a proof of compiled behavior. Dependency,
SDK, compiler, and toolchain-only changes reuse the source-AST fingerprint.
They do not enter source analysis or stage selection. Cache restore still
partitions archives by compatible build environment; that is a byte-format
guard, not a claim that an external dependency changed product behavior.

It hashes canonical ASTs, not source text. Canonicalization removes:

- comments, locations, whitespace, token offsets, and formatting;
- local variable and parameter spellings through alpha-renaming;
- compiler-generated mangled names and source-position metadata.

It retains source control flow, operators, literals, written type syntax,
calls, linkage-visible declarations, exported names, attributes, and resource
bytes. It does not resolve overloads, external declarations, imports, or
compiler-provided types.
Direct runtime/package resources without an AST use a structural parser and
canonical serialization. They have no transitive dependency closure. Binary
assets are direct resource bytes because they have no AST representation.

### Incremental cache

The local AST cache lives at `.git/kpt-semantic-cache/` and is never committed
or uploaded. It stores only per-file AST digests, not complete AST dumps,
dependency edges, or build output.

The pre-commit hook:

1. reads the staged diff;
2. maps changed, renamed, added, and deleted source/header/TypeScript files
   and direct stage resources to their owning stages;
3. reparses only changed source-language files;
4. reuses all unaffected file digests; it does not follow include/import
   graphs;
5. writes the aggregate JSON and stages it.

The first cache population is an explicit bootstrap command. It may scan the
whole project once. Regular commits must never silently fall back to a full
scan; a missing/stale local cache stops the commit and requests bootstrap. A
parser-version change invalidates its source-language namespace. Dependencies,
compiler configuration, SDKs, and generated dependency graphs are intentionally
outside AST invalidation.

The hook checks that generated fragments are current. `--no-verify` is treated
as an exceptional escape hatch. Because CI intentionally does not recompute
all ASTs, the hook is not a cryptographic trust boundary: web merges, rebase,
cherry-pick, and `--no-verify` can leave stale data. Install matching
post-merge/post-rewrite hooks and make CI reject only missing/schema-invalid
fingerprints. CI does not infer dependency or toolchain impact from source
changes; this is deliberate.

The classifier obtains fragments through the GitHub Contents API without
checkout. It dispatches selected stages in parallel and never serially probes
N cache keys. `paths-ignore` applies only to explicitly unpackaged docs after
branch protection accepts an absent CI check (or a separate zero-cost status).
The Contents API only reads committed fingerprint fragments; it never claims a
payload exists. Cache REST results are likewise metrics only. The selected
stage's same-platform restore remains the authority.

## Cache layout

### Release cache modes

The plan has two explicit modes. They are never mixed.

**Legacy mode (current):** cache final versioned artifacts. `VERSION` is in
the release fingerprint, so a version bump intentionally misses all owning
stages. A hit can only serve the exact same version. This is correct for the
current binaries/manifests and is the first cache rollout.

**Payload mode (future):** cache version-neutral payload archives, then run a
reviewed/tested finalizer to inject version/signing metadata. Payload mode may
only replace legacy mode after the activation gate below passes.

Payload mode caches only version-neutral payload archives; never build directories,
`node_modules`, vcpkg installations, Emsdk installations, or complete macOS
build trees. A final `.deb`, `.vsix`, `.dmg`, or ZIP embeds a version and
cannot be reissued as a different version unchanged.

**macOS exception:** even after Payload mode exists, cache a complete,
versioned, signed and notarized DMG under `macos + version + toolchain` when
its bytes are reusable. Notarization is a remote 2–15 minute floor that a
payload hit does not remove; re-notarizing an unchanged DMG is slower than
storing it. The macOS cache budget is separately capped because it is large.

```text
kpt-payload-v4-deb-22-{environment-epoch}-{release.deb_22}
kpt-payload-v4-deb-24-{environment-epoch}-{release.deb_24}
kpt-payload-v4-deb-26-{environment-epoch}-{release.deb_26}
kpt-payload-v4-windows-{environment-epoch}-{release.windows}
kpt-payload-v4-macos-arm64-{environment-epoch}-{release.macos}
kpt-payload-v4-macos-x64-{environment-epoch}-{release.macos}
kpt-payload-v4-vsix-{environment-epoch}-{release.vsix}
```

**Current-state limitation:** KPT compiles `KPT_VERSION_STRING` into native
binaries and embeds versions in CPack, Info.plist, and VSIX manifests. Thus
version-neutral payload reuse is not implementable yet. Until a separately
reviewed runtime-version/finalizer refactor exists, `VERSION` remains a
release input and a version bump correctly misses each affected payload cache.
Legacy mode caches that exact final artifact; it is forbidden to rename it as
a new version.

After that refactor each platform must define/test its exact payload/finalizer
contract: DEB control/payload, Windows ZIP/executable metadata, macOS
lipo/Info.plist/codesign/notarization/DMG, and VSIX manifests/ZIP. README,
LICENSE, NOTICE, icons, locales, and signing epoch are finalizer inputs.

### Test-only artifacts

Test-only caches are distinct and never consumed by release:

```text
kpt-test-v4-native-{test.native}
kpt-test-v4-asan-{test.asan_ubsan}
kpt-test-v4-tsan-{test.tsan}
kpt-test-v4-browser-{test.browser}
kpt-test-v4-electron-{test.electron}
```

They contain the smallest reusable test runtime/binary needed by that suite.
Store one only when measured `restore + decompress` is lower than measured
`compile/runtime saved`; otherwise omit it and rely on ccache.

Browser and Electron extension review are local-only. They may take ten or
more minutes and must not consume ordinary CI or release runner time. Run
`npm --prefix vscode run test:browser` and `npm --prefix vscode run
test:vscode` before an extension review/publish decision. Their fingerprints
remain for local reporting, but select no GitHub Actions job.

### ccache

ccache remains a separate, small compiler-object cache:

```text
kpt-ccache-v4-{exact-preset}-{compiler-content}-{sysroot}-{toolchain-epoch}
```

It is shared only by jobs using the exact same preset/compiler/sysroot, never
merely because both are called “native” or run in different containers. ccache
itself hashes preprocessed compiler inputs and therefore reuses unaffected
objects after a small functional change. `CCACHE_COMPILERCHECK=content` is
mandatory. No commit SHA is present.
Set per-cache `CCACHE_MAXSIZE` (initial limits: 256 MiB native, 128 MiB WASM,
512 MiB packaging) and let ccache evict its own LRU objects. Since GitHub
caches are immutable, first rollout treats each remote ccache key as a
read-only seed; it does not claim continuous remote refresh. Rolling remote
ccache generations need a serialized producer and separate review.

Dependency caches are distinct from artifact caches:

- vcpkg: exact `{OS, arch, triplet, baseline, toolchain epoch}` binary cache;
  restore a default-branch seed before bootstrap and enforce a byte budget.
- Emsdk: exact `{OS, arch, Emscripten version}` cache; tag/dispatch restores
  default-branch seed before downloading the roughly 1 GiB SDK.
- npm: `setup-node` lockfile cache only; payload-hit and publish-only jobs do
  not run `npm ci`. Never cache `node_modules` as release payload.

Sanitizer presets share one Linux setup only if measured wall time and
available parallelism beat separate jobs; record the selected topology.

GitHub cache retention/storage are repository configuration, not workflow
code. The cache ledger records actual limits and treats retention as best effort:
eviction or restore failure is always possible. Maintain a measured cache
ledger (key, bytes, hit count, saved seconds). Begin under the actual repo
limit; do not assume 15–25 GiB or any retention period. Keep at most two
semantic generations per small stage, one for large macOS payloads, and no
test binary cache whose restore time exceeds recompilation.

Before admitting a stage archive, record its compressed bytes, restore plus
decompress seconds, and avoided setup/build seconds. Set its cap and minimum
saving in a checked-in cache-policy table from those measurements; a stage
without positive measured ROI is not cached. This prevents a large, low-hit
DMG or test runtime from evicting high-value compiler/dependency seeds.

## Master CI DAG

`master` has a minimal classifier job that reads fragments through the Contents
API. It performs no checkout/history clone. Each selected platform stage then
uses `actions/cache/restore` on that same platform as the authoritative cache
check. REST/API lookup may provide observability only; it must not decide
job-level skipping.

For each stage:

1. same-platform exact cache restore hit: skip expensive build commands; a
   test runtime hit only skips test compilation, never test execution;
2. exact miss: build/tests only that stage;
3. an unselected stage uses job-level `if` and allocates no runner;
4. a selected hit has only its small restore runner;
5. success saves its immutable new cache. Cleanup is separate, serialized,
   and quota-driven; producers never delete their own previous good cache.

Master produces build payload caches and test-only runtime caches. A native
test change can therefore start only a native test job, while platform
packages and VSIX jobs remain absent. Test conclusions are recorded for human
inspection only; they neither invalidate release payloads nor gate release.

Master is not a release-package farm. It warms only payloads already built by
an existing master stage (currently the Linux/WASM VSIX path) and test caches.
It must not add DEB×3, Windows, or macOS packaging merely to warm release
caches. Those payloads are first built by their release miss, then promoted
to master scope for later reuse.

## Release DAG

Release reads only `release.*`. It never restores test caches or runs tests.

For each package stage:

1. each selected platform performs same-platform exact cache restore;
2. on hit, skip expensive build commands in that platform job;
3. on miss, build only that missing platform payload with tests disabled, then
   save it under the exact semantic key;
4. finalise current-version artifacts only through reviewed platform contracts,
   then run cheap structural checks;
5. derive/publish Open VSX from the VSIX; no immediate consistency GET poll.

`release.publish` is excluded from payload keys. Changing it must
start only the final publishing job and reuse platform/VSIX payloads.

Within a release workflow, final payload handoff uses a one-day artifact only
when independent platform jobs must converge. It is deleted/expired after
publish and never duplicated as a 30-day artifact. Cache is the cross-run
store; the artifact is the short same-run/tag-promotion bridge. Only a cache
miss producer uploads it; cache-hit and publish-only paths upload none.

macOS complete-DMG cache hit skips build/stage/notarize, but changed version or
signing identity still has an unavoidable macOS codesign/notarize runner floor.
Report that floor separately; never advertise it as a zero-runner hit.

## Master/tag cache reachability

GitHub Actions cache scope is ref-aware. A tag run can restore a matching
default-branch cache, so `master` is the authority that warms releaseable
keys. Exact keys are shared by semantic fingerprint, not commit.

REST/API lookup is advisory: Actions Cache also binds an opaque
archive-layout/compression version. Cache-key epoch includes cache-action
major, layout, and compression contract. Consumer restore is truth: a restore
miss after an advisory lookup runs only that stage (with tests disabled on release),
never consumes a near match. If master completes a matching cache shortly
before release restore, release sees the hit and no expensive build commands
run.

Tag-only caches cannot reliably warm master or later tags. When release builds
a miss, it uploads the real cached archive (a Legacy final artifact or a
Payload-mode archive) plus its manifest as a short-retention Actions artifact.
A trusted automatic `workflow_run` promotion on `refs/heads/master` downloads
that artifact, verifies stage/key/payload digest, then uses
`actions/cache/save` to write it in master scope. Manual dispatch is recovery
only. Promotion has no tests and no build, and uses a separate
`kpt-promotion-{stage}-{fingerprint}` concurrency group with
`cancel-in-progress: false`.

Until promotion completes, a later release may fetch only that exact
short-retention bridge artifact by stage, fingerprint, producer provenance,
and manifest digest; it then skips compilation. It may never use a prefix,
PR-produced, or unverified artifact. This closes the short tag-to-master race
without pretending tag-scope caches are globally visible. Release never saves
test-only cache entries.

## Cache lifecycle and cost control

- One exact cache per `{stage, semantic fingerprint}` is immutable.
- Producers never delete old cache entries. A separate low-frequency, trusted
  quota janitor deletes old stage entries by exact cache ID after a newer
  payload has existed; it cannot run concurrently with producers.
- Delete test-only caches before releaseable caches when quota pressure occurs.
- Legacy mode caches only exact compressed final artifacts for their embedded
  version; it never reuses or renames them across versions. Payload mode caches
  only version-neutral archives until its finalizer contract exists. Both modes
  cache bounded ccache directories.
- Do not use Actions artifacts as long-term cache; artifacts have separate
  retention/storage billing and are for same-run handoff.
- Do not upload logs, test reports, source trees, `node_modules`, or build
  directories unless a failure explicitly requests diagnostics.
- Cache cleanup is never part of a producer or consumer, so a failed build
  cannot destroy the last known good cache.
- Each producer uses `concurrency: kpt-payload-{stage}-{fingerprint}` with
  `cancel-in-progress: false`; identical concurrent misses wait rather than
  compile duplicate payloads. ccache writers use an independent compatible
  preset concurrency group.
- Schema migrations dual-read only when a written compatibility proof says
  canonicalization is equivalent; default behavior is no old-key restore. A
  schema bump is never combined with a broad source change.

## Concurrency and correctness

- Same cache key may be written by only one successful producer. Later saves
  treat an already-existing exact key as success and retain immutable content.
- Cache consumers use exact keys only: never `restore-keys`, prefix cache hits,
  or a cache written by a pull request. A restore miss after observation is a
  stage miss, not permission to consume a near match.
- Publish is idempotent: GitHub asset upload uses replacement semantics; Open
  VSX uses `--skip-duplicate`. A version ledger compares remote metadata or
  digest to the current final-artifact manifest; same version plus differing
  bytes fails.
- A cache hit still performs file existence and archive-format checks, not a
  rebuild or test suite.
- Fingerprint schema/tool version is in every cache prefix. Changing
  canonicalization invalidates caches deliberately.

## Implementation order

1. Finish local incremental C++ and TypeScript AST cache; add explicit
   bootstrap and hook installation command.
2. Add deterministic semantic fingerprint fixtures: comments, formatting,
   local rename, parameter rename, header dependency, resource change,
   test-only change, and behavior change.
3. Run adversarial static reviews on canonicalization, direct stage mapping,
   cache scope, cleanup races, version injection, cache byte caps, and DAG
   costs.
4. Add low-cost baseline wins before AST cache: remove commit SHA from current
   ccache keys, make non-diff checkout depth one, set dispatch full=false,
   lower release artifact retention to one day, and add docs-only path ignore
   only after branch-rule status behavior is explicitly accepted.
5. Add the master classifier and per-stage cache producers/consumers.
6. Add release classifier and cache-only release path; remove all release
   test commands.
7. Record jobs started, wall time, cache bytes, restore time, and saved build
   time for every acceptance row; reject a cache that costs more than it saves.
8. Validate YAML and scripts locally without pushing or dispatching Actions.
9. Only after local review approval, make one controlled push.

## Acceptance cases

| Change | Expected master work | Expected release work |
|---|---|---|
| C++ comment/format/local rename | none | none |
| native behavior | affected native release/test stages | affected platform package stages |
| `tests/**` only | affected test stage only | none |
| VS Code browser/Electron test only | local extension review only | none |
| VS Code runtime behavior | VSIX + affected tests | VSIX only |
| Windows icon/package behavior | Windows package stage | Windows package stage |
| Open VSX publish script only | publish validation only | publish job only |
| Unpackaged docs only | zero runner via approved `paths-ignore` | no release stage |
| Packaged README/LICENSE/NOTICE | no compile/test stage | package/finalizer only |
| Runtime locale/icon/manifest | owning runtime stage | owning finalizer or release stage |
