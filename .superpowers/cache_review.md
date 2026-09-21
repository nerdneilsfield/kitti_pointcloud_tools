# glm-5.2 — 純時/空對抗審閱

> 原則：魯棒性、安全性一概不論。只問「這條省了幾秒、幾 GiB，還是反向吃時間」。
> 基線：`ci.yml`（340 行，6 job）、`package-release.yml`（441 行，8 job）、`git ls-files` = 1150 檔。
> 現狀關鍵事實：所有 ccache key **已含 `github.sha`**（ci.yml:129/169/202），方案「No commit SHA」與之直接衝突。

---

## 甲·致命時間稅（方案聲稱省，實際不省或反虧）

### T1. macOS「version-neutral payload」純虧，notarize 不可省

方案 line 138-154 教條：cache version-neutral payload，release 時注入 version + 簽名 + notarize。

實測 release 流程（package-release.yml:236-253）：notarize 是 Apple 服務端異步，**120–900s 網路阻塞**，無論 payload 是否 cache hit 都得跑。

| 步驟 | hit 省 | hit 後仍跑 |
|---|---|---|
| cmake build | ✓ ~15min | |
| stage | ✓ ~30s | |
| 改 plist | | <1s |
| codesign | | 30–60s |
| **notarize** | | **120–900s** |
| stapler | | 10s |
| DMG 重建 | | 20s |

淨省 = build 時間，notarize 每次 release 全付。**而 release 頻率低**，省下的 build 時間復用次數極少。

反之：若直接 cache 完整 `.dmg`（key 含 version），release hit = 跳過 notarize，**省 5–15min/次**。教條在此處**反向吃時**。

**修法**：macOS 走專用完整-artifact cache，破「version-neutral」原則。key = `hash(ast, toolchain, version)`。

---

### T2. `release.deb` 單指紋 → 22/24/26 三連重建或錯命中

方案 line 63 一份 `release.deb` 指紋覆蓋三平台。改 `packaging/linux/Dockerfile` 中僅 24.04 相關行 → 指紋變 → key `deb-22/24/26` 全失效，**重建 3 個 DEB**（package-release.yml:62-103，每個 ~5–10min in Docker）。

反之：三平台共享 src，AST 同 → 指紋同 → 三 key 對一指紋。若 22.04 payload 已存，24.04 改了 Dockerfile，22.04 不該重建卻被牽連。

**時間稅**：5–10min × 誤觸發平台數。

**修法**：指紋分 `deb_common + deb_22_overlay / deb_24_overlay / deb_26_overlay`，key = `hash(common, overlay[i])`。

---

### T3. 跨 workflow ccache「共享」是幻覺，工具鏈異構 0% 命中

方案 line 180 聲稱 master/release 共享 ccache。現實：

- master native: `linux-system-debug` preset，系統 g++（ci.yml:136）
- release DEB: Docker 容器內 toolchain（package-deb.sh）
- release Windows: vcpkg + MSVC
- release macOS: vcpkg + Apple Clang + 不同 triplet

`CCACHE_COMPILERCHECK=content` 下，不同 preprocessor 輸出（g++ 版本、libc 頭、`_GNU_SOURCE` 差異）→ **hash 不同 → 0% 命中**。方案此條**不省一秒**，純文件幻覺。

**修法**：刪除該條聲稱，或改為「同 toolchain 內共享」（如 master native 與 release native-deb 若用同 preset 才共享——當前並非如此）。

---

### T4. Locator job 串行查 6–8 cache，25–40s 浪費/push

方案 line 196-198 承認 locator 不可避免，但**未並行化**。`actions/cache/restore` 每次 ~3–5s API 往返。6 stages 串行 = 30s+。

**修法**：
1. matrix 或同 job 多 step 並行 `actions/cache/restore`
2. locator `fetch-depth: 1` + `sparse-checkout` 僅取 `.github/semantic-fingerprints.json`，checkout 從 ~15s 降到 ~2s
3. 單 workflow 合併 ci + package-release，省一次 runner boot ~10s

---

## 乙·空間帳（方案未算清或反向浪費）

### P1. retention-days: 30 = 雙份空間

現 package-release.yml：deb×3 + windows + dmg + vsix 各 `retention-days: 30`。每 artifact 50–500MiB，6 產物 × 30 天 × N release = **數十 GiB artifact 存储**，與 cache **分開計費**。

方案 line 257 正確指出「artifacts 有獨立計費」，但**未給遷移策略**：cache hit 後是否還上傳 artifact？若 release pipeline 仍需 artifact 做 handoff（package-release.yml:330-346 download-artifact），則 artifact 不可省——**cache + artifact 雙份空間**。

**修法**：release 內同 workflow 用 job output 或 cache 做 handoff，刪 artifact 上傳。

---

### P2. emsdk-cache 跨 workflow scope 不對稱

ci.yml:216 與 package-release.yml:278 用同 key `kpt-emsdk-6.0.5-${{ runner.os }}-${{ runner.arch }}-v1`。GitHub cache 跨 workflow 可讀但 scope 受 ref 影響。

emsdk cache miss = **~1GB 下載 + 解壓**，60–120s。tag release 總在 master push 之後 → 通常 hit。但 `workflow_dispatch` 手動觸發 release 無前置 master → miss。

**修法**：release workflow 內加 master-ref restore 回退（`restore-keys` 或明確 `ref: master` 查詢）。

---

### P3. macOS stage artifact 200MiB–1GiB 下載

package-release.yml:210-218 下 arm64 + x86_64 stage 各一次。universal merge job 淨下載 200MiB–1GiB，30–90s 網路。

方案 payload cache hit 跳過整個 macos-universal job（含此下載）——✓ 真實節省。但見 T1，notarize 抵消大半。

---

### P4. vcpkg bootstrap 每次 Windows/macOS job 重跑，5–15min + 數 GiB

`setup-vcpkg` action 未讀，但典型 vcpkg install：**5–15min + 數 GiB**。方案 line 27 提「download vcpkg / bootstrap vcpkg」應被 cache hit 跳過——✓ 若實現則大省。

但**當前 ci.yml / package-release.yml 完全無 vcpkg cache**，是既有大坑。方案應明列此項收益作為賣點。

---

## 丙·方案未觸及的現存時間稅

### O1. `actions/checkout` × 6 job，~15s × 6 = 90s/push

每 job 獨立 checkout，倉庫 1150 檔 + third_party 子樹大。changes job（ci.yml:34）`fetch-depth: 0` 拉全史。

**修法**：
- 非 changes job 全用 `fetch-depth: 1`
- changes job 用 `paths` filter 或僅 SHA 比較，不需全史
- 純 fingerprint 讀的 locator 用 sparse-checkout 單檔

---

### O2. `npm ci` × 5 job，各 ~10–30s

vscode-build / browser / electron / vsix / release 各 `npm --prefix vscode ci`。setup-node cache 已啟（ci.yml:208），但 install 仍 10–30s/job。

**修法**：vscode 依賴打包成 cache layer，job 間共享。或 single build job 產 dist，下游 download-artifact（現狀部分如此 ci.yml:229）。

---

### O3. Windows `choco install imagemagick` 每次 60–90s

package-release.yml:117-118 無 cache。`release.windows` 指紋含 icon（方案 line 65），icon 不變則跳整個 windows job —— **此處理論省時正確**，方案少數亮點。

---

### O4. sanitizer matrix 各自 setup，2× ccache restore

ci.yml:145-179，asan-ubsan + tsan 兩 matrix 各自裝依賴 + ccache restore。兩 job 共享 `linux-system` base，可合併為單 job 內跑兩 preset，省一次 runner boot + apt install ~30s。

---

### O5. schema bump 全量失效，無共存期

方案 line 275 schema 變 → 全失效。17+ entries 同時 miss = 一次 CI 2+ 小時。

**修法**：key 同時查 v1/v2，漸進遷移。或 schema bump 與 code change 解耦（獨立 PR，擇低峰期）。

---

## 總帳

| 項 | 方案聲稱 | 實際時/空 |
|---|---|---|
| native comment-only | 省 ~10min | ✓ |
| VSIX cache | 省 ~5min | ✓ |
| Windows icon-driven | 省 ~10min + choco | ✓ |
| macOS payload | 省 build ~15min | △ notarize 5–15min 抵消（T1） |
| deb 三平台 | 省重建 | ✗ 單指紋致三連重建（T2） |
| 跨 workflow ccache | 省編譯 | ✗ 0% 命中（T3） |
| locator | 不可避免 | △ 可並行省 25s（T4） |
| artifact 空間 | 未提 | ✗ retention 30 天 = 雙份（P1） |

**真省時處**：native + VSIX + Windows。**教條反吃時處**：macOS version-neutral（T1）、deb 單指紋（T2）。**幻覺處**：跨 workflow ccache（T3）。

**修法優先級**：T1 > T2 > T4 > P1 > T3（刪該條）。餘可後迭代。

---

# k3-256k (Kimi) — 对抗性审阅：时间/空间，1ms 不放过

## 致命（事实性错误，计划不可行）

1. **90 天保留期系虚构。** GitHub Actions cache 硬规则：任何 cache **7 天未访问即自动清除**，无仓库级配置项。L187-191 "set it to 90 days" 不存在。低频发布流必丢 cache：release 到来时 master 的 payload cache 已蒸发，全部 miss。对策：接受 miss、weekly keepalive 触键 job（自身耗分钟）、或换 artifacts 以外的长期存储。
2. **15–25 GiB 目标超出硬上限。** 每仓库 cache 总额固定 **10 GiB**，不可"选择配额"。7 payload key + 5 test key + 每平台 ccache(256+128+512 MiB) 极易顶爆 10 GiB，互相 LRU 逐出。须按 10 GiB 重做尺寸分账。
3. **省钱前提不成立。** L188 自称 "public repository"；GitHub-hosted runner 对 public repo 不收分钟费。全篇 "省 hosted minutes" 动机只剩 wall-clock 延迟与 10 GiB 空间。计划应明说真实目标。

## 高危（正确性漏洞，错了即反噬时间）

4. **Merge commit 绕过 pre-commit hook。** Hook 仅 `git commit` 触发；merge、rebase、cherry-pick、GitHub 网页合并均不刷新 fingerprint。master 上 JSON 可与源码语义脱节 → 假 hit → 发布陈旧二进制 → 返工时间远超所省。须补 post-merge hook + merge commit 强制重算。
5. **CI 盲信本地 hook。** L130 "CI rejects fingerprint/schema mismatch" 只能验 schema，不能验语义——GitHub 不跑 AST。`--no-verify` 提交陈旧 JSON，CI 无从发现。信任边界须写明。
6. **跨平台 AST 失真。** L90-92 要求四平台 compilation database；Linux 机上无 Windows/macOS SDK headers，预处理结果与真实平台编译不同 → 假"未变" → 错 hit。保守 fallback 只覆盖"无法解析"，不覆盖"解析错"。

## 中危（直接浪费时/空）

7. **Locator 不必 checkout。** 读 fingerprint JSON 用 `gh api repos/{owner}/{repo}/contents/.github/semantic-fingerprints.json` 直取，省 `actions/checkout` 5–15 s；locator 从 ~1 min 压到 ~20 s。
8. **缺 `concurrency` group。** 两 producer 并发同 key：双 miss、双构建、双倍 runner 时间。须 `concurrency: kpt-stage-{key}`，`cancel-in-progress: false`。
9. **缺 workflow 级 `paths-ignore`。** docs-only 改动仍触发 locator（1 runner-minute 起跳）。`paths-ignore` 可零 runner；代价是 required status check 不落，需配 ruleset。计划未权衡。
10. **macOS notarization 不可缓存。** Release hit 路径仍须 repackage + 签名 + 公证，公证数分钟且必须 macOS runner。L27/L222 "cache hit 不分配 runner" 对 macOS release 不成立，表述过度乐观。
11. **Schema bump 无灰度。** v4→v5 一次即 7 payload + 5 test 全 miss，叠加 10 GiB 压力，重建风暴。

## 低危

12. L169-170 "test cache 大于所省则略" 无量测流程，靠感觉。
13. Locator 串行查 N 个 key，REST 逐调用，可并行。量小。
14. Fingerprint JSON 跨分支必冲突，rebase churn 无对策。

## 必修清单（按序）

1. 删 90 天幻想，写 7 天未访问即逐的现实模型 + keepalive 或接受 miss。
2. 按 10 GiB 硬顶重做尺寸预算；15–25 GiB 作废。
3. 补 merge/rebase 路径 fingerprint 刷新机制。
4. 写明 "CI 盲信本地 hook" 信任边界与 `--no-verify` 残余风险。
5. 跨平台 AST 的 SDK header 来源与失真防护。
6. Locator 免 checkout、加 concurrency group、评估 paths-ignore。

---

# grok-4.5 — 对抗性审阅（时/空 1ms·1B 尺度）

> 对象：`docs/semantic-ci-cache-plan.md`  
> 对照：`.github/workflows/ci.yml`、`package-release.yml`  
> 原则：省 Actions wall-time + billable minutes + cache/artifact 字节；正确性只在「会反噬时间/空间」时出现。  
> 裁决：**方向对，按现状落地会烧分钟，甚至比 path classify 更贵。Critical 未关前不可实现。**

## Strengths（真能省的部分）

1. 语义分析出 runner：指纹本地提交，GHA 只比 key。
2. `release.*` 排除 `tests/**`：堵 test-only 触发打包。
3. exact key、禁 PR cache、禁判定用 `restore-keys`：防错命中。
4. version-neutral payload + 后注入版本：避开「改 VERSION 全量重编」（macOS 签名/公证除外，见下）。
5. tag 不写权威 cache、master 暖 key：符合 ref scope。
6. locator 挡在贵 job 前：比「job 已分配再 restore miss」省启动税。

## Critical

### C1. Master 变「全量打包农场」——分钟可能暴涨

计划：master 生产 releaseable payload + test cache；acceptance 写 native 行为变 → master 跑 affected **release/test** stages。

现行 master **只测不打包**。落地后一次真行为变可同时点：native + asan + tsan + deb×3 + windows + macos×2 + vsix。**macOS ×10 倍率**。语义 false-positive 一次 = 整包 release 级账单。

**缺**：master 暖哪些 stage 的显式预算（例：master 只暖 `vsix`+`deb-24`+ccache；windows/macos 仅 tag 或 nightly）。不写清 = 优化变加税。

### C2. 本地多平台 AST 在工程上不可闭合

§Canonical AST 要求 Linux / Windows / macOS / Emscripten 各 compile_db 且互不替代。Linux 开发机无 MSVC/Apple 完整 compile_commands → hook 要么挡提交、要么用 Linux AST 顶（计划禁止）、要么 CI 再跑多 OS 分析（吃光「省 runner」）。

未解决跨 OS 语义证据前，「commit 时指纹权威」空；CI 被迫 conservative full rebuild。

**修**：平台权威源（Linux AST 管 deb/test.native；WASM 管 vsix；win/mac 仅在对应 runner 写 fingerprint 片段）或源+flags 结构哈希代理 + residual risk。

### C3. 指纹文件提交模型与并发/合并冲突

`.github/semantic-fingerprints.json` 全局不透明 hash：并行分支 merge 冲突/覆盖；rebase 后 JSON 与树不一致；`--no-verify` + CI mismatch → 全量 miss 或硬失败。冲突解决若重跑 bootstrap = 本地全扫 + 可能误改 key → **cache 全失效**。

**修**：按 stage 分文件 / merge driver；CI 确定性重算须面对 C2。

### C4. 「零 runner」路径未用尽

Acceptance：docs/README only → classifier only。`on.push.paths-ignore` 可让 workflow **完全不调度**（0 runner）。计划仍启 locator；现行 `ci-gate` always() 自带 runner；`workflow_dispatch.full` default true。

**修**：docs 级 workflow skip；locator 与 gate 合并；dispatch default false。

### C5. Locator 实现细节会吃掉「tiny」

未钉死：全树 checkout、`fetch-depth:0`、逐 key `actions/cache`、REST list 分页、`actions: write` 权限。

**修**：sparse-checkout 仅 fingerprint JSON；单 step 一次 lookup；禁 N 次完整 restore 样板；目标 wall <15s，同 run 能合的 job 必须合。

### C6. 缓存空间：exact×多 stage 易爆配额 → 连锁 miss

3 deb + win + 2 macos + vsix + 5 test + 多 ccache；每 semantic 变体整包不可变。活跃开发 20 fingerprint × 大包 ≫ 15–25 GiB 目标 → 逐出仍需 key → miss → 重编 → 正反馈。

**修**：每 stage 只保留最新 K=1|2；test-only 默认不存大二进制只靠 ccache；payload zstd 单文件；macOS 超 cap 不进 Actions cache。

### C7. ccache 多写者 + exact key 与 actions/cache 语义冲突

ccache key 无 SHA、master/release 共享；`actions/cache` 不合并并发 save → hit 率抖、重复编译。Payload「单生产者」未同样约束 ccache。

**修**：分片 key 或 post 串行 save；禁 multi-matrix 同 key 同时 save。

### C8. macOS：双 arch hit 仍不免贵 runner

universal + Developer ID + notary 仍要 macOS 分钟（公证常数级）。「release 全 hit 仍接近零」对 macOS 不成立。

**修**：拆 `release.macos-payload` vs `release.macos-sign`；写死签名 floor 预算。

### C9. Cleanup 自相矛盾

L251–253：producer 从不删、独立 janitor。L260–261：cleanup 属于 successful producer。实现者 save 后删旧 key → 并发删 last good → miss 风暴。

**修**：删权唯一 janitor；与 producer 互斥 concurrency group。

### C10. Tag 与 master 竞态：miss 重建且不回写

Tag 早于 master 写完 cache → release 现场编 → tag 不 save → 同 fingerprint 再 miss 再编。

**修**：release miss 后可写同 key（immutable：已存在 skip）；或 release needs 该 SHA master 绿。

## Important

| ID | 问题 | 时/空影响 |
|----|------|-----------|
| I1 | cheap repackage 可能半次打包（control/plist/json/sign） | 假省 |
| I2 | 命中后仍 download 全量 payload | 数 min + 数 GB |
| I3 | test artifact cache 默认应关，只 ccache | 恢复常慢于热编 |
| I4 | 现行：ccache 含 sha、restore-keys、artifact 30d、gate 独立、full default true、path `run_all` 过宽 | 未写歼灭清单 |
| I5 | locator 串行化全体 `needs` critical path | 15–60s 全局税 |
| I6 | product AST 变 → native/asan/tsan 三 job fan-out | 正确但贵，须写预期 |
| I7 | hit 后检查须 <1s（magic+listing），禁 install 测 | 防命中税 |
| I8 | schema v4→v5 全量 miss | 禁止顺手改 hash |
| I9 | PR 不用 master cache，PR 分钟基本不降 | 目标应只写 default branch+tag |
| I10 | 验收表缺 `jobs/billable min/cache GB` 列 | 无法对抗回归 |

## Minor

- deb key 未绑 toolchain-epoch：错 hit 或手升 v5。
- `SOURCE_DATE_EPOCH` 与 version inject 交互未写。
- Bootstrap 无耗时上限、CI 禁 bootstrap。
- L285 自指 adversarial review：应绑定本文件关闭条件。

## 场景 floor（对抗后）

| 场景 | 计划声称 | 真实 floor |
|------|----------|------------|
| 纯 docs | classifier only | **0 job**（paths-ignore）才达标 |
| 注释/rename | none | 依赖 C2/C3，否则 false miss 全量 |
| tests only | 单 test stage | locator + 可能 gate；无 test 大 cache 时 rebuild+ccache |
| 真 native 行为 | 受影响 stage | master 若暖全平台包 → **可能 ≫ 现行 CI** |
| 同 fingerprint 再 tag | 近零 | locator + version + 全 payload download + 注入 + **macOS 签/公证** + publish |
| 配额抖 | 不谈 | 连环 miss，分钟尖刺 |

## 落地顺序（省分钟优先）

1. 无 AST 硬省：paths-ignore、合 gate、ccache 去 SHA、artifact retention、dispatch default false、禁无意义 fetch-depth:0。
2. fingerprint 仅 Linux native + WASM/TS 闭环；win/mac 代理哈希。
3. Master 暖 cache 白名单 + 字节 cap + janitor 唯一。
4. Release：lookup → 条件 job；miss 允许回写；macOS 签名 floor 写死。
5. 最后全量 alpha-rename AST。

## Assessment

**Ready to implement?** No  

**Reasoning:** 缓存判定与 release/test 分离正确，但 C1–C10（master 打包化、跨 OS AST、指纹合并、零 runner 未用尽、locator/checkout 税、配额与 ccache 多写、macOS 签名 floor、cleanup 矛盾、tag 不回写）会在「省 1ms」目标下反噬分钟与 GiB。  

**关闭 Critical 最低证据：** 各 acceptance 行的 `jobs started / billable min / cache GB`（hit 与 miss）数字表，且 docs-only = **0 runner**。
