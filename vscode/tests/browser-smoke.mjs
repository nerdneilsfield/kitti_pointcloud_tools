import { chromium } from "@playwright/test";
import { existsSync, readFileSync } from "node:fs";

const extensionSource = readFileSync(
  new URL("../src/extension.ts", import.meta.url),
  "utf8",
);
const webviewSource = readFileSync(
  new URL("../webview/main.ts", import.meta.url),
  "utf8",
);
const viewerSource = readFileSync(
  new URL("../webview/viewer.ts", import.meta.url),
  "utf8",
);
if (/decoderResources(Request|Error)?/.test(extensionSource) ||
    /decoderResources(Request|Error)?/.test(webviewSource) ||
    /postMessage\(\{\s*type:\s*["']decoderResources/s.test(webviewSource)) {
  throw new Error("decoder resources must not cross the extension-host boundary");
}
if (/const layerRequestId = \+\+requestId/.test(extensionSource) ||
    !/let nextLayerMessageId = 2_000_000_000/.test(extensionSource)) {
  throw new Error("layer message IDs must not mutate document load generation");
}
if (!/export class LayerPayloadQueue/.test(extensionSource) ||
    !/layerQueue\?\.settle\(message\.requestId\)/.test(extensionSource) ||
    !/layerQueue\?\.retryInFlight\(\)/.test(extensionSource) ||
    !/this\.inFlight !== undefined/.test(extensionSource)) {
  throw new Error("host must serialize Remote layer payloads until render acknowledgement");
}
if (!/function cancelLayerDecodes\(/.test(webviewSource) ||
    !/Layer load cancelled by reload/.test(webviewSource)) {
  throw new Error("Reload must settle a backpressured layer request");
}
if (!/webviewOptions: \{ retainContextWhenHidden: true \}/.test(extensionSource)) {
  throw new Error("layer review must retain its webview while editor is hidden");
}
if (/event\.(origin|source)/.test(webviewSource)) {
  throw new Error("webview must not reject VS Code messages by browser origin");
}
if (!/Math\.max\(0, Math\.min\(size, 5\)\)/.test(viewerSource)) {
  throw new Error("webview renderer does not enforce point size in [0,5]");
}
if (!/if \(this\.cloud\) \{[\s\S]*this\.cloud\.visible = showPoints/s.test(
  viewerSource,
)) {
  throw new Error("point size zero must hide clouds even without a LOD cloud");
}
if (!/function rendererGpuBudget\(/.test(viewerSource) ||
    !/renderQuality: "full" \| "lod"/.test(viewerSource) ||
    !/minimumGpuLodPoints/.test(viewerSource) ||
    !/recoverGpuAllocationFailure/.test(viewerSource) ||
    !/context\.OUT_OF_MEMORY/.test(viewerSource)) {
  throw new Error("layer renderer lacks a bounded full/LOD GPU policy");
}
if (!/cpuBytesInUse/.test(viewerSource) ||
    !/estimatedCpuBytes\(/.test(viewerSource) ||
    !/CPU review budget is exhausted/.test(viewerSource)) {
  throw new Error("layer scene must bound retained CPU snapshots");
}
if (!/cameraDepth\(layer/.test(viewerSource) ||
    !/camera\.matrixWorldInverse/.test(viewerSource)) {
  throw new Error("transparent layers must sort by camera-space depth");
}
if (!/maximumPickingCandidates = 25_000/.test(viewerSource) ||
    !/id="picking-scope"/.test(extensionSource) ||
    !/getPickingScope\(\)/.test(webviewSource)) {
  throw new Error("multi-layer picking must expose its active-only degradation");
}
if (!/roiGeneration/.test(viewerSource) ||
    !/filterClosedWorldRoiCooperatively/.test(viewerSource) ||
    !/window\.setTimeout\([\s\S]*150/.test(viewerSource)) {
  throw new Error("ROI filtering must debounce and discard stale generations");
}
if (!/id="display-toggle"[^>]*aria-controls="overlay-menu"/.test(extensionSource) ||
    !/id="details-toggle"[^>]*aria-controls="information"/.test(extensionSource) ||
    !/id="point-size"[^>]*min="0"[^>]*max="5"[^>]*step="0\.05"/.test(
      extensionSource,
    ) ||
    /\$\{vscode\.l10n\.t\("webview\./.test(extensionSource) ||
    !/#controls-help\s*\{[^}]*position:\s*fixed/s.test(extensionSource) ||
    !/@media \(max-width: 900px\)[\s\S]*#information\s*\{/s.test(
      extensionSource,
    )) {
  throw new Error("production webview lacks polished UI or point-size contract");
}

const baseUrl = process.env.KPT_VSCODE_SMOKE_URL ?? "http://127.0.0.1:8766";
const defaultExecutable = chromium.executablePath();
const headlessShellExecutable = defaultExecutable
  .replace(
    /\/chromium-(\d+)\/chrome-linux64\/chrome$/,
    "/chromium_headless_shell-$1/chrome-headless-shell-linux64/chrome-headless-shell",
  )
  .replace("chromium-", "chromium_headless_shell-")
  .replace(
    "/chrome-linux64/chrome",
    "/chrome-headless-shell-linux64/chrome-headless-shell",
  );
const executablePath = [defaultExecutable, headlessShellExecutable]
  .find((candidate) => existsSync(candidate));
const browser = await chromium.launch({
  headless: process.env.KPT_WEB_HEADED !== "1",
  args: [
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
  ],
  executablePath,
});
try {
  const productionStyle =
    extensionSource.match(/<style>([\s\S]*?)<\/style>/)?.[1];
  if (!productionStyle) throw new Error("cannot extract production webview CSS");
  for (const width of [320, 480, 800, 1150]) {
    const layoutPage = await browser.newPage({
      viewport: { width, height: 600 },
    });
    await layoutPage.setContent(`
      <style>${productionStyle}</style>
      <div id="information"><div id="status" class="glass">Ready</div><section id="cloud-info" class="glass">
        <strong>AABB</strong><span>Min: (-78.9, -36.9, -2.4)</span>
        <span>Max: (77.2, 63.6, 2.9)</span><span>Size: (156, 100, 5)</span>
      </section></div>
      <div id="toolbar" class="glass"><div class="tool-group"><select><option>Intensity</option></select>
        <label id="point-size-control"><span>Size</span><input id="point-size" type="range"><output>1.50</output></label></div>
        <div class="tool-group"><button>Fit</button><button>Reload</button></div>
        <div class="tool-group"><button>Top</button><button>Front</button>
        <button>Left</button><button>Right</button><button>Iso</button></div>
        <div class="tool-group">
        <button id="display-toggle" aria-controls="overlay-menu" aria-expanded="false">Display</button>
        <input type="color"></div>
        <div class="tool-group"><button id="details-toggle" aria-controls="information" aria-expanded="true">Details</button></div></div>
        <div id="overlay-menu" class="glass"><label><input type="checkbox"> Axes</label>
          <label><input type="checkbox"> 3-plane scale grid</label>
          <label><input type="checkbox" checked> Noise</label>
          <label>Fixed <input type="color" value="#ffffff"></label>
          <label>Noise <input type="color" value="#ff0000"></label></div>
      <details id="controls-help" class="glass"><summary>Controls</summary></details>
      <div id="player" class="glass" style="display:flex"><button>Play</button>
        <input id="frame" type="range"><span>1 / 4</span><select><option>5 fps</option></select></div>
    `);
    await layoutPage.locator("body").evaluate((body) => body.classList.add("sequence"));
    const information = await layoutPage.locator("#information").boundingBox();
    const toolbar = await layoutPage.locator("#toolbar").boundingBox();
    if (!information || !toolbar ||
        rectanglesOverlap(information, toolbar)) {
      throw new Error(`production overlays collide at ${width}px`);
    }
    const menu = layoutPage.locator("#overlay-menu");
    const toggleBox = await layoutPage.locator("#display-toggle").boundingBox();
    if (!toggleBox || toggleBox.x < 0 || toggleBox.x + toggleBox.width > width) {
      throw new Error(`display toggle is not initially visible at ${width}px`);
    }
    const menuBox = await menu.boundingBox();
    if (!menuBox || menuBox.x < 0 || menuBox.x + menuBox.width > width) {
      throw new Error(`display menu leaves viewport at ${width}px`);
    }
    for (const input of await menu.locator("input").all()) {
      await input.click();
    }
    const help = await layoutPage.locator("#controls-help").boundingBox();
    const player = await layoutPage.locator("#player").boundingBox();
    if (!help || !player || rectanglesOverlap(help, player)) {
      throw new Error(`sequence controls collide at ${width}px: ${
        JSON.stringify({ help, player })
      }`);
    }
    await layoutPage.close();
  }

  for (const path of [
    "/vscode/tests/bootstrap-failure.html",
    "/vscode/tests/timeout-smoke.html",
    "/vscode/tests/queued-timeout.html",
    "/vscode/tests/worker-recovery.html",
    "/vscode/tests/worker-smoke.html",
    "/vscode/tests/sequence-smoke.html",
    "/vscode/tests/decoder-contract.html",
    "/vscode/tests/converter-smoke.html",
  ]) {
    const page = await browser.newPage();
    const browserErrors = [];
    page.on("console", (message) => {
      if (message.type() === "error") browserErrors.push(message.text());
    });
    page.on("pageerror", (error) => {
      browserErrors.push(error.message);
    });
    await page.goto(`${baseUrl}${path}`);
    try {
      await page.locator("body:not([data-state='loading'])").waitFor({
        timeout: 30_000,
      });
    } catch (error) {
      throw new Error(
        `${path} timed out: ${await page.locator("body").textContent()}\n` +
        `${browserErrors.join("\n")}`,
        { cause: error },
      );
    }
    const body = page.locator("body");
    const state = await body.getAttribute("data-state");
    const text = await body.textContent();
    const unexpectedErrors = path.endsWith("bootstrap-failure.html")
      ? browserErrors.filter(
          (message) => !message.includes("Failed to load resource"),
        )
      : browserErrors;
    if (state !== "passed" || unexpectedErrors.length > 0) {
      throw new Error(
        `${text ?? `smoke ended in state ${state}`}\n${
          unexpectedErrors.join("\n")
        }`,
      );
    }
    if (path.endsWith("worker-smoke.html")) {
      await page.locator("#cloud-info[data-bounds='available']").waitFor();
      await page.evaluate(async () => {
        const first = await fetch("/data/000123.pcd").then((response) =>
          response.arrayBuffer());
        const second = first.slice(0);
        for (const [requestId, sourceKey, name, bytes] of [
          [1_000_000_001, "sha256:" + "1".repeat(64), "overlay-a.pcd", first],
          [1_000_000_002, "sha256:" + "2".repeat(64), "overlay-b.pcd", second],
        ]) {
          window.dispatchEvent(new MessageEvent("message", {
            data: { type: "addLayer", requestId, sourceKey, name, bytes },
          }));
        }
      });
      await page.locator("#layer-list option").nth(2).waitFor();
      if (await page.locator("#layer-list option").count() !== 3) {
        throw new Error("concurrent addLayer messages did not retain all layers");
      }
      await page.evaluate(async () => {
        const [third, fourth] = await Promise.all([
          fetch("/data/000123.pcd").then((response) => response.arrayBuffer()),
          fetch("/data/000123.pcd").then((response) => response.arrayBuffer()),
        ]);
        for (const [requestId, sourceKey, name, bytes] of [
          [1_000_000_003, "sha256:" + "3".repeat(64), "overlay-c.pcd", third],
          [1_000_000_004, "sha256:" + "4".repeat(64), "overlay-d.pcd", fourth],
        ]) {
          window.dispatchEvent(new MessageEvent("message", {
            data: { type: "addLayer", requestId, sourceKey, name, bytes },
          }));
        }
      });
      await page.locator("#layer-list option").nth(4).waitFor();
      if (!/active layer .* only \(5 visible layers\)/.test(
        await page.locator("#picking-scope").textContent() ?? "",
      )) {
        throw new Error("five visible layers did not expose active-only picking");
      }
      await page.locator("#layer-visible").uncheck();
      if (!/Picking: all 4 visible layers/.test(
        await page.locator("#picking-scope").textContent() ?? "",
      )) {
        throw new Error("four visible layers did not return to all-layer picking");
      }
      await page.locator("#layer-visible").check();
      await page.locator("#remove-layer").click();
      await page.locator("#layer-list option").nth(3).waitFor();
      await page.locator("#layer-list").selectOption({ index: 3 });
      await page.locator("#remove-layer").click();
      if (await page.locator("#layer-list option").count() !== 3) {
        throw new Error("temporary picking layers were not removable");
      }
      await page.locator("#layer-list").selectOption({ index: 0 });
      await page.locator("#color-mode").selectOption("fixed");
      await page.locator("#fixed-color").evaluate((input) => {
        input.value = "#00ff00";
        input.dispatchEvent(new Event("input", { bubbles: true }));
      });
      await page.locator("#layer-list").selectOption({ index: 1 });
      await page.locator("#color-mode").selectOption("intensity");
      await page.locator("#layer-list").selectOption({ index: 0 });
      if (await page.locator("#color-mode").inputValue() !== "fixed" ||
          await page.locator("#fixed-color").inputValue() !== "#00ff00") {
        throw new Error("selecting a layer did not restore its toolbar style");
      }
      await page.locator("#layer-list").selectOption({ index: 1 });
      if (await page.locator("#color-mode").inputValue() !== "intensity") {
        throw new Error("active layer style leaked into another layer");
      }
      await page.locator("#layer-pos-x").fill("7");
      await page.locator("#apply-layer-transform").click();
      const activeOverlayKey = await page.locator("#layer-list").inputValue();
      for (const [id, value] of Object.entries({
        "roi-min-x": "-80", "roi-max-x": "80",
        "roi-min-y": "-40", "roi-max-y": "65",
        "roi-min-z": "-3", "roi-max-z": "3",
      })) {
        await page.locator(`#${id}`).fill(value);
      }
      await page.locator("#apply-roi").click();
      await page.locator("#roi-result").getByText(/^ROI: [\d,]+ points$/).waitFor();
      await page.locator("#reload").click();
      await page.locator("body[data-loads='2']").waitFor();
      await page.locator("#status[data-kind='ready']").waitFor();
      await page.locator("#roi-result").getByText(/^ROI: [\d,]+ points$/).waitFor();
      if (await page.locator("#layer-list option").count() !== 3 ||
          await page.locator("#layer-list").inputValue() !== activeOverlayKey ||
          await page.locator("#layer-pos-x").inputValue() !== "7") {
        throw new Error("primary reload lost review layer, active selection, or transform");
      }
      await page.locator("#layer-list").selectOption({ index: 1 });
      await page.locator("#layer-visible").uncheck();
      if (!/^○ /u.test(await page.locator("#layer-list option").nth(1).textContent() ?? "")) {
        throw new Error("layer visibility state did not reach renderer");
      }
      await page.locator("#remove-layer").click();
      await page.locator("#layer-list option").nth(0).waitFor();
      if (await page.locator("#layer-list option").count() !== 2) {
        throw new Error("layer removal did not dispose selected layer");
      }
      await page.locator("#layer-list").selectOption({ index: 1 });
      await page.locator("#remove-layer").click();
      if (await page.locator("#layer-list option").count() !== 1) {
        throw new Error("second queued layer was not removable");
      }
      await page.locator("#layer-list").selectOption({ index: 0 });
      // Let the next WebGL frame consume the post-removal active-layer state
      // before later screenshot/color-map checks inspect the canvas.
      await page.evaluate(() => new Promise((resolve) =>
        requestAnimationFrame(() => requestAnimationFrame(() => resolve()))));
      for (const [id, value] of Object.entries({
        "roi-min-x": "-80", "roi-max-x": "80",
        "roi-min-y": "-40", "roi-max-y": "65",
        "roi-min-z": "-3", "roi-max-z": "3",
      })) {
        await page.locator(`#${id}`).fill(value);
      }
      await page.locator("#apply-roi").click();
      await page.locator("#roi-result").getByText("ROI: filtering…").waitFor();
      if (!await page.locator("#export-roi").isDisabled()) {
        throw new Error("ROI export stayed enabled during cooperative filtering");
      }
      await page.locator("#roi-result").getByText(/^ROI: [\d,]+ points$/).waitFor();
      if (await page.locator("#export-roi").isDisabled()) {
        throw new Error("ROI export stayed disabled after complete filtering");
      }
      const expectedBounds = {
        "aabb-min": "Min: (-78.9542, -36.9768, -2.43231)",
        "aabb-max": "Max: (77.2023, 63.6491, 2.94972)",
        "aabb-size": "Size: (156.156, 100.626, 5.38203)",
      };
      for (const [field, expected] of Object.entries(expectedBounds)) {
        const value = await page.locator(`#${field}`).textContent();
        if (value !== expected) {
          throw new Error(`${field} mismatch: ${value} != ${expected}`);
        }
      }
      const gridLabel = await page.locator("#grid-spacing").textContent();
      if (!gridLabel ||
          !/^Grid: [0-9.e+-]+ units \/ division$/.test(gridLabel)) {
        throw new Error(`invalid adaptive grid spacing: ${gridLabel}`);
      }
      if (await page.locator("#show-axes").isChecked() ||
          await page.locator("#show-grid").isChecked()) {
        throw new Error("reference overlays must be disabled by default");
      }
      const details = page.locator("#information");
      const detailsToggle = page.locator("#details-toggle");
      await detailsToggle.click();
      if (await details.isVisible() ||
          await detailsToggle.getAttribute("aria-expanded") !== "false") {
        throw new Error("details toggle did not collapse point-cloud details");
      }
      await detailsToggle.click();
      if (!await details.isVisible() ||
          await detailsToggle.getAttribute("aria-expanded") !== "true") {
        throw new Error("details toggle did not restore point-cloud details");
      }
      const canvas = page.locator("canvas");
      const cloudOnly = await canvas.screenshot();
      await page.locator("#color-mode").selectOption("intensity");
      await page.locator("#color-map").selectOption("viridis");
      await waitForPaint(page);
      const viridis = await canvas.screenshot();
      if (cloudOnly.equals(viridis)) {
        throw new Error("intensity colormap did not change rendered cloud");
      }
      await page.locator("#equalize-intensity").uncheck();
      await waitForPaint(page);
      const linearIntensity = await canvas.screenshot();
      if (viridis.equals(linearIntensity)) {
        throw new Error("intensity equalization did not change rendered cloud");
      }
      await page.locator("#player-options > summary").click();
      for (const id of ["previous-frame", "next-frame", "reverse-play", "reset-playback"]) {
        await page.locator(`#${id}`).click();
      }
      await page.locator("#loop-playback").check();
      await page.locator("#show-grid").check();
      await waitForPaint(page);
      const withGrid = await canvas.screenshot();
      if (cloudOnly.equals(withGrid)) {
        throw new Error("scale grid toggle did not change rendered canvas");
      }
      for (const view of ["top", "front", "right"]) {
        await page.locator("#show-grid").uncheck();
        await page.locator(`[data-view="${view}"]`).click();
        await waitForPaint(page);
        const withoutPlane = await canvas.screenshot();
        await page.locator("#show-grid").check();
        await waitForPaint(page);
        const withPlane = await canvas.screenshot();
        if (withoutPlane.equals(withPlane)) {
          throw new Error(`${view} view cannot see its orthogonal grid plane`);
        }
      }
      await page.locator("#show-axes").check();
      await waitForPaint(page);
      const withAxes = await canvas.screenshot();
      if (withGrid.equals(withAxes)) {
        throw new Error("axes toggle did not change rendered canvas");
      }
      if (await page.locator("#viewer").getAttribute("data-axes-visible") !==
            "true" ||
          await page.locator("#viewer").getAttribute("data-grid-visible") !==
            "true") {
        throw new Error("viewer did not retain enabled reference overlays");
      }
      if (await page.locator("#viewer").getAttribute("data-grid-planes") !==
            "xy,xz,yz" ||
          await page.locator("#viewer").getAttribute("data-axes-arrowheads") !==
            "true" ||
          await page.locator("#viewer").getAttribute("data-axes-palette") !==
            "pastel") {
        throw new Error("reference overlay geometry or palette is invalid");
      }
      await page.locator("#background").evaluate((input) => {
        const element = input;
        element.value = "#aaaaaa";
        element.dispatchEvent(new Event("input", { bubbles: true }));
      });
      await waitForPaint(page);
      const gridContrast = Number(
        await page.locator("#viewer").getAttribute("data-grid-contrast"),
      );
      if (await page.locator("#viewer").getAttribute("data-axes-palette") !==
            "pastel" ||
          !Number.isFinite(gridContrast) || gridContrast < 1.5) {
        throw new Error("reference overlays lost contrast on light background");
      }
      await page.locator("#controls-help summary").click();
      const helpBefore = await page.locator("#controls-help").boundingBox();
      await page.mouse.move(80, 80);
      await page.mouse.move(700, 420);
      const helpAfter = await page.locator("#controls-help").boundingBox();
      if (!helpBefore || !helpAfter ||
          helpBefore.x !== helpAfter.x || helpBefore.y !== helpAfter.y) {
        throw new Error("mouse controls help followed the pointer");
      }
      await page.locator("#color-mode").selectOption("intensity");
      await page.locator("#point-size").evaluate((input) => {
        const element = input;
        element.value = "3";
        element.dispatchEvent(new Event("input", { bubbles: true }));
      });
      await page.locator("#background").evaluate((input) => {
        const element = input;
        element.value = "#202830";
        element.dispatchEvent(new Event("input", { bubbles: true }));
      });
      for (const view of ["top", "front", "left", "right", "iso", "fit"]) {
        await page.locator(`[data-view="${view}"]`).click();
      }
      await page.evaluate(() => window.loadNoiseFixture());
      await page.locator("body[data-loads='3']").waitFor();
      await page.locator("#status[data-kind='ready']").waitFor();
      if (await page.locator("#noise-info").textContent() !==
            "Noise: 50,000 / 100,001") {
        throw new Error("noise count is missing or incorrect");
      }
      await page.locator("#show-axes").uncheck();
      await page.locator("#show-grid").uncheck();
      await page.locator("#color-mode").selectOption("fixed");
      await page.evaluate(() => window.loadNoiseFixture());
      await page.locator("body[data-loads='4']").waitFor();
      await page.locator("#status[data-kind='ready']").waitFor();
      if (await page.locator("#color-mode").inputValue() !== "fixed") {
        throw new Error("frame load reset the selected fixed color mode");
      }
      await page.locator("[data-view='top']").click();
      await waitForPaint(page);
      const highlightedNoise = await canvas.screenshot();
      await page.locator("#highlight-noise").uncheck();
      await waitForPaint(page);
      const baseOnly = await canvas.screenshot();
      if (highlightedNoise.equals(baseOnly)) {
        throw new Error("noise highlight did not override fixed base color");
      }
      const canvasSize = await page.locator("canvas").evaluate((canvas) => ({
        width: canvas.width,
        height: canvas.height,
      }));
      if (canvasSize.width === 0 || canvasSize.height === 0) {
        throw new Error("Three.js canvas has zero size");
      }
      await page.waitForFunction(() => {
        const body = document.body;
        const current = body.dataset.animationFrames ?? "0";
        const now = performance.now();
        if (body.dataset.idleFrameProbe !== current) {
          body.dataset.idleFrameProbe = current;
          body.dataset.idleFrameSince = String(now);
          return false;
        }
        return now - Number(body.dataset.idleFrameSince ?? now) >= 500;
      }, undefined, { polling: 50, timeout: 5_000 });
      if (browserErrors.length > 0) {
        throw new Error(browserErrors.join("\n"));
      }
    } else if (path.endsWith("sequence-smoke.html")) {
      await page.locator("#show-axes").check();
      await page.locator("#show-grid").check();
      await page.locator("#frame").evaluate((input) => {
        input.value = "2";
        input.dispatchEvent(new Event("input", { bubbles: true }));
      });
      await page.locator("body[data-requested*='2']").waitFor();
      await page.locator("#frame-label").filter({ hasText: "3 / 4" }).waitFor();
      await page.waitForFunction(
        () => Number(document.body.dataset.renders ?? "0") >= 2,
      );
      if (!await page.locator("#show-axes").isChecked() ||
          !await page.locator("#show-grid").isChecked()) {
        throw new Error("reference overlay state changed between frames");
      }
      await page.locator("#play").click();
      await page.waitForFunction(() => {
        const requested = (document.body.dataset.requested ?? "")
          .split(",").map(Number);
        return [0, 1, 2, 3].every((index) => requested.includes(index));
      });
      await page.locator("#play").click();
      const requested = (await page.locator("body").getAttribute("data-requested"))
        ?.split(",").map(Number) ?? [];
      if (!requested.includes(0) || !requested.includes(1) ||
          !requested.includes(2) || !requested.includes(3)) {
        throw new Error(`sequence did not prefetch neighboring frames: ${requested}`);
      }
      const wasmTransfers = await page.locator("body")
        .getAttribute("data-wasm-transfers");
      const workerCount = await page.locator("body")
        .getAttribute("data-worker-count");
      if (!workerCount || wasmTransfers !== workerCount) {
        throw new Error(
          `decoder WASM transferred ${wasmTransfers} times for ` +
          `${workerCount} workers`,
        );
      }
      if (browserErrors.length > 0) throw new Error(browserErrors.join("\n"));
    }
    process.stdout.write(`${path}: ${text?.trim()}\n`);
    await page.close();
  }

  const primaryRacePage = await browser.newPage();
  await primaryRacePage.goto(
    `${baseUrl}/vscode/tests/worker-smoke.html?primaryLayerRace=1`,
  );
  await primaryRacePage.locator("body[data-state='passed']").waitFor({ timeout: 30_000 });
  await primaryRacePage.locator("#layer-list option").nth(1).waitFor();
  if (await primaryRacePage.locator("#layer-list option").count() !== 2) {
    throw new Error("fast addLayer was lost when primary cloud decoded later");
  }
  await primaryRacePage.close();

  for (const query of ["badPrimaryLayerRace=1", "hostErrorPrimaryLayerRace=1"]) {
    const failedPrimaryPage = await browser.newPage();
    await failedPrimaryPage.goto(
      `${baseUrl}/vscode/tests/worker-smoke.html?${query}`,
    );
    await failedPrimaryPage.locator("#layer-list option").waitFor({ timeout: 30_000 });
    await failedPrimaryPage.locator("#status[data-kind='ready']").waitFor({ timeout: 30_000 });
    if (await failedPrimaryPage.locator("#layer-list option").count() !== 1) {
      throw new Error(`queued Add did not recover after failed primary (${query})`);
    }
    await failedPrimaryPage.close();
  }
} finally {
  await browser.close();
}

async function waitForPaint(page) {
  await page.evaluate(() => new Promise((resolve) =>
    requestAnimationFrame(() => requestAnimationFrame(resolve))
  ));
}

function rectanglesOverlap(left, right) {
  return left.x < right.x + right.width &&
    left.x + left.width > right.x &&
    left.y < right.y + right.height &&
    left.y + left.height > right.y;
}
