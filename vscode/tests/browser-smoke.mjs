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
    !/document\.overlayCatalog\.replay\(/.test(extensionSource) ||
    !/layerQueue\?\.enqueue\(replay\)/.test(extensionSource) ||
    !/this\.inFlight !== undefined/.test(extensionSource)) {
  throw new Error("host must serialize Remote layer payloads until render acknowledgement");
}
if (!/showSaveDialog\(/.test(extensionSource) ||
    !/workspace\.fs\.writeFile\(target, new Uint8Array\(message\.bytes\)\)/.test(
      extensionSource,
    ) ||
    !/type: "exportPly"/.test(webviewSource) ||
    /downloadVisiblePly\(/.test(webviewSource)) {
  throw new Error("ROI PLY export must use a Remote-aware host filesystem save");
}
const screenshotSave = extensionSource.match(
  /const saveScreenshot[\s\S]*?\n    \};\n\n    const exportReviewShare/,
);
const reviewShareSave = extensionSource.match(
  /const exportReviewShare[\s\S]*?\n    \};\n\n    const importReviewShare/,
);
if (!/async function confirmRemoteOverwrite\(/.test(extensionSource) ||
    !screenshotSave || !reviewShareSave ||
    !/confirmRemoteOverwrite\(target\)/.test(screenshotSave[0]) ||
    !/confirmRemoteOverwrite\(target\)/.test(reviewShareSave[0])) {
  throw new Error("Remote screenshot/share saves must confirm an existing destination");
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
if (!/size <= 0/.test(viewerSource) ||
    !/min="0\.05" max="5" step="0\.05"/.test(extensionSource)) {
  throw new Error("Review Share point size must stay in (0,5]");
}
if (!/function rendererGpuBudget\(/.test(viewerSource) ||
    !/renderQuality: "full" \| "lod"/.test(viewerSource) ||
    !/minimumGpuLodPoints/.test(viewerSource) ||
    !/recoverGpuAllocationFailure/.test(viewerSource) ||
    !/context\.OUT_OF_MEMORY/.test(viewerSource)) {
  throw new Error("layer renderer lacks a bounded full/LOD GPU policy");
}
const gpuRecovery = viewerSource.match(
  /private recoverGpuAllocationFailure\(\): void \{([\s\S]*?)\n  \}/,
);
if (!gpuRecovery || /removeLayer\(layer\.sourceKey\)/.test(gpuRecovery[1]) ||
    !/retained \$\{name\} at minimum LOD/.test(gpuRecovery[1])) {
  throw new Error("GPU failure must retain existing review layers");
}
if (!/function isRenderableLayerTransform\(/.test(viewerSource) ||
    !/function isRenderableTransformMatrix\(/.test(viewerSource) ||
    !/maximumFloat32 \/ 8/.test(viewerSource)) {
  throw new Error("layer transforms must be validated for WebGL renderability");
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
if (!/label: 3/.test(viewerSource) || !/none: 4/.test(viewerSource) ||
    !/color_by: layer\.style\.reviewColorBy/.test(viewerSource) ||
    !/labelColorFallback/.test(webviewSource)) {
  throw new Error("Review Share ColorBy must retain native Label and None values");
}
if (!/primaryLoadBlocked/.test(extensionSource) ||
    !/reviewSessionActive/.test(webviewSource)) {
  throw new Error("Review Share import must gate stale primary loads");
}
if (!/id="display-toggle"[^>]*aria-controls="overlay-menu"/.test(extensionSource) ||
    !/id="details-toggle"[^>]*aria-controls="information"/.test(extensionSource) ||
    !/id="point-size"[^>]*min="0\.05"[^>]*max="5"[^>]*step="0\.05"/.test(
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
      await page.locator("#fit-visible").click();
      const framingSamples = Number(
        await page.locator("#viewer").getAttribute("data-framing-samples"),
      );
      if (!Number.isSafeInteger(framingSamples) || framingSamples < 1 ||
          framingSamples > 100_000) {
        throw new Error(`visible-fit exceeded global sample budget: ${framingSamples}`);
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
      // Finite JavaScript doubles above Float32/WebGL range must not poison a
      // retained layer matrix. Selecting away/back verifies no partial mutate.
      await page.locator("#layer-pos-x").fill("1e38");
      await page.locator("#apply-layer-transform").click();
      await page.locator("#layer-list").selectOption({ index: 0 });
      await page.locator("#layer-list").selectOption(activeOverlayKey);
      if (await page.locator("#layer-pos-x").inputValue() !== "7") {
        throw new Error("non-renderable layer transform mutated retained state");
      }
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
      // `highlightNoise` is a user intent, not a derived has-noise property.
      // Passing through a no-noise primary layer must not silently turn it off
      // before the next noisy frame arrives.
      await page.locator("#highlight-noise").check();
      await page.locator("#reload").click();
      await page.locator("body[data-loads='5']").waitFor();
      await page.locator("#status[data-kind='ready']").waitFor();
      await page.evaluate(() => window.loadNoiseFixture());
      await page.locator("body[data-loads='6']").waitFor();
      await page.locator("#status[data-kind='ready']").waitFor();
      if (!await page.locator("#highlight-noise").isChecked()) {
        throw new Error("no-noise layer erased the saved noise-highlight intent");
      }
      const canvasSize = await page.locator("canvas").evaluate((canvas) => ({
        width: canvas.width,
        height: canvas.height,
      }));
      if (canvasSize.width === 0 || canvasSize.height === 0) {
        throw new Error("Three.js canvas has zero size");
      }
      // A saved bookmark owns its orientation. Moving the live camera before
      // export must not substitute a basis that was never bookmarked.
      await page.evaluate(() => { window.prompt = () => "Saved top"; });
      await page.locator("[data-view='top']").click();
      await page.locator("#bookmark-save").click();
      await page.evaluate(() => { window.prompt = () => "Local only"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const beforeLiveCameraMove = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), beforeLiveCameraMove.requestId);
      await page.locator("[data-view='front']").click();
      await page.locator("#export-review-share").click();
      const afterLiveCameraMove = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const bookmarkBefore = beforeLiveCameraMove?.document.bookmarks
        .find((bookmark) => bookmark.name === "Saved top");
      const bookmarkAfter = afterLiveCameraMove?.document.bookmarks
        .find((bookmark) => bookmark.name === "Saved top");
      if (!bookmarkBefore || !bookmarkAfter ||
          JSON.stringify(bookmarkBefore.camera.camera_to_world) !==
          JSON.stringify(bookmarkAfter.camera.camera_to_world)) {
        throw new Error("Review Share bookmark exported the live camera basis");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), afterLiveCameraMove.requestId);
      // A share may remain wholly unresolved on this Remote host. Its
      // world-space ROI and every saved measurement still need a lossless
      // export, while host-path logical keys must be rejected at the webview
      // boundary rather than reaching renderer state.
      await page.evaluate(() => {
        const keyA = `sha256:${"a".repeat(64)}`;
        const keyB = `sha256:${"b".repeat(64)}`;
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 77, sessionGeneration: 1, document: {
            schema_version: 2,
            layers: [],
            roi: { minimum: [-1, -2, -3], maximum: [4, 5, 6] },
            measurements: [
              { first_source_key: keyA, first_world: [1, 2, 3], second_source_key: null, second_world: null },
              { first_source_key: keyB, first_world: [4, 5, 6], second_source_key: null, second_world: null },
            ],
            bookmarks: [
              { name: "Saved top", camera: {
                target: [1, 2, 3], rotation_center: [1, 2, 3],
                camera_to_world: [[0,0,1],[0,1,0],[-1,0,0]],
                distance: 8, fov_y_degrees: 50,
              }},
              { name: "Imported only", camera: {
                target: [0, 0, 0], rotation_center: [7, -8, 9],
                camera_to_world: [[1,0,0],[0,1,0],[0,0,1]],
                distance: 5, fov_y_degrees: 45,
              }},
            ],
          },
        }}));
      });
      // A document payload already in VS Code's message queue must not revive
      // the old primary cloud after a Review Share took ownership of the view.
      await page.evaluate(async () => {
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "load", requestId: 99, name: "stale-primary.pcd",
          sourceKey: `sha256:${"d".repeat(64)}`, bytes,
        }}));
      });
      await page.waitForTimeout(300);
      if (await page.locator("#layer-list option").count() !== 0) {
        throw new Error("delayed primary load replaced imported Review Share");
      }
      await page.locator("#export-review-share").click();
      const exportedShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      if (!exportedShare || exportedShare.document.roi?.minimum.join(",") !== "-1,-2,-3" ||
          exportedShare.document.measurements.length !== 2) {
        throw new Error("unresolved Review Share lost ROI or measurements on re-export");
      }
      const exportedBookmarkNames = exportedShare.document.bookmarks
        .map((bookmark) => bookmark.name);
      if (exportedBookmarkNames.filter((name) => name === "Saved top").length !== 1 ||
          !exportedBookmarkNames.includes("Local only") ||
          !exportedBookmarkNames.includes("Imported only")) {
        throw new Error("Review Share import did not merge bookmarks by name");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), exportedShare.requestId);
      await page.locator("#bookmark-list").selectOption({ label: "Imported only" });
      await page.locator("#bookmark-restore").click();
      await page.locator("#export-review-share").click();
      const restoredPivotShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const restoredPivot = restoredPivotShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Imported only")?.camera;
      if (!restoredPivot || JSON.stringify(restoredPivot.target) !== "[0,0,0]" ||
          JSON.stringify(restoredPivot.rotation_center) !== "[7,-8,9]") {
        throw new Error("Review Share bookmark lost its distinct rotation center");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), restoredPivotShare.requestId);
      // Native CameraSnapshot has an orbit pivot separate from its visual
      // target. The first real OrbitControls left-drag must rotate that target
      // around the imported pivot instead of folding it into controls.target.
      const pivotCanvas = await page.locator("canvas").boundingBox();
      if (!pivotCanvas) throw new Error("pivot orbit canvas is unavailable");
      const pivotStart = {
        x: pivotCanvas.x + pivotCanvas.width * 0.72,
        y: pivotCanvas.y + pivotCanvas.height * 0.72,
      };
      await page.mouse.move(pivotStart.x, pivotStart.y);
      await page.mouse.down();
      await page.mouse.move(pivotStart.x - 96, pivotStart.y + 48, { steps: 8 });
      await page.mouse.up();
      await page.waitForTimeout(300);
      await page.evaluate(() => { window.prompt = () => "Orbit pivot"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const orbitPivotShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const orbitPivot = orbitPivotShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Orbit pivot")?.camera;
      const expectedPivotDistance = Math.hypot(7, -8, 9);
      const actualPivotDistance = orbitPivot && Math.hypot(
        orbitPivot.target[0] - orbitPivot.rotation_center[0],
        orbitPivot.target[1] - orbitPivot.rotation_center[1],
        orbitPivot.target[2] - orbitPivot.rotation_center[2],
      );
      if (!orbitPivot || JSON.stringify(orbitPivot.rotation_center) !== "[7,-8,9]" ||
          Math.abs(actualPivotDistance - expectedPivotDistance) > 1e-3 ||
          Math.abs(orbitPivot.distance - 5) > 1e-3 ||
          JSON.stringify(orbitPivot.target) === "[0,0,0]") {
        throw new Error("OrbitControls collapsed imported rotation_center into target");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), orbitPivotShare.requestId);
      // Pan and dolly are also controls changes. They must keep the native
      // pivot fixed while respectively translating target and changing only
      // target-relative camera distance.
      await page.mouse.move(pivotStart.x, pivotStart.y);
      await page.mouse.down({ button: "right" });
      await page.mouse.move(pivotStart.x + 64, pivotStart.y - 32, { steps: 6 });
      await page.mouse.up({ button: "right" });
      await page.waitForTimeout(250);
      await page.evaluate(() => { window.prompt = () => "Pan pivot"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const panPivotShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const panPivot = panPivotShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Pan pivot")?.camera;
      if (!panPivot || JSON.stringify(panPivot.rotation_center) !== "[7,-8,9]" ||
          JSON.stringify(panPivot.target) === JSON.stringify(orbitPivot.target)) {
        throw new Error("OrbitControls pan moved native pivot or did not translate target");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), panPivotShare.requestId);
      await page.mouse.wheel(0, 240);
      await page.waitForTimeout(250);
      await page.evaluate(() => { window.prompt = () => "Zoom pivot"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const zoomPivotShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const zoomPivot = zoomPivotShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Zoom pivot")?.camera;
      if (!zoomPivot || JSON.stringify(zoomPivot.rotation_center) !== "[7,-8,9]" ||
          JSON.stringify(zoomPivot.target) !== JSON.stringify(panPivot.target) ||
          Math.abs(zoomPivot.distance - panPivot.distance) < 1e-6) {
        throw new Error("OrbitControls dolly changed native pivot or target");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), zoomPivotShare.requestId);
      await page.locator("#clear-measurement").click();
      if (!/2 imported measurement\(s\) preserved read-only/.test(
        await page.locator("#measurement-result").textContent() ?? "",
      )) {
        throw new Error("imported measurements were not explicitly read-only");
      }
      await page.locator("#export-review-share").click();
      const afterMeasurementClear = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      if (!afterMeasurementClear || afterMeasurementClear.document.measurements.length !== 2) {
        throw new Error("clearing a transient measurement shrank imported measurements");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), afterMeasurementClear.requestId);
      await page.evaluate(() => window.dispatchEvent(new MessageEvent("message", { data: {
        type: "reviewShareLoaded", requestId: 78, sessionGeneration: 2, document: {
          schema_version: 2,
          layers: [{
            source_key: "path:/remote/private.pcd", runtime_id: "review-9-1", name: "private.pcd",
            local_to_world: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
            style: { color_by: 0, color_map: 0, point_size: 1, opacity: 1,
              scalar_min: 0, scalar_max: 1, fixed_color: [1,1,1], noise_color: [1,0,0],
              highlight_noise: false, intensity_equalize: false }, visible: true,
          }], roi: null, measurements: [], bookmarks: [],
        },
      }})));
      await page.locator("#export-review-share").click();
      const afterPathInjection = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      if (!afterPathInjection || afterPathInjection.document.layers.length !== 0) {
        throw new Error("webview accepted an unsanitized host-path review state");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), afterPathInjection.requestId);
      // Label has no WebGL point attribute but must retain native enum 3 on
      // re-export; None=4 has the same Fixed visual treatment without losing
      // its distinct portable meaning. Imported scalar bounds are semantic.
      await page.evaluate(async () => {
        const labelKey = `sha256:${"e".repeat(64)}`;
        const noneKey = `sha256:${"f".repeat(64)}`;
        const style = (colorBy, scalarMin, scalarMax) => ({
          color_by: colorBy, color_map: 4, point_size: 1, opacity: 1,
          scalar_min: scalarMin, scalar_max: scalarMax,
          fixed_color: [0.25, 0.5, 0.75], noise_color: [1, 0, 0],
          highlight_noise: true, intensity_equalize: false,
        });
        const labelLayer = {
          source_key: labelKey, runtime_id: "review-11-1", name: "label.pcd",
          local_to_world: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
          style: style(3, -12, 42), visible: true,
        };
        const noneLayer = {
          source_key: noneKey, runtime_id: "review-11-2", name: "none.pcd",
          local_to_world: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
          style: style(4, 8, 8), visible: true,
        };
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 80, sessionGeneration: 3, document: {
            schema_version: 2, layers: [labelLayer, noneLayer], roi: null,
            measurements: [], bookmarks: [],
          },
        }}));
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_080, sourceKey: labelKey,
          name: "label.pcd", bytes, sessionGeneration: 3, reviewLayer: labelLayer,
        }}));
      });
      await page.locator("#status[data-kind='ready']").waitFor();
      if (!/Label color is unavailable/.test(
        await page.locator("#color-mode-state").textContent() ?? "",
      )) {
        throw new Error("Label review color did not expose its Fixed fallback");
      }
      await page.locator("#export-review-share").click();
      const colorByExport = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const exportedStyles = colorByExport?.document.layers
        .map((layer) => layer.style).sort((left, right) =>
          left.color_by - right.color_by);
      if (!exportedStyles || JSON.stringify(exportedStyles.map((style) => [
        style.color_by, style.scalar_min, style.scalar_max,
      ])) !== JSON.stringify([[3, -12, 42], [4, 8, 8]])) {
        throw new Error("Review Share Label/None or scalar range did not round-trip");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), colorByExport.requestId);
      // An unresolved share layer later selected through Add arrives as a
      // normal host payload, not a replay message. Its source key must bind
      // to current imported state before defaults materialize in the viewer.
      const manualAffine = [[-1, 0.25, 0, 12], [0, 1, 0.5, -3],
        [0, 0, 1, 4], [0, 0, 0, 1]];
      await page.evaluate(async (localToWorld) => {
        const sourceKey = `sha256:${"9".repeat(64)}`;
        const reviewLayer = {
          source_key: sourceKey, runtime_id: "review-12-1", name: "later.pcd",
          local_to_world: localToWorld,
          style: { color_by: 2, color_map: 9, point_size: 4.5, opacity: 0.25,
            scalar_min: -7.5, scalar_max: 12.25,
            fixed_color: [0.2, 0.4, 0.6], noise_color: [0.8, 0.1, 0.3],
            highlight_noise: true, intensity_equalize: false }, visible: false,
        };
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 81, sessionGeneration: 4, document: {
            schema_version: 2, layers: [reviewLayer], roi: null,
            measurements: [], bookmarks: [],
          },
        }}));
        await new Promise((resolve) => requestAnimationFrame(resolve));
        const list = document.querySelector("#layer-list");
        list.value = reviewLayer.runtime_id;
        list.dispatchEvent(new Event("change", { bubbles: true }));
        document.querySelector("#locate-review-source").click();
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_081, sourceKey,
          name: "manually-added.pcd", bytes, sessionGeneration: 4, reviewLayer,
        }}));
      }, manualAffine);
      const locateRequest = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "locateReviewSource").at(-1),
      );
      if (!locateRequest || locateRequest.sourceKey !== `sha256:${"9".repeat(64)}` ||
          "uri" in locateRequest || "path" in locateRequest) {
        throw new Error("Locate source leaked a Remote path or lost its transport key");
      }
      await page.waitForFunction(() => [...document.querySelectorAll("#layer-list option")]
        .some((option) => !option.disabled && option.value === "review-12-1"));
      await page.locator("#export-review-share").click();
      const manuallyBoundShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const manuallyBound = manuallyBoundShare?.document.layers
        .find((layer) => layer.source_key === `sha256:${"9".repeat(64)}`);
      if (!manuallyBound || JSON.stringify(manuallyBound.local_to_world) !==
          JSON.stringify(manualAffine) || manuallyBound.visible !== false ||
          JSON.stringify(manuallyBound.style) !== JSON.stringify({
            color_by: 2, color_map: 9, point_size: 4.5, opacity: 0.25,
            scalar_min: -7.5, scalar_max: 12.25,
            fixed_color: [0.2, 0.4, 0.6], noise_color: [0.8, 0.1, 0.3],
            highlight_noise: true, intensity_equalize: false,
          })) {
        throw new Error("manual Add overwrote unresolved Review Share layer state");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), manuallyBoundShare.requestId);
      const affine = [[-1, 0.25, 0, 12], [0, 1, 0.5, -3],
        [0, 0, 1, 4], [0, 0, 0, 1]];
      await page.evaluate(async (localToWorld) => {
        const sourceKey = `sha256:${"c".repeat(64)}`;
        const reviewLayer = {
          source_key: sourceKey, runtime_id: "review-10-1", name: "native-affine.pcd",
          local_to_world: localToWorld,
          style: { color_by: 0, color_map: 0, point_size: 1, opacity: 1,
            scalar_min: 0, scalar_max: 1, fixed_color: [1,1,1], noise_color: [1,0,0],
            highlight_noise: false, intensity_equalize: false }, visible: true,
        };
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 79, sessionGeneration: 5, document: {
            schema_version: 2, layers: [reviewLayer], roi: null, measurements: [], bookmarks: [],
          },
        }}));
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_099, sourceKey,
          name: "native-affine.pcd", bytes, sessionGeneration: 5, reviewLayer,
        }}));
      }, affine);
      await page.locator("#layer-list option").waitFor();
      await page.locator("#status[data-kind=\"ready\"]").waitFor();
      if (!await page.locator("#apply-layer-transform").isDisabled() ||
          !await page.locator("#layer-pos-x").isDisabled() ||
          !/Exact affine matrix/.test(
            await page.locator("#layer-transform-state").textContent() ?? "")) {
        throw new Error("sheared native affine was not explicitly locked");
      }
      await page.locator("#export-review-share").click();
      const affineExport = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      if (!affineExport || JSON.stringify(affineExport.document.layers[0].local_to_world) !==
          JSON.stringify(affine)) {
        throw new Error("native shear/reflection affine did not round-trip exactly");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), affineExport.requestId);
      await page.locator("#save-screenshot").click();
      await page.waitForFunction(() => window.kptPostedMessages.some((message) =>
        message.type === "saveScreenshot"), undefined, { timeout: 10_000 });
      const screenshot = await page.evaluate(async () => {
        const message = window.kptPostedMessages.filter((candidate) =>
          candidate.type === "saveScreenshot").at(-1);
        if (!message || !(message.bytes instanceof ArrayBuffer)) return undefined;
        const bytes = new Uint8Array(message.bytes);
        const signature = [...bytes.subarray(0, 8)];
        const bitmap = await createImageBitmap(new Blob([message.bytes], { type: "image/png" }));
        const decoded = document.createElement("canvas");
        decoded.width = bitmap.width;
        decoded.height = bitmap.height;
        const context = decoded.getContext("2d");
        context?.drawImage(bitmap, 0, 0);
        const pixels = context?.getImageData(0, 0, decoded.width, decoded.height).data;
        bitmap.close();
        return {
          byteLength: bytes.byteLength,
          signature,
          dimensions: [decoded.width, decoded.height],
          nonempty: !!pixels && [...pixels].some((value) => value !== 0),
        };
      });
      if (!screenshot || screenshot.byteLength === 0 ||
          screenshot.signature.join(",") !== "137,80,78,71,13,10,26,10" ||
          screenshot.dimensions[0] === 0 || screenshot.dimensions[1] === 0 ||
          !screenshot.nonempty) {
        throw new Error("captured WebGL screenshot is empty or not PNG");
      }
      // A panel reload for session A can finish after the user imports session
      // B. Host request IDs only correlate UI work, so reject A by its
      // monotonic host session generation and retain B's semantic state.
      await page.evaluate(async () => {
        const staleKey = `sha256:${"7".repeat(64)}`;
        const staleLayer = {
          source_key: staleKey, runtime_id: "review-6-1", name: "late-a.pcd",
          local_to_world: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
          style: { color_by: 0, color_map: 0, point_size: 1, opacity: 1,
            scalar_min: 0, scalar_max: 1, fixed_color: [1,1,1], noise_color: [1,0,0],
            highlight_noise: false, intensity_equalize: false }, visible: true,
        };
        const reloadA = {
          schema_version: 2,
          layers: [staleLayer],
          roi: { minimum: [-71, -72, -73], maximum: [-61, -62, -63] },
          measurements: [],
          bookmarks: [],
        };
        const importB = {
          schema_version: 2,
          layers: [],
          roi: { minimum: [101, 102, 103], maximum: [104, 105, 106] },
          measurements: [],
          bookmarks: [],
        };
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 900, sessionGeneration: 7,
          document: importB,
        }}));
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 899, sessionGeneration: 6,
          document: reloadA,
        }}));
        // This mimics an A Remote read that completed only after B's imported
        // state reached the webview. It must not resurrect A's cloud.
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_906, sourceKey: staleKey,
          name: "late-a.pcd", bytes, sessionGeneration: 6, reviewLayer: staleLayer,
        }}));
      });
      await page.waitForTimeout(300);
      await page.locator("#export-review-share").click();
      const interleavedSessionShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      if (!interleavedSessionShare ||
          JSON.stringify(interleavedSessionShare.document.roi?.minimum) !==
          "[101,102,103]" || interleavedSessionShare.document.layers.length !== 0 ||
          await page.locator("#layer-list option").count() !== 0) {
        throw new Error("stale Review Share reload or layer payload overwrote newer import");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), interleavedSessionShare.requestId);
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
