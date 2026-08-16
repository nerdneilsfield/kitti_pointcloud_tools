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
    !/layerQueue\?\.settle\(message\.requestId(?:,|\))/.test(extensionSource) ||
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
      // This fixture exercises the full Review Share path (bookmarks, pivot,
      // unresolved Locate, import/export), not only decoder bootstrap. Keep
      // its required DOM contract explicit so a hidden/missing control cannot
      // make a long browser block silently unexecutable.
      const reviewFixtureControls = [
        "#layer-list", "#locate-review-source", "#remove-layer",
        "#layer-visible", "#layer-opacity", "#layer-size", "#layer-color",
        "#apply-layer-transform", "#bookmark-list", "#bookmark-save",
        "#bookmark-restore", "#bookmark-remove", "#export-review-share",
        "#import-review-share", "#measure-toggle", "#clear-measurement",
      ];
      for (const selector of reviewFixtureControls) {
        if (await page.locator(selector).count() !== 1) {
          throw new Error(`worker Review Share fixture is missing ${selector}`);
        }
      }
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
          type: "reviewShareLoaded", requestId: 77, sessionGeneration: 1, replayEpoch: 1, document: {
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
        // Inspection panel occupies the canvas's right edge throughout this
        // Review Share exercise. Start over exposed WebGL canvas, otherwise
        // this would only drag DOM controls and never exercise OrbitControls.
        x: pivotCanvas.x + pivotCanvas.width * 0.3,
        y: pivotCanvas.y + pivotCanvas.height * 0.72,
      };
      if (await page.evaluate(({ x, y }) =>
        document.elementFromPoint(x, y)?.tagName, pivotStart) !== "CANVAS") {
        throw new Error("pivot drag fixture is obscured before OrbitControls test");
      }
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
      await page.mouse.move(pivotStart.x, pivotStart.y);
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
          !zoomPivot.target.every((coordinate, index) =>
            Math.abs(coordinate - panPivot.target[index]) < 1e-9) ||
          Math.abs(zoomPivot.distance - panPivot.distance) < 1e-6) {
        throw new Error("OrbitControls dolly changed native pivot or target");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), zoomPivotShare.requestId);
      // Shift-left drag is native roll, not an OrbitControls orbit. With a
      // separate CameraSnapshot pivot it rotates both eye and visual target
      // around that pivot while preserving target-relative camera distance.
      const beforeRollTargetRadius = Math.hypot(
        zoomPivot.target[0] - zoomPivot.rotation_center[0],
        zoomPivot.target[1] - zoomPivot.rotation_center[1],
        zoomPivot.target[2] - zoomPivot.rotation_center[2],
      );
      const beforeRollEye = zoomPivot.target.map((coordinate, index) =>
        coordinate + zoomPivot.camera_to_world[index][2] * zoomPivot.distance,
      );
      const beforeRollEyeRadius = Math.hypot(
        beforeRollEye[0] - zoomPivot.rotation_center[0],
        beforeRollEye[1] - zoomPivot.rotation_center[1],
        beforeRollEye[2] - zoomPivot.rotation_center[2],
      );
      await page.mouse.move(pivotStart.x, pivotStart.y);
      await page.keyboard.down("Shift");
      await page.mouse.down();
      await page.mouse.move(pivotStart.x + 72, pivotStart.y, { steps: 6 });
      await page.mouse.up();
      await page.keyboard.up("Shift");
      await page.waitForTimeout(250);
      await page.evaluate(() => { window.prompt = () => "Roll pivot"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const rollPivotShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const rollPivot = rollPivotShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Roll pivot")?.camera;
      const rollEye = rollPivot?.target.map((coordinate, index) =>
        coordinate + rollPivot.camera_to_world[index][2] * rollPivot.distance,
      );
      const rollTargetRadius = rollPivot && Math.hypot(
        rollPivot.target[0] - rollPivot.rotation_center[0],
        rollPivot.target[1] - rollPivot.rotation_center[1],
        rollPivot.target[2] - rollPivot.rotation_center[2],
      );
      const rollEyeRadius = rollEye && rollPivot && Math.hypot(
        rollEye[0] - rollPivot.rotation_center[0],
        rollEye[1] - rollPivot.rotation_center[1],
        rollEye[2] - rollPivot.rotation_center[2],
      );
      if (!rollPivot || !rollEye ||
          JSON.stringify(rollPivot.rotation_center) !== "[7,-8,9]" ||
          JSON.stringify(rollPivot.target) === JSON.stringify(zoomPivot.target) ||
          Math.abs(rollPivot.distance - zoomPivot.distance) > 1e-3 ||
          Math.abs(rollTargetRadius - beforeRollTargetRadius) > 1e-3 ||
          Math.abs(rollEyeRadius - beforeRollEyeRadius) > 1e-3) {
        throw new Error("Shift-drag roll did not rotate camera and target around native pivot");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), rollPivotShare.requestId);
      // Conventional bookmarks begin with C == T. Native pan must freeze that
      // old C before later OrbitControls/roll gestures; otherwise the first
      // right drag silently turns C into the moving visual target.
      await page.locator("#bookmark-list").selectOption({ label: "Saved top" });
      await page.locator("#bookmark-restore").click();
      // Establish a one-step pan baseline. The following multi-step drag must
      // travel much farther; recreating controls in its first `change` event
      // would discard the rest of that pointer sequence.
      await page.mouse.move(pivotStart.x, pivotStart.y);
      await page.mouse.down({ button: "right" });
      await page.mouse.move(pivotStart.x + 14, pivotStart.y + 7);
      await page.mouse.up({ button: "right" });
      await page.waitForTimeout(80);
      await page.evaluate(() => { window.prompt = () => "Unified pan single"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#bookmark-list").selectOption({ label: "Saved top" });
      await page.locator("#bookmark-restore").click();
      await page.mouse.move(pivotStart.x, pivotStart.y);
      await page.mouse.down({ button: "right" });
      await page.mouse.move(pivotStart.x + 70, pivotStart.y + 36, { steps: 7 });
      await page.mouse.up({ button: "right" });
      await page.evaluate(() => { window.prompt = () => "Unified pan immediate"; });
      await page.locator("#bookmark-save").click();
      // The one-way C==T -> independent transition flushes damping first and
      // rebases only after pointerup. Wait for any residual target drift.
      await page.waitForTimeout(350);
      await page.evaluate(() => { window.prompt = () => "Unified pan pivot"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const unifiedPanShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const unifiedPan = unifiedPanShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Unified pan pivot")?.camera;
      const unifiedPanSingle = unifiedPanShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Unified pan single")?.camera;
      const unifiedPanImmediate = unifiedPanShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Unified pan immediate")?.camera;
      const unifiedPanSingleRadius = unifiedPanSingle && Math.hypot(
        unifiedPanSingle.target[0] - unifiedPanSingle.rotation_center[0],
        unifiedPanSingle.target[1] - unifiedPanSingle.rotation_center[1],
        unifiedPanSingle.target[2] - unifiedPanSingle.rotation_center[2],
      );
      if (!unifiedPan || JSON.stringify(unifiedPan.rotation_center) !== "[1,2,3]" ||
          JSON.stringify(unifiedPan.target) === "[1,2,3]" || !unifiedPanImmediate ||
          !unifiedPan.target.every((coordinate, index) =>
            Math.abs(coordinate - unifiedPanImmediate.target[index]) < 1e-8) ||
          !unifiedPanSingleRadius ||
          Math.hypot(
            unifiedPan.target[0] - unifiedPan.rotation_center[0],
            unifiedPan.target[1] - unifiedPan.rotation_center[1],
            unifiedPan.target[2] - unifiedPan.rotation_center[2],
          ) <= unifiedPanSingleRadius * 2) {
        throw new Error("first conventional pan moved native rotation_center");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), unifiedPanShare.requestId);
      const unifiedPanRadius = Math.hypot(
        unifiedPan.target[0] - unifiedPan.rotation_center[0],
        unifiedPan.target[1] - unifiedPan.rotation_center[1],
        unifiedPan.target[2] - unifiedPan.rotation_center[2],
      );
      await page.mouse.move(pivotStart.x, pivotStart.y);
      await page.mouse.down();
      await page.mouse.move(pivotStart.x - 84, pivotStart.y + 42, { steps: 7 });
      await page.mouse.up();
      await page.waitForTimeout(180);
      await page.evaluate(() => { window.prompt = () => "Unified pan orbit"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const unifiedOrbitShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const unifiedOrbit = unifiedOrbitShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Unified pan orbit")?.camera;
      const unifiedOrbitRadius = unifiedOrbit && Math.hypot(
        unifiedOrbit.target[0] - unifiedOrbit.rotation_center[0],
        unifiedOrbit.target[1] - unifiedOrbit.rotation_center[1],
        unifiedOrbit.target[2] - unifiedOrbit.rotation_center[2],
      );
      if (!unifiedOrbit || JSON.stringify(unifiedOrbit.rotation_center) !== "[1,2,3]" ||
          JSON.stringify(unifiedOrbit.target) === JSON.stringify(unifiedPan.target) ||
          Math.abs(unifiedOrbitRadius - unifiedPanRadius) > 1e-3) {
        throw new Error("orbit after conventional pan did not preserve native pivot");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), unifiedOrbitShare.requestId);
      await page.mouse.move(pivotStart.x, pivotStart.y);
      await page.keyboard.down("Shift");
      await page.mouse.down();
      await page.mouse.move(pivotStart.x + 66, pivotStart.y, { steps: 6 });
      await page.mouse.up();
      await page.keyboard.up("Shift");
      await page.waitForTimeout(180);
      await page.evaluate(() => { window.prompt = () => "Unified pan roll"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const unifiedRollShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const unifiedRoll = unifiedRollShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Unified pan roll")?.camera;
      const unifiedRollRadius = unifiedRoll && Math.hypot(
        unifiedRoll.target[0] - unifiedRoll.rotation_center[0],
        unifiedRoll.target[1] - unifiedRoll.rotation_center[1],
        unifiedRoll.target[2] - unifiedRoll.rotation_center[2],
      );
      if (!unifiedRoll || JSON.stringify(unifiedRoll.rotation_center) !== "[1,2,3]" ||
          JSON.stringify(unifiedRoll.target) === JSON.stringify(unifiedOrbit.target) ||
          Math.abs(unifiedRollRadius - unifiedOrbitRadius) > 1e-3) {
        throw new Error("roll after conventional pan did not preserve native pivot");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), unifiedRollShare.requestId);
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
        type: "reviewShareLoaded", requestId: 78, sessionGeneration: 2, replayEpoch: 1, document: {
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
          type: "reviewShareLoaded", requestId: 80, sessionGeneration: 3, replayEpoch: 1, document: {
            schema_version: 2, layers: [labelLayer, noneLayer], roi: null,
            measurements: [], bookmarks: [],
          },
        }}));
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_080, sourceKey: labelKey,
          name: "label.pcd", bytes, sessionGeneration: 3, replayEpoch: 1, reviewLayer: labelLayer,
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
          type: "reviewShareLoaded", requestId: 81, sessionGeneration: 4, replayEpoch: 1, document: {
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
          name: "manually-added.pcd", bytes, sessionGeneration: 4, replayEpoch: 1, reviewLayer,
        }}));
      }, manualAffine);
      const locateRequest = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "locateReviewSource").at(-1),
      );
      if (!locateRequest || locateRequest.sourceKey !== `sha256:${"9".repeat(64)}` ||
          locateRequest.sessionGeneration !== 4 || locateRequest.replayEpoch !== 1 ||
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
      // Rendering returns the effective semantic snapshot to the host. Later
      // inspector edits send an update under the same session generation, so
      // host catalog replay/export does not fall back to URI-derived defaults.
      const renderedManualState = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "rendered" && message.requestId === 1_000_000_081,
        ).at(-1),
      );
      if (!renderedManualState?.reviewLayer ||
          renderedManualState.sessionGeneration !== 4 || renderedManualState.replayEpoch !== 1 ||
          JSON.stringify(renderedManualState.reviewLayer.local_to_world) !==
            JSON.stringify(manualAffine) ||
          renderedManualState.reviewLayer.style.scalar_min !== -7.5 ||
          renderedManualState.reviewLayer.visible !== false) {
        throw new Error("review layer render acknowledgement lost semantic state");
      }
      await page.locator("#layer-list").selectOption("review-12-1");
      await page.locator("#layer-opacity").evaluate((input) => {
        input.value = "0.4";
        input.dispatchEvent(new Event("input", { bubbles: true }));
      });
      const manualStateUpdate = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "reviewLayerState" &&
          message.layer?.source_key === `sha256:${"9".repeat(64)}`,
        ).at(-1),
      );
      if (!manualStateUpdate || manualStateUpdate.sessionGeneration !== 4 ||
          manualStateUpdate.replayEpoch !== 1 ||
          manualStateUpdate.layer.runtime_id !== "review-12-1" ||
          manualStateUpdate.layer.style.opacity !== 0.4 ||
          JSON.stringify(manualStateUpdate.layer.local_to_world) !==
            JSON.stringify(manualAffine)) {
        throw new Error("review layer semantic edit was not acknowledged to host");
      }
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
          type: "reviewShareLoaded", requestId: 79, sessionGeneration: 5, replayEpoch: 1, document: {
            schema_version: 2, layers: [reviewLayer], roi: null, measurements: [], bookmarks: [],
          },
        }}));
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_099, sourceKey,
          name: "native-affine.pcd", bytes, sessionGeneration: 5, replayEpoch: 1, reviewLayer,
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
          JSON.stringify(affine) || affineExport.sessionGeneration !== 5 ||
          affineExport.replayEpoch !== 1) {
        throw new Error("native shear/reflection affine did not round-trip exactly");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), affineExport.requestId);
      // A Fit/default view rebuilds OrbitControls after an imported native
      // pivot. Its programmatic target update must not be mistaken for a pan
      // against the bookmark's old C/T baseline.
      await page.locator("[data-view='top']").click();
      await waitForPaint(page);
      await page.evaluate(() => { window.prompt = () => "Top fit baseline"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#bookmark-list").selectOption({ label: "Imported only" });
      await page.locator("#bookmark-restore").click();
      await page.locator("[data-view='top']").click();
      await waitForPaint(page);
      await page.evaluate(() => { window.prompt = () => "Top fit after pivot"; });
      await page.locator("#bookmark-save").click();
      await page.locator("#export-review-share").click();
      const topFitShare = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const topFitBaseline = topFitShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Top fit baseline")?.camera;
      const topFitAfterPivot = topFitShare?.document.bookmarks
        .find((bookmark) => bookmark.name === "Top fit after pivot")?.camera;
      if (!topFitBaseline || !topFitAfterPivot ||
          !topFitAfterPivot.target.every((value, index) =>
            Math.abs(value - topFitAfterPivot.rotation_center[index]) < 1e-8) ||
          !topFitAfterPivot.target.every((value, index) =>
            Math.abs(value - topFitBaseline.target[index]) < 1e-8) ||
          Math.abs(topFitAfterPivot.distance - topFitBaseline.distance) > 1e-6 ||
          Math.abs(topFitAfterPivot.fov_y_degrees - topFitBaseline.fov_y_degrees) > 1e-6) {
        throw new Error("Fit/default view inherited stale native pivot controls state");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), topFitShare.requestId);
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
      const screenshotRequest = await page.evaluate(() =>
        window.kptPostedMessages.filter((candidate) =>
          candidate.type === "saveScreenshot").at(-1),
      );
      if (screenshotRequest?.sessionGeneration !== 5 ||
          screenshotRequest.replayEpoch !== 1) {
        throw new Error("Review screenshot did not carry session identity");
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
          type: "reviewShareLoaded", requestId: 900, sessionGeneration: 7, replayEpoch: 1,
          document: importB,
        }}));
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 899, sessionGeneration: 6, replayEpoch: 1,
          document: reloadA,
        }}));
        // This mimics an A Remote read that completed only after B's imported
        // state reached the webview. It must not resurrect A's cloud.
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_906, sourceKey: staleKey,
          name: "late-a.pcd", bytes, sessionGeneration: 6, replayEpoch: 1, reviewLayer: staleLayer,
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
      // Review edits live in the webview until the host acknowledges a full
      // semantic snapshot. Replay the exact acknowledged state with a newer
      // epoch: manual layer style, ROI, measurement, and bookmark must all
      // survive; a late epoch-1 payload cannot repopulate the new scene.
      await page.evaluate(async () => {
        const sourceKey = `sha256:${"8".repeat(64)}`;
        const layer = {
          source_key: sourceKey, runtime_id: "review-8-1", name: "evidence.pcd",
          local_to_world: [[1,0,0,3],[0,1,0,-2],[0,0,1,1],[0,0,0,1]],
          style: { color_by: 2, color_map: 1, point_size: 2, opacity: 1,
            scalar_min: -4, scalar_max: 9, fixed_color: [0.2,0.3,0.4],
            noise_color: [1,0,0], highlight_noise: true,
            intensity_equalize: false }, visible: true,
        };
        const state = {
          schema_version: 2, layers: [layer],
          roi: { minimum: [-3,-2,-1], maximum: [4,5,6] },
          measurements: [{ first_source_key: sourceKey, first_world: [1,2,3],
            second_source_key: null, second_world: null }],
          bookmarks: [{ name: "Reload evidence", camera: {
            target: [1,2,3], rotation_center: [4,5,6],
            camera_to_world: [[1,0,0],[0,1,0],[0,0,1]],
            distance: 7, fov_y_degrees: 45,
          }}],
        };
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 901, sessionGeneration: 8,
          replayEpoch: 1, document: state,
        }}));
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_901, sourceKey,
          name: "evidence.pcd", bytes, sessionGeneration: 8, replayEpoch: 1,
          reviewLayer: layer,
        }}));
      });
      await page.locator("#layer-list option[value='review-8-1']").waitFor();
      // The imported state creates an unresolved row before its Remote payload
      // finishes decoding. Wait for the real renderer layer, not that row.
      await page.waitForFunction(() => {
        const input = document.querySelector("#layer-opacity");
        return input instanceof HTMLInputElement && !input.disabled;
      });
      await page.locator("#layer-list").selectOption("review-8-1");
      await page.locator("#layer-opacity").evaluate((input) => {
        input.value = "0.4";
        input.dispatchEvent(new Event("input", { bubbles: true }));
      });
      const acknowledgedSnapshot = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "reviewShareState" && message.sessionGeneration === 8,
        ).at(-1),
      );
      if (!acknowledgedSnapshot || acknowledgedSnapshot.replayEpoch !== 1 ||
          acknowledgedSnapshot.state.layers.length !== 1 ||
          acknowledgedSnapshot.state.layers[0].style.opacity !== 0.4 ||
          JSON.stringify(acknowledgedSnapshot.state.roi) !==
          JSON.stringify({ minimum: [-3,-2,-1], maximum: [4,5,6] }) ||
          acknowledgedSnapshot.state.measurements.length !== 1 ||
          !acknowledgedSnapshot.state.bookmarks.some((bookmark) =>
            bookmark.name === "Reload evidence")) {
        throw new Error("review semantic snapshot omitted manual evidence before Reload");
      }
      await page.evaluate(async (snapshot) => {
        const sourceKey = snapshot.state.layers[0].source_key;
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 902, sessionGeneration: 8,
          replayEpoch: 2, document: snapshot.state,
        }}));
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        // Old asynchronous Remote data from epoch 1 must be discarded.
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_902, sourceKey,
          name: "late-evidence.pcd", bytes: bytes.slice(0),
          sessionGeneration: 8, replayEpoch: 1,
          reviewLayer: snapshot.state.layers[0],
        }}));
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_903, sourceKey,
          name: "evidence.pcd", bytes, sessionGeneration: 8, replayEpoch: 2,
          reviewLayer: snapshot.state.layers[0],
        }}));
      }, acknowledgedSnapshot);
      await page.locator("#layer-list option[value='review-8-1']").waitFor();
      if (await page.locator("#layer-list option").count() !== 1) {
        throw new Error("same-session Replay accepted a stale epoch layer payload");
      }
      await page.locator("#export-review-share").click();
      const replayedEvidence = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportReviewShare").at(-1),
      );
      const replayedLayer = replayedEvidence?.document.layers[0];
      if (!replayedEvidence || replayedLayer?.style.opacity !== 0.4 ||
          JSON.stringify(replayedEvidence.document.roi) !==
            JSON.stringify({ minimum: [-3,-2,-1], maximum: [4,5,6] }) ||
          replayedEvidence.document.measurements.length !== 1 ||
          replayedEvidence.document.bookmarks.find((bookmark) =>
            bookmark.name === "Reload evidence") === undefined) {
        throw new Error("Reload/export lost acknowledged Review Share evidence");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareSaved", requestId, name: "review.json" },
      })), replayedEvidence.requestId);
      // An entirely unresolved import still owns editable world-space evidence.
      // Its first *manual* Add has no reviewLayer envelope, so this catches the
      // otherwise easy-to-miss path where pending ROI is never installed.
      await page.evaluate(() => {
        const unresolvedKey = `sha256:${"d".repeat(64)}`;
        const unresolved = {
          source_key: unresolvedKey, runtime_id: "review-9-1",
          name: "missing-remote.pcd",
          local_to_world: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
          style: { color_by: 4, color_map: 0, point_size: 1, opacity: 1,
            scalar_min: 0, scalar_max: 1, fixed_color: [1,1,1], noise_color: [1,0,0],
            highlight_noise: false, intensity_equalize: false }, visible: true,
        };
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "reviewShareLoaded", requestId: 903, sessionGeneration: 9,
          replayEpoch: 1, document: {
            schema_version: 2, layers: [unresolved],
            roi: { minimum: [-6,-5,-1], maximum: [6,5,1] },
            measurements: [], bookmarks: [],
          },
        }}));
      });
      await page.locator("#layer-list option[value='review-9-1']").waitFor();
      await page.locator("#reset-roi").click();
      let unresolvedRoiState = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "reviewShareState" && message.sessionGeneration === 9,
        ).at(-1),
      );
      if (!unresolvedRoiState || unresolvedRoiState.state.roi !== null) {
        throw new Error("unresolved Review Share ROI cannot be cleared before source attach");
      }
      for (const [id, value] of Object.entries({
        "roi-min-x": "-6", "roi-max-x": "6",
        "roi-min-y": "-5", "roi-max-y": "5",
        "roi-min-z": "-1", "roi-max-z": "1",
      })) {
        await page.locator(`#${id}`).fill(value);
      }
      await page.locator("#apply-roi").click();
      unresolvedRoiState = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "reviewShareState" && message.sessionGeneration === 9,
        ).at(-1),
      );
      if (!unresolvedRoiState || JSON.stringify(unresolvedRoiState.state.roi) !==
          JSON.stringify({ minimum: [-6,-5,-1], maximum: [6,5,1] })) {
        throw new Error("unresolved Review Share ROI cannot be edited before source attach");
      }
      await page.evaluate(async () => {
        const manualKey = `sha256:${"e".repeat(64)}`;
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        // Deliberately omit reviewLayer: this is a user Add after import.
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_904, sourceKey: manualKey,
          name: "manual-after-import.pcd", bytes, sessionGeneration: 9, replayEpoch: 1,
        }}));
      });
      await page.waitForFunction(() => {
        const input = document.querySelector("#layer-opacity");
        return input instanceof HTMLInputElement && !input.disabled;
      });
      await page.locator("#roi-result").getByText(/^ROI: [\d,]+ points$/).waitFor();
      const fullPointCount = 125_980;
      await page.locator("#export-roi").click();
      await page.waitForFunction(() => window.kptPostedMessages.some((message) =>
        message.type === "exportPly" && message.sessionGeneration === 9,
      ));
      const importedRoiExport = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "exportPly" && message.sessionGeneration === 9,
        ).at(-1),
      );
      if (!importedRoiExport || importedRoiExport.replayEpoch !== 1 ||
          importedRoiExport.pointCount <= 0 ||
          importedRoiExport.pointCount >= fullPointCount) {
        throw new Error("first manual Add did not apply imported ROI to PLY export");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "exportedPly", requestId, name: "roi.ply", pointCount: 1 },
      })), importedRoiExport.requestId);
      await page.locator("#layer-list").selectOption("review-9-1");
      if (await page.locator("#remove-layer").isDisabled()) {
        throw new Error("selected unresolved Review Share layer cannot be removed");
      }
      await page.locator("#remove-layer").click();
      const removedUnresolvedState = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "reviewShareState" && message.sessionGeneration === 9,
        ).at(-1),
      );
      const removedUnresolved = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "removeLayer" && message.sessionGeneration === 9,
        ).at(-1),
      );
      if (!removedUnresolvedState || removedUnresolvedState.replayEpoch !== 1 ||
          removedUnresolvedState.state.layers.length !== 1 ||
          JSON.stringify(removedUnresolvedState.state.roi) !==
            JSON.stringify({ minimum: [-6,-5,-1], maximum: [6,5,1] }) ||
          !removedUnresolved || removedUnresolved.replayEpoch !== 1) {
        throw new Error("unresolved Review Share removal lost semantic ROI state");
      }
      // A Remote payload can have crossed postMessage before Remove while its
      // decoder still waits. The stale imported envelope must not recreate the
      // selected unresolved layer after its tombstone is already published.
      const tombstoneLayer = {
        source_key: `sha256:${"f".repeat(64)}`,
        runtime_id: "review-10-1",
        name: "late-remote.pcd",
        local_to_world: [[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
        style: { color_by: 4, color_map: 0, point_size: 1, opacity: 1,
          scalar_min: 0, scalar_max: 1, fixed_color: [1,1,1], noise_color: [1,0,0],
          highlight_noise: false, intensity_equalize: false }, visible: true,
      };
      await page.evaluate((layer) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "reviewShareLoaded", requestId: 904, sessionGeneration: 10,
          replayEpoch: 1, document: {
            schema_version: 2, layers: [layer], roi: null, measurements: [], bookmarks: [],
          } },
      })), tombstoneLayer);
      await page.locator("#layer-list option[value='review-10-1']").waitFor();
      await page.evaluate(async (layer) => {
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        // Start a real worker decode, then remove synchronously before its
        // result can return on the next task. This exercises the decoded-side
        // tombstone gate as well as the host queue's postMessage gate.
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_905, sourceKey: layer.source_key,
          name: layer.name, bytes, sessionGeneration: 10, replayEpoch: 1,
          sourceRevision: 0, reviewLayer: layer,
        }}));
        const list = document.querySelector("#layer-list");
        list.value = layer.runtime_id;
        list.dispatchEvent(new Event("change", { bubbles: true }));
        document.querySelector("#remove-layer").click();
      }, tombstoneLayer);
      await page.waitForTimeout(350);
      if (await page.locator("#layer-list option").count() !== 0) {
        throw new Error("removed unresolved Review Share layer revived from stale Remote payload");
      }
      // A later deliberate Add receives a new source revision and is allowed
      // through. Host catalog layers use source_key as their runtime identity.
      await page.evaluate(async (layer) => {
        const bytes = await fetch("/data/000123.pcd").then((response) => response.arrayBuffer());
        const catalogLayer = { ...layer, runtime_id: layer.source_key, name: "readded.pcd" };
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId: 1_000_000_906, sourceKey: layer.source_key,
          name: catalogLayer.name, bytes, sessionGeneration: 10, replayEpoch: 1,
          sourceRevision: 1, reviewLayer: catalogLayer,
        }}));
      }, tombstoneLayer);
      await page.waitForFunction((sourceKey) =>
        [...document.querySelectorAll("#layer-list option")].some((option) =>
          option.value === sourceKey), tombstoneLayer.source_key);
      await page.locator("#import-review-share").click();
      const importRequest = await page.evaluate(() =>
        window.kptPostedMessages.filter((message) =>
          message.type === "importReviewShare").at(-1),
      );
      if (importRequest?.sessionGeneration !== 10 || importRequest.replayEpoch !== 1) {
        throw new Error("Review import did not carry session identity");
      }
      await page.evaluate((requestId) => window.dispatchEvent(new MessageEvent("message", {
        data: { type: "hostError", requestId, message: "cancelled" },
      })), importRequest.requestId);
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
      await page.locator("#player").waitFor({ state: "visible" });
      const requested = (await page.locator("body").getAttribute("data-requested"))
        ?.split(",").map(Number) ?? [];
      if (!requested.includes(0) || !requested.includes(1) ||
          !requested.includes(2) || !requested.includes(3)) {
        throw new Error(`sequence did not prefetch neighboring frames: ${requested}`);
      }
      // Playback options only exist for a sequence. Cover their real event
      // handlers here instead of clicking hidden single-cloud fixture markup.
      await page.locator("#player-options > summary").click();
      await page.locator("#reset-playback").click();
      await page.locator("#frame-label").filter({ hasText: "1 / 4" }).waitFor();
      await page.locator("#next-frame").click();
      await page.locator("#frame-label").filter({ hasText: "2 / 4" }).waitFor();
      await page.locator("#previous-frame").click();
      await page.locator("#frame-label").filter({ hasText: "1 / 4" }).waitFor();
      // Keep reverse playback alive at frame 0 long enough to assert its
      // running state; Reset below stops it and returns to frame 0.
      await page.locator("#loop-playback").check();
      await page.locator("#reverse-play").click();
      await page.locator("#play").filter({ hasText: "⏸" }).waitFor();
      await page.locator("#reset-playback").click();
      await page.locator("#play").filter({ hasText: "▶" }).waitFor();
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

  // Fit must rebuild the PerspectiveCamera projection after it has derived
  // near/far from *all* visible review layers.  Exercise the failure order:
  // add a very large layer, then a small layer (which refreshes the camera
  // bounds), then Fit both.  With a stale small far plane, hiding the small
  // layer leaves the large layer entirely clipped from the real WebGL frame.
  const frustumPage = await browser.newPage();
  const frustumErrors = [];
  frustumPage.on("console", (message) => {
    if (message.type() === "error") frustumErrors.push(message.text());
  });
  frustumPage.on("pageerror", (error) => frustumErrors.push(error.message));
  try {
    await frustumPage.goto(`${baseUrl}/vscode/tests/worker-smoke.html?fitFrustum=1`);
    await frustumPage.locator("body[data-state='passed']").waitFor({ timeout: 30_000 });
    await frustumPage.evaluate(() => {
      const generation = 21;
      const identity = { sessionGeneration: generation, replayEpoch: 1 };
      const makePcd = (extent) => {
        const positions = [];
        for (const first of [-1, -0.5, 0, 0.5, 1]) {
          for (const second of [-1, -0.5, 0, 0.5, 1]) {
            positions.push(
              [extent, first * extent, second * extent],
              [-extent, first * extent, second * extent],
              [first * extent, extent, second * extent],
              [first * extent, -extent, second * extent],
              [first * extent, second * extent, extent],
              [first * extent, second * extent, -extent],
            );
          }
        }
        const records = new ArrayBuffer(positions.length * 12);
        const view = new DataView(records);
        positions.forEach((position, index) => {
          const offset = index * 12;
          view.setFloat32(offset, position[0], true);
          view.setFloat32(offset + 4, position[1], true);
          view.setFloat32(offset + 8, position[2], true);
        });
        const header = new TextEncoder().encode(`VERSION 0.7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH ${positions.length}
HEIGHT 1
POINTS ${positions.length}
DATA binary
`);
        const bytes = new Uint8Array(header.byteLength + records.byteLength);
        bytes.set(header);
        bytes.set(new Uint8Array(records), header.byteLength);
        return bytes.buffer;
      };
      const style = {
        color_by: 4, color_map: 0, point_size: 5, opacity: 1,
        scalar_min: 0, scalar_max: 1, fixed_color: [1, 1, 1],
        noise_color: [1, 0, 0], highlight_noise: false,
        intensity_equalize: false,
      };
      const matrix = [[1, 0, 0, 0], [0, 1, 0, 0],
        [0, 0, 1, 0], [0, 0, 0, 1]];
      const largeKey = `sha256:${"a".repeat(64)}`;
      const smallKey = `sha256:${"b".repeat(64)}`;
      const large = {
        source_key: largeKey, runtime_id: "review-21-1", name: "large.pcd",
        local_to_world: matrix, style, visible: true,
      };
      const small = {
        source_key: smallKey, runtime_id: "review-21-2", name: "small.pcd",
        local_to_world: matrix, style: { ...style }, visible: true,
      };
      window.dispatchEvent(new MessageEvent("message", { data: {
        type: "reviewShareLoaded", requestId: 2_100, ...identity,
        document: {
          schema_version: 2, layers: [large, small], roi: null,
          measurements: [], bookmarks: [],
        },
      }}));
      for (const [requestId, layer, extent] of [
        [2_101, large, 100_000], [2_102, small, 10],
      ]) {
        window.dispatchEvent(new MessageEvent("message", { data: {
          type: "addLayer", requestId, sourceKey: layer.source_key,
          name: layer.name, bytes: makePcd(extent), reviewLayer: layer,
          sourceRevision: 0, ...identity,
        }}));
      }
    });
    await frustumPage.locator("#layer-list option[value='review-21-2']").waitFor({
      timeout: 30_000,
    });
    await frustumPage.locator("#status[data-kind='ready']").waitFor({ timeout: 30_000 });
    await frustumPage.locator("#background").evaluate((input) => {
      input.value = "#000000";
      input.dispatchEvent(new Event("input", { bubbles: true }));
    });
    await frustumPage.locator("#fit-visible").click();
    await frustumPage.locator("#layer-list").selectOption("review-21-2");
    await frustumPage.locator("#layer-visible").uncheck();
    await frustumPage.locator("#layer-list").selectOption("review-21-1");
    await frustumPage.locator("#layer-color").evaluate((input) => {
      input.value = "#ffffff";
      input.dispatchEvent(new Event("input", { bubbles: true }));
    });
    await frustumPage.locator("#layer-size").evaluate((input) => {
      input.value = "5";
      input.dispatchEvent(new Event("input", { bubbles: true }));
    });
    await waitForPaint(frustumPage);
    await frustumPage.locator("#save-screenshot").click();
    await frustumPage.waitForFunction(() => window.kptPostedMessages.some((message) =>
      message.type === "saveScreenshot"), undefined, { timeout: 10_000 });
    const brightPixels = await frustumPage.evaluate(async () => {
      const message = window.kptPostedMessages.filter((candidate) =>
        candidate.type === "saveScreenshot").at(-1);
      if (!message || !(message.bytes instanceof ArrayBuffer)) return 0;
      const bitmap = await createImageBitmap(new Blob([message.bytes], { type: "image/png" }));
      const image = document.createElement("canvas");
      image.width = bitmap.width;
      image.height = bitmap.height;
      const context = image.getContext("2d");
      context?.drawImage(bitmap, 0, 0);
      bitmap.close();
      const pixels = context?.getImageData(0, 0, image.width, image.height).data;
      if (!pixels) return 0;
      let count = 0;
      for (let index = 0; index < pixels.length; index += 4) {
        if (pixels[index] > 180 && pixels[index + 1] > 180 && pixels[index + 2] > 180) {
          ++count;
        }
      }
      return count;
    });
    if (brightPixels < 40) {
      throw new Error(`Fit clipped large review layer after small bounds refresh (${brightPixels} bright pixels)`);
    }
    if (frustumErrors.length > 0) throw new Error(frustumErrors.join("\n"));
  } finally {
    await frustumPage.close();
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
