import { chromium } from "@playwright/test";
import { existsSync, readFileSync } from "node:fs";

const extensionSource = readFileSync(
  new URL("../src/extension.ts", import.meta.url),
  "utf8",
);
if (!/#controls-help\s*\{[^}]*position:\s*fixed/s.test(extensionSource) ||
    !/@media \(max-width: 1200px\)[\s\S]*#information\s*\{/s.test(
      extensionSource,
    )) {
  throw new Error("production webview CSS lacks fixed, responsive help/info");
}

const baseUrl = process.env.KPT_VSCODE_SMOKE_URL ?? "http://127.0.0.1:8766";
const defaultExecutable = chromium.executablePath();
const fullExecutable = defaultExecutable
  .replace("chromium_headless_shell-", "chromium-")
  .replace(
    "/chrome-headless-shell-linux64/chrome-headless-shell",
    "/chrome-linux64/chrome",
  );
const browser = await chromium.launch({
  headless: process.env.KPT_WEB_HEADED !== "1",
  args: ["--use-gl=angle", "--use-angle=swiftshader"],
  executablePath: existsSync(defaultExecutable)
    ? defaultExecutable
    : fullExecutable,
});
try {
  const productionStyle =
    extensionSource.match(/<style>([\s\S]*?)<\/style>/)?.[1];
  if (!productionStyle) throw new Error("cannot extract production webview CSS");
  for (const width of [800, 1150]) {
    const layoutPage = await browser.newPage({
      viewport: { width, height: 600 },
    });
    await layoutPage.setContent(`
      <style>${productionStyle}</style>
      <div id="information"><div id="status">Ready</div><section id="cloud-info">
        <strong>AABB</strong><span>Min: (-78.9, -36.9, -2.4)</span>
        <span>Max: (77.2, 63.6, 2.9)</span><span>Size: (156, 100, 5)</span>
      </section></div>
      <div id="toolbar"><select><option>Intensity</option></select>
        <input id="point-size" type="range"><button>Fit</button>
        <button>Reload</button><button>Top</button><button>Front</button>
        <button>Left</button><button>Right</button><button>Iso</button>
        <details><summary>Overlays</summary></details>
        <input type="color"></div>
    `);
    const information = await layoutPage.locator("#information").boundingBox();
    const toolbar = await layoutPage.locator("#toolbar").boundingBox();
    if (!information || !toolbar ||
        rectanglesOverlap(information, toolbar)) {
      throw new Error(`production overlays collide at ${width}px`);
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
      const canvas = page.locator("canvas");
      const cloudOnly = await canvas.screenshot();
      await page.locator("#show-grid").check();
      await page.waitForTimeout(100);
      const withGrid = await canvas.screenshot();
      if (cloudOnly.equals(withGrid)) {
        throw new Error("scale grid toggle did not change rendered canvas");
      }
      await page.locator("#show-axes").check();
      await page.waitForTimeout(100);
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
      if (await page.locator("#viewer").getAttribute("data-grid-plane") !==
            "xy-origin" ||
          await page.locator("#viewer").getAttribute("data-axes-palette") !==
            "pastel") {
        throw new Error("reference overlay geometry or palette is invalid");
      }
      await page.locator("#background").evaluate((input) => {
        const element = input;
        element.value = "#aaaaaa";
        element.dispatchEvent(new Event("input", { bubbles: true }));
      });
      await page.waitForTimeout(100);
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
      await page.locator("#reload").click();
      await page.locator("body[data-loads='2']").waitFor();
      await page.locator("#status[data-kind='ready']").waitFor();
      const canvasSize = await page.locator("canvas").evaluate((canvas) => ({
        width: canvas.width,
        height: canvas.height,
      }));
      if (canvasSize.width === 0 || canvasSize.height === 0) {
        throw new Error("Three.js canvas has zero size");
      }
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
      await page.waitForTimeout(120);
      await page.locator("#play").click();
      const requested = (await page.locator("body").getAttribute("data-requested"))
        ?.split(",").map(Number) ?? [];
      if (!requested.includes(0) || !requested.includes(1) ||
          !requested.includes(2) || !requested.includes(3)) {
        throw new Error(`sequence did not prefetch neighboring frames: ${requested}`);
      }
      const wasmTransfers = await page.locator("body")
        .getAttribute("data-wasm-transfers");
      if (wasmTransfers !== "1") {
        throw new Error(`decoder WASM transferred ${wasmTransfers} times`);
      }
      if (browserErrors.length > 0) throw new Error(browserErrors.join("\n"));
    }
    process.stdout.write(`${path}: ${text?.trim()}\n`);
    await page.close();
  }
} finally {
  await browser.close();
}

function rectanglesOverlap(left, right) {
  return left.x < right.x + right.width &&
    left.x + left.width > right.x &&
    left.y < right.y + right.height &&
    left.y + left.height > right.y;
}
