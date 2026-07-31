import { chromium } from "@playwright/test";
import { existsSync } from "node:fs";

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
      await page.locator("#frame").evaluate((input) => {
        input.value = "2";
        input.dispatchEvent(new Event("input", { bubbles: true }));
      });
      await page.locator("body[data-requested*='2']").waitFor();
      await page.locator("#frame-label").filter({ hasText: "3 / 4" }).waitFor();
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
