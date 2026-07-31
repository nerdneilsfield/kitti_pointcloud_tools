import { chromium } from "@playwright/test";
import { existsSync } from "node:fs";

const baseUrl = process.env.KPT_VSCODE_SMOKE_URL ?? "http://127.0.0.1:8766";
const counts = (process.env.KPT_BENCHMARK_COUNTS ?? "100000,1000000,10000000")
  .split(",")
  .map(Number);
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
  for (const count of counts) {
    const page = await browser.newPage();
    await page.goto(
      `${baseUrl}/vscode/tests/benchmark.html?count=${count}`,
    );
    await page.locator("body:not([data-state='loading'])").waitFor({
      timeout: 130_000,
    });
    const body = page.locator("body");
    const state = await body.getAttribute("data-state");
    const text = await body.textContent();
    if (state !== "passed") throw new Error(text ?? "benchmark failed");
    process.stdout.write(`${text}\n`);
    await page.close();
  }
  const degenerate = await browser.newPage();
  await degenerate.goto(
    `${baseUrl}/vscode/tests/benchmark.html?count=200001&degenerate=1`,
  );
  await degenerate.locator("body:not([data-state='loading'])").waitFor({
    timeout: 130_000,
  });
  const degenerateState = await degenerate.locator("body")
    .getAttribute("data-state");
  if (degenerateState !== "passed") {
    throw new Error(await degenerate.locator("body").textContent() ??
      "degenerate octree benchmark failed");
  }
  await degenerate.close();
} finally {
  await browser.close();
}
