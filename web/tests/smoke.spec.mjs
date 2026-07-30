import { expect, test } from "@playwright/test";

test("starts cross-origin-isolated WebGL workbench", async ({ page }) => {
  const pageErrors = [];
  const requestMethods = [];
  page.on("pageerror", (error) => pageErrors.push(error.message));
  page.on("request", (request) => requestMethods.push(request.method()));

  await page.goto("/index.html");
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");
  await expect(page.locator("#fatal-error")).toBeHidden();
  await expect(page.locator("#canvas")).toBeVisible();
  expect(await page.evaluate(() => globalThis.crossOriginIsolated)).toBe(true);
  expect(
    await page.evaluate(() =>
      Module._kpt_web_has_glyph("中".codePointAt(0))),
  ).toBe(1);

  await page.waitForTimeout(1500);
  expect(pageErrors).toEqual([]);
  expect(requestMethods.every((method) => method === "GET")).toBe(true);
});
