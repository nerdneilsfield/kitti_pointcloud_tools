import { expect, test } from "@playwright/test";
import { readFileSync } from "node:fs";

const rendererSource = readFileSync(
  new URL("../../src/gui/backend/opengl/point_renderer.cpp", import.meta.url),
  "utf8",
);
if ((rendererSource.match(/layout\(location = 3\) in float in_noise;/g) ?? [])
      .length !== 2 ||
    (rendererSource.match(
      /if \(highlight_noise && vertex_noise > 0\.5\)/g,
    ) ?? []).length !== 2) {
  throw new Error("desktop and WebGL shaders must both override noise color");
}

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

test("stages and releases selected browser assets transactionally", async ({
  page,
}) => {
  await page.goto("/index.html");
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");

  const result = await page.evaluate(async () => {
    const path = "/kpt-import/clouds/stage.bin";
    await KptWeb.select(1, [new File([new Uint8Array([1, 2, 3])], "stage.bin")]);
    await KptWeb.stage(path, 0x7ffffff0);
    const staged = FS.analyzePath(path).exists;
    KptWeb.release(path);
    const released = !FS.analyzePath(path).exists;

    await KptWeb.stage(`${path}\n/kpt-import/clouds/missing.bin`, 0x7ffffff1);
    const rolledBack = !FS.analyzePath(path).exists;
    return { staged, released, rolledBack };
  });

  expect(result).toEqual({ staged: true, released: true, rolledBack: true });
  await expect(page.locator("#fatal-error")).toBeHidden();
});

test("reference-counts concurrent staging of the same asset", async ({
  page,
}) => {
  await page.goto("/index.html");
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");

  const result = await page.evaluate(async () => {
    let resolveBytes;
    const bytes = new Promise((resolve) => {
      resolveBytes = resolve;
    });
    const file = { name: "shared.bin", arrayBuffer: () => bytes };
    const path = "/kpt-import/clouds/shared.bin";
    await KptWeb.select(1, [file]);
    const first = KptWeb.stage(path, 0x7ffffff2);
    const second = KptWeb.stage(path, 0x7ffffff3);
    resolveBytes(new Uint8Array([4, 5, 6]).buffer);
    await Promise.all([first, second]);
    KptWeb.release(path);
    const retained = FS.analyzePath(path).exists;
    KptWeb.release(path);
    return {
      retained,
      removed: !FS.analyzePath(path).exists,
    };
  });

  expect(result).toEqual({ retained: true, removed: true });
});

test("rejects same-root reselection while staged assets are in use", async ({
  page,
}) => {
  await page.goto("/index.html");
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");

  const result = await page.evaluate(async () => {
    const path = "/kpt-import/clouds/same.bin";
    await KptWeb.select(1, [new File([new Uint8Array([1])], "same.bin")]);
    await KptWeb.stage(path, 0x7ffffff5);
    await KptWeb.select(1, [new File([new Uint8Array([2])], "same.bin")]);
    const error = UTF8ToString(Module._kpt_web_selection_error());
    KptWeb.release(path);
    await KptWeb.select(1, [new File([new Uint8Array([2])], "same.bin")]);
    await KptWeb.stage(path, 0x7ffffff6);
    const bytes = [...FS.readFile(path)];
    KptWeb.release(path);
    return { error, bytes };
  });

  expect(result.error).toContain("still in use");
  expect(result.bytes).toEqual([2]);
});

test("cleans remaining assets after one unlink failure", async ({ page }) => {
  await page.goto("/index.html");
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");

  const result = await page.evaluate(async () => {
    const first = "/kpt-import/clouds/first.bin";
    const second = "/kpt-import/clouds/second.bin";
    await KptWeb.select(1, [
      new File([new Uint8Array([1])], "first.bin"),
      new File([new Uint8Array([2])], "second.bin"),
    ]);
    await KptWeb.stage(`${first}\n${second}`, 0x7ffffff4);
    const unlink = FS.unlink;
    FS.unlink = (path) => {
      if (path === first)
        throw Object.assign(new Error("injected unlink failure"), { errno: 5 });
      return unlink(path);
    };
    try {
      KptWeb.release(`${first}\n${second}`);
    } finally {
      FS.unlink = unlink;
    }
    const secondRemoved = !FS.analyzePath(second).exists;
    unlink(first);
    return { secondRemoved };
  });

  expect(result.secondRemoved).toBe(true);
  await expect(page.locator("#fatal-message"))
    .toContainText("Failed to release staged assets");
});

test("rejects invalid and oversized selection ABI inputs", async ({ page }) => {
  await page.goto("/index.html");
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");

  const errors = await page.evaluate(() => {
    const empty = stringToNewUTF8("");
    try {
      Module._kpt_web_selection_changed(99, empty, empty);
      const invalid = UTF8ToString(Module._kpt_web_selection_error());
      const oversized = stringToNewUTF8("x".repeat(1024 * 1024 + 1));
      try {
        Module._kpt_web_selection_changed(1, oversized, empty);
        return {
          invalid,
          oversized: UTF8ToString(Module._kpt_web_selection_error()),
        };
      } finally {
        _free(oversized);
      }
    } finally {
      _free(empty);
    }
  });

  expect(errors.invalid).toContain("invalid picker kind");
  expect(errors.oversized).toContain("exceeds 1 MiB limit");

  await page.evaluate(async () => {
    await KptWeb.select(1, [
      { name: "bad\nname.bin", arrayBuffer: async () => new ArrayBuffer(0) },
    ]);
  });
  expect(await page.evaluate(() =>
    UTF8ToString(Module._kpt_web_selection_error())))
    .toContain("Invalid filename");
});

test("offers reload recovery after WebGL context loss", async ({ page }) => {
  await page.goto("/index.html");
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");
  await page.locator("#canvas").evaluate((canvas) => {
    canvas.dispatchEvent(new Event("webglcontextlost", { cancelable: true }));
  });
  await expect(page.locator("#fatal-message"))
    .toContainText("WebGL2 context was lost");
  await expect(page.locator("#fatal-reload")).toBeVisible();
  await Promise.all([
    page.waitForEvent("load"),
    page.locator("#fatal-reload").click(),
  ]);
  await expect
    .poll(() => page.evaluate(() => document.body.dataset.kptRuntime))
    .toBe("ready");
});
