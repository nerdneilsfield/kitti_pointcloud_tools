import { runTests } from "@vscode/test-electron";
import { mkdtemp, rm } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const vscodeRoot = resolve(fileURLToPath(new URL("..", import.meta.url)));
const extensionPath = process.env.KPT_EXTENSION_PATH
  ? resolve(process.env.KPT_EXTENSION_PATH)
  : vscodeRoot;
const workspace = await mkdtemp(join(tmpdir(), "kpt-vscode-test-"));
try {
  let failure;
  for (let attempt = 1; attempt <= 3; ++attempt) {
    try {
      await runTests({
        extensionDevelopmentPath: extensionPath,
        extensionTestsPath: join(vscodeRoot, "dist", "integration.js"),
        launchArgs: [
          workspace,
          "--disable-extensions",
          "--use-gl=angle",
          "--use-angle=swiftshader",
          "--enable-unsafe-swiftshader",
        ],
      });
      failure = undefined;
      break;
    } catch (error) {
      failure = error;
      if (attempt === 3) break;
      console.warn(`VS Code test bootstrap failed (attempt ${attempt}/3); retrying`);
      await new Promise((resolve) => setTimeout(resolve, attempt * 1_000));
    }
  }
  if (failure) throw failure;
} finally {
  await rm(workspace, { recursive: true, force: true });
}
