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
  await runTests({
    extensionDevelopmentPath: extensionPath,
    extensionTestsPath: join(vscodeRoot, "dist", "integration.js"),
    launchArgs: [
      workspace,
      "--disable-extensions",
      "--use-gl=angle",
      "--use-angle=swiftshader",
    ],
  });
} finally {
  await rm(workspace, { recursive: true, force: true });
}
