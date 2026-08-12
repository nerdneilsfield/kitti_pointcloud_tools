import * as assert from "node:assert/strict";
import * as vscode from "vscode";
import type { ExtensionApi, ExtensionRenderEvent } from "../src/extension";

class RemoteFixtureProvider implements vscode.FileSystemProvider {
  private readonly emitter =
    new vscode.EventEmitter<vscode.FileChangeEvent[]>();
  readonly onDidChangeFile = this.emitter.event;
  readCount = 0;
  private readonly data = new TextEncoder().encode(
    "1 2 3 0.5\n-1 -2 -3 0.75\n",
  );

  watch(): vscode.Disposable {
    return new vscode.Disposable(() => {});
  }

  stat(uri: vscode.Uri): vscode.FileStat {
    const size = uri.path.endsWith("empty.xyzi") ? 0 : this.data.byteLength;
    return {
      type: vscode.FileType.File,
      ctime: 0,
      mtime: 0,
      size,
    };
  }

  readFile(uri: vscode.Uri): Uint8Array {
    ++this.readCount;
    return uri.path.endsWith("empty.xyzi") ? new Uint8Array() : this.data;
  }

  readDirectory(): [string, vscode.FileType][] {
    return [];
  }

  createDirectory(): void {
    throw vscode.FileSystemError.NoPermissions();
  }
  writeFile(): void {
    throw vscode.FileSystemError.NoPermissions();
  }
  delete(): void {
    throw vscode.FileSystemError.NoPermissions();
  }
  rename(): void {
    throw vscode.FileSystemError.NoPermissions();
  }
}

export async function run(): Promise<void> {
  const extension = vscode.extensions.getExtension<ExtensionApi>(
    "dengqi.pointcloud-tools",
  );
  assert.ok(extension);
  const api = await extension.activate();
  const provider = new RemoteFixtureProvider();
  const registration = vscode.workspace.registerFileSystemProvider(
    "kpt-test",
    provider,
    { isCaseSensitive: true, isReadonly: true },
  );
  let renderEvent: ExtensionRenderEvent | undefined;
  const rendered = api.onDidRender((event) => {
    renderEvent = event;
  });
  try {
    const uri = vscode.Uri.parse("kpt-test:/remote/sample.xyzi");
    for (let attempt = 0; attempt < 2; ++attempt) {
      renderEvent = undefined;
      await vscode.commands.executeCommand(
        "vscode.openWith",
        uri,
        "kpt.pointCloudViewer",
      );
      await waitFor(
        () => renderEvent !== undefined,
        15_000,
        () => `reads=${provider.readCount}, active=${extension.isActive}`,
      );
      assert.equal(renderEvent?.error, undefined);
      assert.equal(renderEvent?.pointCount, 2);
      assert.equal(extension.isActive, true);
      const input = vscode.window.tabGroups.activeTabGroup.activeTab?.input;
      assert.ok(input && "viewType" in input);
      assert.equal(
        (input as vscode.TabInputCustom).viewType,
        "kpt.pointCloudViewer",
      );
      await vscode.commands.executeCommand(
        "workbench.action.closeActiveEditor",
      );
    }
    assert.equal(provider.readCount, 2);

    const emptyUri = vscode.Uri.parse("kpt-test:/remote/empty.xyzi");
    renderEvent = undefined;
    await vscode.commands.executeCommand(
      "vscode.openWith", emptyUri, "kpt.pointCloudViewer",
    );
    await waitFor(() => renderEvent !== undefined, 15_000);
    assert.match(renderEvent?.error ?? "", /empty/i);
    assert.equal(extension.isActive, true);
    await vscode.commands.executeCommand("workbench.action.closeActiveEditor");

    renderEvent = undefined;
    await vscode.commands.executeCommand(
      "vscode.openWith", uri, "kpt.pointCloudViewer",
    );
    await waitFor(() => renderEvent !== undefined, 15_000);
    assert.equal(renderEvent?.error, undefined);
    assert.equal(renderEvent?.pointCount, 2);
    assert.equal(extension.isActive, true);
    assert.equal(provider.readCount, 4);
  } finally {
    rendered.dispose();
    registration.dispose();
  }
}

async function waitFor(
  predicate: () => boolean,
  timeoutMilliseconds: number,
  diagnostic: () => string = () => "no diagnostic",
): Promise<void> {
  const deadline = Date.now() + timeoutMilliseconds;
  while (!predicate()) {
    if (Date.now() >= deadline) {
      throw new Error(`timed out waiting for custom editor read (${diagnostic()})`);
    }
    await new Promise((resolve) => setTimeout(resolve, 50));
  }
}
