import * as assert from "node:assert/strict";
import * as vscode from "vscode";
import type { ExtensionApi, ExtensionRenderEvent } from "../src/extension";
import { convertPointCloud } from "../src/converter";

class RemoteFixtureProvider implements vscode.FileSystemProvider {
  private readonly emitter =
    new vscode.EventEmitter<vscode.FileChangeEvent[]>();
  readonly onDidChangeFile = this.emitter.event;
  readCount = 0;
  private readonly data = new TextEncoder().encode(
    "1 2 3 0.5\n-1 -2 -3 0.75\n",
  );
  private readonly npyData = npyFixture();

  watch(): vscode.Disposable {
    return new vscode.Disposable(() => {});
  }

  stat(uri: vscode.Uri): vscode.FileStat {
    const size = this.bytesFor(uri).byteLength;
    return {
      type: vscode.FileType.File,
      ctime: 0,
      mtime: 0,
      size,
    };
  }

  readFile(uri: vscode.Uri): Uint8Array {
    ++this.readCount;
    return this.bytesFor(uri);
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

  private bytesFor(uri: vscode.Uri): Uint8Array {
    if (uri.path.endsWith("empty.xyzi")) return new Uint8Array();
    if (uri.path.endsWith(".npy")) return this.npyData;
    return this.data;
  }
}

export async function run(): Promise<void> {
  const extension = vscode.extensions.getExtension<ExtensionApi>(
    "dengqi.pointcloud-tools",
  );
  assert.ok(extension);
  const api = await extension.activate();
  const conversionSource = new TextEncoder().encode("1 2 3 0.5\n");
  const converted = await convertPointCloud(
    extension.extensionUri,
    "sample.xyzi",
    "sample.pcd",
    conversionSource.buffer,
  );
  assert.match(new TextDecoder().decode(converted.subarray(0, 64)), /PCD/u);
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
    await vscode.commands.executeCommand("workbench.action.closeActiveEditor");

    const npyUri = vscode.Uri.parse("kpt-test:/remote/sample.npy");
    renderEvent = undefined;
    await vscode.commands.executeCommand("kpt.openPointCloud", npyUri);
    await waitFor(() => renderEvent !== undefined, 15_000);
    assert.equal(renderEvent?.error, undefined);
    assert.equal(renderEvent?.pointCount, 2);
    const npyInput = vscode.window.tabGroups.activeTabGroup.activeTab?.input;
    assert.ok(npyInput && "viewType" in npyInput);
    assert.equal(
      (npyInput as vscode.TabInputCustom).viewType,
      "kpt.binaryPointCloudViewer",
    );
  } finally {
    rendered.dispose();
    registration.dispose();
  }
}

function npyFixture(): Uint8Array {
  const prefix = new Uint8Array([0x93, 0x4e, 0x55, 0x4d, 0x50, 0x59, 1, 0]);
  const rawHeader = "{'descr': '<f4', 'fortran_order': False, 'shape': (2, 4), }";
  const padding = 16 - ((prefix.byteLength + 2 + rawHeader.length + 1) % 16);
  const header = new TextEncoder().encode(`${rawHeader}${" ".repeat(padding)}\n`);
  const result = new Uint8Array(prefix.byteLength + 2 + header.byteLength + 32);
  result.set(prefix);
  new DataView(result.buffer).setUint16(prefix.byteLength, header.byteLength, true);
  result.set(header, prefix.byteLength + 2);
  const values = [1, 2, 3, 0.5, -1, -2, -3, 0.75];
  const data = new DataView(result.buffer, prefix.byteLength + 2 + header.byteLength);
  values.forEach((value, index) => data.setFloat32(index * 4, value, true));
  return result;
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
