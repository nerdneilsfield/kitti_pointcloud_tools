import * as assert from "node:assert/strict";
import * as vscode from "vscode";
import {
  decodeWebviewMessage,
  createHostReviewSession,
  LayerReplayCatalog,
  LayerPayloadQueue,
  readLayerSource,
  relativeUriPath,
  serializeSourceUri,
  sourceKeyForUri,
} from "../src/extension";
import type { ExtensionApi, ExtensionRenderEvent } from "../src/extension";
import { convertPointCloud } from "../src/converter";
import {
  parseReviewShare,
  validRelativeSharePath,
  validateReviewShare,
} from "../src/review-share";

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
    assert.deepEqual(
      decodeWebviewMessage({
        type: "addLayers",
        requestId: 17,
        uri: "file:///client-only/never-trusted.pcd",
      }),
      { type: "addLayers", requestId: 17 },
    );
    assert.equal(
      decodeWebviewMessage({ type: "addLayers", requestId: -1 }),
      undefined,
    );
    const exportBytes = new TextEncoder().encode("ply\n").buffer;
    assert.deepEqual(
      decodeWebviewMessage({
        type: "exportPly",
        requestId: 18,
        suggestedName: "crop-roi.ply",
        pointCount: 2,
        bytes: exportBytes,
      }),
      {
        type: "exportPly",
        requestId: 18,
        suggestedName: "crop-roi.ply",
        pointCount: 2,
        bytes: exportBytes,
      },
    );
    assert.equal(
      decodeWebviewMessage({
        type: "exportPly",
        requestId: 18,
        suggestedName: "../client-path.ply",
        pointCount: 2,
        bytes: exportBytes,
      }),
      undefined,
    );
    const firstSourceKey = `sha256:${"a".repeat(64)}`;
    assert.deepEqual(
      decodeWebviewMessage({ type: "removeLayer", sourceKey: firstSourceKey }),
      { type: "removeLayer", sourceKey: firstSourceKey },
    );
    assert.deepEqual(
      decodeWebviewMessage({ type: "importReviewShare", requestId: 19 }),
      { type: "importReviewShare", requestId: 19 },
    );
    assert.equal(validRelativeSharePath("clouds/scan.xyzi"), true);
    assert.equal(validRelativeSharePath("../scan.xyzi"), false);
    const nativeFixtureUri = vscode.Uri.joinPath(
      extension.extensionUri, "tests", "fixtures", "native-review-share.json",
    );
    const nativeReview = parseReviewShare(
      await vscode.workspace.fs.readFile(nativeFixtureUri),
    );
    assert.equal(nativeReview.layers[0].source_key, "path:/srv/kitti/scan.xyzi");
    assert.throws(() => parseReviewShare(new TextEncoder().encode(JSON.stringify({
      ...nativeReview,
      layers: [{ ...nativeReview.layers[0], source_path: "../escape.xyzi" }],
    }))), /schema/u);
    const nativeSession = await createHostReviewSession(
      vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/reviews/review.json"),
      nativeReview,
    );
    const nativeTransportKey = nativeSession.state.layers[0].source_key;
    assert.match(nativeTransportKey, /^sha256:[a-f0-9]{64}$/u);
    assert.notEqual(nativeTransportKey, nativeReview.layers[0].source_key);
    assert.equal(
      nativeSession.originalKeyForAlias.get(nativeTransportKey),
      nativeReview.layers[0].source_key,
    );
    assert.equal(nativeSession.layers[0].uri?.scheme, "kpt-test");
    assert.equal(
      nativeSession.state.measurements[0].first_source_key,
      nativeTransportKey,
    );
    assert.doesNotMatch(JSON.stringify(nativeSession.state), /path:\/srv\/kitti/u);
    const opaqueReview = {
      ...nativeReview,
      layers: [{ ...nativeReview.layers[0], source_key: "opaque:remote/path" }],
      measurements: [{
        ...nativeReview.measurements[0],
        first_source_key: "opaque:remote/path",
      }],
    };
    const opaqueSession = await createHostReviewSession(
      vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/reviews/review.json"),
      opaqueReview,
    );
    assert.match(opaqueSession.state.layers[0].source_key, /^sha256:/u);
    assert.doesNotMatch(JSON.stringify(opaqueSession.state), /opaque:remote\/path/u);
    // `sha256:` is a logical source identity in v1, not a Remote URI proof.
    // A moved share may resolve a different URI and remains loadable/unverified.
    const logicalHash = `sha256:${"f".repeat(64)}`;
    const movedReview = {
      ...nativeReview,
      layers: [{ ...nativeReview.layers[0], source_key: logicalHash }],
      measurements: [{
        ...nativeReview.measurements[0], first_source_key: logicalHash,
      }],
    };
    const movedSession = await createHostReviewSession(
      vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/reviews/review.json"),
      movedReview,
    );
    assert.equal(movedSession.state.layers[0].source_key, logicalHash);
    assert.equal(movedSession.layers[0].uri?.scheme, "kpt-test");
    assert.equal(
      relativeUriPath(
        vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/reviews"),
        vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/reviews/clouds/scan.xyzi"),
      ),
      "clouds/scan.xyzi",
    );
    assert.equal(
      relativeUriPath(
        vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/reviews"),
        vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/scan.xyzi"),
      ),
      undefined,
    );
    const manyLayers = Array.from({ length: 257 }, (_, index) => ({
      ...nativeReview.layers[0],
      source_key: `sha256:${index.toString(16).padStart(64, "0")}`,
      source_path: null,
    }));
    assert.equal(validateReviewShare({
      ...nativeReview, layers: manyLayers, measurements: [], bookmarks: [],
    }), true);
    const remoteUri = vscode.Uri.parse(
      "kpt-test://ssh-remote+fixture/workspace/layer.xyzi",
    );
    const layer = await readLayerSource(remoteUri);
    assert.equal(layer.name, "layer.xyzi");
    assert.ok(layer.bytes.byteLength > 0);
    assert.match(layer.sourceKey, /^sha256:[a-f0-9]{64}$/u);
    assert.notEqual(layer.sourceKey, remoteUri.toString());
    assert.doesNotMatch(layer.sourceKey, /ssh-remote|workspace|layer/u);
    assert.equal(layer.sourceKey, await sourceKeyForUri(remoteUri));
    const vscodeRemoteUri = vscode.Uri.parse(
      "vscode-remote://ssh-remote+fixture/workspace/layer.xyzi",
    );
    assert.equal(
      serializeSourceUri(vscodeRemoteUri),
      vscodeRemoteUri.toString(),
    );
    assert.notEqual(
      await sourceKeyForUri(vscodeRemoteUri),
      await sourceKeyForUri(vscode.Uri.parse(
        "vscode-remote://ssh-remote+fixture/workspace/other.xyzi",
      )),
    );
    const queueReads: string[] = [];
    const queuePosts: Array<{ requestId: number; name: string }> = [];
    let queueRequestId = 2_000_000_000;
    const queue = new LayerPayloadQueue(
      async (candidate) => {
        queueReads.push(candidate.path);
        return {
          sourceKey: `sha256:${queueReads.length}`,
          name: candidate.path.split("/").pop() ?? "layer.xyzi",
          bytes: new ArrayBuffer(4),
        };
      },
      async (message) => {
        queuePosts.push({ requestId: message.requestId, name: message.name });
      },
      async () => assert.fail("queued Remote source should not fail"),
      () => ++queueRequestId,
      () => false,
    );
    const queueUris = ["a", "b", "c"].map((name) => vscode.Uri.parse(
      `kpt-test://ssh-remote+fixture/workspace/${name}.xyzi`,
    ));
    queue.enqueue(queueUris.map((candidate) => ({ uri: candidate, requestId: 41 })));
    await waitFor(() => queuePosts.length === 1, 1_000);
    assert.deepEqual(queueReads, ["/workspace/a.xyzi"]);
    queue.settle(123); // Stale acknowledgement cannot release another payload.
    await new Promise((resolve) => setTimeout(resolve, 25));
    assert.equal(queueReads.length, 1);
    // Recreated webview retries its URI with a new request ID. Its old ack
    // remains stale and cannot release B before the retry settles.
    queue.retryInFlight();
    await waitFor(() => queuePosts.length === 2, 1_000);
    assert.deepEqual(queueReads, ["/workspace/a.xyzi", "/workspace/a.xyzi"]);
    queue.settle(queuePosts[0].requestId);
    await new Promise((resolve) => setTimeout(resolve, 25));
    assert.equal(queuePosts.length, 2);
    // Reload cancellation uses this same renderError acknowledgement path.
    queue.settle(queuePosts[1].requestId);
    await waitFor(() => queuePosts.length === 3, 1_000);
    assert.deepEqual(queueReads, ["/workspace/a.xyzi", "/workspace/a.xyzi", "/workspace/b.xyzi"]);
    queue.settle(queuePosts[2].requestId);
    await waitFor(() => queuePosts.length === 4, 1_000);
    assert.deepEqual(queueReads, [
      "/workspace/a.xyzi", "/workspace/a.xyzi", "/workspace/b.xyzi", "/workspace/c.xyzi",
    ]);
    queue.settle(queuePosts[3].requestId);
    const catalog = new LayerReplayCatalog();
    const firstOverlay = vscode.Uri.parse(
      "vscode-remote://ssh-remote+fixture/workspace/first.xyzi",
    );
    const secondOverlay = vscode.Uri.parse(
      "vscode-remote://ssh-remote+fixture/workspace/second.xyzi",
    );
    catalog.record({
      pending: { uri: firstOverlay, requestId: 1 },
      sourceKey: firstSourceKey,
    });
    catalog.record({
      pending: { uri: secondOverlay, requestId: 2 },
      sourceKey: `sha256:${"b".repeat(64)}`,
    });
    const replay = catalog.replay(
      [{ uri: firstOverlay, requestId: 3 }],
      99,
    );
    assert.deepEqual(
      replay.map((entry) => [entry.uri.toString(), entry.requestId]),
      [
        [firstOverlay.toString(), 99],
        [secondOverlay.toString(), 99],
      ],
    );
    catalog.remove(firstSourceKey);
    assert.deepEqual(
      catalog.replay([], 100).map((entry) => entry.uri.toString()),
      [secondOverlay.toString()],
    );
    provider.readCount = 0;

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
