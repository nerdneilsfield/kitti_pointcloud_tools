import * as assert from "node:assert/strict";
import * as vscode from "vscode";
import {
  beginReviewSourceReattachment,
  clearPendingManualReviewSource,
  decodeWebviewMessage,
  createHostReviewSession,
  LayerReplayCatalog,
  LayerPayloadQueue,
  markReviewLayerResolved,
  readLayerSource,
  readReviewShareBounded,
  relativeUriPath,
  serializeSourceUri,
  sourceKeyForUri,
  reviewSessionLayerPayloads,
} from "../src/extension";
import type { ExtensionApi, ExtensionRenderEvent } from "../src/extension";
import { convertPointCloud } from "../src/converter";
import {
  hasPortableCameraFov,
  hasValidSourceKeyByteLength,
  maximumSourceKeyBytes,
} from "../src/protocol";
import {
  encodeReviewShare,
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
    const size = uri.path.endsWith("oversized.json")
      ? 4 * 1024 * 1024 + 1
      : this.bytesFor(uri).byteLength;
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
      decodeWebviewMessage({
        type: "locateReviewSource", requestId: 18, sourceKey: firstSourceKey,
        uri: "file:///client-only/never-trusted.xyzi",
      }),
      { type: "locateReviewSource", requestId: 18, sourceKey: firstSourceKey },
    );
    assert.equal(
      decodeWebviewMessage({
        type: "locateReviewSource", requestId: 18, sourceKey: "path:/never-trusted.xyzi",
      }),
      undefined,
    );
    assert.deepEqual(
      decodeWebviewMessage({ type: "importReviewShare", requestId: 19 }),
      { type: "importReviewShare", requestId: 19 },
    );
    assert.equal(validRelativeSharePath("clouds/scan.xyzi"), true);
    assert.equal(validRelativeSharePath("../scan.xyzi"), false);
    const legacyFixtureUri = vscode.Uri.joinPath(
      extension.extensionUri, "tests", "fixtures", "native-review-share.json",
    );
    const legacyReviewBytes = await vscode.workspace.fs.readFile(legacyFixtureUri);
    assert.throws(
      () => parseReviewShare(legacyReviewBytes),
      /schema/u,
    );
    const nativeFixtureUri = vscode.Uri.joinPath(
      extension.extensionUri, "tests", "fixtures",
      "review-share-v2-cross-runtime.json",
    );
    const nativeReview = parseReviewShare(
      await vscode.workspace.fs.readFile(nativeFixtureUri),
    );
    assert.equal(nativeReview.schema_version, 2);
    assert.equal(nativeReview.layers[0].source_key, "path:/srv/kitti/intensity.xyzi");
    assert.deepEqual(
      nativeReview.layers.map((layer) => layer.style.color_by),
      [0, 1, 2, 3, 4],
    );
    // Native uses full affine matrices, not only decomposable TRS. Keep this
    // reflection + shear fixture exact through extension parsing/sanitizing.
    assert.deepEqual(nativeReview.layers[0].local_to_world, [
      [-1, 0.25, 0, 12], [0, 1, 0.5, -3], [0, 0, 1, 4], [0, 0, 0, 1],
    ]);
    assert.deepEqual(nativeReview.roi, {
      minimum: [-1, -2, -3], maximum: [4, 5, 6],
    });
    assert.equal(nativeReview.bookmarks[0].camera.distance, 12);
    // `camera_to_world` is a native CameraSnapshot rotation, not a generic
    // 3×3 transform. Keep wire validation aligned with native's finite,
    // orthonormal, right-handed restore contract.
    const nativeRelativeAccepted = structuredClone(nativeReview) as typeof nativeReview;
    nativeRelativeAccepted.bookmarks[0].camera.camera_to_world = [
      [Math.sqrt(1.00015), 0, 0], [0, 1, 0], [0, 0, 1],
    ];
    assert.equal(validateReviewShare(nativeRelativeAccepted), true);
    // Native parses fov_y_degrees as float before its [0.01, 180) test.
    // Its float boundaries, rather than JavaScript double comparisons, define
    // the portable wire contract.
    assert.equal(hasPortableCameraFov(1e-45), false);
    assert.equal(hasPortableCameraFov(0.001), false);
    assert.equal(hasPortableCameraFov(0.01), true);
    // 179.999999 rounds to exactly 180f; the preceding float value remains
    // valid, so wire validation must make the same distinction.
    assert.equal(hasPortableCameraFov(179.999999), false);
    assert.equal(hasPortableCameraFov(179.99999), true);
    const fovRoundedToNativeLimit = structuredClone(nativeReview) as typeof nativeReview;
    fovRoundedToNativeLimit.bookmarks[0].camera.fov_y_degrees = 179.999999;
    assert.equal(validateReviewShare(fovRoundedToNativeLimit), false);
    const fovBelowNativeLimit = structuredClone(nativeReview) as typeof nativeReview;
    fovBelowNativeLimit.bookmarks[0].camera.fov_y_degrees = 179.99999;
    assert.equal(validateReviewShare(fovBelowNativeLimit), true);
    const fovBelowPortableMinimum = structuredClone(nativeReview) as typeof nativeReview;
    fovBelowPortableMinimum.bookmarks[0].camera.fov_y_degrees = 0.001;
    assert.equal(validateReviewShare(fovBelowPortableMinimum), false);
    const fovAtPortableMinimum = structuredClone(nativeReview) as typeof nativeReview;
    fovAtPortableMinimum.bookmarks[0].camera.fov_y_degrees = 0.01;
    assert.equal(validateReviewShare(fovAtPortableMinimum), true);
    const distributedNearSkew = structuredClone(nativeReview) as typeof nativeReview;
    const nearSkew = 4.95e-5;
    distributedNearSkew.bookmarks[0].camera.camera_to_world = [
      [1, nearSkew, nearSkew], [nearSkew, 1, nearSkew], [nearSkew, nearSkew, 1],
    ];
    assert.equal(validateReviewShare(distributedNearSkew), false);
    for (const basis of [
      [[1, 0.25, 0], [0, 1, 0], [0, 0, 1]], // skew
      [[0, 0, 0], [0, 0, 0], [0, 0, 0]], // zero basis
      [[-1, 0, 0], [0, 1, 0], [0, 0, 1]], // reflection
      [[Number.POSITIVE_INFINITY, 0, 0], [0, 1, 0], [0, 0, 1]],
    ]) {
      const invalidCamera = structuredClone(nativeReview) as typeof nativeReview;
      invalidCamera.bookmarks[0].camera.camera_to_world = basis;
      assert.equal(validateReviewShare(invalidCamera), false);
    }
    const largestRenderableCamera = structuredClone(nativeReview) as typeof nativeReview;
    largestRenderableCamera.bookmarks[0].camera.target = [1e30, -1e30, 1e30];
    largestRenderableCamera.bookmarks[0].camera.rotation_center = [1e30, -1e30, 1e30];
    largestRenderableCamera.bookmarks[0].camera.distance = 1e30;
    assert.equal(validateReviewShare(largestRenderableCamera), true);
    for (const mutate of [
      (camera: typeof nativeReview.bookmarks[number]["camera"]) => {
        camera.target = [Number.MAX_VALUE, 0, 0];
      },
      (camera: typeof nativeReview.bookmarks[number]["camera"]) => {
        camera.rotation_center = [0, -Number.MAX_VALUE, 0];
      },
      (camera: typeof nativeReview.bookmarks[number]["camera"]) => {
        camera.distance = Number.MAX_VALUE;
      },
    ]) {
      const nonRenderableCamera = structuredClone(nativeReview) as typeof nativeReview;
      mutate(nonRenderableCamera.bookmarks[0].camera);
      assert.equal(validateReviewShare(nonRenderableCamera), false);
    }
    const dbMaxCamera = structuredClone(nativeReview) as typeof nativeReview;
    dbMaxCamera.bookmarks[0].camera.target = [Number.MAX_VALUE, 0, 0];
    assert.throws(
      () => parseReviewShare(new TextEncoder().encode(JSON.stringify(dbMaxCamera))),
      /schema/u,
    );
    for (const mutate of [
      (style: Record<string, unknown>) => { style.color_by = 5; },
      (style: Record<string, unknown>) => { style.point_size = 0; },
      (style: Record<string, unknown>) => { style.scalar_min = 2; style.scalar_max = 1; },
      (style: Record<string, unknown>) => { style.fixed_color = [1.1, 0, 0]; },
    ]) {
      const invalidStyle = structuredClone(nativeReview) as typeof nativeReview;
      mutate(invalidStyle.layers[0].style as unknown as Record<string, unknown>);
      assert.equal(validateReviewShare(invalidStyle), false);
    }
    const opaqueUtf8FixtureUri = vscode.Uri.joinPath(
      extension.extensionUri, "tests", "fixtures",
      "native-review-share-opaque-utf8.json",
    );
    const opaqueUtf8Review = parseReviewShare(
      await vscode.workspace.fs.readFile(opaqueUtf8FixtureUri),
    );
    assert.equal(opaqueUtf8Review.layers[0].source_key, "opaque:点云/🔭");
    // Source-key capacity is a UTF-8 byte contract, not a JavaScript UTF-16
    // code-unit count. These exact-limit identities include BMP and astral
    // scalars so a cross-runtime implementation cannot count `.length`.
    const exactOpaqueSourceKey =
      `opaque:${"点".repeat(5_455)}${"🔭".repeat(3)}`;
    const exactPathSourceKey = `path:/${"点".repeat(5_458)}🔭`;
    const oversizedOpaqueSourceKey = `${exactOpaqueSourceKey}x`;
    for (const sourceKey of [exactOpaqueSourceKey, exactPathSourceKey]) {
      assert.equal(
        new TextEncoder().encode(sourceKey).byteLength,
        maximumSourceKeyBytes,
      );
      assert.equal(hasValidSourceKeyByteLength(sourceKey), true);
    }
    assert.equal(
      hasValidSourceKeyByteLength(`sha256:${"a".repeat(64)}`),
      true,
    );
    assert.equal(
      new TextEncoder().encode(oversizedOpaqueSourceKey).byteLength,
      maximumSourceKeyBytes + 1,
    );
    assert.equal(hasValidSourceKeyByteLength(oversizedOpaqueSourceKey), false);
    const reviewWithSourceKey = (sourceKey: string) => ({
      ...nativeReview,
      layers: [{ ...nativeReview.layers[0], source_key: sourceKey }],
      measurements: [{
        ...nativeReview.measurements[0],
        first_source_key: sourceKey,
      }],
    });
    for (const sourceKey of [exactOpaqueSourceKey, exactPathSourceKey]) {
      const boundaryReview = reviewWithSourceKey(sourceKey);
      assert.equal(validateReviewShare(boundaryReview), true);
      assert.equal(
        parseReviewShare(new TextEncoder().encode(JSON.stringify(boundaryReview)))
          .layers[0].source_key,
        sourceKey,
      );
    }
    const oversizedReview = reviewWithSourceKey(oversizedOpaqueSourceKey);
    assert.equal(validateReviewShare(oversizedReview), false);
    assert.throws(
      () => parseReviewShare(
        new TextEncoder().encode(JSON.stringify(oversizedReview)),
      ),
      /schema/u,
    );
    const opaqueC1FixtureUri = vscode.Uri.joinPath(
      extension.extensionUri, "tests", "fixtures",
      "native-review-share-opaque-c1-invalid.json",
    );
    await assert.rejects(
      async () => parseReviewShare(
        await vscode.workspace.fs.readFile(opaqueC1FixtureUri),
      ),
      /schema/u,
    );
    // JSON accepts an escaped lone surrogate; portable source keys do not.
    const malformedOpaqueJson = JSON.stringify(opaqueUtf8Review).replaceAll(
      "opaque:点云/🔭", "opaque:\\ud800",
    );
    assert.throws(
      () => parseReviewShare(new TextEncoder().encode(malformedOpaqueJson)),
      /schema/u,
    );
    assert.throws(() => parseReviewShare(new TextEncoder().encode(JSON.stringify({
      ...nativeReview,
      layers: [{ ...nativeReview.layers[0], source_path: "../escape.xyzi" }],
    }))), /schema/u);
    const nativeSession = await createHostReviewSession(
      vscode.Uri.parse("kpt-test://ssh-remote+fixture/workspace/reviews/review.json"),
      nativeReview,
    );
    assert.ok(nativeSession.generation > 0);
    const nativeTransportKey = nativeSession.state.layers[0].source_key;
    assert.match(nativeTransportKey, /^sha256:[a-f0-9]{64}$/u);
    assert.notEqual(nativeTransportKey, nativeReview.layers[0].source_key);
    assert.equal(
      nativeSession.originalKeyForAlias.get(nativeTransportKey),
      nativeReview.layers[0].source_key,
    );
    assert.equal(nativeSession.layers[0].uri?.scheme, "kpt-test");
    assert.deepEqual(
      nativeSession.state.layers.map((layer) => layer.style.color_by),
      [0, 1, 2, 3, 4],
    );
    assert.deepEqual(
      nativeSession.state.layers.map((layer) => [
        layer.style.scalar_min, layer.style.scalar_max,
      ]),
      [[-4, 17], [0, 1], [-20, 40], [0, 255], [8, 8]],
    );
    // Schema v2 is lossless at the host boundary. Native Label=3 and
    // None=4 must not be rewritten to the webview shader's fixed mode.
    assert.deepEqual(
      parseReviewShare(encodeReviewShare(nativeReview)).layers.map(
        (layer) => layer.style.color_by,
      ),
      [0, 1, 2, 3, 4],
    );
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
    // A user can explicitly reattach an unresolved native share source on a
    // different Remote URI. Its URI hash is not proof of the share's logical
    // identity; the host deliberately sends the imported transport key.
    const unresolvedReview = {
      ...nativeReview,
      layers: [{ ...nativeReview.layers[0], source_path: null }],
      measurements: [],
    };
    const unresolvedSession = await createHostReviewSession(
      vscode.Uri.parse("vscode-remote://ssh-remote+fixture/workspace/reviews/review.json"),
      unresolvedReview,
    );
    const unresolvedLayer = unresolvedSession.layers[0];
    assert.equal(unresolvedLayer.uri, undefined);
    const locatedRemoteUri = vscode.Uri.parse(
      "vscode-remote://ssh-remote+fixture/workspace/relocated/scan.xyzi",
    );
    const locatedUriHash = await sourceKeyForUri(locatedRemoteUri);
    assert.notEqual(locatedUriHash, unresolvedLayer.state.source_key);
    const locatedPending = beginReviewSourceReattachment(
      unresolvedSession,
      unresolvedSession,
      unresolvedLayer.state.source_key,
      locatedRemoteUri,
      52,
    );
    assert.ok(locatedPending);
    assert.equal(unresolvedLayer.uri?.toString(), locatedRemoteUri.toString());
    assert.equal(unresolvedLayer.manuallyLocated, true);
    // Reload can discard an in-flight payload before `rendered`; it rebuilds
    // from the early host binding, not from a URI hash or webview state.
    const replayBeforeRendered = reviewSessionLayerPayloads(unresolvedSession, 53);
    assert.equal(replayBeforeRendered.length, 1);
    assert.equal(replayBeforeRendered[0].uri.toString(), locatedRemoteUri.toString());
    assert.equal(replayBeforeRendered[0].reviewLayer, unresolvedLayer.state);
    assert.equal(replayBeforeRendered[0].sessionGeneration, unresolvedSession.generation);
    assert.equal(replayBeforeRendered[0].manuallyLocated, true);
    const supersedingSession = await createHostReviewSession(
      vscode.Uri.parse("vscode-remote://ssh-remote+fixture/workspace/reviews/new-review.json"),
      unresolvedReview,
    );
    assert.ok(supersedingSession.generation > unresolvedSession.generation);
    assert.equal(
      beginReviewSourceReattachment(
        supersedingSession,
        unresolvedSession,
        unresolvedLayer.state.source_key,
        locatedRemoteUri,
        54,
      ),
      undefined,
    );
    let locatedMessage: {
      sourceKey: string;
      reviewLayer?: unknown;
      sessionGeneration?: number;
    } | undefined;
    const locatedQueue = new LayerPayloadQueue(
      async () => ({
        sourceKey: locatedUriHash,
        name: "scan.xyzi",
        bytes: new ArrayBuffer(4),
      }),
      async (message) => { locatedMessage = message; },
      async () => assert.fail("reattached Remote source should not fail"),
      () => 2_000_001_000,
      () => false,
    );
    locatedQueue.enqueue([locatedPending]);
    await waitFor(() => locatedMessage !== undefined, 1_000);
    assert.equal(locatedMessage?.sourceKey, unresolvedLayer.state.source_key);
    assert.equal(locatedMessage?.reviewLayer, unresolvedLayer.state);
    assert.equal(locatedMessage?.sessionGeneration, unresolvedSession.generation);
    const locatedSettled = locatedQueue.settle(2_000_001_000);
    assert.ok(locatedSettled);
    assert.equal(markReviewLayerResolved(unresolvedSession, locatedSettled), true);
    assert.equal(unresolvedSession.layers[0].uri?.toString(), locatedRemoteUri.toString());
    const reattachedCatalog = new LayerReplayCatalog();
    reattachedCatalog.record(locatedSettled);
    assert.equal(
      reattachedCatalog.uriFor(unresolvedLayer.state.source_key)?.toString(),
      locatedRemoteUri.toString(),
    );
    const failedReview = {
      ...nativeReview,
      layers: [{ ...nativeReview.layers[0], source_path: null }],
      measurements: [],
    };
    const failedSession = await createHostReviewSession(
      vscode.Uri.parse("vscode-remote://ssh-remote+fixture/workspace/reviews/failed.json"),
      failedReview,
    );
    const failedPending = beginReviewSourceReattachment(
      failedSession,
      failedSession,
      failedSession.layers[0].state.source_key,
      locatedRemoteUri,
      55,
    );
    assert.ok(failedPending);
    assert.equal(clearPendingManualReviewSource(failedSession, failedPending), true);
    assert.equal(failedSession.layers[0].uri, undefined);
    assert.equal(failedSession.layers[0].manuallyLocated, false);
    // A user Add after Review Share import is registered in the host-owned
    // URI catalog before decoding. Reload merges this catalog after imported
    // session layers, even when it cancelled the first payload pre-render.
    const manualAddSession = await createHostReviewSession(
      vscode.Uri.parse("vscode-remote://ssh-remote+fixture/workspace/reviews/manual.json"),
      unresolvedReview,
    );
    const manualAddUri = vscode.Uri.parse(
      "vscode-remote://ssh-remote+fixture/workspace/manual/added.xyzi",
    );
    const manualAddKey = await sourceKeyForUri(manualAddUri);
    const manualCatalog = new LayerReplayCatalog();
    manualCatalog.recordSource(manualAddKey, manualAddUri);
    const replayManualAdd = reviewSessionLayerPayloads(
      manualAddSession, 57, manualCatalog,
    );
    assert.equal(replayManualAdd.length, 1);
    assert.equal(replayManualAdd[0].uri.toString(), manualAddUri.toString());
    assert.equal(replayManualAdd[0].reviewLayer, undefined);
    assert.equal(replayManualAdd[0].sessionGeneration, manualAddSession.generation);
    let manualReplayMessage: { sourceKey: string; sessionGeneration?: number;
      reviewLayer?: unknown } | undefined;
    const manualReplayQueue = new LayerPayloadQueue(
      async () => ({ sourceKey: manualAddKey, name: "added.xyzi", bytes: new ArrayBuffer(4) }),
      async (message) => { manualReplayMessage = message; },
      async () => assert.fail("manual catalog replay should not fail"),
      () => 2_000_001_001,
      () => false,
    );
    manualReplayQueue.enqueue(replayManualAdd);
    await waitFor(() => manualReplayMessage !== undefined, 1_000);
    assert.equal(manualReplayMessage?.sourceKey, manualAddKey);
    assert.equal(manualReplayMessage?.sessionGeneration, manualAddSession.generation);
    assert.equal(manualReplayMessage?.reviewLayer, undefined);
    // The old in-flight payload can be cancelled by Reload before rendered.
    // Catalog state, unlike the transient queue entry, still rebuilds it.
    assert.equal(manualReplayQueue.drainForReplay().length, 1);
    assert.equal(reviewSessionLayerPayloads(
      manualAddSession, 58, manualCatalog,
    ).length, 1);
    assert.equal(manualCatalog.uriFor(manualAddKey)?.scheme, "vscode-remote");
    assert.doesNotMatch(JSON.stringify(manualAddSession.state), /workspace\/manual/u);
    manualCatalog.remove(manualAddKey);
    assert.equal(reviewSessionLayerPayloads(
      manualAddSession, 59, manualCatalog,
    ).length, 0);
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
    const readsBeforeOversizedShare = provider.readCount;
    await assert.rejects(
      readReviewShareBounded(vscode.Uri.parse(
        "kpt-test://ssh-remote+fixture/workspace/reviews/oversized.json",
      )),
      /exceeds 4 MiB/u,
    );
    assert.equal(provider.readCount, readsBeforeOversizedShare);
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
