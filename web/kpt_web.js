globalThis.KptWeb = (() => {
  const registry = new Map();
  const staged = new Map();
  let stagedBytes = 0;
  let pendingStageRequests = 0;
  const maxAssetBytes = 512 * 1024 * 1024;
  const maxStagedBytes = 768 * 1024 * 1024;
  const maxSelectionPayloadBytes = 1024 * 1024;
  const maxSelectionPaths = 20000;
  const maxSelectionPathBytes = 64 * 1024;
  const maxStageRequests = 64;
  const maxScreenshotBytes = 64 * 1024 * 1024;
  const roots = {
    0: "/kpt-import/viewer",
    1: "/kpt-import/clouds",
    2: "/kpt-import/labels",
    3: "/kpt-import/poses",
    4: "/kpt-import/poses2",
  };

  const extension = (name) => {
    const dot = name.lastIndexOf(".");
    return dot < 0 ? "" : name.slice(dot).toLowerCase();
  };

  const allowedClouds = new Set([
    ".bin", ".pcd", ".ply", ".xyz", ".xyzi", ".xyzrgb", ".xyzrgbi",
  ]);

  const hasRoot = (kind) =>
    Object.prototype.hasOwnProperty.call(roots, kind);

  const safeBasename = (name) =>
    typeof name === "string" && name.length > 0 && name.length <= 255 &&
    name !== "." && name !== ".." && !(/[\\/]/u.test(name)) &&
    !(/[\u0000-\u001f\u007f-\u009f]/u.test(name));

  const safeVirtualPath = (path) => {
    if (typeof path !== "string" || path.length > 1024)
      return false;
    const root = Object.values(roots).find((candidate) =>
      path.startsWith(`${candidate}/`));
    return root !== undefined && safeBasename(path.slice(root.length + 1));
  };

  const decodePaths = (payload) => {
    if (typeof payload !== "string")
      throw new Error("staging payload must be a string");
    if (Module.lengthBytesUTF8(payload) > maxSelectionPayloadBytes)
      throw new Error("staging payload exceeds 1 MiB limit");
    if (!payload)
      return [];
    const paths = payload.split("\n");
    if (paths[paths.length - 1] === "")
      paths.pop();
    if (paths.length > maxSelectionPaths)
      throw new Error("staging path count exceeds 20000 limit");
    const unique = new Set();
    for (const path of paths) {
      if (Module.lengthBytesUTF8(path) > maxSelectionPathBytes ||
          !safeVirtualPath(path) || unique.has(path))
        throw new Error("invalid or duplicate staging path");
      unique.add(path);
    }
    return paths;
  };

  const callSelection = (kind, paths, error = "") => {
    const payload = paths.join("\n");
    const payloadPtr = Module.stringToNewUTF8(payload);
    const errorPtr = Module.stringToNewUTF8(error);
    try {
      Module._kpt_web_selection_changed(
        kind,
        payloadPtr,
        Module.lengthBytesUTF8(payload),
        errorPtr,
        Module.lengthBytesUTF8(error),
      );
    } finally {
      Module._free(payloadPtr);
      Module._free(errorPtr);
    }
  };

  const ensureParent = (path) => {
    const slash = path.lastIndexOf("/");
    Module.FS.mkdirTree(path.slice(0, slash));
  };

  const writeFile = async (path, file) => {
    if (!safeVirtualPath(path) || !file ||
        !Number.isSafeInteger(file.size) || file.size < 0 ||
        file.size > maxAssetBytes)
      throw new Error("staged asset exceeds safety limits");
    ensureParent(path);
    const bytes = new Uint8Array(await file.arrayBuffer());
    if (bytes.byteLength !== file.size)
      throw new Error("staged asset size changed while reading");
    Module.FS.writeFile(path, bytes);
  };

  const isMissingFileError = (reason) =>
    reason && typeof reason === "object" && reason.errno === 44;

  const removePaths = (paths) => {
    const errors = [];
    for (const path of paths) {
      const entry = staged.get(path);
      if (entry) {
        --entry.references;
        if (entry.references > 0)
          continue;
        staged.delete(path);
        stagedBytes -= entry.bytes;
      }
      try {
        Module.FS.unlink(path);
      } catch (reason) {
        if (!isMissingFileError(reason))
          errors.push(reason);
      }
    }
    if (errors.length)
      throw new AggregateError(errors, "failed to remove staged assets");
  };

  const acquirePath = async (path) => {
    if (!safeVirtualPath(path))
      throw new Error("staging path is outside the virtual import root");
    const existing = staged.get(path);
    if (existing) {
      ++existing.references;
      return existing.promise;
    }
    const file = registry.get(path);
    if (!file)
      throw new Error(`Selected asset is no longer available: ${path}`);
    if (!Number.isSafeInteger(file.size) || file.size < 0 ||
        file.size > maxAssetBytes || stagedBytes > maxStagedBytes - file.size)
      throw new Error("staged asset working set exceeds safety limits");
    const entry = {
      references: 1,
      bytes: file.size,
      promise: writeFile(path, file),
    };
    staged.set(path, entry);
    stagedBytes += file.size;
    try {
      await entry.promise;
    } catch (reason) {
      if (staged.get(path) === entry) {
        staged.delete(path);
        stagedBytes -= entry.bytes;
      }
      throw reason;
    }
  };

  const select = async (kind, files) => {
    if (files.length === 0)
      return;
    if (!hasRoot(kind)) {
      callSelection(kind, [], `Invalid picker kind: ${kind}`);
      return;
    }
    if (files.length > maxSelectionPaths) {
      callSelection(kind, [], "Too many selected files");
      return;
    }
    const names = new Set();
    let selectedBytes = 0;
    for (const file of files) {
      if (!file || !safeBasename(file.name)) {
        callSelection(kind, [], "Invalid filename");
        return;
      }
      if (!Number.isSafeInteger(file.size) || file.size < 0 ||
          file.size > maxAssetBytes) {
        callSelection(kind, [], "Selected file violates safety limits");
        return;
      }
      if (names.has(file.name)) {
        callSelection(kind, [], `Duplicate filename: ${file.name}`);
        return;
      }
      names.add(file.name);
      if (selectedBytes > maxStagedBytes - file.size) {
        callSelection(kind, [], "Selected files exceed staging budget");
        return;
      }
      selectedBytes += file.size;
      if ((kind === 0 || kind === 1) && !allowedClouds.has(extension(file.name))) {
        callSelection(kind, [], `Unsupported point-cloud format: ${file.name}`);
        return;
      }
      if (kind === 2 && extension(file.name) !== ".label") {
        callSelection(kind, [], `Unsupported label format: ${file.name}`);
        return;
      }
    }

    const root = roots[kind];
    if ([...staged.keys()].some((path) => path.startsWith(`${root}/`))) {
      callSelection(
        kind,
        [],
        "Cannot replace files while selected assets are still in use",
      );
      return;
    }
    const paths = files.map((file) => `${root}/${file.name}`);
    if (Module.lengthBytesUTF8(paths.join("\n")) > maxSelectionPayloadBytes) {
      callSelection(kind, [], "Selected paths exceed payload limit");
      return;
    }
    for (const path of registry.keys()) {
      if (path.startsWith(`${root}/`))
        registry.delete(path);
    }
    paths.forEach((path, index) => registry.set(path, files[index]));
    callSelection(kind, paths);
  };

  const pick = (kind) => {
    if (!hasRoot(kind)) {
      callSelection(kind, [], `Invalid picker kind: ${kind}`);
      return;
    }
    const input = document.createElement("input");
    input.type = "file";
    input.multiple = kind === 1 || kind === 2;
    if (kind === 0 || kind === 1)
      input.accept = ".bin,.pcd,.ply,.xyz,.xyzi,.xyzrgb,.xyzrgbi";
    else if (kind === 2)
      input.accept = ".label";
    else
      input.accept = ".txt,.csv";
    input.addEventListener("change", () => select(kind, [...input.files]));
    input.click();
  };

  const stage = async (payload, requestId) => {
    let error = "";
    let paths = [];
    const validRequest = Number.isSafeInteger(requestId) &&
      requestId > 0 && requestId <= 0xffffffff;
    if (!validRequest) {
      error = "invalid staging request id";
    } else if (pendingStageRequests >= maxStageRequests) {
      error = "staging request queue is full";
    } else {
      ++pendingStageRequests;
      try {
        paths = decodePaths(payload);
        const results = await Promise.allSettled(paths.map(acquirePath));
        const failure = results.find((result) => result.status === "rejected");
        if (failure)
          error = String(failure.reason);
        if (failure) {
          try {
            removePaths(paths);
          } catch (reason) {
            error += `; cleanup failed: ${String(reason)}`;
          }
        }
      } catch (reason) {
        error = String(reason);
        if (paths.length) {
          try {
            removePaths(paths);
          } catch (cleanupReason) {
            error += `; cleanup failed: ${String(cleanupReason)}`;
          }
        }
      } finally {
        --pendingStageRequests;
      }
    }
    const errorPtr = Module.stringToNewUTF8(error);
    try {
      Module._kpt_web_stage_complete(
        requestId, errorPtr, Module.lengthBytesUTF8(error),
      );
    } finally {
      Module._free(errorPtr);
    }
  };

  const release = (payload) => {
    try {
      removePaths(decodePaths(payload ?? ""));
    } catch (reason) {
      console.error("Failed to release staged assets", reason);
      fatal(`Failed to release staged assets: ${String(reason)}`);
    }
  };

  const completePngDownload = (requestId, error = "") => {
    let errorPtr = 0;
    try {
      const message = typeof error === "string" ? error : String(error);
      errorPtr = Module.stringToNewUTF8(message);
      Module._kpt_web_viewport_png_complete(
        requestId, errorPtr, Module.lengthBytesUTF8(message),
      );
      // toBlob completes outside input handling. Wake a throttled workbench so
      // App can drain and display this result without waiting for its idle tick.
      if (typeof Module._kpt_web_wake_main_loop === "function")
        Module._kpt_web_wake_main_loop();
    } catch (reason) {
      // This can only happen for a mismatched WASM/JS deployment. Keep it
      // visible instead of pretending that an asynchronous download succeeded.
      console.error("Failed to report KPT PNG download result", reason);
    } finally {
      if (errorPtr) Module._free(errorPtr);
    }
  };

  // Copy WASM-owned top-left RGBA rows before returning.  Canvas PNG encoding
  // is asynchronous, so a direct HEAPU8 view would become invalid as soon as
  // C++ releases or reallocates its capture vector.
  const downloadPng = (
    name, sourcePtr, sourceBytes, width, height, bytesPerRow, requestId,
  ) => {
    try {
      if (!safeBasename(name) || !name.endsWith(".png") ||
          !Number.isSafeInteger(sourcePtr) || sourcePtr < 0 ||
          !Number.isSafeInteger(sourceBytes) || sourceBytes <= 0 ||
          sourceBytes > maxScreenshotBytes ||
          !Number.isSafeInteger(width) || !Number.isSafeInteger(height) ||
          width <= 0 || height <= 0 ||
          width > Math.floor(Number.MAX_SAFE_INTEGER / 4) ||
          !Number.isSafeInteger(bytesPerRow) || bytesPerRow < width * 4 ||
          height > Math.floor(sourceBytes / bytesPerRow) ||
          sourcePtr > Module.HEAPU8.length ||
          sourceBytes > Module.HEAPU8.length - sourcePtr ||
          !Number.isSafeInteger(requestId) || requestId <= 0 ||
          requestId > 0xffffffff)
        return false;
      const packedRow = width * 4;
      if (packedRow > maxScreenshotBytes ||
          height > Math.floor(maxScreenshotBytes / packedRow))
        return false;
      const requiredBytes = bytesPerRow * height;
      if (requiredBytes > sourceBytes) return false;

      const image = new ImageData(width, height);
      const source = Module.HEAPU8.subarray(
        sourcePtr, sourcePtr + requiredBytes,
      );
      for (let row = 0; row < height; ++row) {
        image.data.set(
          source.subarray(row * bytesPerRow, row * bytesPerRow + packedRow),
          row * packedRow,
        );
      }
      const canvas = document.createElement("canvas");
      canvas.width = width;
      canvas.height = height;
      const context = canvas.getContext("2d");
      if (!context) return false;
      context.putImageData(image, 0, 0);
      canvas.toBlob((blob) => {
        if (!blob) {
          completePngDownload(requestId, "browser PNG encoder returned no data");
          return;
        }
        let url = "";
        let anchor = null;
        try {
          url = URL.createObjectURL(blob);
          anchor = document.createElement("a");
          anchor.href = url;
          anchor.download = name;
          anchor.hidden = true;
          document.body.append(anchor);
          anchor.click();
          // Browser accepted the handoff; it may still apply user download
          // settings afterwards, so this remains a queued/started result.
          completePngDownload(requestId);
        } catch (reason) {
          console.error("Failed to queue KPT PNG download", reason);
          const message = String(reason);
          completePngDownload(
            requestId,
            message.length <= 4096 ? message : "browser could not queue PNG download",
          );
        } finally {
          if (anchor) anchor.remove();
          if (url)
            globalThis.setTimeout(() => URL.revokeObjectURL(url), 0);
        }
      }, "image/png");
      return true;
    } catch (reason) {
      console.error("Failed to start KPT PNG download", reason);
      return false;
    }
  };

  const fatal = (message) => {
    const overlay = document.getElementById("fatal-error");
    document.getElementById("fatal-message").textContent = message;
    overlay.hidden = false;
  };

  return { pick, select, stage, release, downloadPng, fatal };
})();
