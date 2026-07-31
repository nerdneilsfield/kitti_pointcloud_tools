globalThis.KptWeb = (() => {
  const registry = new Map();
  const staged = new Map();
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

  const callSelection = (kind, paths, error = "") => {
    const payload = paths.join("\n");
    const payloadPtr = stringToNewUTF8(payload);
    const errorPtr = stringToNewUTF8(error);
    try {
      Module._kpt_web_selection_changed(kind, payloadPtr, errorPtr);
    } finally {
      _free(payloadPtr);
      _free(errorPtr);
    }
  };

  const ensureParent = (path) => {
    const slash = path.lastIndexOf("/");
    FS.mkdirTree(path.slice(0, slash));
  };

  const writeFile = async (path, file) => {
    ensureParent(path);
    const bytes = new Uint8Array(await file.arrayBuffer());
    FS.writeFile(path, bytes);
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
      }
      try {
        FS.unlink(path);
      } catch (reason) {
        if (!isMissingFileError(reason))
          errors.push(reason);
      }
    }
    if (errors.length)
      throw new AggregateError(errors, "failed to remove staged assets");
  };

  const acquirePath = async (path) => {
    const existing = staged.get(path);
    if (existing) {
      ++existing.references;
      return existing.promise;
    }
    const file = registry.get(path);
    if (!file)
      throw new Error(`Selected asset is no longer available: ${path}`);
    const entry = {
      references: 1,
      promise: writeFile(path, file),
    };
    staged.set(path, entry);
    try {
      await entry.promise;
    } catch (reason) {
      if (staged.get(path) === entry)
        staged.delete(path);
      throw reason;
    }
  };

  const select = async (kind, files) => {
    if (files.length === 0)
      return;
    if (!(kind in roots)) {
      callSelection(kind, [], `Invalid picker kind: ${kind}`);
      return;
    }
    const names = new Set();
    for (const file of files) {
      if (/[\r\n]/u.test(file.name)) {
        callSelection(kind, [], `Invalid filename: ${file.name}`);
        return;
      }
      if (names.has(file.name)) {
        callSelection(kind, [], `Duplicate filename: ${file.name}`);
        return;
      }
      names.add(file.name);
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
    for (const path of registry.keys()) {
      if (path.startsWith(`${root}/`))
        registry.delete(path);
    }
    const paths = files.map((file) => `${root}/${file.name}`);
    paths.forEach((path, index) => registry.set(path, files[index]));
    callSelection(kind, paths);
  };

  const pick = (kind) => {
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
    const paths = payload ? payload.split("\n") : [];
    let error = "";
    const results = await Promise.allSettled(paths.map(acquirePath));
    const failure = results.find((result) => result.status === "rejected");
    if (failure) {
      error = String(failure.reason);
      try {
        removePaths(paths);
      } catch (reason) {
        error += `; cleanup failed: ${String(reason)}`;
      }
    }
    const errorPtr = stringToNewUTF8(error);
    try {
      Module._kpt_web_stage_complete(requestId, errorPtr);
    } finally {
      _free(errorPtr);
    }
  };

  const release = (payload) => {
    try {
      removePaths(payload ? payload.split("\n") : []);
    } catch (reason) {
      console.error("Failed to release staged assets", reason);
      fatal(`Failed to release staged assets: ${String(reason)}`);
    }
  };

  const fatal = (message) => {
    const overlay = document.getElementById("fatal-error");
    document.getElementById("fatal-message").textContent = message;
    overlay.hidden = false;
  };

  return { pick, select, stage, release, fatal };
})();
