globalThis.KptWeb = (() => {
  const registry = new Map();
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

  const select = async (kind, files) => {
    if (files.length === 0)
      return;
    const names = new Set();
    for (const file of files) {
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
    try {
      await Promise.all(paths.map(async (path) => {
        const file = registry.get(path);
        if (!file)
          throw new Error(`Selected asset is no longer available: ${path}`);
        await writeFile(path, file);
      }));
    } catch (reason) {
      error = String(reason);
    }
    const errorPtr = stringToNewUTF8(error);
    try {
      Module._kpt_web_stage_complete(requestId, errorPtr);
    } finally {
      _free(errorPtr);
    }
  };

  const release = (payload) => {
    for (const path of payload ? payload.split("\n") : []) {
      try {
        FS.unlink(path);
      } catch (_) {
        // A stale or cancelled request may already have removed its staging file.
      }
    }
  };

  const fatal = (message) => {
    const overlay = document.getElementById("fatal-error");
    overlay.textContent = message;
    overlay.hidden = false;
  };

  return { pick, stage, release, fatal };
})();
