self.onmessage = (event) => {
  self.postMessage({
    type: "decodeStarted",
    requestId: event.data.requestId,
    frameIndex: event.data.frameIndex,
    generation: event.data.generation,
  });
  // Intentionally never respond. Exercises webview timeout recovery.
};
