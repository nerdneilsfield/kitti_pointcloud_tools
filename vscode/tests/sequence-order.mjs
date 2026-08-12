import { build } from "esbuild";
import { readFile } from "node:fs/promises";

const result = await build({
  entryPoints: [new URL("../src/sequence-order.ts", import.meta.url).pathname],
  bundle: true,
  format: "esm",
  platform: "node",
  write: false,
});
const source = Buffer.from(result.outputFiles[0].contents).toString("base64");
const { createSequenceNameComparator } = await import(`data:text/javascript;base64,${source}`);

function expectOrder(input, expected) {
  const actual = [...input].sort(createSequenceNameComparator(input));
  if (actual.join("\n") !== expected.join("\n")) {
    throw new Error(`unexpected sequence order:\n${actual.join("\n")}`);
  }
}

const sharedFixtures = await readFile(
  new URL("../../tests/data/sequence-order-fixtures.tsv", import.meta.url),
  "utf8",
);
for (const line of sharedFixtures.split(/\r?\n/u)) {
  if (!line || line.startsWith("#")) continue;
  const [name, input, expected] = line.split("\t");
  if (!name || !input || !expected) throw new Error(`invalid fixture: ${line}`);
  try {
    expectOrder(input.split("|"), expected.split("|"));
  } catch (error) {
    throw new Error(`shared sequence fixture ${name} failed`, { cause: error });
  }
}

expectOrder(
  ["frame_10.bin", "frame_2.bin", "frame_1.bin"],
  ["frame_1.bin", "frame_2.bin", "frame_10.bin"],
);

const fallback = createSequenceNameComparator([]);
const cycleNames = ["a2.bin", "a100.1.bin", "b1.bin"];
for (const left of cycleNames) {
  for (const right of cycleNames) {
    const forward = Math.sign(fallback(left, right));
    const reverse = Math.sign(fallback(right, left));
    if (forward !== -reverse) throw new Error("fallback comparator is asymmetric");
  }
}
const raggedNames = [
  "cam_frame_1.bin", "cam_frame_10.bin", "cam_without_frame.bin",
  "other_frame_2.bin", "other_100.1.bin",
];
const raggedComparator = createSequenceNameComparator(raggedNames);
for (const left of raggedNames) {
  for (const middle of raggedNames) {
    for (const right of raggedNames) {
      if (raggedComparator(left, middle) <= 0 &&
          raggedComparator(middle, right) <= 0 &&
          raggedComparator(left, right) > 0) {
        throw new Error("catalog comparator is not transitive");
      }
    }
  }
}
const largeCatalog = Array.from({ length: 30_000 }, (_, index) =>
  `camera${index}part_frame${index}.bin`
);
const started = performance.now();
createSequenceNameComparator(largeCatalog);
if (performance.now() - started > 2_000) {
  throw new Error("sequence inference exceeds linear-time budget");
}
expectOrder(
  ["capture_1000.001_frame_10.bin", "capture_1000.003_frame_2.bin",
    "capture_999.999_frame_1.bin"],
  ["capture_999.999_frame_1.bin", "capture_1000.003_frame_2.bin",
    "capture_1000.001_frame_10.bin"],
);
expectOrder(
  ["scene_2_ts_100.1.bin", "scene_1_ts_100.4.bin",
    "scene_2_ts_100.2.bin", "scene_1_ts_100.3.bin"],
  ["scene_1_ts_100.3.bin", "scene_1_ts_100.4.bin",
    "scene_2_ts_100.1.bin", "scene_2_ts_100.2.bin"],
);
expectOrder(
  ["xxx1xxxyyy12yyxsjd732.bin", "xxx1xxxyyy12yyxsjd9.bin",
    "xxx1xxxyyy12yyxsjd101.bin"],
  ["xxx1xxxyyy12yyxsjd9.bin", "xxx1xxxyyy12yyxsjd101.bin",
    "xxx1xxxyyy12yyxsjd732.bin"],
);
