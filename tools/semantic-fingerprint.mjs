#!/usr/bin/env node
/**
 * Incremental, source-only semantic fingerprint generator.
 *
 * This deliberately has no dependency graph and no platform compiler model.
 * C++/TS are canonicalized from repository source syntax; packaging resources
 * are direct inputs only. State remains local under .git/.
 */
import { createHash } from "node:crypto";
import { existsSync, mkdirSync, readFileSync, rmSync, writeFileSync } from "node:fs";
import { dirname, join, relative, resolve } from "node:path";
import { execFileSync } from "node:child_process";

const root = resolve(import.meta.dirname, "..");
const statePath = join(root, ".git", "kpt-semantic-cache", "state-v1.json");
const outputDir = join(root, ".github", "semantic-fingerprints");
const mode = new Set(process.argv.slice(2));
const bootstrap = mode.has("--bootstrap");
const check = mode.has("--check");
if ([...mode].some((item) => !["--bootstrap", "--check"].includes(item))) {
  throw new Error("usage: semantic-fingerprint.mjs [--bootstrap] [--check]");
}

const stages = {
  "release.deb_22": ["native", "linux-package", "version", "package-doc"],
  "release.deb_24": ["native", "linux-package", "version", "package-doc"],
  "release.deb_26": ["native", "linux-package", "version", "package-doc"],
  "release.windows": ["native", "windows-package", "version", "package-doc"],
  "release.macos": ["native", "macos-package", "version", "package-doc"],
  "release.vsix": ["wasm", "vscode", "version"],
  "release.publish": ["publish"],
  "test.native": ["native", "native-test"],
  "test.asan_ubsan": ["native", "asan-test"],
  "test.tsan": ["native", "tsan-test"],
  "test.browser": ["wasm", "vscode", "browser-test"],
  "test.electron": ["wasm", "vscode", "electron-test"],
};

const sha256 = (value) => createHash("sha256").update(value).digest("hex");
const git = (args, options = {}) => execFileSync("git", args, {
  cwd: root,
  encoding: "utf8",
  stdio: ["ignore", "pipe", "pipe"],
  ...options,
});

function trackedFiles() {
  return git(["ls-files", "-z"]).split("\0").filter(Boolean).sort();
}

function stagedFiles() {
  const raw = git(["diff", "--cached", "--name-status", "-z"]);
  const fields = raw.split("\0");
  const files = new Set();
  for (let index = 0; index < fields.length - 1;) {
    const status = fields[index++];
    if (!status) continue;
    if (status.startsWith("R") || status.startsWith("C")) {
      files.add(fields[index++]);
      files.add(fields[index++]);
    } else {
      files.add(fields[index++]);
    }
  }
  return files;
}

function stagedText(file) {
  try {
    return git(["show", `:${file}`]);
  } catch {
    return undefined;
  }
}

// Lexing makes comment/format-only changes inert without interpreting includes.
function tokens(source) {
  const result = [];
  let index = 0;
  while (index < source.length) {
    const current = source[index];
    const next = source[index + 1];
    if (/\s/.test(current)) { index++; continue; }
    if (current === "/" && next === "/") {
      index = source.indexOf("\n", index + 2);
      if (index < 0) break;
      continue;
    }
    if (current === "/" && next === "*") {
      index = source.indexOf("*/", index + 2);
      if (index < 0) throw new Error("unterminated block comment");
      index += 2;
      continue;
    }
    if (current === '"' || current === "'" || current === "`") {
      const quote = current;
      let token = quote;
      index++;
      for (; index < source.length; index++) {
        token += source[index];
        if (source[index] === "\\" && index + 1 < source.length) token += source[++index];
        else if (source[index] === quote) { index++; break; }
      }
      result.push(token);
      continue;
    }
    const match = source.slice(index).match(/^(?:[A-Za-z_$][\w$]*|(?:0x[\da-fA-F]+|\d+(?:\.\d*)?(?:[eE][+-]?\d+)?)|===|!==|=>|::|->|\+\+|--|&&|\|\||<<|>>|<=|>=|==|!=|\+=|-=|\*=|\/=|\.\.\.|.)/);
    if (!match) throw new Error(`cannot tokenize at ${index}`);
    result.push(match[0]);
    index += match[0].length;
  }
  return result;
}

const reserved = new Set((
  "alignas alignof asm auto bool break case catch char char8_t char16_t char32_t class " +
  "concept const consteval constexpr constinit const_cast continue co_await co_return co_yield " +
  "decltype default delete do double dynamic_cast else enum explicit export extern false float for " +
  "friend goto if inline int long mutable namespace new noexcept nullptr operator private protected " +
  "public register reinterpret_cast requires return short signed sizeof static static_assert static_cast " +
  "struct switch template this thread_local throw true try typedef typeid typename union unsigned using " +
  "virtual void volatile wchar_t while let var function interface extends implements import from async await " +
  "yield readonly declare keyof typeof instanceof in of null undefined any unknown never number string boolean"
).split(" "));
const declarationLead = new Set((
  "auto bool char char8_t char16_t char32_t const constexpr constinit double float int long short signed " +
  "static unsigned void wchar_t let var readonly declare"
).split(" "));
const controlBeforeParen = new Set(["if", "for", "while", "switch", "catch"]);
const identifier = (token) => /^[A-Za-z_$][\w$]*$/.test(token) && !reserved.has(token);

function matching(tokens_, start, open = "(", close = ")") {
  let depth = 0;
  for (let index = start; index < tokens_.length; index++) {
    if (tokens_[index] === open) depth++;
    if (tokens_[index] === close && --depth === 0) return index;
  }
  return -1;
}

function parameterNames(tokens_, start, end) {
  const names = [];
  let segmentStart = start + 1;
  let depth = 0;
  const collect = (from, to) => {
    const candidates = [];
    for (let index = from; index < to; index++) {
      if (identifier(tokens_[index]) && tokens_[index - 1] !== "." && tokens_[index - 1] !== "::") candidates.push(tokens_[index]);
      if (tokens_[index] === "=") break;
    }
    if (candidates.length) names.push(candidates.at(-1));
  };
  for (let index = start + 1; index <= end; index++) {
    const token = tokens_[index];
    if (["(", "[", "<", "{"].includes(token)) depth++;
    if ([")", "]", ">", "}"].includes(token)) depth--;
    if ((token === "," && depth === 0) || index === end) {
      collect(segmentStart, index);
      segmentStart = index + 1;
    }
  }
  return names;
}

// Binding-aware only for lexical locals/parameters. Global/exported/member
// names remain literal tokens, so a public API rename still changes a digest.
function alphaNormalize(tokens_) {
  const functionScopes = new Map();
  for (let index = 0; index < tokens_.length; index++) {
    if (tokens_[index] !== "(") continue;
    const before = tokens_[index - 1];
    if (!identifier(before) || controlBeforeParen.has(before)) continue;
    const end = matching(tokens_, index);
    if (end < 0) continue;
    let body = end + 1;
    while (["const", "noexcept", "->", "requires"].includes(tokens_[body])) body++;
    if (tokens_[body] === "{") functionScopes.set(body, parameterNames(tokens_, index, end));
  }

  const scopes = [new Map()];
  let nextBinding = 0;
  const lookup = (name) => {
    for (let index = scopes.length - 1; index >= 0; index--) {
      const value = scopes[index].get(name);
      if (value) return value;
    }
    return undefined;
  };
  const bind = (name) => {
    if (!identifier(name) || scopes.at(-1).has(name)) return;
    scopes.at(-1).set(name, `$${nextBinding++}`);
  };
  const result = [];
  for (let index = 0; index < tokens_.length; index++) {
    const token = tokens_[index];
    if (token === "{") {
      const scope = new Map();
      scopes.push(scope);
      for (const name of functionScopes.get(index) ?? []) bind(name);
      result.push(token);
      continue;
    }
    if (token === "}") {
      result.push(token);
      if (scopes.length > 1) scopes.pop();
      continue;
    }
    // Common local declarations: `const auto value`, `int value`, and
    // `PointCloud value`. This deliberately avoids declaration lookup.
    const prior = tokens_[index - 1];
    const next = tokens_[index + 1];
    const statementStart = index === 0 || [";", "{", "(", ","].includes(prior);
    const typedName = identifier(token) && (
      declarationLead.has(prior) ||
      (identifier(prior) && /^[A-Z]/.test(prior) && statementStart === false) ||
      (index >= 2 && declarationLead.has(tokens_[index - 2]) && ["*", "&", "&&"].includes(prior))
    ) && ["=", ";", ",", ")", "["].includes(next);
    if (typedName) bind(token);
    const replacement = identifier(token) && prior !== "." && prior !== "->" && prior !== "::" ? lookup(token) : undefined;
    result.push(replacement ?? token);
  }
  return result;
}

function canonicalSource(file, source) {
  const language = file.endsWith(".ts") || file.endsWith(".mts") ? "ts" : "cpp";
  // Parser schema belongs in every digest. This is source-only syntax: imports,
  // preprocessor includes, compiler flags, and macro expansion are intentionally absent.
  return `kpt-source-ast-v2\0${language}\0${alphaNormalize(tokens(source)).join("\x1f")}`;
}

function category(file) {
  if (file === "VERSION") return ["version"];
  // Documentation is a packaging-only input. It must never select native,
  // browser, Electron, or WASM compilation in ordinary CI.
  if (/^(README(?:\.zh-CN)?\.md|LICENSE|THIRD_PARTY_NOTICES(?:\.md)?)$/.test(file)) return ["package-doc"];
  if (/^(src|bindings)\//.test(file) && /\.(c|cc|cpp|cxx|h|hh|hpp)$/.test(file)) {
    const categories = ["native"];
    if (/^(src\/kpt\/(core|io|wasm)\/|bindings\/wasm\/)/.test(file)) categories.push("wasm");
    return categories;
  }
  if (/^vscode\/(src|webview|scripts)\/.*\.(ts|mts|js|mjs)$/.test(file)) return ["vscode", "wasm"];
  if (/^vscode\/tests\/(browser-|worker-|decoder-).*/.test(file)) return ["browser-test"];
  if (/^vscode\/tests\/run-vscode-test\.mjs$/.test(file)) return ["electron-test"];
  if (/^tests\//.test(file)) return ["native-test", "asan-test", "tsan-test"];
  if (/^(packaging\/linux|packaging\/deb|tools\/package-deb\.sh|packaging\/icons\/linux)/.test(file)) return ["linux-package"];
  if (/^(packaging\/windows|tools\/package-windows\.ps1|packaging\/icons\/windows)/.test(file)) return ["windows-package"];
  if (/^(packaging\/macos|tools\/package-macos\.sh|packaging\/icons\/macos)/.test(file)) return ["macos-package"];
  if (/^(\.github\/workflows\/package-release\.yml|tools\/package-openvsx-vsix\.sh|tools\/validate-release-assets\.sh)/.test(file)) return ["publish"];
  return [];
}

function canonicalFile(file, source) {
  if (/\.(c|cc|cpp|cxx|h|hh|hpp|ts|mts|js|mjs)$/.test(file)) return canonicalSource(file, source);
  // Direct, non-source runtime/package assets have no AST. Keep their bytes local
  // to the owning stage; never traverse their dependencies.
  return `kpt-direct-resource-v1\0${source}`;
}

function readState() {
  if (!existsSync(statePath)) return { schema: 1, files: {} };
  return JSON.parse(readFileSync(statePath, "utf8"));
}

function output(file, data) {
  mkdirSync(dirname(file), { recursive: true });
  writeFileSync(file, `${JSON.stringify(data, null, 2)}\n`);
}

const all = trackedFiles();
const changed = bootstrap ? new Set(all) : stagedFiles();
let state = readState();
if (!bootstrap && Object.keys(state.files ?? {}).length === 0) {
  throw new Error("semantic cache missing: run node tools/semantic-fingerprint.mjs --bootstrap once");
}

for (const file of changed) {
  const source = stagedText(file);
  if (source === undefined) delete state.files[file];
  else state.files[file] = { categories: category(file), digest: sha256(canonicalFile(file, source)) };
}

if (bootstrap) {
  for (const file of all) {
    const source = readFileSync(join(root, file), "utf8");
    state.files[file] = { categories: category(file), digest: sha256(canonicalFile(file, source)) };
  }
}

const fragments = {};
for (const [stage, categories] of Object.entries(stages)) {
  const inputs = Object.entries(state.files)
    .filter(([, entry]) => entry.categories.some((item) => categories.includes(item)))
    .sort(([left], [right]) => left.localeCompare(right));
  fragments[stage] = sha256(`kpt-stage-v1\0${stage}\0${inputs.map(([file, entry]) => `${file}\0${entry.digest}`).join("\0")}`);
}

if (check) {
  for (const [stage, digest] of Object.entries(fragments)) {
    const file = join(outputDir, `${stage.replace(".", "-")}.json`);
    if (!existsSync(file) || JSON.parse(readFileSync(file, "utf8")).digest !== digest) {
      throw new Error(`stale fingerprint: ${relative(root, file)}`);
    }
  }
} else {
  for (const [stage, digest] of Object.entries(fragments)) {
    output(join(outputDir, `${stage.replace(".", "-")}.json`), {
      schema: 1,
      stage,
      digest,
      sourceOnly: true,
    });
  }
  mkdirSync(dirname(statePath), { recursive: true });
  output(statePath, state);
  const generated = Object.keys(fragments).map((stage) => join(".github", "semantic-fingerprints", `${stage.replace(".", "-")}.json`));
  execFileSync("git", ["add", "--", ...generated], { cwd: root, stdio: "inherit" });
}
