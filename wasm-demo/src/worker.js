// Web Worker: the only place that touches OPFS *synchronous* access handles
// (`createSyncAccessHandle`, worker-only).
//
// Two paths, chosen by the main thread from the file size vs the memory slider:
//   • in-RAM (small files): read the whole input, convert bytes→bytes, write out.
//   • streaming (large files): hand wasm an `Io` object backed by three sync
//     handles (input / output / temp). Rust reads the input *by range*, appends
//     the GLB, and spills geometry to the temp file — nothing big is held in
//     wasm memory. Progress is posted to the UI as it arrives.
import init, {
  convert_step_to_glb_opts,
  convert_streaming,
  version,
} from '../pkg/step2glb_wasm.js';
// Import the .wasm as an explicit asset URL so Vite emits it and resolves it
// under the configured `base` (e.g. /step2glb/ on GitHub Pages). This avoids
// relying on the glue's `import.meta.url` rewrite inside the worker chunk — and
// means the wasm never has to be inlined to deploy under a subpath.
import wasmUrl from '../pkg/step2glb_wasm_bg.wasm?url';

const ready = init(wasmUrl).then(() => version());
const root = () => navigator.storage.getDirectory();

async function openSync(path, create) {
  const fh = await (await root()).getFileHandle(path, { create });
  return fh.createSyncAccessHandle();
}

// whole-file read/write for the in-RAM path
async function readAllSync(path) {
  const h = await openSync(path, false);
  try {
    const buf = new Uint8Array(h.getSize());
    h.read(buf, { at: 0 });
    return buf;
  } finally {
    h.close();
  }
}
async function writeAllSync(path, bytes) {
  const h = await openSync(path, true);
  try {
    h.truncate(0);
    h.write(bytes, { at: 0 });
    h.flush();
  } finally {
    h.close();
  }
}

// streaming path: Rust drives I/O through these sync handles by range
async function runStreaming(m, o) {
  const inH = await openSync(m.inputPath, false);
  const outH = await openSync(m.outputPath, true);
  const tmpPath = `${m.outputPath}.tmp`;
  const tmpH = await openSync(tmpPath, true);
  outH.truncate(0);
  tmpH.truncate(0);
  let outLen = 0;
  let tmpLen = 0;
  const slice = (h, off, len) => {
    const b = new Uint8Array(len);
    const n = h.read(b, { at: off });
    return n === len ? b : b.subarray(0, n);
  };
  const io = {
    size: () => inH.getSize(),
    read: (off, len) => slice(inH, off, len),
    writeOutput: (bytes) => {
      outH.write(bytes, { at: outLen });
      outLen += bytes.length;
    },
    writeTemp: (off, bytes) => {
      tmpH.write(bytes, { at: off });
      if (off + bytes.length > tmpLen) tmpLen = off + bytes.length;
    },
    readTemp: (off, len) => slice(tmpH, off, len),
    tempLen: () => tmpLen,
    progress: (done, total) => self.postMessage({ progress: { done, total } }),
  };
  try {
    const info = convert_streaming(
      io,
      o.deflectionMm ?? 1.0,
      o.maxAngleDeg ?? 25.0,
      o.yUp ?? true,
      o.keepNormals ?? false,
      o.cleanup ?? false,
      o.merged ?? true,
    );
    outH.flush();
    return info;
  } finally {
    inH.close();
    outH.close();
    tmpH.close();
    (await root()).removeEntry(tmpPath).catch(() => {});
  }
}

async function runInRam(m, o) {
  const stepBytes = await readAllSync(m.inputPath);
  const result = convert_step_to_glb_opts(
    stepBytes,
    o.deflectionMm ?? 1.0,
    o.maxAngleDeg ?? 25.0,
    o.yUp ?? true,
    o.keepNormals ?? false,
    o.cleanup ?? false,
    o.merged ?? true,
  );
  const glb = result.glb;
  const info = result.info;
  result.free();
  await writeAllSync(m.outputPath, glb);
  return info;
}

self.onmessage = async (e) => {
  const m = e.data;
  try {
    const v = await ready;
    if (m.init) {
      self.postMessage({ ok: true, ready: true, version: v });
      return;
    }
    const t0 = performance.now();
    const o = m.opts || {};
    const info = m.streaming ? await runStreaming(m, o) : await runInRam(m, o);
    const ms = Math.round(performance.now() - t0);
    self.postMessage({
      ok: true,
      outputPath: m.outputPath,
      displayName: m.displayName,
      streaming: m.streaming,
      info,
      ms,
      version: v,
    });
  } catch (err) {
    self.postMessage({
      ok: false,
      error: String(err && err.message ? err.message : err),
    });
  }
};
