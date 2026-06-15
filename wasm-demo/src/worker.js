// Web Worker: the only place that touches OPFS *synchronous* access handles
// (`createSyncAccessHandle`, worker-only). It is handed OPFS paths, reads the
// input synchronously, runs the synchronous wasm converter, writes the GLB
// synchronously, and returns the output path plus the JSON diagnostics report.
// Nothing here is `async` except acquiring the handles by name.
import init, { convert_step_to_glb, version } from '../pkg/step2glb_wasm.js';

const ready = init().then(() => version());

async function readAllSync(path) {
  const root = await navigator.storage.getDirectory();
  const fh = await root.getFileHandle(path);
  const h = await fh.createSyncAccessHandle(); // sync handle (input)
  try {
    const size = h.getSize();
    const buf = new Uint8Array(size);
    h.read(buf, { at: 0 });
    return buf;
  } finally {
    h.close();
  }
}

async function writeAllSync(path, bytes) {
  const root = await navigator.storage.getDirectory();
  const fh = await root.getFileHandle(path, { create: true });
  const h = await fh.createSyncAccessHandle(); // sync handle (output)
  try {
    h.truncate(0);
    h.write(bytes, { at: 0 });
    h.flush();
  } finally {
    h.close();
  }
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
    const stepBytes = await readAllSync(m.inputPath); // sync read
    const result = convert_step_to_glb(stepBytes); // synchronous, CPU-bound
    const glb = result.glb; // Uint8Array (getter)
    const info = result.info; // JSON diagnostics (getter)
    result.free(); // release the wasm-side object
    await writeAllSync(m.outputPath, glb); // sync write
    const ms = Math.round(performance.now() - t0);

    self.postMessage({
      ok: true,
      outputPath: m.outputPath,
      displayName: m.displayName,
      info,
      ms,
      version: v,
    });
  } catch (err) {
    self.postMessage({ ok: false, error: String(err && err.message ? err.message : err) });
  }
};
