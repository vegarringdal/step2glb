/**
 * Web Worker for {@link StepConvert} — the only place OPFS *synchronous* access
 * handles exist (`createSyncAccessHandle` is worker-only). It loads the
 * step2glb wasm core and runs exactly one conversion per spawn; the parent
 * terminates it afterwards, reclaiming the grow-only wasm heap.
 *
 * Copy this together with stepConvert.ts and the pkg/ wasm output, keeping the
 * `../pkg` relative path (the `wasm-pack build --target web` layout).
 *
 * Two paths, chosen by the parent from input size vs the memory ceiling:
 *   • in-RAM (small inputs): read the whole file, convert bytes→bytes, write out.
 *   • streaming (large inputs): hand wasm an `io` object backed by three sync
 *     handles (input / output / temp). Rust reads the input *by range*, appends
 *     the GLB, and spills geometry to the temp file — nothing big stays in wasm
 *     memory. Progress is posted as it arrives.
 */
import init, {
  convert_step_to_glb_opts,
  convert_streaming,
  version,
} from '../pkg/step2glb_wasm.js';
// Import the .wasm as an explicit asset URL so the bundler emits it and resolves
// it under the configured base path (e.g. /step2glb/ on GitHub Pages) — no need
// to inline the wasm or rely on the glue's own `import.meta.url` rewrite.
import wasmUrl from '../pkg/step2glb_wasm_bg.wasm?url';
import type { ConvertOptions, WorkerRequest, WorkerResponse } from './stepConvert';

// `self` is a dedicated worker scope; narrow it so postMessage/onmessage type
// against the worker API regardless of which lib the tsconfig loads first.
const ctx = self as unknown as DedicatedWorkerGlobalScope;

const ready: Promise<string> = init(wasmUrl).then(() => version());
const root = (): Promise<FileSystemDirectoryHandle> => navigator.storage.getDirectory();

function post(msg: WorkerResponse): void {
  ctx.postMessage(msg);
}

async function openSync(path: string, create: boolean): Promise<FileSystemSyncAccessHandle> {
  const fh = await (await root()).getFileHandle(path, { create });
  return fh.createSyncAccessHandle();
}

/** Read [offset, offset+len) from a sync handle (short read trims the buffer). */
function readRange(h: FileSystemSyncAccessHandle, offset: number, len: number): Uint8Array {
  const buf = new Uint8Array(len);
  const n = h.read(buf, { at: offset });
  return n === len ? buf : buf.subarray(0, n);
}

// whole-file read/write for the in-RAM path
async function readAll(path: string): Promise<Uint8Array> {
  const h = await openSync(path, false);
  try {
    const buf = new Uint8Array(h.getSize());
    h.read(buf, { at: 0 });
    return buf;
  } finally {
    h.close();
  }
}

async function writeAll(path: string, bytes: Uint8Array): Promise<void> {
  const h = await openSync(path, true);
  try {
    h.truncate(0);
    h.write(bytes, { at: 0 });
    h.flush();
  } finally {
    h.close();
  }
}

// The `io` object Rust calls during a streaming conversion. The shapes mirror
// the wasm-bindgen `Io` extern (size/read/writeOutput/writeTemp/readTemp/
// tempLen/progress); we cast to the glue's parameter type at the call so we
// don't depend on whether wasm-bindgen exports the `Io` name.
interface StreamingIo {
  size(): number;
  read(offset: number, len: number): Uint8Array;
  writeOutput(bytes: Uint8Array): void;
  writeTemp(offset: number, bytes: Uint8Array): void;
  readTemp(offset: number, len: number): Uint8Array;
  tempLen(): number;
  progress(done: number, total: number): void;
}

// Progress sink for the in-RAM path: `report(done, total)` fires as product
// nodes finish — the in-RAM counterpart of `io.progress`.
interface ProgressSink {
  report(done: number, total: number): void;
}

async function runInRam(inputPath: string, outputPath: string, o: ConvertOptions): Promise<string> {
  const stepBytes = await readAll(inputPath);
  const progress: ProgressSink = {
    report: (done, total) => post({ kind: 'progress', progress: { done, total } }),
  };
  const result = convert_step_to_glb_opts(
    stepBytes,
    o.deflectionMm,
    o.maxAngleDeg,
    o.yUp,
    o.keepNormals,
    o.cleanup,
    o.merged,
    progress as Parameters<typeof convert_step_to_glb_opts>[7],
  );
  const glb = result.glb;
  const info = result.info;
  result.free();
  await writeAll(outputPath, glb);
  return info;
}

async function runStreaming(
  inputPath: string,
  outputPath: string,
  o: ConvertOptions,
): Promise<string> {
  const inH = await openSync(inputPath, false);
  const outH = await openSync(outputPath, true);
  const tmpPath = `${outputPath}.tmp`;
  const tmpH = await openSync(tmpPath, true);
  outH.truncate(0);
  tmpH.truncate(0);
  let outLen = 0;
  let tmpLen = 0;
  const io: StreamingIo = {
    size: () => inH.getSize(),
    read: (off, len) => readRange(inH, off, len),
    writeOutput: (bytes) => {
      outH.write(bytes, { at: outLen });
      outLen += bytes.length;
    },
    writeTemp: (off, bytes) => {
      tmpH.write(bytes, { at: off });
      if (off + bytes.length > tmpLen) tmpLen = off + bytes.length;
    },
    readTemp: (off, len) => readRange(tmpH, off, len),
    tempLen: () => tmpLen,
    progress: (done, total) => post({ kind: 'progress', progress: { done, total } }),
  };
  try {
    const info = convert_streaming(
      io as Parameters<typeof convert_streaming>[0],
      o.deflectionMm,
      o.maxAngleDeg,
      o.yUp,
      o.keepNormals,
      o.cleanup,
      o.merged,
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

ctx.onmessage = async (e: MessageEvent<WorkerRequest>) => {
  const req = e.data;
  try {
    const v = await ready;
    if (req.kind === 'init') {
      post({ kind: 'ready', version: v });
      return;
    }
    const t0 = performance.now();
    const info = req.streaming
      ? await runStreaming(req.inputPath, req.outputPath, req.opts)
      : await runInRam(req.inputPath, req.outputPath, req.opts);
    post({
      kind: 'done',
      outputPath: req.outputPath,
      info,
      ms: Math.round(performance.now() - t0),
      streaming: req.streaming,
    });
  } catch (err) {
    post({ kind: 'error', error: err instanceof Error ? err.message : String(err) });
  }
};
