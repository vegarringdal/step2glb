/**
 * StepConvert — a self-contained, browser-side STEP → GLB converter.
 *
 * Everything needed to run the step2glb wasm core from a web app lives in this
 * file and its sibling worker. To drop it into another project, copy THREE
 * things and keep their relative layout:
 *
 *   1. this file                  src/stepConvert.ts
 *   2. the worker it spawns       src/stepConvertWorker.ts
 *   3. the wasm-pack output dir   pkg/   (step2glb_wasm.js + _bg.wasm + .d.ts)
 *
 * The worker imports the wasm from `../pkg`, so keep `src/` and `pkg/` as
 * siblings (the `wasm-pack build --target web` layout). Then:
 *
 *   const converter = new StepConvert();
 *   const result = await converter.convert(file, file.name);
 *   viewer.src = await result.objectUrl();   // render
 *   await result.download();                 // save
 *   await result.dispose();                  // drop OPFS scratch
 *
 * The class owns the whole lifecycle: it streams the input into OPFS, spawns a
 * throwaway Web Worker per conversion (so the grow-only wasm heap is reclaimed
 * between runs), drives the synchronous wasm core through OPFS *synchronous*
 * access handles, and returns a {@link ConversionResult} that reads / exports /
 * disposes the GLB on demand. Nothing here touches the DOM except the optional
 * {@link ConversionResult.download} helper.
 */

/** Tessellation + output knobs forwarded to the wasm core. */
export interface ConvertOptions {
  /** chordal deflection (max sag), in millimetres. */
  deflectionMm: number;
  /** max chord turn angle, in degrees (smaller = rounder curves). */
  maxAngleDeg: number;
  /** rotate Z-up (STEP) to glTF Y-up. */
  yUp: boolean;
  /**
   * Keep per-vertex normals (off = smaller; viewer flat-shades). Mutually
   * exclusive with {@link cleanup}: cleanup welds positions and drops normals,
   * so this is forced off whenever `cleanup` is set.
   */
  keepNormals: boolean;
  /**
   * One node/mesh per colour baked to world space (vs the per-part tree).
   * Merged can't stream (one buffer per colour), so it is always in RAM and
   * ignores {@link StepConvert.memoryCeilingMb} — it never takes the streaming
   * path regardless of input size.
   */
  merged: boolean;
  /**
   * rvm-style position weld (simplify needs meshopt, omitted in the wasm
   * build). Welding drops normals, so this is mutually exclusive with
   * {@link keepNormals} (cleanup wins).
   */
  cleanup: boolean;
}

/** Diagnostics report produced by the core (parsed from its JSON). */
export interface ConvertReport {
  facesOk: number;
  facesSkipped: number;
  degenerateFaces: number;
  colorMeshes: number;
  unitAssumedMillimetres: boolean;
  unitScaleToMetres: number;
  deflectionMm: number;
  unsupportedSurfaces: Record<string, number>;
  unsupportedCurves: Record<string, number>;
  unsupportedItems: Record<string, number>;
  approximatedSurfaces: Record<string, number>;
  skippedSurfaces: Record<string, number>;
  warnings: string[];
}

/** Conversion progress, as product nodes finish (the core throttles to ~5%). */
export interface ConvertProgress {
  done: number;
  total: number;
}

/** Optional callbacks for status text and progress ticks during a conversion. */
export interface ConvertHandlers {
  onStatus?: (message: string) => void;
  onProgress?: (progress: ConvertProgress) => void;
}

// ---------------------------------------------------------------------------
// Worker message protocol (shared with stepConvertWorker.ts via `import type`).
// ---------------------------------------------------------------------------

/** Request sent to the worker (one per spawn). */
export type WorkerRequest =
  | { kind: 'init' }
  | {
      kind: 'convert';
      inputPath: string;
      outputPath: string;
      streaming: boolean;
      opts: ConvertOptions;
    };

/** Response from the worker; `progress` is repeatable, the rest are terminal. */
export type WorkerResponse =
  | { kind: 'ready'; version: string }
  | { kind: 'progress'; progress: ConvertProgress }
  | { kind: 'done'; outputPath: string; info: string; ms: number; streaming: boolean }
  | { kind: 'error'; error: string };

/** Defaults matching the wasm core's `ConvertOptions::default()`. */
export const DEFAULT_CONVERT_OPTIONS: ConvertOptions = {
  deflectionMm: 1.0,
  maxAngleDeg: 25.0,
  yUp: true,
  keepNormals: false,
  merged: false,
  cleanup: true,
};

function describeMode(streaming: boolean, merged: boolean): string {
  if (streaming) return 'streaming via OPFS (hierarchical, geometry spilled)';
  return merged ? 'in memory (merged)' : 'in memory (hierarchical)';
}

export class StepConvert {
  /**
   * Memory ceiling, in MB. A non-merged input larger than this is converted on
   * the streaming path (input read by range + geometry spilled to OPFS, so peak
   * wasm memory stays bounded); smaller inputs convert all in RAM (faster).
   * Merged mode can't stream (one buffer per colour), so it ignores this.
   * Mutable so a UI can adjust it between conversions.
   */
  memoryCeilingMb: number;

  /** OPFS scratch files this instance created and has not yet removed. */
  private readonly scratch = new Set<string>();
  private active: ConversionResult | null = null;

  constructor(options: { memoryCeilingMb?: number } = {}) {
    this.memoryCeilingMb = options.memoryCeilingMb ?? 100;
    // best-effort: drop any leftover OPFS scratch when the tab closes
    if (typeof addEventListener === 'function') {
      addEventListener('beforeunload', () => this.sweep());
    }
  }

  /** Probe the underlying converter's version string (spawns a throwaway worker). */
  async version(): Promise<string> {
    const res = await this.runJob({ kind: 'init' });
    if (res.kind === 'ready') return res.version;
    throw new Error(res.kind === 'error' ? res.error : 'worker failed to start');
  }

  /**
   * Convert a STEP blob to a GLB. Stages the input into OPFS, runs the wasm core
   * in a fresh worker, and resolves to a {@link ConversionResult}. Disposes the
   * previous result's scratch first; throws on conversion failure (after
   * cleaning up its own scratch).
   */
  async convert(
    input: Blob,
    displayName: string,
    options: Partial<ConvertOptions> = {},
    handlers: ConvertHandlers = {},
  ): Promise<ConversionResult> {
    const opts: ConvertOptions = { ...DEFAULT_CONVERT_OPTIONS, ...options };
    // cleanup-position welds positions and drops normals, so keeping normals is
    // meaningless alongside it: force them apart (cleanup wins, matching the
    // core). The demo UI enforces this too, but a direct caller might pass both.
    if (opts.cleanup) opts.keepNormals = false;
    await this.active?.dispose();
    this.active = null;

    handlers.onStatus?.(`staging ${displayName}…`);
    const inputPath = await this.stage(input);
    const outputPath = this.track(`${crypto.randomUUID()}.glb`);
    // merged can't stream (one world-baked buffer per colour); hierarchical
    // spills geometry to OPFS above the ceiling.
    const streaming = !opts.merged && input.size > this.memoryCeilingMb * 1024 * 1024;
    handlers.onStatus?.(`converting ${displayName}… (${describeMode(streaming, opts.merged)})`);

    let res: WorkerResponse;
    try {
      res = await this.runJob(
        { kind: 'convert', inputPath, outputPath, streaming, opts },
        handlers.onProgress,
      );
    } finally {
      await this.remove(inputPath); // the input is never needed past the conversion
    }

    if (res.kind !== 'done') {
      await this.remove(outputPath);
      throw new Error(res.kind === 'error' ? res.error : 'unexpected worker response');
    }

    const report = JSON.parse(res.info) as ConvertReport;
    const result = new ConversionResult(this, outputPath, displayName, report, res.ms, res.streaming);
    this.active = result;
    return result;
  }

  // --------------------------------------------------------------- worker glue

  private spawnWorker(): Worker {
    // Resolved relative to THIS module, so copying stepConvert.ts +
    // stepConvertWorker.ts together is enough for a bundler to find the worker.
    return new Worker(new URL('./stepConvertWorker.ts', import.meta.url), { type: 'module' });
  }

  private runJob(
    req: WorkerRequest,
    onProgress?: (p: ConvertProgress) => void,
  ): Promise<WorkerResponse> {
    return new Promise((resolve) => {
      const worker = this.spawnWorker();
      const finish = (msg: WorkerResponse): void => {
        worker.terminate(); // discards the whole wasm instance + its grown heap
        resolve(msg);
      };
      worker.onmessage = (e: MessageEvent<WorkerResponse>) => {
        const msg = e.data;
        if (msg.kind === 'progress') {
          onProgress?.(msg.progress);
          return;
        }
        finish(msg); // ready / done / error are terminal — one per worker
      };
      worker.onerror = (e: ErrorEvent) => finish({ kind: 'error', error: e.message || String(e) });
      worker.postMessage(req);
    });
  }

  // ------------------------------------------------------------ OPFS lifecycle

  private root(): Promise<FileSystemDirectoryHandle> {
    return navigator.storage.getDirectory();
  }

  private track(name: string): string {
    this.scratch.add(name);
    return name;
  }

  /**
   * Stream the blob straight into an OPFS file in chunks (never materialising
   * the whole thing in memory) and return its name. Uses an async writable —
   * fine off the worker; sync handles are only needed inside the worker.
   */
  private async stage(blob: Blob): Promise<string> {
    const name = this.track(`${crypto.randomUUID()}.stp`);
    const fh = await (await this.root()).getFileHandle(name, { create: true });
    const writable = await fh.createWritable();
    await blob.stream().pipeTo(writable);
    return name;
  }

  private async remove(name: string): Promise<void> {
    this.scratch.delete(name);
    try {
      (await this.root()).removeEntry(name);
    } catch {
      /* already gone */
    }
  }

  private sweep(): void {
    void this.root().then((root) => {
      for (const name of this.scratch) root.removeEntry(name).catch(() => {});
      this.scratch.clear();
    });
  }

  /** @internal — used by {@link ConversionResult}. */
  async _file(path: string): Promise<File> {
    const fh = await (await this.root()).getFileHandle(path);
    return fh.getFile();
  }

  /** @internal — used by {@link ConversionResult}. */
  async _remove(path: string): Promise<void> {
    await this.remove(path);
  }
}

/**
 * The output of one conversion. Holds only the OPFS path until you ask for the
 * bytes, so nothing of the GLB sits in memory until {@link objectUrl} /
 * {@link blob} / {@link download} is called. {@link dispose} deletes the OPFS
 * file and revokes any object URL.
 */
export class ConversionResult {
  private url: string | null = null;
  private disposed = false;

  constructor(
    private readonly owner: StepConvert,
    readonly outputPath: string,
    readonly displayName: string,
    readonly report: ConvertReport,
    readonly ms: number,
    readonly streaming: boolean,
  ) {}

  /** GLB size in bytes (metadata only — does not load the bytes). */
  async size(): Promise<number> {
    return (await this.owner._file(this.outputPath)).size;
  }

  /** The GLB as a Blob (reads it from OPFS). */
  async blob(): Promise<Blob> {
    return this.owner._file(this.outputPath);
  }

  /** A cached object URL for the GLB (e.g. for `<model-viewer src>`). */
  async objectUrl(): Promise<string> {
    if (this.url === null) this.url = URL.createObjectURL(await this.blob());
    return this.url;
  }

  /** Trigger a browser download of the GLB. */
  async download(filename?: string): Promise<void> {
    const url = await this.objectUrl();
    const a = document.createElement('a');
    a.href = url;
    a.download = filename ?? `${this.displayName.replace(/\.[^.]*$/, '')}.glb`;
    a.click();
  }

  /** Delete the OPFS output and revoke the object URL. Idempotent. */
  async dispose(): Promise<void> {
    if (this.disposed) return;
    this.disposed = true;
    if (this.url !== null) {
      URL.revokeObjectURL(this.url);
      this.url = null;
    }
    await this.owner._remove(this.outputPath);
  }
}
