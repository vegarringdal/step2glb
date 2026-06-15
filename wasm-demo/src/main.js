// Main thread: orchestrates OPFS files + the worker. It never touches a sync
// access handle (those are worker-only) — it writes the upload to OPFS with an
// async writable stream, hands the worker a path, and renders/downloads the GLB
// the worker writes back. Files use UUID names and are deleted on error or once
// the user has downloaded the result.
import sampleUrl from './sample-box-cylinder.stp?url';

const statusEl = document.getElementById('status');
const viewer = document.getElementById('viewer');
const sampleBtn = document.getElementById('sample');
const fileInput = document.getElementById('file');
const downloadBtn = document.getElementById('download');
const infoEl = document.getElementById('info');
const deflEl = document.getElementById('deflection');
const deflValEl = document.getElementById('deflVal');
const maxAngleEl = document.getElementById('maxangle');
const maxAngleValEl = document.getElementById('maxangleVal');
const yupEl = document.getElementById('yup');
const normalsEl = document.getElementById('normals');
const mergedEl = document.getElementById('merged');
const cleanupEl = document.getElementById('cleanup');
const renderEl = document.getElementById('render');
const memEl = document.getElementById('mem');
const memValEl = document.getElementById('memVal');
const memLabel = document.getElementById('memLabel');

// live-update the slider read-outs
deflEl.addEventListener('input', () => {
  deflValEl.textContent = Number(deflEl.value).toFixed(2);
});
maxAngleEl.addEventListener('input', () => {
  maxAngleValEl.textContent = maxAngleEl.value;
});
memEl.addEventListener('input', () => {
  memValEl.textContent = memEl.value;
});

// merged mode is always in-RAM (it can't stream — one buffer per color), so the
// memory ceiling does not apply: grey out the slider while merged is selected.
function syncMemEnabled() {
  const off = mergedEl.checked;
  memEl.disabled = off;
  memLabel.style.opacity = off ? '0.5' : '';
}
mergedEl.addEventListener('change', syncMemEnabled);
syncMemEnabled();

// A wasm module's linear memory only ever grows — it is never returned to the
// OS while the instance lives. So instead of one long-lived worker we spawn a
// fresh one per job and terminate it when the job ends: killing the worker
// discards the whole wasm instance, and the browser reclaims all of its memory
// before the next conversion starts from a small, fresh heap.
function runWorker(job, onProgress) {
  return new Promise((resolve) => {
    const w = new Worker(new URL('./worker.js', import.meta.url), {
      type: 'module',
    });
    const finish = (msg) => {
      w.terminate();
      resolve(msg);
    };
    w.onmessage = (e) => {
      const m = e.data;
      // progress ticks arrive during a (blocked) streaming conversion
      if (m.progress) {
        onProgress?.(m.progress);
        return;
      }
      finish(m); // ready / result / error are all terminal — one msg per worker
    };
    w.onerror = (err) =>
      finish({ ok: false, error: String(err?.message || err) });
    w.postMessage(job);
  });
}

let current = null; // { inputPath, outputPath, displayName, glbUrl }
let busy = false;

const opfsRoot = () => navigator.storage.getDirectory();

async function deleteFile(name) {
  if (!name) return;
  try {
    (await opfsRoot()).removeEntry(name);
  } catch {
    /* already gone */
  }
}

async function cleanup() {
  if (!current) return;
  await deleteFile(current.inputPath);
  await deleteFile(current.outputPath);
  if (current.glbUrl) URL.revokeObjectURL(current.glbUrl);
  current = null;
}

// Stream the picked file straight to OPFS in chunks — never materialise the
// whole thing in main-thread memory. A File is backed by the OS file on disk,
// so `blob.stream()` reads it lazily and `pipeTo` writes each chunk to OPFS
// (and closes the writable on completion). Peak main-thread RAM is one chunk.
async function writeUpload(blob) {
  const name = `${crypto.randomUUID()}.stp`;
  const fh = await (await opfsRoot()).getFileHandle(name, { create: true });
  const w = await fh.createWritable(); // async writable — fine on the main thread
  await blob.stream().pipeTo(w);
  return name;
}

async function convert(displayName, blob) {
  if (busy) return;
  busy = true;
  downloadBtn.disabled = true;
  sampleBtn.disabled = true;
  infoEl.textContent = '';
  viewer.src = ''; // clear the old model from the canvas before a new run
  statusEl.textContent = `staging ${displayName}…`;
  await cleanup();
  const inputPath = await writeUpload(blob);
  const outputPath = `${crypto.randomUUID()}.glb`;
  current = { inputPath, outputPath, displayName };
  // merged mode accumulates one huge per-color buffer baked to world space — it
  // can't stream, so it always runs in RAM and ignores the memory ceiling.
  // Hierarchical mode spills each mesh's geometry to OPFS as it goes, so above
  // the ceiling it streams (bounded RAM); below it, all in RAM (faster).
  const merged = mergedEl.checked;
  const streaming = !merged && blob.size > Number(memEl.value) * 1024 * 1024;
  const opts = {
    deflectionMm: Number(deflEl.value),
    maxAngleDeg: Number(maxAngleEl.value),
    yUp: yupEl.checked,
    keepNormals: normalsEl.checked,
    merged,
    cleanup: cleanupEl.checked,
  };
  const mode = streaming
    ? 'streaming via OPFS (hierarchical, geometry spilled)'
    : merged
      ? 'in memory (merged)'
      : 'in memory (hierarchical)';
  statusEl.textContent = `converting ${displayName}… (${mode})`;

  // spawn a throwaway worker for just this conversion; it is terminated (and
  // its wasm memory reclaimed) the moment we get a result back.
  const m = await runWorker(
    { inputPath, outputPath, displayName, opts, streaming },
    ({ done, total }) => {
      const pct = total ? Math.round((100 * done) / total) : 0;
      statusEl.textContent = `converting ${displayName}… ${pct}% (${done}/${total})`;
    },
  );

  busy = false;
  sampleBtn.disabled = false;

  if (!m.ok) {
    statusEl.textContent = `error: ${m.error}`;
    await cleanup(); // delete the staged files on failure
    return;
  }

  // the GLB the worker wrote to OPFS. getFile()/.size is just metadata — it
  // does not load the bytes; only model-viewer (below) or Export does.
  const fh = await (await opfsRoot()).getFileHandle(m.outputPath);
  const size = (await fh.getFile()).size;
  renderInfo(m.displayName, size, m.ms, JSON.parse(m.info));

  // render only if asked — model-viewer decodes the glTF and uploads it to the
  // GPU, which is a big chunk of memory *after* conversion. Skipping it leaves
  // the canvas cleared and the post-conversion footprint to the converter.
  if (renderEl.checked) {
    current.glbUrl = URL.createObjectURL(await fh.getFile());
    viewer.src = current.glbUrl;
  } else {
    statusEl.textContent += ' — render skipped (Export to save)';
  }

  // the input is no longer needed
  await deleteFile(current.inputPath);
  current.inputPath = null;

  downloadBtn.disabled = false;
  downloadBtn.onclick = async () => {
    // create the object URL on demand — when render is off none exists yet, so
    // nothing of the GLB is held in memory until the user actually exports
    const url =
      current.glbUrl ??
      URL.createObjectURL(await (await opfsRoot()).getFileHandle(current.outputPath).then((h) => h.getFile()));
    const a = document.createElement('a');
    a.href = url;
    a.download = `${current.displayName.replace(/\.[^.]*$/, '')}.glb`;
    a.click();
    if (!current.glbUrl) URL.revokeObjectURL(url);
    // delete the OPFS output once the user has taken the file
    await deleteFile(current.outputPath);
    current.outputPath = null;
    downloadBtn.disabled = true;
  };
}

function table(title, obj) {
  const keys = Object.keys(obj || {});
  if (!keys.length) return '';
  const rows = keys.map((k) => `<div>${k}: <b>${obj[k]}</b></div>`).join('');
  return `<details open><summary>${title} (${keys.length})</summary>${rows}</details>`;
}

function renderInfo(name, glbSize, ms, r) {
  const parts = [];
  parts.push(`<h3>${name}</h3>`);
  parts.push(
    `<div>GLB ${(glbSize / 1024).toFixed(1)} KB · ${ms} ms · ${r.colorMeshes} color mesh(es)</div>`,
  );
  parts.push(`<div>faces ok: <b>${r.facesOk}</b>, skipped: <b>${r.facesSkipped}</b>, degenerate: <b>${r.degenerateFaces}</b></div>`);
  // defaults used
  if (r.unitAssumedMillimetres)
    parts.push(`<div>⚠ no length unit declared — assumed millimetres</div>`);
  parts.push(`<div>deflection: ${r.deflectionMm} mm · unit→m scale: ${r.unitScaleToMetres}</div>`);
  // issues
  parts.push(table('skipped surfaces', r.skippedSurfaces));
  parts.push(table('unsupported surfaces', r.unsupportedSurfaces));
  parts.push(table('approximated surfaces (flattened)', r.approximatedSurfaces));
  parts.push(table('unsupported edge curves', r.unsupportedCurves));
  parts.push(table('unsupported items', r.unsupportedItems));
  if (r.warnings?.length)
    parts.push(table('parser warnings', Object.fromEntries(r.warnings.map((w, i) => [i + 1, w]))));
  infoEl.innerHTML = parts.join('');
}

sampleBtn.addEventListener('click', async () => {
  // pass the Blob through — writeUpload streams it to OPFS without ever holding
  // the whole file in main-thread memory
  const res = await fetch(sampleUrl);
  await convert('sample', await res.blob());
});

fileInput.addEventListener('change', async () => {
  const f = fileInput.files?.[0];
  if (f) await convert(f.name, f); // the File is backed by disk; stream it
});

window.addEventListener('beforeunload', () => {
  // best-effort: drop any leftover OPFS scratch when the tab closes
  if (current) {
    navigator.storage.getDirectory().then((root) => {
      if (current.inputPath) root.removeEntry(current.inputPath).catch(() => {});
      if (current.outputPath) root.removeEntry(current.outputPath).catch(() => {});
    });
  }
});

// probe the version once at load (its worker is terminated immediately after)
runWorker({ init: true }).then((m) => {
  if (m.ready) statusEl.textContent = `${m.version} — ready`;
});
