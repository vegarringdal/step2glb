/**
 * Demo page: wires the HTML controls to {@link StepConvert} and renders the
 * result with <model-viewer>. All the wasm / OPFS / worker plumbing lives in
 * stepConvert.ts — this file is just the UI on top of it.
 */
import sampleUrl from './sample-box-cylinder.stp?url';
import { StepConvert, type ConvertOptions, type ConvertReport, type ConversionResult } from './stepConvert';

const byId = <T extends HTMLElement>(id: string): T => {
  const el = document.getElementById(id);
  if (!el) throw new Error(`missing #${id}`);
  return el as T;
};

const statusEl = byId('status');
const viewer = byId('viewer');
const sampleBtn = byId<HTMLButtonElement>('sample');
const fileInput = byId<HTMLInputElement>('file');
const downloadBtn = byId<HTMLButtonElement>('download');
const infoEl = byId('info');
const deflEl = byId<HTMLInputElement>('deflection');
const deflValEl = byId('deflVal');
const maxAngleEl = byId<HTMLInputElement>('maxangle');
const maxAngleValEl = byId('maxangleVal');
const yupEl = byId<HTMLInputElement>('yup');
const normalsEl = byId<HTMLInputElement>('normals');
const mergedEl = byId<HTMLInputElement>('merged');
const cleanupEl = byId<HTMLInputElement>('cleanup');
const renderEl = byId<HTMLInputElement>('render');
const memEl = byId<HTMLInputElement>('mem');
const memValEl = byId('memVal');
const memLabel = byId('memLabel');

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

// merged mode is always in-RAM (one buffer per colour baked to world space — it
// can't stream), so the memory threshold does not apply: grey out the slider
// while merged is selected. (StepConvert enforces this anyway: merged never
// takes the streaming path regardless of the ceiling.)
function syncMemEnabled(): void {
  const off = mergedEl.checked;
  memEl.disabled = off;
  memLabel.style.opacity = off ? '0.5' : '';
}
mergedEl.addEventListener('change', syncMemEnabled);
syncMemEnabled();

// normals and cleanup-position are mutually exclusive: cleanup welds positions
// on a grid, after which a vertex shared across faces has no single valid
// normal, so the converter always drops NORMAL when cleanup is on. You can't
// keep normals AND weld — checking one clears the other (neither is fine too).
function syncNormalsCleanup(justChanged: HTMLInputElement): void {
  if (justChanged === normalsEl && normalsEl.checked) cleanupEl.checked = false;
  if (justChanged === cleanupEl && cleanupEl.checked) normalsEl.checked = false;
}
normalsEl.addEventListener('change', () => syncNormalsCleanup(normalsEl));
cleanupEl.addEventListener('change', () => syncNormalsCleanup(cleanupEl));

const converter = new StepConvert();
let busy = false;
let current: ConversionResult | null = null;

async function convert(displayName: string, blob: Blob): Promise<void> {
  if (busy) return;
  busy = true;
  downloadBtn.disabled = true;
  sampleBtn.disabled = true;
  infoEl.textContent = '';
  viewer.removeAttribute('src'); // clear the old model before a new run
  current = null;

  converter.memoryCeilingMb = Number(memEl.value);
  const opts: ConvertOptions = {
    deflectionMm: Number(deflEl.value),
    maxAngleDeg: Number(maxAngleEl.value),
    yUp: yupEl.checked,
    keepNormals: normalsEl.checked,
    merged: mergedEl.checked,
    cleanup: cleanupEl.checked,
  };

  try {
    const result = await converter.convert(blob, displayName, opts, {
      onStatus: (message) => {
        statusEl.textContent = message;
      },
      onProgress: ({ done, total }) => {
        const pct = total ? Math.round((100 * done) / total) : 0;
        statusEl.textContent = `converting ${displayName}… ${pct}% (${done}/${total})`;
      },
    });
    current = result;
    renderInfo(result.displayName, await result.size(), result.ms, result.report);
    statusEl.textContent = `done in ${result.ms} ms (${result.streaming ? 'streamed via OPFS' : 'in memory'})`;

    // render only if asked — model-viewer decodes the glTF and uploads it to the
    // GPU, a big chunk of memory *after* conversion. Skipping it leaves the
    // canvas cleared and the footprint to the converter.
    if (renderEl.checked) {
      viewer.setAttribute('src', await result.objectUrl());
    } else {
      statusEl.textContent += ' — render skipped (Export to save)';
    }
    downloadBtn.disabled = false;
  } catch (err) {
    statusEl.textContent = `error: ${err instanceof Error ? err.message : String(err)}`;
  } finally {
    busy = false;
    sampleBtn.disabled = false;
  }
}

downloadBtn.addEventListener('click', async () => {
  if (!current) return;
  await current.download();
  await current.dispose(); // drop the OPFS output once the user has taken it
  current = null;
  downloadBtn.disabled = true;
});

const escapeHtml = (s: string): string =>
  s.replace(/[&<>"]/g, (c) => {
    switch (c) {
      case '&':
        return '&amp;';
      case '<':
        return '&lt;';
      case '>':
        return '&gt;';
      default:
        return '&quot;';
    }
  });

function table(title: string, obj: Record<string, string | number> | undefined): string {
  const entries = Object.entries(obj ?? {});
  if (!entries.length) return '';
  const rows = entries
    .map(([k, v]) => `<div>${escapeHtml(k)}: <b>${escapeHtml(String(v))}</b></div>`)
    .join('');
  return `<details open><summary>${escapeHtml(title)} (${entries.length})</summary>${rows}</details>`;
}

function renderInfo(name: string, glbSize: number, ms: number, r: ConvertReport): void {
  const parts: string[] = [];
  parts.push(`<h3>${escapeHtml(name)}</h3>`);
  parts.push(
    `<div>GLB ${(glbSize / 1024).toFixed(1)} KB · ${ms} ms · ${r.colorMeshes} color mesh(es)</div>`,
  );
  parts.push(
    `<div>faces ok: <b>${r.facesOk}</b>, skipped: <b>${r.facesSkipped}</b>, degenerate: <b>${r.degenerateFaces}</b></div>`,
  );
  if (r.unitAssumedMillimetres) {
    parts.push(`<div>⚠ no length unit declared — assumed millimetres</div>`);
  }
  parts.push(
    `<div>deflection: ${r.deflectionMm} mm · unit→m scale: ${r.unitScaleToMetres}</div>`,
  );
  parts.push(table('skipped surfaces', r.skippedSurfaces));
  parts.push(table('unsupported surfaces', r.unsupportedSurfaces));
  parts.push(table('approximated surfaces (flattened)', r.approximatedSurfaces));
  parts.push(table('unsupported edge curves', r.unsupportedCurves));
  parts.push(table('unsupported items', r.unsupportedItems));
  if (r.warnings.length) {
    parts.push(table('parser warnings', Object.fromEntries(r.warnings.map((w, i) => [String(i + 1), w]))));
  }
  infoEl.innerHTML = parts.join('');
}

sampleBtn.addEventListener('click', async () => {
  // pass the Blob through — StepConvert streams it to OPFS without ever holding
  // the whole file in main-thread memory
  const res = await fetch(sampleUrl);
  await convert('sample', await res.blob());
});

fileInput.addEventListener('change', async () => {
  const f = fileInput.files?.[0];
  if (f) await convert(f.name, f); // the File is backed by disk; stream it
});

// probe the version once at load
converter
  .version()
  .then((v) => {
    statusEl.textContent = `${v} — ready`;
  })
  .catch((err) => {
    statusEl.textContent = `failed to load wasm: ${err instanceof Error ? err.message : String(err)}`;
  });
