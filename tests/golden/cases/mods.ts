import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/mods.ts'
import { nodeAt, Selection } from '../../../../../../apps/Fountain/fountain-ts/src/curves/selection.ts'
import { lineCubic as L } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { P, ser, type Spec } from './paths.ts'
import { record } from './record.ts'

// A selection is named by data: a path spec and the nodes picked (all when left out).
const sel = (path: Spec, nodes?: number[]) => Selection.of(P(path), nodes ? nodeAt(...nodes) : undefined)
const outSel = (s: Selection) => ({ paths: s.paths.map(ser), weights: s.weights })

const mod = {
  all: () => m.mods.map(x => ({ name: x.name, group: x.group, doc: x.doc, params: Object.entries(x.params) })),
  modDefaults: (name: string) => m.modDefaults(m.mod(name)),
  modCode: (name: string, v?: m.Values) => m.modCode(m.mod(name), v),
  applyMod: (path: Spec, nodes: number[] | null, name: string, v?: m.Values) => outSel(m.applyMod(sel(path, nodes ?? undefined), m.mod(name), v)),
  chain: (path: Spec, nodes: number[] | null, steps: m.Step[]) => outSel(m.chain(sel(path, nodes ?? undefined), steps)),
  chainCode: (steps: m.Step[]) => m.chainCode(steps),
}

const k = 0.5523 * 0.5
const ring: Spec = { start: [1, 0.5], closed: true, segs: [
  [[1, 0.5], [1, 0.5 + k], [0.5 + k, 1], [0.5, 1]], [[0.5, 1], [0.5 - k, 1], [0, 0.5 + k], [0, 0.5]],
  [[0, 0.5], [0, 0.5 - k], [0.5 - k, 0], [0.5, 0]], [[0.5, 0], [0.5 + k, 0], [1, 0.5 - k], [1, 0.5]]] }
const zig: Spec = { start: [0, 0], segs: [L([0, 0], [0.2, 0.6]), L([0.2, 0.6], [0.5, 0.3]), L([0.5, 0.3], [0.8, 0.8]), L([0.8, 0.8], [1, 0.1])] }
const bend: Spec = { start: [0, 0], segs: [L([0, 0], [0, 0.5]), [[0, 0.5], [0, 0.8], [0.2, 1], [0.5, 1]], L([0.5, 1], [1, 1])] }

const values: Record<string, m.Values[]> = {
  soften: [{}, { reach: 0.8 }, { falloff: 'linear' }, { falloff: 'sharp', along: 'plane', reach: 1.2 }, { along: 'plane' }],
  weigh: [{}, { from: 0.3, to: 0.3 }, { from: 0, to: 1 }],
  invert: [{}],
  shift: [{}, { axis: 0 }, { dist: -0.3, axis: 45 }, { dist: 0.123456 }],
  put: [{}, { along: 'x', x: 0.2 }, { along: 'both', x: 0.3, y: 0.9 }, { y: 1.25 }],
  rotate: [{}, { deg: -90 }, { deg: 12.5 }],
  scale: [{}, { x: 2, y: 2 }, { x: 0.5, y: 1.5 }, { x: -1 }],
  shear: [{}, { axis: 90 }, { deg: -30, axis: 45 }],
  mirror: [{}, { axis: 0 }, { axis: 33 }],
  inflate: [{}, { wave: 'sine' }, { wave: 'cosine', cycles: 2, handles: 0.5 }, { wave: 'square', smooth: 0.4 }, { wave: 'triangle', along: 'path' }, { wave: 'saw', cycles: 5, r: -0.2 }, { along: 'path' }],
  kind: [{}, { kind: 'symmetric' }, { kind: 'corner' }],
  handles: [{}, { deg: 40 }, { aim: 'angle' }, { aim: 'angle', deg: -45, length: 0.5 }, { length: 2, side: 'in' }, { side: 'out', deg: 10 }],
  smooth: [{}, { amount: 0.5 }, { amount: 0 }],
  fair: [{}, { reach: 0.2 }, { reach: 1 }],
}

export default record(mod, F => {
  F('all')
  for (const x of m.mods) F('modDefaults', x.name)
  for (const [name, vs] of Object.entries(values)) for (const v of vs) {
    F('modCode', name, v)
    for (const [path, nodes] of [[ring, null], [ring, [1]], [zig, [1, 2]], [bend, [1, 2]], [zig, null]] as const)
      F('applyMod', path, nodes, name, v)
  }
  F('modCode', 'shift')
  F('modCode', 'shift', { dist: 1e-5 })
  F('modCode', 'shift', { dist: 0.00005 })
  F('modCode', 'shift', { dist: -0.00015 })
  F('modCode', 'rotate', { deg: 1234567.98765 })
  F('modCode', 'put', { along: 'x', x: 1e-8 })
  const steps: m.Step[] = [{ mod: 'soften', values: { reach: 0.5 } }, { mod: 'inflate', values: { r: 0.05, wave: 'sine' } }, { mod: 'rotate' }, { mod: 'kind', values: { kind: 'smooth' } }]
  F('chain', ring, [0], steps)
  F('chain', zig, [2], steps)
  F('chain', bend, null, [])
  F('chainCode', steps)
  F('chainCode', [])
})
