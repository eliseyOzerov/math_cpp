import * as m from '../../../../../../apps/Fountain/fountain-ts/src/ink/ink.ts'
import type { Cubic } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import type { NibInput } from '../../../../../../apps/Fountain/fountain-ts/src/ink/nib.ts'
import type { Keys } from '../../../../../../apps/Fountain/fountain-ts/src/curves/profile.ts'
import { record } from './record.ts'

// Strokes the ink and outline cases share, and a nib maker with coarse sampling (small goldens).

const K = 0.5523
const L = (a: [number, number], b: [number, number]): Cubic => [a, [a[0] + (b[0] - a[0]) / 3, a[1] + (b[1] - a[1]) / 3], [a[0] + 2 * (b[0] - a[0]) / 3, a[1] + 2 * (b[1] - a[1]) / 3], b]
const circle = (r: number, cx: number, cy: number): Cubic[] => [
  [[cx + r, cy], [cx + r, cy + K * r], [cx + K * r, cy + r], [cx, cy + r]],
  [[cx, cy + r], [cx - K * r, cy + r], [cx - r, cy + K * r], [cx - r, cy]],
  [[cx - r, cy], [cx - r, cy - K * r], [cx - K * r, cy - r], [cx, cy - r]],
  [[cx, cy - r], [cx + K * r, cy - r], [cx + r, cy - K * r], [cx + r, cy]],
]

export const strokes: Record<string, m.InkStroke> = {
  line: { segments: [L([0, 0], [300, 0])] },
  vline: { segments: [L([0, 0], [0, 300])] },
  diag: { segments: [L([0, 0], [250, 300])] },
  arch: { segments: [[[0, 0], [0, 200], [300, 300], [300, 0]]] },
  sCurve: { segments: [[[0, 0], [150, 300], [150, -300], [300, 0]]] },
  bowl: { segments: [[[300, 150], [300, 250], [200, 300], [150, 300]], [[150, 300], [50, 300], [0, 250], [0, 150]], [[0, 150], [0, 50], [50, 0], [150, 0]]] },
  ring: { segments: circle(150, 150, 150), closed: true },
  vee: { segments: [L([0, 300], [150, 0]), L([150, 0], [300, 300])] },
  chevron: { segments: [L([0, 0], [200, 100]), L([200, 100], [0, 200])] },
  boxed: { segments: [L([0, 300], [0, 0]), L([0, 0], [200, 0]), L([200, 0], [200, 300])] },
  hook: { segments: [[[0, 300], [0, 150], [0, 100], [0, 50]], [[0, 50], [0, 0], [100, 0], [150, 0]], L([150, 0], [250, 120])] },
}

/** A nib given coarsely sampled: few stamps per span, few points per corner. */
export const nib = (n: Record<string, unknown>): NibInput => ({ perSpan: 6, perCorner: 3, ...n }) as NibInput

const { line, vline, diag, arch, sCurve, bowl, ring, vee, chevron, boxed, hook } = strokes
const cut = (mode: m.Cut['mode'], extra: Partial<m.Cut> = {}): m.Cut => ({ mode, ...extra })
const rect = nib({ w: 50, h: 24, angle: 30 }), round = nib({ w: 44, h: 44 }), square = nib({ w: 40, h: 40, rx: 0, ry: 0 })
const pen = nib({ weight: 40, contrast: 0.5, stress: 25 })

export default record({ ...m }, F => {
  // The default and number nibs on each kind of stroke.
  for (const s of [line, vline, diag, arch, sCurve, bowl, ring, vee, chevron, boxed, hook]) {
    F('ink', s, round)
    F('ink', s, rect)
  }
  F('ink', line); F('ink', line, 30); F('ink', arch, 30, { perSpan: 5 }); F('ink', sCurve, 60, { perSpan: 4 })
  F('ink', ring, 36, { perSpan: 6 }); F('ink', vee, 36, { perSpan: 3 })
  F('ink', line, pen); F('ink', arch, pen); F('ink', bowl, pen); F('ink', ring, pen)
  F('ink', arch, square); F('ink', vee, square); F('ink', boxed, square)
  // Nib angle, follow, a broad nib turning with the stroke.
  F('ink', arch, nib({ w: 50, h: 20, follow: 1 })); F('ink', sCurve, nib({ w: 50, h: 20, follow: 0.5, angle: 20 }))
  F('ink', ring, nib({ w: 50, h: 20, follow: 1 })); F('ink', bowl, nib({ w: 50, h: 20, follow: 0.3, angle: -40 }))
  // Caps.
  for (const caps of ['butt', 'nib'] as const) for (const s of [line, arch, sCurve, vee, ring]) { F('ink', s, rect, { caps }); F('ink', s, round, { caps }) }
  F('ink', diag, rect, { caps: 'nib' })
  // Every cut mode at each end.
  const modes: m.Cut[] = [cut('flat'), cut('nib'), cut('round'), cut('level'), cut('angle', { deg: 30 }), cut('angle', { deg: 100 }), cut('angle', { deg: -45 }), cut('auto')]
  for (const c of modes) for (const s of [arch, diag, sCurve]) {
    F('ink', { ...s, cuts: { start: c } }, rect)
    F('ink', { ...s, cuts: { end: c } }, rect)
    F('ink', { ...s, cuts: { start: c, end: c } }, round)
  }
  F('ink', { ...arch, cuts: { start: cut('level'), end: cut('angle', { deg: 60 }) } }, rect)
  F('ink', { ...bowl, cuts: { start: cut('angle', { deg: 20 }), end: cut('flat') } }, pen)
  F('ink', { ...line, cuts: { start: cut('angle', { deg: 8 }), end: cut('angle', { deg: 90 }) } }, rect)
  F('ink', { ...line, cuts: { start: cut('angle', { deg: 14.9 }) } }, rect)
  F('ink', { ...vline, cuts: { start: cut('level'), end: cut('level') } }, rect)
  F('ink', { ...vline, cuts: { start: cut('level') } }, rect, { caps: 'nib' })
  F('ink', { ...arch, cuts: { start: cut('level'), end: cut('flat') } }, rect, { cuts: false })
  F('ink', { ...arch, cuts: { start: cut('level'), end: cut('flat') } }, rect, { cuts: false, caps: 'nib' })
  // Taper and reach.
  for (const c of [cut('flat', { taper: -0.5 }), cut('flat', { taper: 0.4, reach: 60 }), cut('nib', { taper: -0.8, reach: 100 }), cut('round', { taper: -0.3 }),
    cut('auto', { taper: -0.6 }), cut('level', { taper: -0.5, reach: 30 }), cut('angle', { deg: 40, taper: 0.5 })]) {
    F('ink', { ...arch, cuts: { start: c, end: c } }, rect)
    F('ink', { ...sCurve, cuts: { end: c } }, round)
  }
  F('ink', { ...line, cuts: { start: cut('flat', { taper: -1 }), end: cut('flat', { taper: -1 }) } }, round)
  // Levels: bare ends on a level are cut level; cuts off ignores them.
  F('ink', arch, rect, { levels: [0] }); F('ink', arch, rect, { levels: [0, 300] }); F('ink', arch, rect, { levels: [300.3] })
  F('ink', arch, rect, { levels: [0], cuts: false }); F('ink', vline, rect, { levels: [0, 300] }); F('ink', sCurve, round, { levels: [0] })
  F('ink', arch, rect, { levels: [0], caps: 'nib' }); F('ink', { ...arch, cuts: { start: cut('nib') } }, rect, { levels: [0] })
  F('ink', { ...arch, cuts: { start: cut('auto') } }, rect, { levels: [0] })
  // The stroke's curves.
  const ks = (...v: [number, number][]): Keys => v
  F('ink', { ...arch, size: ks([0, 1], [0.5, 1.5], [1, 0.6]) }, rect)
  F('ink', { ...arch, size: ks([1, 0.5], [0, 2]) }, rect)
  F('ink', { ...arch, size: [[0, 1, { out: [0.2, 1] }], [1, 2, { in: [-0.3, 0] }]] }, round)
  F('ink', { ...sCurve, rotate: ks([0, 0], [1, 60]) }, rect)
  F('ink', { ...sCurve, rotate: ks([0, 45]) }, rect)
  F('ink', { ...arch, follow: ks([0, 0], [1, 1]) }, rect)
  F('ink', { ...ring, follow: ks([0, 1]) }, rect)
  F('ink', { ...arch, offset: ks([0, -1], [1, 1]) }, rect)
  F('ink', { ...sCurve, offset: ks([0, 0.5]) }, round)
  F('ink', { ...line, offset: ks([0, 1], [1, -1]) }, rect, { caps: 'nib' })
  F('ink', { ...arch, shapes: [[0, { w: 1, h: 1, rx: 1, ry: 1 }], [1, { w: 2, h: 0.5, rx: 0, ry: 0 }]] }, rect)
  F('ink', { ...bowl, shapes: [[0, { w: 1, h: 1, rx: [1, 0, 1, 0], ry: 1 }], [0.5, { w: 0.5, h: 1.5, rx: 0.5, ry: 0.5 }], [1, { w: 1, h: 1, rx: 1, ry: 1 }]] }, round)
  F('ink', { ...arch, shapes: [[0.5, { w: 1.5, h: 1, rx: 0, ry: 0 }]] }, rect)
  F('ink', { ...arch, shapes: [[1, { w: 0.5, h: 2, rx: 1, ry: 1 }], [0, { w: 1, h: 1, rx: 0, ry: 1 }]], size: ks([0, 1], [1, 1.2]), rotate: ks([0, 10], [1, -10]) }, rect)
  F('ink', { ...ring, size: ks([0, 1], [0.5, 0.5], [1, 1]) }, round)
  F('ink', { ...ring, shapes: [[0, { w: 1, h: 1, rx: 1, ry: 1 }], [0.5, { w: 2, h: 0.5, rx: 0, ry: 0 }], [1, { w: 1, h: 1, rx: 1, ry: 1 }]] }, rect)
  F('ink', { ...vee, size: ks([0, 1], [1, 2]), rotate: ks([0, 0], [1, 30]) }, rect)
  F('ink', { ...arch, weight: 1.5 }, rect); F('ink', { ...arch, weight: 0.5 }, round); F('ink', { ...ring, weight: 0.8 }, pen)
  F('ink', { ...arch, cuts: { start: cut('flat', { taper: -0.5 }) }, size: ks([0, 1], [1, 1.5]), offset: ks([0, 0.3], [1, -0.3]) }, rect)
  // Stamps per span, spacing.
  F('ink', arch, rect, { perSpan: 3 }); F('ink', arch, nib({ w: 50, h: 24, angle: 35 }), { perSpan: 0 }); F('ink', arch, rect, { perSpan: 7.6 }); F('ink', bowl, round, { perSpan: 2 })
  F('ink', arch, rect, { spacing: 40 }); F('ink', bowl, rect, { spacing: 25 }); F('ink', line, round, { spacing: 100 }); F('ink', ring, round, { spacing: 60 })
  F('ink', arch, rect, { spacing: 0, perSpan: 4 })
  F('ink', arch, nib({ w: 40, h: 20, perSpan: 3, perCorner: 1 })); F('ink', arch, nib({ w: 40, h: 20, perSpan: 5, perCorner: 0 })); F('ink', arch, nib({ w: 40, h: 20, perSpan: 5, perCorner: 2.6 }))
  // Chord S and E.
  F('ink', { ...arch, S: [0, 0], E: [300, 0] }, rect); F('ink', { ...line, S: [0, 0], E: [0, 100], cuts: { start: cut('level') } }, rect)
  F('ink', { ...arch, S: [0, 0], E: [0, 0] }, rect, { levels: [0] })
  // Sharp corners: the nib sweeps round them.
  for (const s of [vee, chevron, boxed, hook]) {
    F('ink', s, round, { caps: 'nib' }); F('ink', s, rect, { caps: 'nib' }); F('ink', s, square); F('ink', s, pen, { caps: 'nib' })
  }
  F('ink', vee, nib({ w: 40, h: 40, follow: 1 })); F('ink', chevron, nib({ w: 50, h: 20, follow: 0.5 }))
  // Pointed pen.
  for (const s of [line, arch, sCurve, ring, vee, boxed, hook]) { F('ink', s, rect, { model: 'pointed' }); F('ink', s, pen, { model: 'pointed', caps: 'nib' }) }
  F('ink', { ...arch, cuts: { start: cut('round'), end: cut('round') } }, rect, { model: 'pointed' })
  F('ink', { ...vee, cuts: { start: cut('flat', { taper: -0.5 }) } }, rect, { model: 'pointed' })
  F('ink', { ...arch, size: ks([0, 1], [0.5, 2], [1, 1]) }, rect, { model: 'pointed' })
  // Round ends.
  F('ink', { ...arch, cuts: { start: cut('round'), end: cut('round') } }, rect); F('ink', { ...sCurve, cuts: { end: cut('round') } }, pen)
  F('ink', { ...ring, cuts: { start: cut('round'), end: cut('level') } }, rect)
  // Closed stroke with nib caps.
  F('ink', ring, rect, { caps: 'nib', levels: [0] })
})
