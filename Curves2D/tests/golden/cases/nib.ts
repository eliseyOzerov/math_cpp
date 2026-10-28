import * as m from '../../../../../../apps/Fountain/fountain-ts/src/ink/nib.ts'
import type { Cubic } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { record } from './record.ts'

const NIB = m.NIB
const nibs: m.NibInput[] = [
  undefined, null, 40, 0, 72.5,
  {}, { w: 60 }, { w: 60, h: 30, angle: 30 }, { w: 50, h: 20, rx: 0, ry: 0, angle: -20 },
  { w: 40, h: 80, rx: [1, 0, 1, 0], ry: 0.5 }, { w: 70, h: 30, rx: [0.2, 0.4, 0.6, 0.8], ry: [1, 0.5, 0, 0.25], follow: 1 },
  { w: 64, h: 64, follow: 0.5, angle: 15, perCorner: 3 }, { w: 90, h: 24, angle: 45, perCorner: 8 },
  { weight: 60 }, { weight: 60, contrast: 0.5 }, { weight: 80, contrast: 0.4, stress: 20 }, { weight: 40, contrast: 2, stress: 85 },
  { weight: 50, stress: -30 },
]
const shapes: m.Shape[] = [
  { w: 60, h: 40, rx: 1, ry: 1 }, { w: 60, h: 40, rx: 0, ry: 0 }, { w: 100, h: 20, rx: 0.5, ry: 0.5 },
  { w: 50, h: 50, rx: [1, 0, 1, 0], ry: [1, 0, 1, 0] }, { w: 80, h: 30, rx: [0.2, 0.4, 0.6, 0.8], ry: 0.3 },
  { w: 30, h: 90, rx: 2, ry: -1 }, { w: 40, h: 40, rx: [0.5], ry: [0.5, 1] }, { w: 70, h: 20, rx: 1e-7, ry: 1 },
  { w: 0, h: 0, rx: 1, ry: 1 },
]
const lines: Cubic[] = [
  [[0, 0], [100, 0], [200, 0], [300, 0]], [[0, 0], [0, 100], [0, 200], [0, 300]], [[0, 0], [100, 100], [200, 200], [300, 300]],
  [[0, 0], [100, 10], [200, 20], [300, 30]], [[0, 0], [100, 40], [200, 80], [300, 120]], [[0, 0], [-100, 100], [-200, 200], [-300, 300]],
  [[0, 0], [100, 55], [200, 110], [300, 165]], [[0, 0], [100, 0], [200, 0], [300, 5]],
]
const curves: Cubic[] = [
  [[0, 0], [0, 150], [150, 300], [300, 300]], [[0, 0], [50, 100], [50, -100], [100, 0]], [[0, 0], [0, 0], [100, 50], [100, 100]],
  [[0, 0], [10, 0], [20, 0], [300, 2]],
]
const segs = (...c: Cubic[]) => c
const comps: m.Compensation[] = [{}, { round: 1.1 }, { diagonal: 0.9 }, { round: 1.06, diagonal: 0.92 }, { round: 1, diagonal: 1 }]

// transitionKeys' angle along the stroke as a line a + b·u.
const tk = { transitionKeysAt: (n: m.NibInput, tr: number, [a, b]: [number, number], samples?: number) => m.transitionKeys(n, tr, u => a + b * u, samples) }

export default record({ ...m, ...tk }, F => {
  // nibOf, stemNib, stemPenOf
  for (const n of nibs) F('nibOf', n)
  F('nibOf')
  for (const p of [{ weight: 60 }, { weight: 60, contrast: 0.5 }, { weight: 80, contrast: 0.4, stress: 20 }, { weight: 30, contrast: -1 }, { weight: 40, contrast: 0.2, stress: 90 }, { weight: 40, contrast: 0.99, stress: 10 }, { weight: 55, stress: 45 }])
    F('stemNib', p)
  for (const n of nibs) F('stemPenOf', n)
  F('stemPenOf')
  // radii, uniform, mixShape
  for (const r of [1, 0, 0.5, -1, [1, 0, 1, 0], [0.3], [0.1, 0.2], [0.1, 0.2, 0.3, 0.4, 0.5]]) { F('radii', r); F('uniform', r) }
  F('uniform', [0.5, 0.5, 0.5, 0.5]); F('uniform', [0.5, 0.5, 0.5, 0.5 + 1e-10]); F('uniform', [0.5, 0.5, 0.5, 0.5 + 1e-8])
  for (const u of [0, 0.25, 0.5, 1, -0.5, 1.5]) {
    F('mixShape', shapes[0], shapes[2], u)
    F('mixShape', shapes[3], shapes[4], u)
  }
  F('mixShape', shapes[0], shapes[1], 0.3); F('mixShape', shapes[4], shapes[0], 0.7); F('mixShape', shapes[5], shapes[6], 0.5)
  F('mixShape', shapes[3], shapes[3], 0.5); F('mixShape', shapes[0], shapes[3], 0.4); F('mixShape', shapes[8], shapes[7], 0.5)
  // cornerArcs
  for (const s of shapes) F('cornerArcs', s)
  // stampPolygon
  for (const s of shapes) { F('stampPolygon', s); F('stampPolygon', s, 0.7, 3) }
  F('stampPolygon', shapes[0], -1.2, 8); F('stampPolygon', shapes[1], 0.3, 1); F('stampPolygon', shapes[2], Math.PI / 2, 5)
  F('stampPolygon', shapes[3], 2, 0); F('stampPolygon', shapes[4], 0.1, 12); F('stampPolygon', shapes[1], 0, 5)
  F('stampPolygon', shapes[0], 0.5, 2.5)
  // nibWidth, stemWidth
  for (const n of nibs) { F('stemWidth', n); F('stemWidth', n, 0.5); F('nibWidth', n, 0.4); F('nibWidth', n, -1, 2); F('nibWidth', n, Math.PI / 4) }
  F('stemWidth'); F('nibWidth', undefined, 1)
  // shapesOf
  const nib = { ...NIB, rx: [0.5, 1, 0.5, 1], ry: 0.7 }
  F('shapesOf', null, NIB); F('shapesOf', undefined, NIB); F('shapesOf', [], NIB)
  F('shapesOf', [[0, shapes[0]], [1, shapes[1]]], NIB); F('shapesOf', [[0.5, shapes[3]]], nib)
  F('shapesOf', [{ w: 1, h: 1 }], NIB); F('shapesOf', [{ w: 2 }, { h: 0.5 }], NIB); F('shapesOf', [{ w: 2 }, { h: 0.5 }, { rx: 0 }], nib)
  F('shapesOf', [{}, { ry: [0, 1, 0, 1] }], nib); F('shapesOf', [{ w: 1.5, h: 0.8, rx: 0, ry: 0 }, {}, { w: 0.5 }, {}], NIB)
  // segmentWeight
  for (const c of comps) for (const l of [...lines, ...curves]) F('segmentWeight', l, c)
  F('segmentWeight', [[0, 0], [0, 0], [0, 0], [0, 0]], { round: 1.1, diagonal: 0.8 })
  // weightCurve
  const bowl = segs(curves[0]!, [[300, 300], [450, 300], [600, 150], [600, 0]])
  const mixed = segs(lines[0]!, curves[0]!, lines[2]!)
  const square = segs([[0, 0], [100, 0], [200, 0], [300, 0]], [[300, 0], [300, 100], [300, 200], [300, 300]], [[300, 300], [200, 300], [100, 300], [0, 300]], [[0, 300], [0, 200], [0, 100], [0, 0]])
  const diamond = segs([[0, 0], [50, 50], [100, 100], [150, 150]], [[150, 150], [100, 200], [50, 250], [0, 300]], [[0, 300], [-50, 250], [-100, 200], [-150, 150]], [[-150, 150], [-100, 100], [-50, 50], [0, 0]])
  const ring = segs([[0, 0], [100, 55], [200, 110], [300, 165]], [[300, 165], [300, 265], [300, 365], [300, 465]], [[300, 465], [150, 300], [0, 150], [0, 0]])
  const sets = [bowl, mixed, square, diamond, ring, [lines[0]!], [curves[0]!], [], [lines[0]!, lines[1]!]]
  for (const s of sets) for (const c of comps) for (const closed of [false, true]) F('weightCurve', s, closed, c, 40)
  F('weightCurve', mixed, false, comps[3], 0); F('weightCurve', mixed, false, comps[3], 1000); F('weightCurve', square, true, comps[3], 150)
  F('weightCurve', ring, true, { round: 1.1, diagonal: 1 }, 60); F('weightCurve', diamond, true, { round: 1, diagonal: 0.8 }, 60)
  for (const n of [40, { w: 60, h: 30, angle: 30 }, { weight: 80, contrast: 0.4, stress: 20 }, { weight: 60, contrast: 0.5 }])
    for (const tr of [0, 0.6, -0.5, 2, -3]) F('transitionKeysAt', n, tr, [0, Math.PI], 17)
  F('transitionKeysAt', { weight: 90, contrast: 0.6, stress: 20 }, 0.6, [0.3, 2])
})
