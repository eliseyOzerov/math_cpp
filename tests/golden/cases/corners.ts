import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/corners.ts'
import { arcCubic, arcCubics, type Contour, type Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { reverseContour, union } from '../../../../../../apps/Fountain/fountain-ts/src/curves/boolean.ts'
import { record } from './record.ts'

// Callbacks by name, since the TS takes functions (tests/corners.cpp has the same tables).
const radii: Record<string, (k: m.Corner) => number> = {
  byTurn: k => (k.turn < 0 ? 10 : 4),
  byAt: k => (k.at % 2 ? 0 : 12),
}
const reaches: Record<string, (k: m.Corner) => number> = {
  ten: () => 10, byTurn: k => (k.turn < 0 ? 0 : 15), none: () => 0,
}
const makes: Record<string, (a: Pt, ta: Pt, b: Pt, tb: Pt, k: m.Corner) => [Pt, Pt][] | Pt[][]> = {
  chamfer: (a, _ta, b) => [[a, b]],
  fillet: (a, ta, b, tb) => [arcCubic(a, ta, b, tb)],
  bulge: (a, _ta, b) => arcCubics(a, b, 0.6),
}
const sizes: Record<string, (k: m.Corner) => number> = { ten: () => 10, twenty: () => 20, byTurn: k => (k.turn < 0 ? 8 : 14) }
const shapes: Record<string, (k: m.Corner) => number | m.CornerShape> = {
  zero: () => 0, out: () => 0.5, in: () => -0.5, outFull: () => 1, inFull: () => -1, over: () => 1.5, under: () => -1.5,
  mixed: k => (k.turn < 0 ? 0.7 : -0.7),
  rect: () => ({ angle: 0.5, profile: m.caps.rect(1) }),
  rectIn: () => ({ angle: -0.8, profile: m.caps.rect(0.6) }),
  triangle: () => ({ angle: -0.6, profile: m.caps.triangle(1) }),
  arc: () => ({ angle: 0.4, profile: m.caps.arc(1) }),
  flatProfile: () => ({ angle: 0.5, profile: [[[-1, 0], [1, 0]]] }),
  emptyProfile: () => ({ angle: 0.5, profile: [] }),
  profileOnly: () => ({ angle: 0, profile: m.caps.rect(0.5) }),
  objectNoProfile: () => ({ angle: 0.5 }),
  curveNotch: () => ({ angle: 0, curve: [[[-1, 0], [0, 1.5]], [[0, 1.5], [1, 0]]] }),
  curveRound: () => ({ angle: 0, curve: [[[-1, 0], [-1, 0.9], [-0.55, 1.6], [0, 1.6]], [[0, 1.6], [0.55, 1.6], [1, 0.9], [1, 0]]] }),
  curveLoose: () => ({ angle: 0, curve: [[[-0.8, 0.1], [0, 2]], [[0, 2], [0.9, 0.2], [0.7, -0.3], [1.1, 0.3]]] }),
  curveWell: k => ({ angle: 0, curve: k.turn > 0 ? [[[-1, 0], [-0.7, 0.2], [-0.45, 0.35], [0, 0.35]], [[0, 0.35], [0.45, 0.35], [0.7, 0.2], [1, 0]]] : undefined }),
  curveEmpty: () => ({ angle: 0.5, curve: [] }),
}

const rect = (x: number, y: number, w: number, h: number): Contour => {
  const ps: Pt[] = [[x, y], [x, y + h], [x + w, y + h], [x + w, y]]
  return ps.map((p, i) => [p, ps[(i + 1) % 4]!])
}
const poly = (ps: Pt[]): Contour => ps.map((p, i) => [p, ps[(i + 1) % ps.length]!])
const K = 0.5522847498
const circle = (cx: number, cy: number, r: number): Contour => {
  const pts: Pt[] = [[cx + r, cy], [cx, cy + r], [cx - r, cy], [cx, cy - r]]
  return reverseContour(pts.map((p, i) => {
    const q = pts[(i + 1) % 4]!, a = (i * Math.PI) / 2, b = a + Math.PI / 2
    return [p, [p[0] - Math.sin(a) * r * K, p[1] + Math.cos(a) * r * K], [q[0] + Math.sin(b) * r * K, q[1] - Math.cos(b) * r * K], q]
  }))
}

// Outlines as a boolean leaves them: outer contours clockwise, holes counter-clockwise.
const square = [rect(0, 0, 100, 100)]
const bar = [rect(0, 0, 100, 20)]
const cross = union([rect(0, 40, 100, 20), rect(40, 0, 20, 100)])
const tee = union([rect(0, 80, 100, 20), rect(40, 0, 20, 90)])
const ell = [poly([[0, 0], [0, 100], [30, 100], [30, 30], [100, 30], [100, 0]])]
const tri = [poly([[0, 0], [50, 100], [100, 0]])]
const wedge = [poly([[0, 0], [100, 20], [0, 8]].reverse() as Pt[])]
const disc = [circle(50, 50, 40)]
const blob = union([rect(0, 0, 100, 60), circle(50, 60, 40)])
const ring = [rect(0, 0, 100, 100), reverseContour(rect(25, 25, 50, 50))]
const dotted = [(() => { const c = rect(0, 0, 100, 100); c.splice(1, 0, [[0, 100], [0, 100]]); return c })()]
const slant = [poly([[0, 0], [10, 100], [40, 90], [30, 0]])]
const fixtures: Record<string, Contour[]> = { square, bar, cross, tee, ell, tri, wedge, disc, blob, ring, dotted, slant, two: [...square.map(c => c.map(q => q.map(p => [p[0] + 300, p[1]] as Pt))), ...tri] }
const all = Object.values(fixtures)

// A straight stroke and a curved one, ending at the top.
const stroke = [rect(0, 0, 20, 100)]
const bent: Contour[] = [[
  [[0, 0], [0, 40], [4, 70], [10, 100]], [[10, 100], [30, 100]], [[30, 100], [34, 70], [30, 40], [20, 0]], [[20, 0], [0, 0]],
]]
const endTop = { p: [10, 100] as Pt, t: [0, 1] as Pt, r: 10 }
const endBent = { p: [20, 100] as Pt, t: [0.2, 1] as Pt, r: 10 }
const endSide = { p: [100, 10] as Pt, t: [1, 0] as Pt, r: 10 }

export default record({
  ...m,
  roundCornersBy: (cs: Contour[], name: string, opts?: object) => m.roundCorners(cs, radii[name]!, opts),
  replaceCornersBy: (cs: Contour[], reach: string, make: string, opts?: object) => m.replaceCorners(cs, reaches[reach]!, makes[make]!, opts),
  shapeCornersBy: (cs: Contour[], size: string, shape: string) => m.shapeCorners(cs, sizes[size]!, shapes[shape]!),
  capShape: (name: 'triangle' | 'rect' | 'arc', h?: number) => m.caps[name](h),
  shapeCornersSeen: (cs: Contour[], size: string, shape: string) => {
    const seen: unknown[] = []
    const contours = m.shapeCorners(cs, sizes[size]!, shapes[shape]!, (k, a, b) => seen.push({ contour: k.contour, at: k.at, a, b }))
    return { contours, seen }
  },
  cornerFrameAt: (a: Pt, b: Pt, p: Pt, q: Pt) => { const f = m.cornerFrame(a, b, p); return { to: f.to(q), from: f.from(q) } },
}, F => {
  for (const cs of all) { F('corners', cs); F('pits', cs); F('peaks', cs) }
  F('corners', []); F('corners', square, 100); F('corners', cross, 0); F('corners', blob, 30); F('corners', disc, 0.001)
  F('pits', cross, 100); F('pits', cross, 175); F('pits', blob, 120); F('pits', tee, 91)
  F('peaks', square, 100); F('peaks', tri, 70); F('peaks', slant, 5); F('peaks', slant, 40)

  for (const [name, cs] of Object.entries(fixtures)) { F('roundCorners', cs, 8); F('roundCorners', cs, 0); if (name !== 'disc') F('roundCorners', cs, 30) }
  F('roundCorners', bar, 50); F('roundCorners', bar, 10); F('roundCorners', square, 50); F('roundCorners', square, -5)
  F('roundCorners', cross, 6, { minDeg: 45 }); F('roundCorners', cross, 10, { share: 0.25 }); F('roundCorners', cross, 30, { share: 1 })
  F('roundCorners', blob, 6, { share: 0.9 }); F('roundCorners', ell, 12, { minDeg: 100 }); F('roundCorners', tri, 20, { share: 0.3, minDeg: 10 })
  for (const cs of [cross, tee, blob, ell, ring, tri, slant, dotted]) { F('roundCornersBy', cs, 'byTurn'); F('roundCornersBy', cs, 'byAt', { share: 0.4 }) }

  for (const reach of Object.keys(reaches)) for (const make of Object.keys(makes))
    for (const cs of [cross, blob, ring, tri]) F('replaceCornersBy', cs, reach, make)
  F('replaceCornersBy', tee, 'ten', 'chamfer', { share: 1 }); F('replaceCornersBy', ell, 'ten', 'bulge', { minDeg: 100, share: 0.3 })

  for (const shape of Object.keys(shapes)) for (const cs of [cross, tee, square, tri]) F('shapeCornersBy', cs, 'ten', shape)
  for (const size of Object.keys(sizes)) for (const shape of ['zero', 'mixed', 'rect', 'triangle', 'arc', 'inFull', 'over'])
    for (const cs of [blob, ring, ell, slant]) F('shapeCornersBy', cs, size, shape)

  for (const shape of ['curveNotch', 'curveRound', 'curveLoose', 'curveWell', 'curveEmpty'])
    for (const cs of [cross, tee, blob, ell, slant]) { F('shapeCornersBy', cs, 'byTurn', shape); F('shapeCornersSeen', cs, 'ten', shape) }
  F('cornerFrameAt', [0, 0], [10, 0], [5, 4], [0.5, 1.5]); F('cornerFrameAt', [0, 0], [10, 0], [5, 4], [33, -7])
  F('cornerFrameAt', [3, 1], [-4, 9], [8, 12], [-1, 0.2]); F('cornerFrameAt', [0, 0], [10, 0], [5, 0], [0.3, 0.7])
  F('cornerFrameAt', [2, 2], [8, 8], [5, 5], [-0.4, 2])

  F('capShape', 'triangle'); F('capShape', 'triangle', 0.5); F('capShape', 'rect'); F('capShape', 'rect', 1)
  F('capShape', 'arc'); F('capShape', 'arc', 0.3)

  const cp = (shape: unknown, align?: number) => (align === undefined ? { shape } : { shape, align })
  for (const h of [undefined, 0.3, 1]) for (const name of ['triangle', 'rect', 'arc'] as const) {
    const shape = m.caps[name](h)
    for (const align of [undefined, 1, 0.5, 0, -0.5, -1, 3]) F('capped', stroke, endTop, cp(shape, align))
  }
  const shape = m.caps.arc(0.8)
  for (const align of [undefined, 1, 0, -1]) {
    F('capped', bent, endBent, cp(m.caps.triangle(0.6), align)); F('capped', bent, endBent, cp(shape, align))
    F('capped', [rect(0, 0, 100, 20)], endSide, cp(m.caps.rect(0.4), align))
    F('capped', ring, { p: [100, 50], t: [1, 0], r: 10 }, cp(m.caps.triangle(), align))
  }
  F('capped', stroke, endTop, cp([])); F('capped', stroke, { p: [10, 100], t: [0, 2], r: 10 }, cp(m.caps.arc()))
  F('capped', stroke, { p: [10, 100], t: [0, 1], r: 10 }, cp([[[-1, 0], [-1, 1.5], [1, 1.5], [1, 0]]] as unknown, 0.5))
  F('capped', stroke, { p: [10, 130], t: [0, 1], r: 10 }, cp(m.caps.rect(1))); F('capped', [], endTop, cp(m.caps.rect(1)))
  F('capped', [[[[5, 5], [5, 5], [5, 5], [5, 5]]]], endTop, cp(m.caps.rect(1)))
})
