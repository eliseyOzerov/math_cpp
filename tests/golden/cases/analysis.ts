import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/analysis.ts'
import { Path } from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import type { Cubic, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { record } from './record.ts'

// Paths aren't serialisable: a case holds one as plain data {start, segs, closed, metric?}, rebuilt on both sides. A
// returned path is written as {start, segs, closed, kinds}; a function (nearness) by its values at progresses `us`;
// a Selection by each picked path with its nodes and weights.
type D = { start: Pt, segs: Cubic[], closed: boolean, metric?: Pt }
const P = (d: D) => new Path(d.start, d.segs, d.closed, undefined, d.metric ?? [1, 1])
const ser = (p: Path) => ({ start: p.start, segs: p.segs, closed: p.closed, kinds: p.kinds })
const ln = (a: Pt, b: Pt): Cubic => [a, [a[0] + (b[0] - a[0]) / 3, a[1] + (b[1] - a[1]) / 3], [a[0] + 2 * (b[0] - a[0]) / 3, a[1] + 2 * (b[1] - a[1]) / 3], b]
const K = 0.5522847498307936
const ramp = (d0: number, d1: number) => (u: number) => d0 + (d1 - d0) * u

const wave: D = { start: [0, 0], segs: [[[0, 0], [30, 60], [70, 60], [100, 0]], [[100, 0], [130, -60], [170, -60], [200, 0]]], closed: false }
const arch: D = { start: [0, 0], segs: [[[0, 0], [0, 55], [45, 100], [100, 100]], ln([100, 100], [180, 100]), [[180, 100], [230, 100], [260, 60], [260, 0]]], closed: false }
const square: D = { start: [0, 0], segs: [ln([0, 0], [100, 0]), ln([100, 0], [100, 60]), ln([100, 60], [0, 60]), ln([0, 60], [0, 0])], closed: true }
const open3: D = { start: [0, 0], segs: [ln([0, 0], [100, 0]), ln([100, 0], [100, 60]), ln([100, 60], [30, 90])], closed: false }
const circle: D = {
  start: [110, 60], closed: true,
  segs: [[[110, 60], [110, 60 + 50 * K], [60 + 50 * K, 110], [60, 110]], [[60, 110], [60 - 50 * K, 110], [10, 60 + 50 * K], [10, 60]],
    [[10, 60], [10, 60 - 50 * K], [60 - 50 * K, 10], [60, 10]], [[60, 10], [60 + 50 * K, 10], [110, 60 - 50 * K], [110, 60]]],
}
const loop: D = { start: [0, 0], segs: [[[0, 0], [150, 100], [-50, 100], [100, 0]]], closed: false }
const eight: D = { start: [0, 0], segs: [[[0, 0], [80, 0], [100, 40], [60, 60]], [[60, 60], [20, 80], [0, 40], [80, 0]]], closed: false }
const closedLoop: D = { start: [0, 0], segs: [[[0, 0], [150, 100], [-50, 100], [100, 0]], [[100, 0], [60, -40], [20, -40], [0, 0]]], closed: true }
const straight: D = { start: [0, 0], segs: [ln([0, 0], [100, 50])], closed: false }
const dot: D = { start: [5, 5], segs: [], closed: false }
const waveM: D = { ...wave, metric: [2, 0.5] }
const archM: D = { ...arch, metric: [0.5, 2] }
const circleM: D = { ...circle, metric: [2, 1] }
const squareM: D = { ...square, metric: [1, 3] }
const open = [wave, arch, open3, straight, waveM, archM]
const all = [...open, square, circle, loop, closedLoop, circleM, squareM, dot]
const ps: Pt[] = [[50, 50], [-20, 30], [130, 120], [0, 0], [100, 30]]
const us = [0, 0.1, 0.25, 0.5, 0.7, 1]

const mod = {
  project: (d: D, p: Pt) => m.project(P(d), p),
  extrema: (d: D) => m.extrema(P(d)),
  inflections: (d: D) => m.inflections(P(d)),
  side: (d: D, way: m.Way) => m.side(P(d), way),
  turns: (d: D, minDeg?: number) => m.turns(P(d), minDeg),
  nearness: (d: D, ats: number[], reach: number, falloff: string | undefined, xs: number[]) =>
    xs.map(m.nearness(P(d), ats, reach, falloff as never)),
  bbox: (d: D) => m.bbox(P(d)),
  curvature: (d: D, at: number) => m.curvature(P(d), at),
  normal: (d: D, at: number) => m.normal(P(d), at),
  osculating: (d: D, at: number) => m.osculating(P(d), at),
  mold: (d: D, at: number, to: Pt) => ser(m.mold(P(d), at, to)),
  offset: (d: D, r: number) => ser(m.offset(P(d), r)),
  offsetRamp: (d: D, d0: number, d1: number) => ser(m.offset(P(d), ramp(d0, d1))),
  outline: (d: D, d1: number, d2?: number) => m.outline(P(d), d1, d2),
  lineCrossings: (d: D, a: Pt, b: Pt, segment?: boolean) => m.lineCrossings(P(d), a, b, segment === undefined ? {} : { segment }),
  circleCrossings: (d: D, o: Pt, r: number) => m.circleCrossings(P(d), o, r),
  selfCrossings: (d: D) => m.selfCrossings(P(d)),
  arcs: (d: D, threshold?: number) => m.arcs(P(d), threshold),
  selectAt: (d: D, at: number, reach: number, o: { falloff?: string, along?: 'path' | 'plane', cut?: boolean } = {}) =>
    m.selectAt(P(d), at, reach, o as never).picked.map(p => ({ path: ser(p.path), nodes: p.nodes, weights: p.weights })),
}

export default record(mod, F => {
  for (const d of all) {
    for (const p of ps) F('project', d, p)
    F('extrema', d); F('inflections', d); F('bbox', d)
    for (const w of ['left', 'right', 'top', 'bottom']) F('side', d, w)
    F('turns', d); F('turns', d, 60); F('turns', d, 0)
    for (const at of us) { F('curvature', d, at); F('normal', d, at); F('osculating', d, at) }
    F('offset', d, 8); F('offset', d, -8); F('offsetRamp', d, 2, 12)
    F('outline', d, 5); F('outline', d, 6, 2)
    F('arcs', d); F('arcs', d, 3)
  }
  for (const d of all.filter(d => d.segs.length)) {
    F('mold', d, 0.5, [60, 70]); F('mold', d, 0, [10, 10]); F('mold', d, 0.8, [-5, 40])
    for (const f of ['smooth', 'linear', 'sphere', 'root', 'sharp', 'constant', undefined]) F('nearness', d, [0.3, 0.8], 60, f, us)
    F('nearness', d, [0.5], 0, 'smooth', us)
    F('nearness', d, [], 50, 'linear', us)
    F('nearness', d, [0, 1], 80, 'smooth', [0, 0.05, 0.5, 0.95, 1])
  }
  F('nearness', dot, [0], 10, 'smooth', us)
  const lines: [Pt, Pt][] = [[[50, -100], [50, 100]], [[-20, 20], [300, 20]], [[0, 30], [20, 30]], [[0, -70], [260, 130]], [[10, 10], [10, 10]]]
  for (const d of all.filter(d => d.segs.length)) for (const [a, b] of lines) { F('lineCrossings', d, a, b); F('lineCrossings', d, a, b, false); F('lineCrossings', d, a, b, true) }
  for (const d of all) {
    F('circleCrossings', d, [60, 60], 30); F('circleCrossings', d, [100, 0], 50); F('circleCrossings', d, [0, 0], 500); F('circleCrossings', d, [50, 50], 1)
  }
  for (const d of [loop, eight, closedLoop, wave, arch, square, circle, open3, straight, circleM, waveM]) F('selfCrossings', d)
  for (const d of all) {
    F('selectAt', d, 0.5, 40); F('selectAt', d, 0.37, 30, { falloff: 'linear' }); F('selectAt', d, 0.37, 30, { falloff: 'sharp', along: 'plane' })
    F('selectAt', d, 0.37, 50, { cut: false }); F('selectAt', d, 0.9, 25, { falloff: 'sphere', along: 'path', cut: true })
    F('selectAt', d, 0, 20); F('selectAt', d, 1, 20, { cut: false })
  }
})
