import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/attach.ts'
import { Path } from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import type { Cubic, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { record } from './record.ts'

// Paths aren't serialisable: a case holds one as plain data {start, segs, closed, metric?}, rebuilt on both sides, and
// a returned path is written as {start, segs, closed, kinds}. Mounts, targets, pairs and aligns are plain data with
// their paths as such data; a Segs is a number or [a, b].
type D = { start: Pt, segs: Cubic[], closed: boolean, metric?: Pt }
type Segs = number | [number, number]
const P = (d: D) => new Path(d.start, d.segs, d.closed, undefined, d.metric ?? [1, 1])
const ser = (p: Path) => ({ start: p.start, segs: p.segs, closed: p.closed, kinds: p.kinds })
const ln = (a: Pt, b: Pt): Cubic => [a, [a[0] + (b[0] - a[0]) / 3, a[1] + (b[1] - a[1]) / 3], [a[0] + 2 * (b[0] - a[0]) / 3, a[1] + 2 * (b[1] - a[1]) / 3], b]
const K = 0.5522847498307936

type MountD = { on: D, at: number, from?: number, segs?: Segs }
type TargetD = { on: D, at: number, segs?: Segs, from?: number } | { point: Pt, from?: number } | { cross: D, with: D, which?: 'first' | 'last' | number, from?: number }
type AlignD = { on: D, at: number, segs?: Segs, to?: 'tangent' | 'normal', angle?: number, from?: number }
const mount = (a: MountD) => ({ ...a, on: P(a.on) })
const target = (t: TargetD): m.Pair => 'cross' in t ? { ...t, cross: P(t.cross), with: P(t.with) } : 'on' in t ? { ...t, on: P(t.on) } : t

const mod = {
  attach: (p: D, a: MountD, b?: MountD) => ser(m.attach(P(p), mount(a), b && mount(b))),
  target: (t: TargetD) => m.target(target(t)),
  place: (p: D, pairs: TargetD[], align?: AlignD) => ser(m.place(P(p), pairs.map(target), align && { ...align, on: P(align.on) })),
  pathAttach: (p: D, on: D, at: number, o?: { from?: number, segs?: Segs, end?: number | { on?: D, at: number, from?: number, segs?: Segs } }) => {
    const end = o?.end === undefined || typeof o.end === 'number' ? o?.end : { ...o.end, on: o.end.on && P(o.end.on) }
    return ser(P(p).attach(P(on), at, { ...o, end }))
  },
}

const stem: D = { start: [0, 0], segs: [ln([0, 0], [0, 100])], closed: false }
const wave: D = { start: [0, 0], segs: [[[0, 0], [30, 60], [70, 60], [100, 0]], [[100, 0], [130, -60], [170, -60], [200, 0]]], closed: false }
const arch: D = { start: [0, 0], segs: [[[0, 0], [0, 55], [45, 100], [100, 100]], ln([100, 100], [180, 100]), [[180, 100], [230, 100], [260, 60], [260, 0]]], closed: false }
const square: D = { start: [0, 0], segs: [ln([0, 0], [100, 0]), ln([100, 0], [100, 60]), ln([100, 60], [0, 60]), ln([0, 60], [0, 0])], closed: true }
const circle: D = {
  start: [110, 60], closed: true,
  segs: [[[110, 60], [110, 60 + 50 * K], [60 + 50 * K, 110], [60, 110]], [[60, 110], [60 - 50 * K, 110], [10, 60 + 50 * K], [10, 60]],
    [[10, 60], [10, 60 - 50 * K], [60 - 50 * K, 10], [60, 10]], [[60, 10], [60 + 50 * K, 10], [110, 60 - 50 * K], [110, 60]]],
}
const bar: D = { start: [-20, 40], segs: [ln([-20, 40], [300, 40])], closed: false }
const diag: D = { start: [-20, 120], segs: [ln([-20, 120], [260, -50])], closed: false }
const dot: D = { start: [5, 5], segs: [], closed: false }
const waveM: D = { ...wave, metric: [2, 0.5] }
const archM: D = { ...arch, metric: [0.5, 2] }
const stemM: D = { ...stem, metric: [3, 1] }
const circleM: D = { ...circle, metric: [2, 1] }
const barM: D = { ...bar, metric: [2, 3] }
const kinked: D = { start: [0, 0], segs: [ln([0, 0], [100, 0]), [[100, 0], [140, 10], [150, 50], [140, 90]], ln([140, 90], [60, 90]), [[60, 90], [30, 90], [10, 70], [0, 50]]], closed: false }

export default record(mod, F => {
  // attach
  const ons = [wave, arch, square, circle, waveM, circleM, dot]
  for (const p of [stem, wave, arch, stemM, waveM, kinked, dot])
    for (const on of ons) {
      F('attach', p, { on, at: 0.3 })
      F('attach', p, { on, at: 0.6, from: 0.5 })
      F('attach', p, { on, at: 0.25 }, { on, at: 0.75 })
    }
  for (const p of [stem, wave, arch, stemM, kinked]) {
    F('attach', p, { on: wave, at: 0.5, segs: 1 }, { on: arch, at: 0.9, segs: [1, 2] })
    F('attach', p, { on: arch, at: 0.5, segs: [0, 1], from: 0.2 }, { on: circle, at: 0.2, from: 0.8 })
    F('attach', p, { on: arch, at: 0.5, segs: -1 }, { on: arch, at: 0.1, segs: -1 })
    F('attach', p, { on: wave, at: 0.4 }, { on: wave, at: 0.4 })
    F('attach', p, { on: wave, at: 0.2, from: 0.5 }, { on: wave, at: 0.9, from: 0.5 })
    F('attach', p, { on: barM, at: 0.1 }, { on: circleM, at: 0.6 })
  }
  F('attach', stem, { on: wave, at: 0 }, { on: wave, at: 1 })
  F('attach', stem, { on: bar, at: 0.2 }, { on: diag, at: 0.9, from: 0.7 })
  // target
  for (const on of ons) { F('target', { on, at: 0.4 }); F('target', { on, at: 0 }); if (on !== dot) F('target', { on, at: 1, segs: 0 }) }
  F('target', { on: arch, at: 0.5, segs: [1, 2] })
  F('target', { point: [3, 4] })
  F('target', { cross: bar, with: wave })
  F('target', { cross: bar, with: wave, which: 'first' }); F('target', { cross: bar, with: wave, which: 'last' })
  F('target', { cross: bar, with: wave, which: 0 }); F('target', { cross: bar, with: wave, which: 1 })
  F('target', { cross: wave, with: bar, which: 'last' }); F('target', { cross: diag, with: circle, which: 1 })
  F('target', { cross: circleM, with: barM, which: 'first' })
  // place
  const T = (at: number): TargetD => ({ on: arch, at })
  for (const p of [stem, wave, arch, stemM, waveM, kinked, square]) {
    F('place', p, [])
    F('place', p, [T(0.3)]); F('place', p, [{ ...T(0.5), from: 0.4 }]); F('place', p, [{ point: [40, 50] }])
    F('place', p, [T(0.1), T(0.8)]); F('place', p, [{ ...T(0.1), from: 0.2 }, { point: [200, 10], from: 0.9 }])
    F('place', p, [T(0.1), { cross: bar, with: wave }])
    F('place', p, [{ point: [0, 0] }, { point: [100, 20] }, { point: [10, 120] }])
    F('place', p, [{ point: [0, 0], from: 0.1 }, { point: [100, 100], from: 0.5 }, { point: [200, 0], from: 1 }])
    F('place', p, [{ point: [0, 0] }, { point: [50, 50] }, { point: [100, 100] }])
    F('place', p, [{ point: [0, 0] }, { point: [100, 0] }, { point: [100, 100] }, { point: [0, 100] }])
    F('place', p, [{ point: [0, 0] }, { point: [30, 40] }, { point: [100, 60] }, { point: [150, 20] }, { point: [220, 0] }])
    F('place', p, [{ on: wave, at: 0.1 }, { on: wave, at: 0.4 }, { on: arch, at: 0.5 }, { on: arch, at: 0.9 }])
    F('place', p, [{ point: [0, 0] }, { point: [10, 10] }, { point: [20, 20] }, { point: [50, 50] }])
    F('place', p, [{ point: [0, 0], from: 0.5 }, { point: [40, 40], from: 0.5 }, { point: [80, 10], from: 0.5 }, { point: [20, 90], from: 0.5 }])
    F('place', p, [{ point: [0, 0], from: 0.9 }, { point: [80, 20], from: 0.2 }, { point: [160, -10], from: 0.5 }, { point: [200, 30], from: 0.1 }])
    F('place', p, [T(0.3)], { on: wave, at: 0.5 })
    F('place', p, [T(0.3)], { on: wave, at: 0.5, to: 'normal' })
    F('place', p, [T(0.3)], { on: wave, at: 0.5, to: 'normal', angle: 30 })
    F('place', p, [T(0.3)], { on: arch, at: 0.5, angle: -45, from: 0.5, segs: [0, 1] })
    F('place', p, [], { on: circle, at: 0.2 })
    F('place', p, [], { on: circle, at: 0.2, to: 'normal', from: 0.5 })
    F('place', p, [{ point: [10, 10], from: 0.5 }], { on: circleM, at: 0.7, angle: 90 })
  }
  F('place', dot, [{ point: [1, 2] }, { point: [3, 4] }])
  F('place', dot, [{ point: [1, 2] }, { point: [3, 4] }, { point: [9, 4] }, { point: [0, 0] }])
  // Path.attach
  for (const p of [stem, wave, stemM])
    for (const on of [wave, arch, circleM]) {
      F('pathAttach', p, on, 0.4)
      F('pathAttach', p, on, 0.4, {})
      F('pathAttach', p, on, 0.2, { from: 0.5 })
      F('pathAttach', p, on, 0.2, { segs: 0 })
      F('pathAttach', p, on, 0.2, { end: 0.8 })
      F('pathAttach', p, on, 0.2, { from: 0.1, segs: 0, end: 0.8 })
      F('pathAttach', p, on, 0.2, { end: { at: 0.7, from: 0.9 } })
      F('pathAttach', p, on, 0.2, { end: { on: bar, at: 0.7, segs: 0 } })
      F('pathAttach', p, on, 0.2, { end: { on: barM, at: 0.2 } })
    }
})
