import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import type { Cubic, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cubics } from './fixtures.ts'
import { record } from './record.ts'

// Paths are objects, so a case names one by a spec, { start, segs, closed?, arrives?, metric?, kinds? }, and every
// path it answers is written as { start, segs, closed, arrives, metric, kinds } (kinds read for every node). A drawing
// is a list of steps from a point: ['line', to] | ['curve', c1, c2, to] | ['quarter', to, from?, tension?] |
// ['arch', to, peak, tension?] | ['join', spec] | ['close']. aligned isn't exported in the TS: handle covers it.
type Spec = { start: Pt, segs: Cubic[], closed?: boolean, arrives?: m.Heading, metric?: Pt, kinds?: (m.NodeKind | null)[] }
const P = (s: Spec) => new m.Path(s.start, s.segs, s.closed ?? false, s.arrives, s.metric ?? [1, 1], s.kinds?.map(k => k ?? undefined))
const ser = (p: m.Path) => ({ start: p.start, segs: p.segs, closed: p.closed, arrives: p.arrives ?? null, metric: p.metric, kinds: p.kinds })
type Step = [string, ...any[]]
const draw = (from: Pt, steps: Step[]) => steps.reduce((p, [k, a, b, c]) =>
  k === 'line' ? p.line(a) : k === 'curve' ? p.curve(a, b, c) :
  k === 'quarter' ? p.quarter(a, { from: b ?? undefined, tension: c ?? undefined }) :
  k === 'arch' ? p.arch(a, b, { tension: c ?? undefined }) : k === 'join' ? p.join(P(a)) : p.close(), m.from(from))
const affine = (k: number[]) => (p: Pt): Pt => [k[0]! * p[0] + k[2]! * p[1] + k[4]!, k[1]! * p[0] + k[3]! * p[1] + k[5]!]

const mod = {
  path: (s: Spec) => ser(P(s)),
  of: (segs: Cubic[], closed?: boolean, metric?: Pt) => ser(m.Path.of(segs, closed, metric)),
  measured: (s: Spec, metric: Pt) => ser(P(s).measured(metric)),
  reshaped: (s: Spec, segs: Cubic[]) => ser(P(s).reshaped(segs)),
  handle: (s: Spec, i: number, side: m.HandleSide, to: Pt) => ser(P(s).handle(i, side, to)),
  length: (s: Spec) => P(s).length,
  locate: (s: Spec, ps: number[]) => ps.map(p => P(s).locate(p)),
  progress: (s: Spec, i: number, ts: number[]) => ts.map(t => P(s).progress(i, t)),
  end: (s: Spec) => P(s).end,
  draw: (from: Pt, steps: Step[]) => ser(draw(from, steps)),
  at: (s: Spec, ps: number[]) => ps.map(p => P(s).at(p)),
  crossY: (s: Spec, y: number) => P(s).crossY(y),
  crossX: (s: Spec, x: number) => P(s).crossX(x),
  atY: (s: Spec, y: number, which?: m.Which) => P(s).atY(y, which),
  atX: (s: Spec, x: number, which?: m.Which) => P(s).atX(x, which),
  tAtY: (s: Spec, y: number, which?: m.Which) => P(s).tAtY(y, which),
  tAtX: (s: Spec, x: number, which?: m.Which) => P(s).tAtX(x, which),
  chordY: (s: Spec, y: number) => ser(P(s).chordY(y)),
  chordX: (s: Spec, x: number) => ser(P(s).chordX(x)),
  slice: (s: Spec, p0: number, p1: number) => ser(P(s).slice(p0, p1)),
  aperture: (s: Spec, offset: number, angle?: number) => ser(P(s).aperture(offset, angle)),
  divide: (s: Spec, n: number) => ser(P(s).divide(n)),
  direction: (s: Spec, ps: number[]) => ps.map(p => P(s).direction(p)),
  crossings: (s: Spec, o: Spec) => P(s).crossings(P(o)),
  cut: (s: Spec, ps: number[]) => ser(P(s).cut(ps)),
  angle: (s: Spec) => P(s).angle,
  nodeCount: (s: Spec) => P(s).nodeCount,
  node: (s: Spec, i: number) => P(s).node(i),
  within: (s: Spec, segs: m.Segs, p: number) => P(s).within(segs, p),
  nodeProgress: (s: Spec) => Array.from({ length: P(s).nodeCount + 1 }, (_, i) => P(s).nodeProgress(i)),
  nodes: (s: Spec) => P(s).nodes,
  distances: (s: Spec) => P(s).distances,
  reverse: (s: Spec) => ser(P(s).reverse()),
  startAt: (s: Spec, r: number) => ser(P(s).startAt(r)),
  map: (s: Spec, k: number[]) => ser(P(s).map(affine(k))),
  warp: (s: Spec, k: number[]) => ser(P(s).warp(affine(k))),
  line: (a: Pt, b: Pt) => ser(m.line(a, b)),
  poly: (p: Pt, ps: Pt[]) => ser(m.poly(p, ...ps)),
}

const { arch, s, loop, skew, line, handleOnNode } = cubics
// An oval of four quarters (all smooth), a D (two corners), an open S of two cubics, a triangle, and an open hook.
const oval: Spec = { start: [0, 50], segs: [[[0, 50], [0, 77.6], [22.4, 100], [50, 100]], [[50, 100], [77.6, 100], [100, 77.6], [100, 50]], [[100, 50], [100, 22.4], [77.6, 0], [50, 0]], [[50, 0], [22.4, 0], [0, 22.4], [0, 50]]], closed: true }
const D: Spec = { start: [0, 0], segs: [[[0, 0], [0, 33], [0, 66], [0, 100]], [[0, 100], [80, 100], [80, 0], [0, 0]]], closed: true }
const S: Spec = { start: [0, 0], segs: [s!, [[100, 0], [150, 100], [180, 40], [200, 60]]] }
const tri: Spec = { start: [0, 0], segs: [[[0, 0], [33, 0], [66, 0], [100, 0]], [[100, 0], [83, 33], [66, 66], [50, 100]], [[50, 100], [33, 66], [16, 33], [0, 0]]], closed: true }
const hook: Spec = { start: [10, 0], segs: [[[10, 0], [10, 60], [30, 90], [60, 90]], [[60, 90], [90, 90], [100, 60], [100, 40]]] }
const sym: Spec = { start: [0, 0], segs: [[[0, 0], [0, 50], [25, 100], [50, 100]], [[50, 100], [75, 100], [100, 50], [100, 0]]] }
const wide: Spec = { ...oval, metric: [2, 1] }
const kinded: Spec = { ...oval, kinds: ['corner', null, 'symmetric', null] }
const openKinded: Spec = { ...hook, kinds: [null, 'smooth', 'corner'] }
const single: Spec = { start: [0, 0], segs: [arch!] }
const dot: Spec = { start: [5, 5], segs: [] }
const paths = { oval, D, S, tri, hook, sym, wide, kinded, openKinded, single }
const ps = [0, 0.1, 0.25, 0.5, 0.73, 1, -0.2, 1.3]

// Throws can't be recorded, so a case that throws in the TS is left out.
export default record(mod, R => {
  const F = (fn: string, ...a: unknown[]) => { try { (mod as any)[fn](...structuredClone(a)) } catch { return } R(fn, ...a) }
  for (const p of [...Object.values(paths), dot]) {
    F('path', p); F('length', p); F('end', p); F('nodeCount', p); F('nodes', p); F('distances', p); F('angle', p)
    F('at', p, ps); F('direction', p, ps); F('locate', p, ps); F('nodeProgress', p)
  }
  for (const p of Object.values(paths)) {
    F('reverse', p); F('divide', p, 3); F('divide', p, 1); F('cut', p, [0.3, 0.6, 0.6000000001, 0, 1])
    F('map', p, [1.1, 0.2, -0.3, 0.9, 5, -7]); F('warp', p, [1.1, 0.2, -0.3, 0.9, 5, -7])
    F('slice', p, 0.2, 0.7); F('slice', p, 0, 1); F('slice', p, 0.1, 0.15)
    F('progress', p, p.segs.length - 1, [0, 0.3, 1]); F('node', p, 1)
    F('within', p, 0, 0.5); F('within', p, -1, 0.25); F('within', p, [0, 1], 0.5)
  }
  F('progress', oval, 0, [0, 0.5, 1]); F('progress', wide, 2, [0.2, 0.9])
  F('nodeCount', dot); F('end', dot); F('at', dot, [0.5]); F('direction', dot, [0.5])
  F('of', [arch, s], false); F('of', [loop], true, [1.5, 1])
  F('measured', oval, [3, 1]); F('measured', kinded, [0.5, 2])
  F('reshaped', kinded, oval.segs.map(c => c.map(([x, y]) => [x + 1, y * 2]))); F('reshaped', kinded, [arch])
  F('reshaped', dot, [])
  for (const p of [oval, kinded, sym, tri, wide]) {
    F('handle', p, 1, 'in', [40, 110]); F('handle', p, 1, 'out', [70, 90])
  }
  F('handle', oval, 0, 'in', [-5, 40]); F('handle', kinded, 0, 'out', [5, 70]); F('handle', kinded, 2, 'out', [60, 10])
  F('handle', hook, 1, 'in', [50, 95]); F('handle', openKinded, 1, 'out', [80, 95]); F('handle', hook, 0, 'out', [10, 50])
  for (const p of [oval, D, tri, hook, wide]) {
    F('crossY', p, 50); F('crossY', p, 0); F('crossX', p, 50); F('crossX', p, 10)
    F('atY', p, 30); F('atY', p, 30, 'last'); F('atY', p, 30, 0); F('atX', p, 50, 'first'); F('atX', p, 50, 'last')
    F('tAtY', p, 30, 'last'); F('tAtX', p, 50); F('chordY', p, 40); F('chordX', p, 50)
  }
  F('crossY', S, 0); F('atY', S, 0, 1); F('atY', S, 0, 2); F('tAtY', S, 0, 2); F('crossX', single, 50)
  for (const p of [oval, D, tri, wide, kinded]) {
    F('aperture', p, 0); F('aperture', p, 0.5); F('aperture', p, -0.4, 90); F('aperture', p, 0.2, 30)
    F('startAt', p, 1); F('startAt', p, 2); F('startAt', p, 0)
  }
  F('crossings', oval, D); F('crossings', oval, tri); F('crossings', S, hook); F('crossings', oval, single)
  F('crossings', S, { start: [0, 50], segs: [[[0, 50], [60, 50], [140, 50], [200, 50]]] })
  F('cut', kinded, [0.1, 0.4]); F('cut', openKinded, [0.25, 0.75]); F('cut', S, [0, 1, 0.5])
  F('draw', [0, 0], [['line', [100, 0]], ['line', [100, 100]], ['close']])
  F('draw', [0, 0], [['quarter', [50, 100]], ['quarter', [100, 0]]])
  F('draw', [0, 0], [['quarter', [50, 100], 'h']])
  F('draw', [0, 0], [['quarter', [50, 100], null, 0.7], ['quarter', [100, 0], null, 0.4], ['line', [0, 0]]])
  F('draw', [0, 0], [['arch', [100, 0], 120]]); F('draw', [10, 20], [['arch', [100, 0], 80, 0.6], ['close']])
  F('draw', [0, 0], [['curve', [0, 50], [50, 100], [100, 100]], ['join', hook]])
  F('draw', [0, 0], [['line', [10, 0]], ['join', { start: [10, 0], segs: [[[10, 0], [20, 10], [30, 10], [40, 0]]] }]])
  F('draw', [0, 0], [['line', [10, 0]], ['join', openKinded]])
  F('draw', [0, 0], [['line', [10, 0]], ['join', dot]])
  F('draw', [0, 0], [['join', openKinded], ['close']])
  F('draw', [0, 0], [['line', [100, 0]], ['line', [0, 0]], ['close']])
  F('draw', [5, 5], [['close']])
  F('line', [0, 0], [30, 40]); F('poly', [0, 0], [[10, 0], [10, 10], [0, 10]]); F('poly', [1, 2], [])
})
