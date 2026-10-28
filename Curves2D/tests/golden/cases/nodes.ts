import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/nodes.ts'
import { Path, type Heading, type NodeKind } from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import type { Cubic, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { record } from './record.ts'

// A path is named by a spec as in path.ts and every path answered is written whole. Nodes are plain objects
// { p, in?, out?, kind? }. Callbacks are given as data: shiftNodes' pick as a list of indices (null: all), remapLocks'
// `to` as a list of new indices (null: gone). lockOf answers the index of its lock in `locks`, or null.
type Spec = { start: Pt, segs: Cubic[], closed?: boolean, arrives?: Heading, metric?: Pt, kinds?: (NodeKind | null)[] }
const P = (s: Spec) => new Path(s.start, s.segs, s.closed ?? false, s.arrives, s.metric ?? [1, 1], s.kinds?.map(k => k ?? undefined))
const ser = (p: Path) => ({ start: p.start, segs: p.segs, closed: p.closed, arrives: p.arrives ?? null, metric: p.metric, kinds: p.kinds })
type N = m.Node
type Ref = m.HandleRef

const mod = {
  nodesOf: (s: Spec) => m.nodesOf(P(s)),
  segmentOf: (a: N, b: N) => m.segmentOf(a, b),
  segmentsOf: (ns: N[], closed: boolean) => m.segmentsOf(ns, closed),
  pathOf: (ns: N[], closed: boolean, metric?: Pt) => ser(m.pathOf(ns, closed, metric)),
  pointOn: (ns: N[], closed: boolean, seg: number, ts: number[]) => ts.map(t => m.pointOn(ns, closed, seg, t)),
  nearestOn: (ns: N[], closed: boolean, ps: Pt[]) => ps.map(p => m.nearestOn(ns, closed, p)),
  crossingsOn: (ns: N[], closed: boolean, j: 0 | 1, v: number) => m.crossingsOn(ns, closed, j, v),
  moldNodes: (ns: N[], closed: boolean, seg: number, t: number, to: Pt) => m.moldNodes(ns, closed, seg, t, to),
  nodeKind: (n: N) => m.nodeKind(n),
  withHandle: (n: N, side: 'in' | 'out', to: Pt) => m.withHandle(n, side, to),
  withNodeKind: (n: N, kind: NodeKind) => m.withNodeKind(n, kind),
  insertNode: (ns: N[], seg: number, t: number) => m.insertNode(ns, seg, t),
  shiftNodes: (ns: N[], d: Pt, pick: number[] | null) => m.shiftNodes(ns, d, pick ? i => pick.includes(i) : undefined),
  simplified: (ns: N[], closed: boolean, tol: number) => m.simplified(ns, closed, tol),
  lockOf: (locks: m.HandleLock[], r: Ref) => { const l = m.lockOf(locks, r); return l ? locks.indexOf(l) : null },
  withLockedHandle: (lists: N[][], locks: m.HandleLock[], r: Ref, to: Pt) => m.withLockedHandle(lists, locks, r, to),
  locked: (locks: m.HandleLock[], refs: Ref[], how: { angle?: boolean, length?: boolean }) => m.locked(locks, refs, how),
  unlocked: (locks: m.HandleLock[], refs: Ref[]) => m.unlocked(locks, refs),
  remapLocks: (locks: m.HandleLock[], s: number, to: (number | null)[]) => m.remapLocks(locks, s, i => to[i] ?? undefined),
  lockedKind: (locks: m.HandleLock[], n: N, s: number, i: number) => m.lockedKind(locks, n, s, i),
  lockedKinds: (lists: N[][], locks: m.HandleLock[]) => m.lockedKinds(lists, locks),
  kindLocks: (lists: N[][]) => m.kindLocks(lists),
}

const oval: Spec = { start: [0, 50], segs: [[[0, 50], [0, 77.6], [22.4, 100], [50, 100]], [[50, 100], [77.6, 100], [100, 77.6], [100, 50]], [[100, 50], [100, 22.4], [77.6, 0], [50, 0]], [[50, 0], [22.4, 0], [0, 22.4], [0, 50]]], closed: true }
const D: Spec = { start: [0, 0], segs: [[[0, 0], [0, 33], [0, 66], [0, 100]], [[0, 100], [80, 100], [80, 0], [0, 0]]], closed: true }
const hook: Spec = { start: [10, 0], segs: [[[10, 0], [10, 60], [30, 90], [60, 90]], [[60, 90], [90, 90], [100, 60], [100, 40]]] }
const S: Spec = { start: [0, 0], segs: [[[0, 0], [50, 100], [50, -100], [100, 0]], [[100, 0], [150, 100], [180, 40], [200, 60]]] }
const kinded: Spec = { ...oval, kinds: ['corner', null, 'symmetric', null] }
const wide: Spec = { ...hook, metric: [2, 1] }
const paths = { oval, D, hook, S, kinded, wide }

const open3: N[] = [{ p: [0, 0], out: [0, 40] }, { p: [50, 100], in: [-30, 0], out: [30, 0], kind: 'smooth' }, { p: [100, 0], in: [0, 40] }]
const closed3: N[] = [{ p: [0, 0], in: [-20, -10], out: [20, 10], kind: 'symmetric' }, { p: [80, 10], in: [-10, 30], out: [10, -30] }, { p: [40, 90], in: [15, 0], out: [-15, 0], kind: 'corner' }]
const bare: N[] = [{ p: [0, 0] }, { p: [100, 0] }, { p: [100, 100] }, { p: [0, 100] }]
const mixed: N[] = [{ p: [0, 0], out: [0, 0] }, { p: [60, 0], in: [0, 0], out: [20, 0] }, { p: [120, 40], in: [-20, -10], out: [0, 0] }, { p: [60, 80], in: [20, 0] }]
const circle = (k: number): N[] => Array.from({ length: k }, (_, i) => ({ p: [50 + 40 * Math.cos(2 * Math.PI * i / k), 50 + 40 * Math.sin(2 * Math.PI * i / k)] as Pt }))
const wave: N[] = Array.from({ length: 41 }, (_, i) => ({ p: [i * 5, 30 * Math.sin(i / 6)] as Pt }))
const nodeSets: Record<string, [N[], boolean]> = {
  open3: [open3, false], closed3: [closed3, true], bare: [bare, true], bareOpen: [bare, false], mixed: [mixed, false],
  ovalN: [m.nodesOf(P(oval)), true], DN: [m.nodesOf(P(D)), true], hookN: [m.nodesOf(P(hook)), false],
}
const pts: Pt[] = [[47, 52], [50, 50], [-20, 30], [130, 120], [0, 0], [60, 99]]
const ts = [0, 0.25, 0.5, 0.8, 1]

export default record(mod, R => {
  // Throws can't be recorded, so a case that throws in the TS is left out.
  const F = (fn: string, ...a: unknown[]) => { try { (mod as any)[fn](...structuredClone(a)) } catch { return } R(fn, ...a) }
  for (const p of Object.values(paths)) F('nodesOf', p)
  F('nodesOf', { start: [5, 5], segs: [] }); F('nodesOf', { ...oval, metric: [3, 1] })
  F('segmentOf', open3[0]!, open3[1]!); F('segmentOf', open3[1]!, open3[2]!); F('segmentOf', bare[0]!, bare[1]!)
  F('segmentOf', mixed[0]!, mixed[1]!)
  for (const [ns, closed] of Object.values(nodeSets)) {
    F('segmentsOf', ns, closed); F('pathOf', ns, closed); F('pathOf', ns, closed, [2, 0.5])
    for (let seg = 0; seg < ns.length - (closed ? 0 : 1); seg++) F('pointOn', ns, closed, seg, ts)
    F('nearestOn', ns, closed, pts)
    F('crossingsOn', ns, closed, 0, 50); F('crossingsOn', ns, closed, 1, 50); F('crossingsOn', ns, closed, 1, 0); F('crossingsOn', ns, closed, 0, 1000)
    F('moldNodes', ns, closed, 0, 0.5, [30, 45]); F('moldNodes', ns, closed, ns.length - 2, 0.3, [70, 20])
    F('moldNodes', ns, closed, ns.length - 1, 0.7, [10, 90])
    F('insertNode', ns, 0, 0.5); F('insertNode', ns, ns.length - 2, 0.25); F('insertNode', ns, ns.length - 1, 0.75)
    F('shiftNodes', ns, [5, -3], null); F('shiftNodes', ns, [5, -3], [0, 2]); F('shiftNodes', ns, [0, 0], [])
  }
  F('segmentsOf', [], false); F('segmentsOf', [bare[0]!], true); F('segmentsOf', [bare[0]!], false)
  F('pathOf', [bare[0]!], true); F('pathOf', [bare[0]!], false); F('pathOf', [], false)
  F('pathOf', [{ p: [0, 0], kind: 'smooth' }, { p: [10, 0], kind: 'corner' }], false)
  F('pointOn', bare, false, 7, [0.5])
  // nodeKind
  for (const n of [...open3, ...closed3, ...bare, ...mixed]) F('nodeKind', n)
  F('nodeKind', { p: [0, 0], in: [-10, 0], out: [10, 0] }); F('nodeKind', { p: [0, 0], in: [-10, 0], out: [30, 0] })
  F('nodeKind', { p: [0, 0], in: [-10, 0], out: [10, 5] }); F('nodeKind', { p: [0, 0], in: [-10, 0], out: [10, 0.0005] })
  F('nodeKind', { p: [0, 0], in: [-10, 0], out: [10, 0], kind: 'corner' }); F('nodeKind', { p: [0, 0], in: [0, 0], out: [10, 0] })
  F('nodeKind', { p: [0, 0], in: [-1e-3, 0], out: [1e-3, 0] }); F('nodeKind', { p: [0, 0], in: [-1e-3, 0], out: [1.0000005e-3, 0] })
  // withHandle on every kind, side, and missing handle
  const probes: N[] = [open3[0]!, open3[1]!, open3[2]!, closed3[0]!, closed3[1]!, closed3[2]!, bare[0]!, mixed[1]!, mixed[2]!,
    { p: [0, 0], in: [-10, 0], out: [10, 0] }, { p: [0, 0], in: [-10, 0], out: [30, 0] }, { p: [0, 0], in: [-10, 0], out: [30, 0], kind: 'symmetric' }]
  for (const n of probes) for (const side of ['in', 'out'] as const) { F('withHandle', n, side, [7, 25]); F('withHandle', n, side, [0, 0]) }
  // withNodeKind
  for (const n of [...probes, { p: [0, 0], in: [0, 0], out: [0, 0] }, { p: [0, 0], in: [0, 0], out: [10, 5] }, { p: [0, 0], in: [10, 5], out: [0, 0] }, { p: [0, 0], in: [-10, 10], out: [30, 0] }])
    for (const k of ['corner', 'smooth', 'symmetric'] as const) F('withNodeKind', n, k)
  // simplified
  F('simplified', circle(60), true, 0.5); F('simplified', circle(60), true, 0.05); F('simplified', circle(24), true, 1)
  F('simplified', wave, false, 0.5); F('simplified', wave, false, 3)
  F('simplified', bare, true, 0.5); F('simplified', bare, false, 0.5)
  F('simplified', open3, false, 0.5); F('simplified', closed3, true, 0.5); F('simplified', m.nodesOf(P(oval)), true, 0.5)
  F('simplified', m.nodesOf(P(D)), true, 0.5); F('simplified', m.nodesOf(P(S)), false, 0.5); F('simplified', m.nodesOf(P(hook)), false, 0.2)
  F('simplified', [bare[0]!], true, 0.5); F('simplified', bare.slice(0, 2), false, 0.5); F('simplified', mixed, false, 1)
  F('simplified', [...bare, { p: [0, 0] }].slice(0, 5), false, 0.5)
  // locks
  const H = (s: number, i: number, side: 'in' | 'out', flip?: true) => ({ s, i, side, ...(flip && { flip }) })
  const lists: N[][] = [open3, closed3, [{ p: [0, 0], in: [-20, 0], out: [20, 0] }, { p: [50, 0], in: [0, 0], out: [10, 30] }]]
  const lk: m.HandleLock[] = [
    { handles: [H(0, 1, 'in'), H(0, 1, 'out')], angle: true },
    { handles: [H(1, 0, 'in'), H(1, 0, 'out')], angle: true, length: true },
    { handles: [H(0, 0, 'out'), H(1, 2, 'in', true), H(2, 0, 'out')], angle: true, length: true },
    { handles: [H(1, 1, 'in'), H(2, 1, 'out')], length: true },
  ]
  F('lockOf', lk, H(0, 1, 'out')); F('lockOf', lk, H(1, 2, 'in')); F('lockOf', lk, H(0, 1, 'in', true)); F('lockOf', lk, H(0, 2, 'in')); F('lockOf', [], H(0, 0, 'in'))
  const drags: [Ref, Pt][] = [[H(0, 1, 'in'), [-10, 40]], [H(0, 1, 'out'), [60, 10]], [H(1, 0, 'out'), [0, 40]], [H(1, 0, 'in'), [-5, 5]],
    [H(0, 0, 'out'), [20, 30]], [H(1, 2, 'in'), [0, 25]], [H(2, 0, 'out'), [-30, 5]], [H(1, 1, 'in'), [-20, 60]], [H(2, 1, 'out'), [0, 0]],
    [H(1, 0, 'out'), [0, 0]], [H(2, 1, 'in'), [3, 4]], [H(5, 0, 'in'), [3, 4]], [H(0, 9, 'out'), [3, 4]], [H(0, 0, 'in'), [1, 1]], [H(2, 0, 'in'), [8, 8]]]
  for (const [r, to] of drags) { F('withLockedHandle', lists, lk, r, to); F('withLockedHandle', lists, [], r, to) }
  F('locked', [], [H(0, 0, 'in'), H(0, 0, 'out')], { angle: true }); F('locked', lk, [H(0, 1, 'in'), H(2, 1, 'in')], { angle: true, length: true })
  F('locked', lk, [H(0, 1, 'in'), H(1, 1, 'in')], {}); F('locked', lk, [H(3, 3, 'out'), H(3, 4, 'out')], { length: true })
  F('locked', lk, [H(0, 0, 'out')], { angle: false, length: false }); F('locked', lk, [], { angle: true }); F('locked', lk, [H(9, 9, 'in'), H(9, 9, 'in')], { angle: true })
  F('unlocked', lk, [H(0, 1, 'in')]); F('unlocked', lk, [H(0, 0, 'out')]); F('unlocked', lk, [H(1, 2, 'in'), H(2, 0, 'out')]); F('unlocked', lk, [])
  F('unlocked', lk, [H(9, 9, 'in')]); F('unlocked', lk, lk.flatMap(l => l.handles))
  F('remapLocks', lk, 0, [0, 1, 2]); F('remapLocks', lk, 0, [0, 1, null]); F('remapLocks', lk, 0, [null, 0, 1]); F('remapLocks', lk, 1, [2, 1, 0])
  F('remapLocks', lk, 1, [null, null, 0]); F('remapLocks', lk, 2, [1, 0]); F('remapLocks', lk, 7, [0]); F('remapLocks', lk, 0, [null, null, null])
  for (const [s, ns] of lists.entries()) ns.forEach((n, i) => F('lockedKind', lk, n, s, i))
  F('lockedKind', lk, { p: [0, 0], in: [-20, 5], out: [20, 0] }, 1, 0); F('lockedKind', lk, { p: [0, 0] }, 1, 0); F('lockedKind', lk, { p: [0, 0], in: [1, 0] }, 0, 1)
  F('lockedKind', [{ handles: [H(0, 0, 'in'), H(0, 0, 'out')], length: true }], { p: [0, 0], in: [-1, 0], out: [1, 0] }, 0, 0)
  F('lockedKind', [{ handles: [H(0, 0, 'in'), H(0, 0, 'out')], angle: true, length: true }], { p: [0, 0], in: [-1, 0], out: [1, 0] }, 0, 0)
  F('lockedKind', [{ handles: [H(0, 0, 'in'), H(0, 0, 'out')], angle: true, length: true }], { p: [0, 0], in: [-1, 0], out: [2, 0] }, 0, 0)
  F('lockedKind', [{ handles: [H(0, 0, 'in'), H(0, 0, 'out')], angle: true, length: true }], { p: [0, 0], out: [2, 0] }, 0, 0)
  F('lockedKinds', lists, lk); F('lockedKinds', lists, []); F('lockedKinds', [], lk)
  F('kindLocks', lists); F('kindLocks', [[...open3, ...mixed], closed3]); F('kindLocks', []); F('kindLocks', [[]])
  F('kindLocks', [[{ p: [0, 0], in: [-1, 0], out: [1, 0], kind: 'smooth' }, { p: [0, 0], in: [0, 0], out: [1, 0], kind: 'smooth' }, { p: [1, 1], in: [-1, 0], out: [1, 0], kind: 'symmetric' }, { p: [1, 1], in: [-1, 0], out: [1, 0], kind: 'corner' }]])
})
