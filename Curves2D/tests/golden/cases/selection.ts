import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/selection.ts'
import { Path } from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import * as pr from '../../../../../../apps/Fountain/fountain-ts/src/curves/profile.ts'
import { lineCubic as L, type Cubic, type Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { record } from './record.ts'

// Selections and paths aren't serialisable, so the cases name them by data.
// A path: { start, segs, closed?, metric?, kinds? } (kinds: a kind name or null for each node).
// A pick: a list of nodes, or { k: 'nodeAt' | 'cubicAt', i } | { k: 'section', from, to } (m.Pick), or a function of
//   { k: 'above', y } (a boolean), 'ramp' (y0, y1: a weight by height), 'prog' (a, b: by progress, weight 0.5…1),
//   'odd' (every odd node), 'num' (a constant v, may be NaN or beyond 0…1), 'bool' (a constant v).
// A step of a chain is [method, ...args], applied to a Selection in turn; see `step` for the argument specs.
type PS = { start: Pt, segs: Cubic[], closed?: boolean, metric?: Pt, kinds?: (string | null)[] }
const P = (s: PS) => new Path(s.start, s.segs, s.closed ?? false, undefined, s.metric, s.kinds?.map(k => k ?? undefined) as any)
const ser = (p: Path) => ({ start: p.start, segs: p.segs, closed: p.closed, kinds: p.kinds })

const pickOf = (s: any): m.NodePick | m.Pick | undefined => {
  if (s === undefined || s === null) return undefined
  if (Array.isArray(s)) return s
  switch (s.k) {
    case 'nodeAt': return m.nodeAt(...s.i)
    case 'cubicAt': return m.cubicAt(...s.i)
    case 'section': return m.section(s.from, s.to)
    case 'above': return p => p[1] > s.y
    case 'ramp': return p => (p[1] - s.y0) / (s.y1 - s.y0)
    case 'prog': return (_, i, at) => at >= s.a && at <= s.b ? 0.5 + 0.5 * at : 0
    case 'odd': return (_, i) => i % 2
    case 'num': return () => s.v
    case 'bool': return () => s.v
  }
  throw new Error('pick ' + JSON.stringify(s))
}
const selOf = (path: PS, pick?: any) => m.Selection.of(P(path), pickOf(pick))

const fall = (s: any): m.FalloffName | m.Falloff => s === undefined ? 'smooth' : typeof s === 'string' ? s as m.FalloffName
  : s.fn === 'sq' ? (x => 1 - x * x) : s.fn === 'over' ? (x => 2 - x) : (x => x - 0.5)
const wave = (s: any): pr.Strength => typeof s === 'number' || Array.isArray(s) ? s : s.w === 'saw' ? pr.saw(s) : pr.sine(s)
const pivot = (s: any): m.Pivot | undefined => s === 'center' || s === undefined || s === null ? undefined : s
const moveMap = (s: any): ((node: Pt, path: Path, i: number) => (p: Pt) => Pt) => {
  switch (s.k) {
    case 'shift': return () => p => [p[0] + s.d[0], p[1] + s.d[1]]
    case 'byIndex': return (_, __, i) => p => [p[0] + s.d[0] * i, p[1] + s.d[1]]
    case 'aboutNode': return node => p => [node[0] + (p[0] - node[0]) * s.s, node[1] + (p[1] - node[1]) * s.s]
    default: return (_, path) => p => [p[0] + path.segs.length, p[1]]
  }
}
const pointMap = (s: any) => (p: Pt, i: number): Pt => [p[0] + s.dx * (i + 1), p[1] + s.dy * (i % 2 ? 1 : -1)]

const step = (sel: m.Selection, [name, ...a]: any[]): m.Selection => {
  switch (name) {
    case 'soften': return sel.soften(a[0], a[1] && { ...a[1], falloff: a[1].falloff === undefined ? undefined : fall(a[1].falloff) })
    case 'weigh': return sel.weigh(wave(a[0]))
    case 'invert': return sel.invert()
    case 'transform': return sel.transform(a[0])
    case 'moveEach': return sel.moveEach(moveMap(a[0]))
    case 'put': return sel.put(a[0])
    case 'kind': return sel.kind(a[0])
    case 'handles': return sel.handles(a[0])
    case 'inflate': return sel.inflate(a[0], a[1] && { ...a[1], strength: a[1].strength === undefined ? undefined : wave(a[1].strength) })
    case 'smooth': return sel.smooth(a[0])
    case 'fair': return sel.fair(a[0])
    case 'rotate': return sel.rotate(a[0], pivot(a[1]))
    case 'shear': return sel.shear(a[0], a[1] && { ...a[1], about: pivot(a[1].about) })
    case 'shift': return sel.shift(a[0], a[1])
    case 'scale': return sel.scale(a[0], pivot(a[1]))
    case 'mirror': return sel.mirror(a[0], pivot(a[1]))
    case 'select': return sel.select(pickOf(a[0]))
    case 'and': return sel.and(...a.map((x: any) => selOf(x[0], x[1])))
  }
  throw new Error('step ' + name)
}
const outSel = (s: m.Selection) => {
  const pts = s.points
  return {
    picked: s.picked.map(p => ({ path: ser(p.path), nodes: [...p.nodes], weights: [...p.weights] })),
    weights: s.weights, points: pts, center: pts.length ? s.center : null, bounds: pts.length ? s.bounds : null, metric: s.metric,
    path: s.picked.length === 1 ? ser(s.path) : null, paths: s.paths.map(ser),
  }
}
const chain = (path: PS, pick: any, steps: any[]) => outSel(steps.reduce(step, selOf(path, pick)))
const pickOn = (path: PS, p: m.Pick) => {
  const on = p.on(P(path)), sel = m.Selection.of(P(path), p)
  return { path: ser(on.path), pick: Array.isArray(on.pick) ? on.pick : null, weights: sel.picked[0]!.weights }
}

const mod = {
  ...m,
  chain,
  nodeAt: (path: PS, ...i: number[]) => pickOn(path, m.nodeAt(...i)),
  cubicAt: (path: PS, ...i: number[]) => pickOn(path, m.cubicAt(...i)),
  section: (path: PS, from: number, to: number) => pickOn(path, m.section(from, to)),
  falloffAt: (name: string, xs: number[]) => xs.map(m.falloffs[name as m.FalloffName]),
  moveNodes: (segs: Cubic[], closed: boolean, nodes: number[], s: any) => m.moveNodes(segs, closed, nodes, pointMap(s)),
  smoothNodes: (segs: Cubic[], closed: boolean, nodes: number[], amount: number | undefined | { byNode: number[] }, metric?: Pt) =>
    m.smoothNodes(segs, closed, nodes, amount === undefined || typeof amount === 'number' ? amount : (i => amount.byNode[i]!), metric),
  selectOf: (path: PS, pick?: any) => outSel(selOf(path, pick)),
  pathSelect: (path: PS, pick?: any) => outSel(P(path).select(pickOf(pick))),
  withKind: (path: PS, kind: any, pick?: any) => ser(P(path).withKind(kind, pickOf(pick) as m.NodePick | undefined)),
  group: (items: any[], steps: any[] = []) => outSel(steps.reduce(step, m.group(items.map(it => it.sel ? selOf(it.sel[0], it.sel[1]) : P(it))))),
}

const poly: PS = { start: [0, 0], segs: [L([0, 0], [20, 60]), L([20, 60], [50, 30]), L([50, 30], [80, 80]), L([80, 80], [100, 10])] }
const polyC: PS = { ...poly, closed: true, segs: [...poly.segs, L([100, 10], [0, 0])] }
const wavy: PS = { start: [0, 0], segs: [[[0, 0], [10, 40], [30, 50], [40, 30]], [[40, 30], [50, 10], [70, 10], [80, 30]], [[80, 30], [90, 50], [100, 60], [110, 90]]] }
const k = 0.5523 * 50
const ring: PS = { start: [100, 50], closed: true, segs: [
  [[100, 50], [100, 50 + k], [50 + k, 100], [50, 100]], [[50, 100], [50 - k, 100], [0, 50 + k], [0, 50]],
  [[0, 50], [0, 50 - k], [50 - k, 0], [50, 0]], [[50, 0], [50 + k, 0], [100, 50 - k], [100, 50]]] }
const oval: PS = { start: [100, 50], closed: true, segs: [
  [[100, 50], [100, 50 + 0.5 * 50], [50 + 0.8 * 50, 100], [50, 100]], [[50, 100], [50 - 0.4 * 50, 100], [0, 50 + 0.7 * 50], [0, 50]],
  [[0, 50], [0, 50 - 0.45 * 50], [50 - 0.6 * 50, 0], [50, 0]], [[50, 0], [50 + 0.55 * 50, 0], [100, 50 - 0.5 * 50], [100, 50]]] }
const blob: PS = { start: [10, 0], closed: true, segs: [
  [[10, 0], [40, -10], [60, 5], [80, 20]], [[80, 20], [95, 40], [90, 60], [70, 80]], [[70, 80], [50, 95], [30, 90], [20, 70]],
  [[20, 70], [10, 50], [-10, 40], [-5, 20]], [[-5, 20], [-3, 8], [0, 3], [10, 0]]] }
const q = 0.5523 * 40
const stem: PS = { start: [0, 0], segs: [L([0, 0], [0, 60]), [[0, 60], [0, 60 + q], [40 - q, 100], [40, 100]], L([40, 100], [80, 100])] }
const stemShort: PS = { start: [0, 0], segs: [L([0, 0], [0, 6]), [[0, 6], [0, 6 + q], [40 - q, 46], [40, 46]], L([40, 46], [48, 46])] }
const r = 0.5523 * 20
const stadium: PS = { start: [80, 0], closed: true, segs: [
  [[80, 0], [80 + r, 0], [100, 20 - r], [100, 20]], [[100, 20], [100, 20 + r], [80 + r, 40], [80, 40]], L([80, 40], [20, 40]),
  [[20, 40], [20 - r, 40], [0, 20 + r], [0, 20]], [[0, 20], [0, 20 - r], [20 - r, 0], [20, 0]], L([20, 0], [80, 0])] }
const stadiumOpen: PS = { start: [50, 0], segs: [L([50, 0], [80, 0]), ...stadium.segs.slice(0, 2), L([80, 40], [20, 40])] }
const u = 0.5523 * 0.5
const unitRing: PS = { start: [1, 0.5], closed: true, metric: [80, 120], segs: [
  [[1, 0.5], [1, 0.5 + u], [0.5 + u, 1], [0.5, 1]], [[0.5, 1], [0.5 - u, 1], [0, 0.5 + u], [0, 0.5]],
  [[0, 0.5], [0, 0.5 - u], [0.5 - u, 0], [0.5, 0]], [[0.5, 0], [0.5 + u, 0], [1, 0.5 - u], [1, 0.5]]] }
const unitWavy: PS = { start: [0, 0], metric: [60, 140], segs: [[[0, 0], [0.1, 0.4], [0.3, 0.5], [0.4, 0.3]], [[0.4, 0.3], [0.5, 0.1], [0.7, 0.1], [0.8, 0.3]], [[0.8, 0.3], [0.9, 0.5], [1, 0.6], [1, 0.9]]] }
const kinded: PS = { ...wavy, kinds: [null, 'smooth', null, 'corner'] }
const dot: PS = { start: [5, 7], segs: [] }
const far: PS = { start: [200, 200], segs: [L([200, 200], [260, 240]), L([260, 240], [300, 200])] }

export default record(mod, F => {
  // The picks.
  for (const p of [poly, polyC, wavy, ring, dot]) {
    F('nodeAt', p, 0); F('nodeAt', p, -1); F('nodeAt', p, 1, 2); F('nodeAt', p, -2, 0, 1); F('nodeAt', p)
    F('cubicAt', p, 0); F('cubicAt', p, -1); F('cubicAt', p, 0, 1); F('cubicAt', p, 1, 1, -2); F('cubicAt', p)
    F('section', p, 0, 0.5); F('section', p, 0.3, 0.7); F('section', p, 0.7, 0.3); F('section', p, 0, 1); F('section', p, 0.25, 0.25)
  }
  F('section', ring, 0.1, 0.9); F('section', stem, 0.2, 0.6); F('section', unitWavy, 0.05, 0.45); F('section', unitRing, 0, 0.5)
  F('section', poly, -0.2, 0.4); F('section', poly, 0.6, 1.3)

  // Falloffs.
  const xs = [0, 0.1, 0.25, 0.5, 0.75, 0.9, 1, 1.2, -0.1]
  for (const name of ['smooth', 'linear', 'sphere', 'root', 'sharp', 'constant']) F('falloffAt', name, xs)

  // Inflation.
  for (const [p, from, r] of [[[10, 0], [0, 0], 5], [[3, 4], [0, 0], -2], [[0, 0], [0, 0], 7], [[1e-13, 0], [0, 0], 3], [[-6, 8], [0, 0], 0], [[5, 5], [10, 20], 1.5], [[0, 10], [0, 0], -10]] as [Pt, Pt, number][])
    F('inflation', p, from, r)

  // Moving nodes.
  const segs = wavy.segs, rs = ring.segs
  F('moveNodes', segs, false, [], { dx: 1, dy: 2 }); F('moveNodes', segs, false, [0], { dx: 1, dy: 2 }); F('moveNodes', segs, false, [1], { dx: 1, dy: 2 })
  F('moveNodes', segs, false, [3], { dx: 1, dy: 2 }); F('moveNodes', segs, false, [0, 1, 2, 3], { dx: -2, dy: 1 }); F('moveNodes', segs, false, [2, 2, 1, 2], { dx: 0.5, dy: 3 })
  F('moveNodes', rs, true, [0], { dx: 1, dy: 2 }); F('moveNodes', rs, true, [1, 3], { dx: 1, dy: 2 }); F('moveNodes', rs, true, [4], { dx: 1, dy: 2 })
  F('moveNodes', rs, true, [0, 4], { dx: 1, dy: 2 }); F('moveNodes', rs, true, [3, 0, 2], { dx: -1, dy: 1 }); F('moveNodes', rs, true, [], { dx: 1, dy: 1 })

  // Smoothing nodes.
  for (const [s, c] of [[poly.segs, false], [polyC.segs, true], [wavy.segs, false], [ring.segs, true], [blob.segs, true], [stem.segs, false], [stadium.segs, true], [unitRing.segs, true]] as [Cubic[], boolean][]) {
    const n = s.length, all = Array.from({ length: n + (c ? 0 : 1) }, (_, i) => i)
    F('smoothNodes', s, c, all); F('smoothNodes', s, c, all, 0.5); F('smoothNodes', s, c, [1], 1); F('smoothNodes', s, c, [0, n - 1], 0.7); F('smoothNodes', s, c, [], 1)
    F('smoothNodes', s, c, all, { byNode: all.map(i => (i % 3) / 2) }); F('smoothNodes', s, c, all, 0); F('smoothNodes', s, c, all, 1, [0.5, 2])
  }
  F('smoothNodes', unitWavy.segs, false, [0, 1, 2, 3], 1, [60, 140]); F('smoothNodes', unitRing.segs, true, [0, 2], 0.8, [80, 120])
  F('smoothNodes', ring.segs, true, [4, 5, 0], 1); F('smoothNodes', wavy.segs, false, [9], 1)
  F('smoothNodes', [[[0, 0], [0, 0], [10, 0], [10, 0]], [[10, 0], [10, 0], [10, 10], [10, 10]]], false, [0, 1, 2], 1)
  F('smoothNodes', [[[0, 0], [0, 0], [0, 0], [0, 0]], [[0, 0], [5, 5], [5, 5], [10, 0]]], false, [0, 1, 2], 1)
  F('smoothNodes', [[[0, 0], [10, 0], [20, 0], [30, 0]]], false, [0, 1], 1)
  F('smoothNodes', [[[0, 0], [10, 0], [20, 0], [30, 0]], [[30, 0], [20, 0], [10, 0], [0, 0]]], true, [0, 1], 1)

  // Fairing nodes.
  const fairs: [PS, number[], number?, Pt?, number[]?][] = [
    [stem, [1, 2]], [stem, [1, 2], 0.25], [stem, [1, 2], 1], [stem, [1], 0.5], [stem, [2], 0.5], [stem, [0, 3]], [stem, [], 0.5], [stem, [1, 2], 0.5, [1, 1], [1, 0.5, 0.5, 1]],
    [stem, [1, 2], 0.5, undefined, [0, 1, 0, 0]], [stem, [1, 2], 0.5, undefined, [0, 0.4, 0.7, 0]], [stemShort, [1, 2], 0.5], [stemShort, [1, 2], 0.9], [stem, [1, 2], 0.5, [0.5, 2]],
    [stadium, [0, 1, 2, 3, 4, 5]], [stadium, [0, 1, 2, 3, 4, 5], 0.3], [stadium, [0], 0.5], [stadium, [1, 3], 1], [stadium, [2, 6], 0.5], [stadium, [0, 1, 2, 3, 4, 5], 0.5, [1, 1], [1, 1, 0.5, 1, 0.5, 1]],
    [stadium, [5, 0], 0.4], [stadium, [6], 0.4], [stadiumOpen, [1, 2, 3, 4]], [stadiumOpen, [0, 1, 2, 3, 4], 0.4], [stadiumOpen, [1, 2, 3, 4], 0.5, [0.7, 1.4]],
    [ring, [0, 1, 2, 3]], [oval, [0, 1, 2, 3]], [oval, [1, 2]], [oval, [0, 1, 2, 3, 4], 0.5, [1, 1], [1, 1, 1, 1]], [blob, [0, 1, 2, 3, 4]], [wavy, [1, 2]], [wavy, [0, 1, 2, 3]],
    [unitRing, [0, 1, 2, 3]], [unitWavy, [1, 2]], [poly, [1, 2, 3]], [polyC, [0, 1, 2, 3, 4]],
  ]
  for (const [p, nodes, reach, metric, ws] of fairs)
    F('fairNodes', p.segs, p.closed ?? false, nodes, reach, metric ?? p.metric, ws)

  // Selection.of and its picks.
  for (const p of [poly, polyC, wavy, ring, blob, unitRing, dot, stem]) {
    F('selectOf', p); F('selectOf', p, [0]); F('selectOf', p, [1, 2]); F('selectOf', p, []); F('selectOf', p, [p.segs.length]); F('selectOf', p, [-1, 99])
    F('selectOf', p, { k: 'nodeAt', i: [0, -1] }); F('selectOf', p, { k: 'cubicAt', i: [0] }); F('selectOf', p, { k: 'section', from: 0.2, to: 0.7 })
    F('selectOf', p, { k: 'above', y: 40 }); F('selectOf', p, { k: 'ramp', y0: 10, y1: 60 }); F('selectOf', p, { k: 'prog', a: 0.2, b: 0.8 }); F('selectOf', p, { k: 'odd' })
    F('selectOf', p, { k: 'num', v: 0.3 }); F('selectOf', p, { k: 'num', v: 5 }); F('selectOf', p, { k: 'num', v: -2 }); F('selectOf', p, { k: 'num', v: NaN })
    F('selectOf', p, { k: 'bool', v: true }); F('selectOf', p, { k: 'bool', v: false })
    F('pathSelect', p); F('pathSelect', p, [1]); F('pathSelect', p, { k: 'nodeAt', i: [-1] }); F('pathSelect', p, { k: 'above', y: 20 })
  }
  F('selectOf', kinded, [1, 2])

  // withKind.
  for (const p of [wavy, ring, blob, poly, kinded, unitWavy, stem, dot]) for (const kind of ['smooth', 'symmetric', 'corner']) {
    F('withKind', p, kind); F('withKind', p, kind, [1]); F('withKind', p, kind, []); F('withKind', p, kind, { k: 'nodeAt', i: [0, -1] }); F('withKind', p, kind, { k: 'above', y: 30 })
  }

  // Selection methods, each as a step of a chain.
  const all = [wavy, ring, poly, blob, unitRing, stadium]
  const mid = [1, 2], tops = { k: 'prog', a: 0.2, b: 0.9 }
  for (const p of all) {
    F('chain', p, undefined, [])
    F('chain', p, mid, [['shift', 5, 90]]); F('chain', p, mid, [['shift', 5]]); F('chain', p, mid, [['shift', -3.5, 35]])
    F('chain', p, undefined, [['rotate', 30]]); F('chain', p, mid, [['rotate', 30]]); F('chain', p, mid, [['rotate', -45, [10, 20]]]); F('chain', p, tops, [['rotate', 20, 'center']])
    F('chain', p, mid, [['shear', 20]]); F('chain', p, mid, [['shear', 15, { axis: 90 }]]); F('chain', p, undefined, [['shear', -10, { axis: 30, about: [5, 5] }]])
    F('chain', p, mid, [['scale', 1.5]]); F('chain', p, mid, [['scale', [2, 0.5]]]); F('chain', p, undefined, [['scale', 0.7, [10, 10]]]); F('chain', p, mid, [['scale', [1, 3], 'center']])
    F('chain', p, mid, [['mirror']]); F('chain', p, mid, [['mirror', 45]]); F('chain', p, undefined, [['mirror', 0, [20, 30]]]); F('chain', p, tops, [['mirror', 90, [10, 0]]])
    F('chain', p, mid, [['transform', [1.1, 0.2, -0.3, 0.9, 5, -7]]]); F('chain', p, undefined, [['transform', [1, 0, 0, 1, 3, 4]]])
    F('chain', p, mid, [['moveEach', { k: 'shift', d: [4, -2] }]]); F('chain', p, mid, [['moveEach', { k: 'byIndex', d: [2, 3] }]]); F('chain', p, undefined, [['moveEach', { k: 'aboutNode', s: 1.3 }]])
    F('chain', p, mid, [['moveEach', { k: 'paths' }]])
    F('chain', p, mid, [['put', [30, 60]]]); F('chain', p, mid, [['put', { x: 25 }]]); F('chain', p, mid, [['put', { y: 55 }]]); F('chain', p, undefined, [['put', {}]]); F('chain', p, [0], [['put', [0, 0]]])
    F('chain', p, mid, [['kind', 'smooth']]); F('chain', p, mid, [['kind', 'symmetric']]); F('chain', p, undefined, [['kind', 'corner']]); F('chain', p, tops, [['kind', 'smooth']])
    F('chain', p, mid, [['handles', { angle: 30 }]]); F('chain', p, mid, [['handles', { turn: 25 }]]); F('chain', p, mid, [['handles', { length: 0.5 }]])
    F('chain', p, mid, [['handles', { angle: 90, turn: 10, length: 1.5, side: 'in' }]]); F('chain', p, undefined, [['handles', { angle: -20, side: 'out' }]])
    F('chain', p, undefined, [['handles', { length: 0 }]]); F('chain', p, mid, [['handles', {}]]); F('chain', p, mid, [['handles', { angle: 170, side: 'both' }]])
    F('chain', p, undefined, [['inflate', 6]]); F('chain', p, mid, [['inflate', -4]]); F('chain', p, undefined, [['inflate', 8, { align: [0, 0] }]]); F('chain', p, undefined, [['inflate', 8, { align: [0.2, 0.9], handles: 0 }]])
    F('chain', p, undefined, [['inflate', 8, { smooth: 1 }]]); F('chain', p, undefined, [['inflate', 8, { smooth: 0.4, handles: 0.5 }]])
    F('chain', p, undefined, [['inflate', 10, { strength: { w: 'sine', cycles: 2 } }]]); F('chain', p, undefined, [['inflate', 10, { strength: [0, 1, 0.5], along: 'path' }]])
    F('chain', p, undefined, [['inflate', 10, { strength: { w: 'saw', cycles: 3, low: 0, high: 1 }, along: 'ring' }]]); F('chain', p, undefined, [['inflate', 5, { strength: 2, along: 'path' }]])
    F('chain', p, tops, [['inflate', 7]])
    F('chain', p, undefined, [['smooth']]); F('chain', p, undefined, [['smooth', 0.5]]); F('chain', p, mid, [['smooth', 1]]); F('chain', p, mid, [['smooth', 0]])
    F('chain', p, undefined, [['fair']]); F('chain', p, undefined, [['fair', 0.25]]); F('chain', p, mid, [['fair', 1]]); F('chain', p, undefined, [['fair', 0.5]])
    F('chain', p, [0], [['soften', 40]]); F('chain', p, [0], [['soften', 40, { falloff: 'linear' }]]); F('chain', p, [1], [['soften', 60, { falloff: 'root', along: 'plane' }]])
    F('chain', p, [0], [['soften', 1e6, { falloff: 'constant' }]]); F('chain', p, [1], [['soften', 30, { falloff: 'sphere' }]]); F('chain', p, [2], [['soften', 50, { falloff: 'sharp', along: 'path' }]])
    F('chain', p, [1], [['soften', 50, { falloff: { fn: 'sq' } }]]); F('chain', p, [1], [['soften', 50, { falloff: { fn: 'over' } }]]); F('chain', p, [1], [['soften', 50, { falloff: { fn: 'neg' } }]])
    F('chain', p, [1], [['soften', 0]]); F('chain', p, [], [['soften', 50]]); F('chain', p, [0], [['soften', 25, {}]])
    F('chain', p, [1], [['soften', 30], ['weigh', 0.5]]); F('chain', p, [1], [['soften', 60], ['weigh', [1, 0]]]); F('chain', p, { k: 'num', v: 0.8 }, [['weigh', { w: 'sine', cycles: 1.5 }]])
    F('chain', p, undefined, [['weigh', 3]]); F('chain', p, undefined, [['weigh', [-1, 2, 0.5]]]); F('chain', p, undefined, [['weigh', 0]])
    F('chain', p, mid, [['invert']]); F('chain', p, { k: 'ramp', y0: 0, y1: 100 }, [['invert']]); F('chain', p, undefined, [['invert']])
    F('chain', p, mid, [['select', [0]]]); F('chain', p, mid, [['select', { k: 'nodeAt', i: [-1] }]]); F('chain', p, mid, [['select']]); F('chain', p, mid, [['select', tops]])
    F('chain', p, mid, [['select', { k: 'section', from: 0, to: 0.5 }]])
    F('chain', p, mid, [['and', [far, [0]]]]); F('chain', p, mid, [['and', [far, [0, 2]], [wavy, { k: 'above', y: 30 }]]])
  }
  // Soft selections moved; weights that carry through.
  F('chain', wavy, [0], [['soften', 50], ['rotate', 40]])
  F('chain', wavy, [0], [['soften', 50], ['shift', 10, 90]])
  F('chain', wavy, [0], [['soften', 50], ['scale', 2, [0, 0]]])
  F('chain', wavy, [0], [['soften', 50], ['shear', 20]])
  F('chain', wavy, [0], [['soften', 50], ['mirror', 90, [0, 0]]])
  F('chain', wavy, [0], [['soften', 50], ['inflate', 6, { align: [0, 0] }]])
  F('chain', wavy, [0], [['soften', 80], ['smooth']])
  F('chain', wavy, [0], [['soften', 80], ['kind', 'smooth']])
  F('chain', wavy, [0], [['soften', 80], ['handles', { turn: 40, length: 2 }]])
  F('chain', wavy, [0], [['soften', 80], ['put', [10, 10]]])
  F('chain', wavy, [0], [['soften', 80], ['moveEach', { k: 'shift', d: [0, 9] }]])
  F('chain', ring, [0], [['soften', 120, { along: 'plane' }], ['inflate', 5]])
  F('chain', ring, { k: 'ramp', y0: 20, y1: 90 }, [['rotate', 30], ['shift', 3], ['fair']])
  F('chain', ring, { k: 'ramp', y0: 20, y1: 90 }, [['smooth'], ['inflate', 3, { smooth: 0.5 }], ['mirror']])
  F('chain', stem, [1, 2], [['fair'], ['soften', 30]])
  F('chain', stem, [1, 2], [['fair', 0.3], ['rotate', 10]])
  F('chain', stem, undefined, [['smooth'], ['fair']])
  F('chain', stem, [1, 2], [['kind', 'smooth']])
  F('chain', stadium, [0, 1, 2, 3, 4, 5], [['fair', 0.5], ['inflate', 2]])
  F('chain', stadium, { k: 'ramp', y0: 0, y1: 40 }, [['fair', 0.5]])
  F('chain', stadiumOpen, undefined, [['fair', 0.5]])
  F('chain', stemShort, undefined, [['fair', 0.5]])
  // Kinds set on a path survive moves of the selection.
  F('chain', kinded, [1], [['rotate', 20]]); F('chain', kinded, [1], [['kind', 'symmetric']]); F('chain', kinded, [0, 1, 2, 3], [['kind', 'corner']]); F('chain', kinded, [1, 2], [['smooth']])
  F('chain', kinded, [1, 2], [['fair']]); F('chain', kinded, [1], [['handles', { turn: 15 }]]); F('chain', kinded, [2], [['inflate', 4, { align: [0, 0] }]])
  // Metric other than [1, 1]: turns stay turns.
  for (const p of [unitRing, unitWavy]) {
    F('chain', p, [0, 1], [['rotate', 30]]); F('chain', p, undefined, [['rotate', 45, [0.5, 0.5]]]); F('chain', p, [1], [['shear', 20]]); F('chain', p, [1, 2], [['shift', 10, 90]]); F('chain', p, [1, 2], [['shift', 10, 0]])
    F('chain', p, undefined, [['inflate', 8]]); F('chain', p, undefined, [['inflate', 8, { smooth: 1 }]]); F('chain', p, [1, 2], [['handles', { angle: 60, length: 1.4 }]]); F('chain', p, undefined, [['smooth']]); F('chain', p, undefined, [['fair', 0.4]])
    F('chain', p, [0], [['soften', 40]]); F('chain', p, [0], [['soften', 80, { along: 'plane', falloff: 'linear' }]]); F('chain', p, [1], [['scale', [2, 1]]]); F('chain', p, [1, 2], [['mirror', 0, [0.5, 0.5]]])
    F('chain', p, [1], [['put', [0.5, 0.5]]]); F('chain', p, [1], [['put', { y: 0.2 }]])
  }
  // Paths of no segments, and selections of none.
  F('chain', dot, undefined, [['rotate', 30, [0, 0]]]); F('chain', dot, [0], [['shift', 3]]); F('chain', dot, [0], [['scale', 2, [0, 0]]]); F('chain', dot, [], [['shift', 3]])
  F('chain', dot, [0], [['smooth']]); F('chain', dot, [0], [['fair']]); F('chain', dot, [0], [['inflate', 3]]); F('chain', dot, [0], [['handles', { turn: 30 }]]); F('chain', dot, [0], [['soften', 10]])
  F('chain', dot, [0], [['put', [1, 2]]]); F('chain', dot, [0], [['moveEach', { k: 'shift', d: [1, 1] }]]); F('chain', dot, [0], [['mirror']]); F('chain', dot, [0], [['invert']])
  F('chain', poly, [], [['rotate', 30, [10, 10]]]); F('chain', poly, [], [['shift', 3]]); F('chain', poly, [], [['smooth']]); F('chain', poly, [], [['fair']]); F('chain', poly, [], [['handles', { turn: 5 }]])
  F('chain', poly, [], [['kind', 'smooth']]); F('chain', poly, [], [['soften', 20]]); F('chain', poly, [], [['invert']]); F('chain', poly, [], [['weigh', 1]])

  // Groups: several paths as one.
  F('group', []); F('group', [], [['rotate', 20, [3, 4]]])
  F('group', [wavy]); F('group', [wavy, far]); F('group', [wavy, ring, far], [['rotate', 15]])
  F('group', [wavy, ring], [['select', { k: 'nodeAt', i: [-1] }]]); F('group', [wavy, ring], [['select', { k: 'section', from: 0, to: 0.5 }], ['shift', 6, 90]])
  F('group', [wavy, ring, unitRing], [['scale', 1.2], ['shear', 10]]); F('group', [wavy, far], [['mirror', 90]]); F('group', [wavy, ring], [['inflate', 5]])
  F('group', [{ sel: [wavy, [1]] }, ring]); F('group', [{ sel: [wavy, [1]] }, ring, { sel: [far, [0]] }], [['rotate', 30]]); F('group', [{ sel: [poly, tops_()] }], [['soften', 30]])
  F('group', [wavy, unitRing], [['soften', 60, { along: 'plane' }]]); F('group', [wavy, ring], [['select', [0]], ['soften', 80, { along: 'plane' }]]); F('group', [wavy, far], [['select', [0]], ['soften', 300, { along: 'plane', falloff: 'linear' }]])
  F('group', [wavy, ring], [['smooth']]); F('group', [wavy, ring], [['fair']]); F('group', [wavy, ring], [['kind', 'smooth']]); F('group', [wavy, dot], [['shift', 2]])
})

function tops_() { return { k: 'above', y: 40 } }
