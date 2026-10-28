import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/morph.ts'
import { oval } from '../../../../../../apps/Fountain/fountain-ts/src/curves/shape.ts'
import { from, poly } from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import { P, ser, type Spec } from './paths.ts'
import { record } from './record.ts'

const mod = {
  alike: (a: Spec, b: Spec, eps?: number) => { const r = m.alike(P(a), P(b), eps === undefined ? {} : { eps }); return { from: ser(r.from), to: ser(r.to), matched: r.matched } },
  morph: (a: Spec, b: Spec, ks: number[], eps?: number) => ks.map(k => ser(m.morph(P(a), P(b), k, eps === undefined ? {} : { eps }))),
  between: (a: Spec, b: Spec, k: number, eps?: number) => ser(m.between(P(a), P(b), k, eps)),
}

const spec = (p: ReturnType<typeof from>): Spec => ({ start: p.start, segs: p.segs, closed: p.closed, metric: p.metric })
const O = spec(oval(0, 0, 100, 200))
const Osq = spec(oval(10, 0, 110, 190, { tension: 0.75, start: 'right' }))
const sq = spec(poly([0, 0], [100, 0], [100, 100], [0, 100]).close())
const tri = spec(poly([0, 0], [120, 0], [60, 90]).close())
const hex = spec(poly([50, 0], [100, 25], [100, 75], [50, 100], [0, 75], [0, 25]).close())
const wideO = { ...spec(oval(0, 0, 100, 100)), metric: [2, 1] as [number, number] }
const stroke = spec(poly([0, 0], [50, 100], [100, 0]))
const stroke2 = spec(from([0, 10]).curve([20, 120], [80, 120], [100, 10]))
const zig = spec(poly([0, 0], [20, 40], [40, 0], [60, 40], [80, 0], [100, 40]))
const back = spec(poly([100, 0], [50, 100], [0, 0]))
const dot = spec(poly([0, 0], [0, 0], [100, 0]))
const ks = [0, 0.25, 0.5, 1]

const closedSet = [O, Osq, sq, tri, hex, wideO]
const openSet = [stroke, stroke2, zig, back, dot]

export default record(mod, R => {
  // Cases the TS throws on (open into closed) aren't recorded.
  const F = (fn: string, ...a: unknown[]) => { try { (mod as any)[fn](...structuredClone(a)) } catch { return } R(fn, ...a) }
  for (const set of [closedSet, openSet])
    for (const a of set) for (const b of set) { F('alike', a, b); F('morph', a, b, ks) }
  F('alike', O, Osq, 1e3)
  F('morph', sq, hex, [0.5], 50)
  F('morph', O, stroke, ks)
  F('between', sq, sq, 0.5)
  F('between', dot, dot, 0.5)
  F('between', dot, stroke, 0.3, 1e-9)
})
