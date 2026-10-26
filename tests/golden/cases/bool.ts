import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/bool.ts'
import type { Contour, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { oval } from '../../../../../../apps/Fountain/fountain-ts/src/curves/shape.ts'
import { poly } from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import { P, type Spec } from './paths.ts'
import { record } from './record.ts'

// A Bool is named by data: { op, operands, eps? }, each operand { contours } | { path: spec } | { bool }.
type O = { contours: Contour[] } | { path: Spec } | { bool: B }
type B = { op: m.Op, operands: O[], eps?: number }
const operand = (o: O): m.Operand => 'contours' in o ? o.contours : 'path' in o ? P(o.path) : bool(o.bool)
const bool = (b: B) => new m.Bool(b.op, b.operands.map(operand), b.eps)
const corner = (c: m.BoolCorner) => ({ ...c })

const mod = {
  bool: (b: B) => {
    const x = bool(b), cs = x.contours
    return {
      ok: x.ok, contours: cs, sources: x.sources,
      outward: cs.map((c, k) => c.map((_, j) => x.outward(k, j))),
    }
  },
  corners: (b: B, a?: number, c?: number, same?: boolean) =>
    bool(b).corners(a ?? undefined, c ?? undefined, same === undefined ? {} : { same }).map(corner),
  with: (b: B, operands: O[]) => bool(b).with(operands.map(operand)).contours,
}

const square = (x: number, y: number, s: number, cw = false): Contour => {
  const ps: Pt[] = [[x, y], [x + s, y], [x + s, y + s], [x, y + s]]
  if (cw) ps.reverse()
  return ps.map((p, i) => [p, ps[(i + 1) % 4]!])
}
const spec = (p: ReturnType<typeof poly>): Spec => ({ start: p.start, segs: p.segs, closed: p.closed, metric: p.metric })
const A: O = { contours: [square(0, 0, 100)] }, Bq: O = { contours: [square(50, 50, 100)] }
const ring: O = { contours: [square(0, 0, 100), square(25, 25, 50, true)] }
const far: O = { contours: [square(500, 500, 10)] }
const stem: O = { path: spec(poly([0, 0], [0, 200], [30, 200], [30, 0]).close()) }
const bowl: O = { path: spec(oval(10, 0, 110, 110)) }
const tri: O = { path: spec(poly([0, 0], [120, 0], [60, 90]).close()) }
const ops: m.Op[] = ['union', 'intersect', 'subtract', 'exclude', 'cut']

export default record(mod, F => {
  const sets: O[][] = [[A, Bq], [stem, bowl], [A, ring], [ring, tri], [A, far], [stem, bowl, Bq], [A], [], [tri, tri]]
  for (const op of ops) for (const operands of sets) {
    F('bool', { op, operands })
    F('corners', { op, operands })
    F('corners', { op, operands }, 0)
    F('corners', { op, operands }, 0, 1)
    F('corners', { op, operands }, 1, 0, true)
    F('corners', { op, operands }, null, null, true)
  }
  const inner: B = { op: 'union', operands: [stem, bowl] }
  for (const op of ops) {
    F('bool', { op, operands: [{ bool: inner }, Bq] })
    F('corners', { op, operands: [{ bool: inner }, tri] }, 0, 1)
  }
  F('bool', { op: 'union', operands: [A, Bq], eps: 5 })
  F('with', { op: 'subtract', operands: [A, Bq] }, [stem, bowl])
})
