import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/boolean.ts'
import type { Contour, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cubics } from './fixtures.ts'
import { record } from './record.ts'

// Rules over the groups' windings, by name, since the TS takes a function.
const rules: Record<string, (w: number[]) => boolean> = {
  union: w => w.some(n => n !== 0),
  intersect: w => w.every(n => n !== 0),
  difference: w => w[0] !== 0 && w[1] === 0,
  xor: w => w.filter(n => n !== 0).length === 1,
  nonzero: w => w[0] !== 0,
}
const by = <A extends unknown[], R>(f: (input: Contour[][], inside: (w: number[]) => boolean, ...a: A) => R) =>
  (rule: string, input: Contour[][], ...a: A) => f(input, rules[rule]!, ...a)

const square = (x: number, y: number, s: number, cw = false): Contour => {
  const ps: Pt[] = [[x, y], [x + s, y], [x + s, y + s], [x, y + s]]
  if (cw) ps.reverse()
  return ps.map((p, i) => [p, ps[(i + 1) % 4]!])
}
const K = 0.5522847498
const circle = (cx: number, cy: number, r: number): Contour => {
  const pts: Pt[] = [[cx + r, cy], [cx, cy + r], [cx - r, cy], [cx, cy - r]]
  return pts.map((p, i) => {
    const q = pts[(i + 1) % 4]!, a = (i * Math.PI) / 2, b = a + Math.PI / 2
    return [p, [p[0] - Math.sin(a) * r * K, p[1] + Math.cos(a) * r * K], [q[0] + Math.sin(b) * r * K, q[1] - Math.cos(b) * r * K], q]
  })
}
const A = [square(0, 0, 100)], B = [square(50, 50, 100)], C = [circle(100, 50, 40)], D = [square(100, 0, 100)]
const ring = [square(0, 0, 100), square(25, 25, 50, true)], far = [square(500, 500, 10)]

export default record({ ...m, booleanBy: by(m.boolean), tryBooleanBy: by(m.tryBoolean), traceBooleanBy: by(m.traceBoolean) }, F => {
  F('crossings', cubics.arch, cubics.s); F('crossings', cubics.arch, cubics.loop); F('crossings', cubics.loop, cubics.s, { tol: 0.01 })
  F('crossings', cubics.arch, cubics.arch); F('crossings', cubics.arch, [[0, 50], [30, 50], [60, 50], [100, 50]])
  F('crossings', cubics.arch, cubics.skew, { shared: [[0, 0]] }); F('crossings', cubics.s, [[200, 0], [210, 5], [220, 5], [230, 0]])
  for (const p of [[50, 50], [-5, 50], [100, 50], [50, 100], [70, 70], [10, 10], [200, 200]] as Pt[]) {
    F('winding', A, p); F('winding', ring, p); F('winding', C, p); F('winding', [...A, ...C], p)
  }
  for (const rule of Object.keys(rules)) for (const [x, y] of [[A, B], [A, C], [A, D], [ring, C], [A, far], [C, B]])
    F('booleanBy', rule, [x, y])
  F('booleanBy', 'nonzero', [[...A, ...B, ...C]]); F('booleanBy', 'union', [A, B], 0.01); F('booleanBy', 'union', [A, B], { eps: 2 })
  F('tryBooleanBy', 'union', [A, C]); F('traceBooleanBy', 'difference', [A, C]); F('traceBooleanBy', 'union', [ring, B])
  F('union', [...A, ...B]); F('union', [...ring, ...C]); F('union', [])
  for (const c of [...A, ...ring, ...C]) { F('contourArea', c); F('reverseContour', c) }
  F('orient', ring); F('orient', [...C, ...A]); F('orient', [square(0, 0, 10, true)])
  F('flatten', [...A, ...C]); F('flatten', C, 5, 3); F('flatten', C, 0.5)
  const polys = m.flatten([...ring, ...C], 10, 4)
  F('edges', polys); F('edges', [[[0, 0], [10, 0], [10, 10]]])
  const es = m.edges(polys)
  for (const y of [0, 10, 25, 50, 75, 100, 101]) F('scanline', es, y)
  F('narrowest', polys[2]!, 20); F('narrowest', polys[0]!, 50); F('narrowest', polys[0]!, 1e6)
  F('polyGap', polys[0]!, polys[2]!); F('polyGap', polys[1]!, [[500, 500]])
})
