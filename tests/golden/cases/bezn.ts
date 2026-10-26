import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/bezn.ts'
import type { Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { bez } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cs, ts_ } from './fixtures.ts'
import { record } from './record.ts'

const line2: Pt[] = [[0, 0], [10, 20]]
const quad: Pt[] = [[0, 0], [50, 100], [100, 0]]
const quart: Pt[] = [[0, 0], [20, 80], [50, -30], [80, 90], [100, 10]]
const quint: Pt[] = [[0, 0], [10, 60], [40, 90], [60, -40], [90, 50], [120, 0]]
const raised = m.raise(m.raise(quad))
const all: Pt[][] = [line2, quad, ...cs, quart, quint]
const sample = (c: Pt[], n: number): Pt[] => Array.from({ length: n + 1 }, (_, i) => m.bezierAt(c, i / n))
const wobble = (c: Pt[], n: number): Pt[] => sample(c, n).map((p, i) => [p[0] + Math.sin(i * 1.7) * 0.8, p[1] + Math.cos(i * 2.3) * 0.8] as Pt)

export default record(m, F => {
  for (const c of all) for (const t of ts_) { F('bezierAt', c, t); F('casteljauN', c, t); F('splitN', c, t) }
  F('bezierAt', [[3, 4]], 0.5)
  F('bezierAt', cs[0]!, -0.5); F('bezierAt', cs[0]!, 1.5)
  for (const c of all) { F('derivativeN', c); F('raise', c); F('lower', c) }
  F('derivativeN', [[3, 4]])
  F('raise', [[3, 4]])
  F('lower', raised); F('lower', m.raise(quart)); F('lower', m.raise(cs[1]!)); F('lower', [[1, 1], [1, 1], [1, 1], [1, 1]])
  F('lower', [[0, 0], [0, 0], [5, 5]])
  // fitBezier: every order, short and long point lists, repeated points, a straight line
  for (const order of [1, 2, 3, 4, 5]) {
    F('fitBezier', sample(cs[0]!, 12), order); F('fitBezier', sample(cs[1]!, 20), order)
    F('fitBezier', wobble(cs[6]!, 16), order); F('fitBezier', sample(quart, 24), order)
  }
  F('fitBezier', sample(cs[0]!, 12)); F('fitBezier', wobble(cs[1]!, 30)); F('fitBezier', sample(quint, 40))
  F('fitBezier', [[0, 0], [10, 5], [20, 0]]); F('fitBezier', [[0, 0], [30, 40]]); F('fitBezier', [[0, 0], [30, 40]], 2)
  F('fitBezier', [[0, 0], [10, 5], [20, 0], [30, 5], [40, 0]], 2)
  F('fitBezier', [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5]], 3)
  F('fitBezier', [[2, 2], [2, 2], [2, 2], [2, 2], [2, 2], [2, 2]], 3)
  F('fitBezier', [[0, 0], [10, 0], [20, 0], [30, 0], [40, 0], [50, 0]], 2)
  F('fitBezier', [[0, 0], [50, 0], [100, 0], [50, 1], [0, 0], [10, 10]], 3)
  // rationalAt
  for (const t of ts_) {
    F('rationalAt', quad, [1, Math.SQRT1_2, 1], t)
    F('rationalAt', quad, [1, 1, 1], t); F('rationalAt', quad, [1, 3, 1], t); F('rationalAt', quad, [1, 0.2, 1], t)
    F('rationalAt', cs[0]!, [1, 2, 0.5, 1], t); F('rationalAt', cs[0]!, [1], t); F('rationalAt', quart, [1, 2, 3, 2, 1], t)
  }
  // toCubics: lines, quadratics and cubics exactly; higher and rational fitted
  F('toCubics', line2); F('toCubics', quad); F('toCubics', [[5, 5]])
  for (const c of cs) F('toCubics', c)
  F('toCubics', quad, { weights: [1, 1, 1] }); F('toCubics', quad, { weights: [2, 2, 2] }); F('toCubics', quad, { weights: [] })
  F('toCubics', quart); F('toCubics', quart, { tol: 0.05 }); F('toCubics', quint, { tol: 0.01, samples: 32 })
  F('toCubics', quad, { weights: [1, Math.SQRT1_2, 1] })
  F('toCubics', quad, { weights: [1, Math.SQRT1_2, 1], tol: 1e-4, samples: 100 })
  F('toCubics', cs[0]!, { weights: [1, 3, 0.5, 1] })
  F('toCubics', quart, { weights: [1, 2, 3, 2, 1], tol: 0.01 })
  F('toCubics', line2, { weights: [1, 5] })
  // bernstein, solve
  for (const n of [0, 1, 2, 3, 4, 6]) for (const t of [0, 0.3, 0.5, 1]) F('bernstein', n, t)
  F('solve', [[2, 1], [1, 3]], [3, 5])
  F('solve', [[0, 1], [1, 0]], [4, 7])
  F('solve', [[1, 2, 3], [4, 5, 6], [7, 8, 10]], [1, 2, 3])
  F('solve', [[1, 2], [2, 4]], [3, 6])
  F('solve', [[0, 0], [0, 0]], [1, 1])
  F('solve', [[4, 1, 0, 0], [1, 4, 1, 0], [0, 1, 4, 1], [0, 0, 1, 4]], [1, 2, 3, 4])
  F('solve', [[1e-20, 1], [1, 1]], [1, 2])
  F('solve', [[5]], [10])
  F('solve', [], [])
})
