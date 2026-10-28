import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/fit.ts'
import type { Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { bez } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cs, cubics } from './fixtures.ts'
import { record } from './record.ts'

const sample = (c: (typeof cs)[number], n: number): Pt[] => Array.from({ length: n + 1 }, (_, i) => bez(c, i / n))
const wobble = (c: (typeof cs)[number], n: number): Pt[] => sample(c, n).map((p, i) => [p[0] + Math.sin(i * 1.7) * 0.8, p[1] + Math.cos(i * 2.3) * 0.8] as Pt)
const zig: Pt[] = [[0, 0], [10, 0], [20, 0], [30, 10], [30, 20], [30, 30], [20, 40], [10, 40]]
const star: Pt[] = [[50, 0], [61, 35], [98, 35], [68, 57], [79, 91], [50, 70], [21, 91], [32, 57], [2, 35], [39, 35]]
const sq: Pt[] = [[0, 0], [10, 0], [10, 10], [0, 10]]
const cloud: Pt[] = [[3, 4], [1, 1], [8, 2], [5, 5], [2, 9], [9, 9], [5, 5], [7, 3], [4, 4], [0, 6], [6, 0], [1, 1], [10, 5]]

export default record({
  ...m,
}, F => {
  // fitCubic: general, two points, one point, repeated points, straight, closed-ish
  F('fitCubic', sample(cubics.arch!, 8), [0, 1], [0, 1])
  F('fitCubic', sample(cubics.s!, 12), [0.4472135954999579, 0.8944271909999159], [-0.4472135954999579, 0.8944271909999159])
  F('fitCubic', sample(cubics.skew!, 10), [0.3, 0.9539392014169456], [0.5, 0.8660254037844386])
  F('fitCubic', wobble(cubics.skew!, 14), [0.3, 0.9539392014169456], [0.5, 0.8660254037844386])
  F('fitCubic', [[0, 0], [30, 40]], [1, 0], [0, 1])
  F('fitCubic', [[5, 5]], [1, 0], [0, 1])
  F('fitCubic', [[5, 5], [5, 5], [5, 5]], [1, 0], [0, 1])
  F('fitCubic', [[0, 0], [0, 0], [30, 40], [30, 40]], [1, 0], [0, 1])
  F('fitCubic', [[0, 0], [10, 10], [20, 20], [30, 30]], [0.7071067811865476, 0.7071067811865476], [-0.7071067811865476, -0.7071067811865476])
  F('fitCubic', [[0, 0], [10, 0], [20, 0], [30, 0]], [1, 0], [-1, 0])
  F('fitCubic', [[0, 0], [10, 5], [20, -5], [30, 0]], [0, 1], [0, 1])
  F('fitCubic', [[0, 0], [50, 0], [100, 0], [50, 1], [0, 0]], [1, 0], [1, 0])
  // fitCubics
  for (const c of cs) { F('fitCubics', sample(c, 20), 0.5); F('fitCubics', sample(c, 40), 0.05); F('fitCubics', wobble(c, 30), 1) }
  F('fitCubics', zig, 0.5)
  F('fitCubics', zig, 0.5, { corner: 120 })
  F('fitCubics', star, 1)
  F('fitCubics', star, 1, { corner: 100 })
  F('fitCubics', sq, 0.1, { corner: 30 })
  F('fitCubics', cloud, 0.5, { corner: 179 })
  F('fitCubics', [], 1)
  F('fitCubics', [[1, 1]], 1)
  F('fitCubics', [[1, 1], [1, 1]], 1)
  F('fitCubics', [[0, 0], [10, 10]], 1)
  F('fitCubics', [[0, 0], [5, 5], [10, 10]], 1)
  F('fitCubics', [[0, 0], [5, 5], [10, 10], [20, 20], [30, 30], [40, 40]], 0.1)
  // cubicToQuads
  for (const c of cs) for (const tol of [5, 1, 0.1, 0.01]) F('cubicToQuads', c, tol)
  F('cubicToQuads', [[0, 0], [100, 200], [-100, 200], [100, 0]], 0.5)
  F('cubicToQuads', [[0, 0], [300, 300], [-300, 300], [0, 0]], 0.01)
  F('cubicToQuads', [[0, 0], [100, 0], [200, 0], [300, 0]], 0.01)
  F('cubicToQuads', [[0, 0], [0, 100], [100, 100], [100, 0]], 1000)
  // hull
  F('hull', [])
  F('hull', [[1, 2]])
  F('hull', [[3, 1], [1, 1]])
  F('hull', sq)
  F('hull', cloud)
  F('hull', star)
  F('hull', [[0, 0], [1, 1], [2, 2], [3, 3]])
  F('hull', [[0, 0], [5, 0], [10, 0], [10, 10], [0, 10], [5, 5], [5, 0]])
  F('hull', [[0, 0], [10, 0], [10, 10], [0, 10], [5, 5]], true)
  F('hull', [[0, 0], [0, 10], [10, 0], [10, 10]], true)
  F('hull', [[2, 3], [2, 1], [2, 2], [2, 3]])
  // mergeHulls (convex polygons, counter-clockwise)
  const sqA: Pt[] = [[0, 0], [10, 0], [10, 10], [0, 10]]
  const sqB: Pt[] = [[5, 5], [15, 5], [15, 15], [5, 15]]
  const tri: Pt[] = [[20, 0], [30, 5], [22, 12]]
  const hex: Pt[] = [[0, 0], [4, -3], [10, -3], [14, 0], [10, 3], [4, 3]]
  const hex2 = hex.map(([x, y]) => [x + 6, y + 2] as Pt)
  F('mergeHulls', sqA, sqB)
  F('mergeHulls', sqA, tri)
  F('mergeHulls', hex, hex2)
  F('mergeHulls', hex2, hex)
  F('mergeHulls', tri, hex)
  F('mergeHulls', sqA, sqA)
  F('mergeHulls', [[1, 1]], [[4, 4]])
  F('mergeHulls', [[1, 1], [5, 1]], sqB)
  F('mergeHulls', sqA.slice().reverse(), sqB)
  F('mergeHulls', [[0, 0], [10, 0], [10, 10], [0, 10]], [[10, 0], [20, 0], [20, 10], [10, 10]])
  // clipPolygon
  F('clipPolygon', sqA, { at: [5, 0], n: [1, 0] })
  F('clipPolygon', sqA, { at: [5, 5], n: [1, 1] })
  F('clipPolygon', sqA, { at: [-5, 0], n: [1, 0] })
  F('clipPolygon', sqA, { at: [50, 0], n: [1, 0] })
  F('clipPolygon', sqA, { at: [10, 0], n: [-1, 0] })
  F('clipPolygon', hex, { at: [7, 0], n: [0.6, 0.8] })
  F('clipPolygon', [], { at: [0, 0], n: [1, 0] })
})
