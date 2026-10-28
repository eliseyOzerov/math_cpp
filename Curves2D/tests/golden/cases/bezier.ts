import * as b from '../../../../../../apps/Fountain/fountain-ts/src/curves/bezier.ts'
import type { Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cs, cubics, pts, ts_ } from './fixtures.ts'
import { record } from './record.ts'

export default record(b, B => {
for (const [a, bb, c, d] of [[1, -6, 11, -6], [1, 0, 0, -8], [0, 1, -3, 2], [0, 0, 2, -1], [0, 0, 0, 1], [1, -3, 3, -1], [2, 0, 1, 0], [1e-15, 1, -3, 2]])
  B('cubicRoots', a, bb, c, d)
B('inUnit', [0.5, -1e-10, 1 + 1e-10, 0.5 + 1e-11, 2, -0.1, 0.25])
for (const c of cs) {
  B('cubicCoordRoots', c, 0, 50); B('cubicCoordRoots', c, 1, 20)
  B('extrema', c); B('bbox', c); B('align', c); B('align', c, [0, 0], [0, 10]); B('inflections', c); B('tightBox', c)
  B('canonical', c); B('lut', c, 4)
  for (const t of ts_) { B('casteljau', c, t); B('tangent', c, t); B('normal', c, t); B('osculating', c, t) }
  for (const p of pts) B('project', c, p)
  B('mold', c, 0.5, [60, 40]); B('mold', c, 0, [10, 10])
}
B('coordRoots', 0, 10, -10, 5, 1)
const [S, M, E]: Pt[] = [[0, 0], [40, 60], [100, 0]]
B('quadThrough', S, M, E); B('quadThrough', S, M, E, 0.3)
B('cubicThrough', S, M, E); B('cubicThrough', S, M, E, { t: 0.4 }); B('cubicThrough', S, M, E, { d1: 20, dir: [1, 0.2] })
B('cubicThrough', [0, 0], [0, 0], [0, 0])

})
