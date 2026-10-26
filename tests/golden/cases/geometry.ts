import * as g from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import type { Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cs, cubics, pts, ts_ } from './fixtures.ts'
import { record } from './record.ts'

export default record(g, G => {
for (const v of [[3, 4], [0, 0], [-1e-9, 2]] as Pt[]) G('norm', v)
for (const a of [0, 2, -2, 4.5, -4.5, Math.PI / 2]) G('halfTurn', a)
G('turn', [1, 0], [0, 1]); G('turn', [1, 0], [-1, -0.001])
G('mean', pts)
for (const c of cs) for (const t of ts_) {
  G('bez', c, t); G('bezD', c, t); G('bezDD', c, t); G('curvature', c, t)
}
for (const c of cs) {
  G('isLine', c); G('split', c, 0.3); G('trim', c, 0.2, 0.7); G('trim', c, 0.7, 0.2)
  G('arcLength', c); G('arcLength', c, 0.4); G('tAtLength', c, 10); G('tAtLength', c, 1e6)
  G('splitEven', c, 3); G('cutAt', c, [0.25, 0.5]); G('cutAt', c, [0.3], [1, 0.5]); G('hullLength', c)
  for (const coord of [0, 1] as const) G('roots', c, coord, 40)
  G('nearest', [c], [50, 50]); G('nearest', [c], [130, 120], 8)
}
G('nearest', cs, [60, -20])
G('nearSegment', [5, 1], [0, 0], [10, 0], 2); G('nearSegment', [12, 1], [0, 0], [10, 0], 2); G('nearSegment', [12, 1], [0, 0], [10, 0], 2, 3)
G('meet', [0, 0], [1, 0], [5, 5], [0, -1]); G('meet', [0, 0], [1, 0], [5, 5], [1, 0])
G('meet', [0, 0], [1, 1], [5, 0], [-1, 1], { inside: 0 }); G('meet', [0, 0], [1, 0], [5, 5], [0, -1], { inside: 1 })
const pieces: Pt[][] = [[[0, 0], [10, 0]], cubics.arch!, [[100, 0], [0, 0]]]
for (const q of pieces) { G('endTangents', q); G('reverse', q); G('pieceLength', q); G('asCubic', q) }
G('endTangents', cubics.handleOnNode); G('endTangents', cubics.dot)
G('box', pieces); G('box', pieces, 4)
G('stitch', [[[0, 0], [10, 0]], [[10.000001, 0], [10, 10]], [[10, 10], [0, 0.000002]]])
for (const al of [10, 90, 180, -120]) G('arcCubics', [0, 0], [100, 0], al)
G('arcCubic', [0, 0], [0, 1], [100, 100], [1, 0])
const poly: Pt[] = [[0, 0], [10, 0], [10, 10], [0, 10]]
const run = g.runLengths(poly)
G('runLengths', poly); G('area', poly); G('area', [...poly].reverse())
for (const s of [-1, 0, 5, 15, 30, 99]) G('pointAlong', poly, run, s)
G('bounds', pts); G('bounds', [[1, 2], [NaN, 5], [3, -1]]); G('center', pts)
G('overlaps', [[0, 0], [10, 10]], [[10, 5], [20, 20]]); G('overlaps', [[0, 0], [10, 10]], [[10.5, 5], [20, 20]], 1)
const m = g.rotation(30, [10, 20])
G('rotation', 30); G('rotation', 30, [10, 20]); G('shear', 12); G('shear', 12, 90, [5, 5]); G('inMetric', m, [1, 0.5])
G('scaling', 2); G('scaling', 2, 3, [1, 1]); G('partly', m, 0.25); G('shift', 10); G('shift', 10, 45)
G('then', m, g.shear(12)); G('apply', m, [3, 4])

})
