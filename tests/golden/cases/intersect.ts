import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/intersect.ts'
import type { Cubic } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cs, cubics } from './fixtures.ts'
import { record } from './record.ts'

export default record(m, F => {
for (const c of cs) {
  F('lineCrossings', c, [0, 50], [100, 50]); F('lineCrossings', c, [0, 50], [100, 50], { segment: false })
  F('lineCrossings', c, [-10, -10], [110, 110]); F('lineCrossings', c, [50, -20], [50, 200])
  F('lineCrossings', c, [0, 0], [0, 0])
  F('circleCrossings', c, [50, 50], 40); F('circleCrossings', c, [50, 0], 30, 16); F('circleCrossings', c, c[0], 0)
  F('circleCrossings', c, [50, 50], 1000)
}
F('curveCrossings', cubics.arch, [[0, 50], [40, 50], [60, 50], [100, 50]])
F('curveCrossings', cubics.s, cubics.arch)
F('curveCrossings', cubics.arch, cubics.arch)
F('curveCrossings', cubics.arch, [[100, 0], [150, 50], [150, 50], [200, 0]], { shared: [[1, 0]] })
F('curveCrossings', cubics.arch, [[100, 0], [150, 50], [150, 50], [200, 0]])
F('curveCrossings', cubics.s, [[0, 50], [40, 50], [60, 50], [100, 50]], { tol: 0.1 })
const loop = cubics.loop as Cubic
F('selfCrossings', [loop])
F('selfCrossings', [cubics.arch, [[100, 0], [100, -100], [0, -100], [0, 0]]], true)
const eight: Cubic[] = [[[0, 0], [100, 100], [100, -100], [0, 0]], [[0, 0], [-100, 100], [-100, -100], [0, 0]]]
F('selfCrossings', eight, true); F('selfCrossings', eight, false)
const zig: Cubic[] = [[[0, 0], [30, 0], [70, 100], [100, 100]], [[100, 100], [100, 50], [0, 50], [0, 100]], [[0, 100], [50, 100], [50, 0], [100, 0]]]
F('selfCrossings', zig); F('selfCrossings', zig, true)
F('selfCrossings', [cubics.s]); F('selfCrossings', [cubics.line, cubics.arch])
})
