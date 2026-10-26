import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/spline.ts'
import type { Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { ser } from './paths.ts'
import { record } from './record.ts'

const mod = {
  ...m,
  catmullRomPath: (pts: Pt[], o?: m.CatmullRomOptions) => ser(m.catmullRomPath(pts, o)),
  bsplinePath: (pts: Pt[], o?: m.BSplineOptions & { metric?: Pt }) => ser(m.bsplinePath(pts, o)),
  deBoorAt: (pts: Pt[], us: number[], o?: m.BSplineOptions) => us.map(u => m.deBoor(pts, u, o)),
}

const zig: Pt[] = [[0, 0], [100, 200], [200, 0], [300, 200], [400, 0]]
const close: Pt[] = [[0, 0], [1, 0], [1, 1e-13], [50, 80], [51, 80.5], [120, 0]]
const square: Pt[] = [[0, 0], [100, 0], [100, 100], [0, 100]]
const loop: Pt[] = [[0, 0], [100, 0], [100, 100], [0, 100], [0, 0]]
const sets: Pt[][] = [[], [[5, 5]], [[0, 0], [100, 50]], [[0, 0], [100, 50], [200, 0]], zig, close, square, loop]
const us = [0, 0.1, 0.33, 0.5, 0.77, 1, -1, 2]

export default record(mod, R => {
  // Cases the TS throws on (a path of no segments) aren't recorded.
  const F = (fn: string, ...a: unknown[]) => { try { (mod as any)[fn](...structuredClone(a)) } catch { return } R(fn, ...a) }
  for (const s of sets) {
    F('catmullRom', s)
    F('catmullRom', s, { closed: true })
    for (const alpha of [0, 0.5, 1]) F('catmullRom', s, { alpha })
    F('catmullRom', s, { tension: 0 })
    F('catmullRom', s, { tension: 1.5, metric: [2, 1] })
    F('catmullRomPath', s, { closed: true, metric: [1, 3] })
    if (s.length >= 2) {
      for (const degree of [1, 2, 3, 4, 7]) {
        F('bspline', s, { degree })
        F('bspline', s, { degree, closed: true })
        F('deBoorAt', s, us, { degree })
        F('deBoorAt', s, us, { degree, closed: true })
      }
      F('bspline', s, { weights: s.map((_, i) => 1 + (i % 3) * 0.5) })
      F('bspline', s, { weights: s.map((_, i) => 1 + (i % 2)), closed: true, tol: 1e-2 })
      F('bsplinePath', s, { closed: true, metric: [2, 1] })
      F('bsplinePath', s)
    }
  }
  // Exact circle quarter arcs: degree 2, corner weights √½, a knot doubled at each corner.
  const r = Math.SQRT1_2
  const circle: Pt[] = [[100, 0], [100, 100], [0, 100], [-100, 100], [-100, 0], [-100, -100], [0, -100], [100, -100], [100, 0]]
  const cw = [1, r, 1, r, 1, r, 1, r, 1], ck = [0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 4]
  F('bspline', circle, { degree: 2, weights: cw, knots: ck })
  F('deBoorAt', circle, us, { degree: 2, weights: cw, knots: ck })
  // A knot repeated degree times: a corner on the control point.
  F('bspline', zig, { degree: 3, knots: [0, 0, 0, 0, 1, 1, 1, 2, 2] })
  F('deBoorAt', zig, us, { degree: 3, knots: [0, 0, 0, 0, 1, 1, 1, 2, 2] })
  F('bspline', zig, { degree: 2, knots: [0, 0, 0, 0.1, 0.9, 1, 1, 1] })
})
