import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/shape.ts'
import type { Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { ser } from './paths.ts'
import { record } from './record.ts'

const mod = {
  ...m,
  soft: (pts: Pt[], s?: m.Softness, metric?: Pt) => ser(m.soft(pts, s, metric)),
  oval: (x0: number, y0: number, x1: number, y1: number, o?: any) => ser(m.oval(x0, y0, x1, y1, o)),
  lobe: (a: Pt, b: Pt, o?: any, metric?: Pt) => ser(m.lobe(a, b, o, metric)),
}

const polys: Pt[][] = [
  [], [[0, 0]], [[0, 0], [100, 0]], [[0, 0], [100, 0], [100, 100]], [[0, 0], [100, 0], [0, 10]],
  [[0, 0], [50, 100], [100, 0], [150, 100], [200, 0]], [[0, 0], [0, 0], [100, 0], [100, 100]],
  [[0, 0], [100, 0], [100, 100], [0, 100], [0, 0]],
]

export default record(mod, R => {
  // Cases the TS throws on (a lobe with no room) aren't recorded.
  const F = (fn: string, ...a: unknown[]) => { try { (mod as any)[fn](...structuredClone(a)) } catch { return } R(fn, ...a) }
  for (const b of [0, 0.2, -0.3, 1]) F('bowed', [0, 0], [100, 40], b)
  F('bowed', [0, 0], [100, 40])
  F('bowed', [5, 5], [5, 5], 0.5)
  for (const p of polys) {
    F('fitBulges', p, p.slice(1).map(() => 0.3))
    F('fitBulges', p, p.slice(1).map((_, i) => (i % 2 ? -1 : 1) * 2))
    for (const radius of [0, 0.5, 1, 2]) for (const bulge of [0, 0.15, -0.4, 3]) F('softPolyline', p, { radius, bulge })
    F('softPolyline', p)
    F('softPolyline', p, { radius: 0.7, bulge: [0.1, -0.1] })
    F('soft', p, { radius: 1, bulge: 0.1 }, [2, 1])
    F('soft', p)
  }
  for (const start of ['bottom', 'right', 'top', 'left']) {
    F('oval', 0, 0, 100, 200, { start })
    F('oval', -10, 5, 40, 5, { start, tension: 0.6 })
  }
  F('oval', 0, 0, 100, 100)
  const ends: [Pt, Pt][] = [[[0, 100], [0, 0]], [[10, 100], [0, 0]], [[0, 0], [0, 100]], [[0, 100], [5, 50]]]
  for (const [a, b] of ends) {
    for (const nodes of [1, 2, 3, 4.6, 0]) F('lobe', a, b, { nodes, reach: 60 })
    for (const radius of [0, 0.5]) for (const bulge of [0, 0.2, -0.2]) F('lobe', a, b, { radius, bulge, reach: 80 })
    F('lobe', a, b, { reach: 50, axis: 90 })
    F('lobe', a, b, { reach: 50, axis: 80, reachFrom: 'start' })
    F('lobe', a, b, { reach: 40 }, [2, 1])
    F('lobe', a, b)
    F('lobe', a, b, { reach: -5 })
  }
})
