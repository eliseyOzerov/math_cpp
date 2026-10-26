import createCurves from '../../build-wasm/curves.js'
import * as geo from '../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import * as off from '../../../../../apps/Fountain/fountain-ts/src/curves/offset.ts'
const cv: any = await createCurves()
const time = (name: string, n: number, f: () => unknown) => {
  for (let i = 0; i < Math.min(n, 1000); i++) f()
  const t = performance.now(); for (let i = 0; i < n; i++) f()
  return ((performance.now() - t) * 1000 / n)
}
const c: geo.Cubic = [[0, 0], [30, 80], [70, 80], [100, 0]]
for (const [name, n, ts, wa] of [
  ['lerp', 1e6, () => geo.lerp(c[0], c[1], 0.3), () => cv.lerp(c[0], c[1], 0.3)],
  ['split', 2e5, () => geo.split(c, 0.3), () => cv.split(c, 0.3)],
  ['arcLength', 1e5, () => geo.arcLength(c), () => cv.arcLength(c)],
  ['outline (cubic)', 2e4, () => off.outline(c, 4, 6), () => cv.outline(c, 4, 6)],
] as const) {
  const a = time(name, n, ts), b = time(name, n, wa)
  console.log(`${name.padEnd(16)} ts ${a.toFixed(3)} µs  wasm ${b.toFixed(3)} µs  ×${(b / a).toFixed(1)}`)
}
