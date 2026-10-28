import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/offset.ts'
import { cs, ts_ } from './fixtures.ts'
import { record } from './record.ts'

// Function reaches can't be serialised: a linear ramp d0…d1 over t stands in, rebuilt in C++ from the two numbers.
const ramp = (d0: number, d1: number) => (t: number) => d0 + (d1 - d0) * t
const mod = {
  ...m,
  scaleRamp: (c: Parameters<typeof m.scale>[0], d0: number, d1: number) => m.scale(c, ramp(d0, d1)),
  offsetRamp: (c: Parameters<typeof m.offset>[0], d0: number, d1: number) => m.offset(c, ramp(d0, d1)),
}

export default record(mod, F => {
for (const c of cs) {
  F('simple', c); F('reduce', c)
  F('scale', c, 10); F('scale', c, -7)
  F('scaleRamp', c, 5, 15); F('scaleRamp', c, -3, 8)
  F('offset', c, 10); F('offset', c, -12.5)
  F('offsetRamp', c, 4, 12)
  for (const t of ts_) F('offsetAt', c, t, 6)
  F('outline', c, 8); F('outline', c, 8, 3); F('outline', c, 8, 3, 2, 6)
  F('outlineShapes', c, 8); F('outlineShapes', c, 8, 3, 2, 6)
  F('arcs', c); F('arcs', c, 0.05); F('arcs', c, 5)
}
})
