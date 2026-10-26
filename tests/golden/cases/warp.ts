import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/warp.ts'
import * as pr from '../../../../../../apps/Fountain/fountain-ts/src/curves/profile.ts'
import type { Contour, Cubic, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import { cs, cubics, pts } from './fixtures.ts'
import { record } from './record.ts'

// Warps are functions, so the cases name them: { k: 'affine', m } | 'quad' | 'sin' | { k: 'spine', dx: number | wave,
// y0, y1 } | { k: 'mesh', lo, hi, grid } | { k: 'wobble', w } | { k: 'pinned', f: spec, ys, reach }. A wave is
// { w: 'sine' | 'saw', cycles?, low?, high? }.
type Spec = { k: string, [key: string]: any }
const wave = (s: any): number | pr.Wave => typeof s === 'number' ? s : s.w === 'saw' ? pr.saw(s) : pr.sine(s)
const warp = (s: Spec): m.Warp => {
  switch (s.k) {
    case 'affine': return p => [s.m[0] * p[0] + s.m[2] * p[1] + s.m[4], s.m[1] * p[0] + s.m[3] * p[1] + s.m[5]]
    case 'quad': return p => [p[0] + 0.004 * p[1] * p[1] - 0.001 * p[0] * p[1], p[1] + 0.002 * p[0] * p[0]]
    case 'sin': return p => [p[0] + 8 * Math.sin(p[1] / 20), p[1] + 3 * Math.cos(p[0] / 30)]
    case 'spine': return m.spine(wave(s.dx), s.y0, s.y1)
    case 'mesh': return m.mesh(s.lo, s.hi, s.grid)
    case 'wobble': return m.wobble(s.w)
    default: return m.pinned(warp(s.f), s.ys, s.reach)
  }
}
const at = (s: Spec, ps: Pt[]) => ps.map(warp(s))

const mod = {
  ...m,
  jacobianAt: (s: Spec, p: Pt, eps?: number) => m.jacobian(warp(s), p, eps),
  warpNodeAt: (n: { p: Pt, in?: Pt, out?: Pt }, s: Spec, eps?: number) => m.warpNode(n, warp(s), eps),
  warpCubicAt: (c: Cubic, s: Spec, o?: m.WarpOptions) => m.warpCubic(c, warp(s), o),
  warpCubicsAt: (c: Cubic[], s: Spec, o?: m.WarpOptions) => m.warpCubics(c, warp(s), o),
  warpContoursAt: (c: Contour[], s: Spec, o?: m.WarpOptions) => m.warpContours(c, warp(s), o),
  warpAt: at,
  noise2At: (seed: number, ps: Pt[]) => ps.map(([x, y]) => m.noise2(seed)(x, y)),
  wobbleWaveAt: (w: m.Wobble, length: number, closed: boolean | undefined, xs: number[]) => xs.map(m.wobbleWave(w, length, closed)),
}

const aff: Spec = { k: 'affine', m: [1.1, 0.2, -0.3, 0.9, 5, -7] }
const grid = (lo: Pt, hi: Pt) => m.meshGrid(lo, hi)
const lo: Pt = [0, 0], hi: Pt = [100, 200]
const moved = grid(lo, hi).map((p, i): Pt => [p[0] + (i % 4) * 3 - 4, p[1] + (i % 3) * 5 - 5])
const grid2: Pt[] = grid(lo, hi).map((p): Pt => [p[0] * 1.2 + 4, p[1] * 0.8 + 10])
const sample: Pt[] = [[0, 0], [50, 100], [100, 200], [25, 150], [-30, 250], [130, -20], [73.5, 41.2], [10, 10]]
const bow: Spec = { k: 'spine', dx: 20, y0: 0, y1: 200 }
const wob: Spec = { k: 'wobble', w: { amplitude: 12, frequency: 8, seed: 3 } }
const rect: Contour = [[[0, 0], [100, 0]], [[100, 0], [100, 100]], [[100, 100], [0, 100]], [[0, 100], [0, 0]]] as any
const ring: Contour = [[[0, 0], [0, 55], [55, 100], [100, 100]], [[100, 100], [50, 0]], [[50, 0], [0, 0]]] as any
const line: Cubic = [[0, 0], [0, 33.3], [0, 66.6], [0, 100]]

export default record(mod, F => {
  for (const s of [aff, bow, wob, 'quad', 'sin'].map(x => typeof x === 'string' ? { k: x } : x)) {
    for (const p of pts) F('jacobianAt', s, p)
    F('jacobianAt', s, [50, 50], 0.5); F('jacobianAt', s, [12.5, 80], 1e-5)
  }

  F('warpNodeAt', { p: [10, 20] }, aff)
  F('warpNodeAt', { p: [10, 20], in: [-5, 3], out: [8, -2] }, aff)
  F('warpNodeAt', { p: [40, 60], out: [10, 30] }, bow)
  F('warpNodeAt', { p: [40, 60], in: [-10, -30], tag: 'x' }, { k: 'quad' }, 1e-2)
  F('warpNodeAt', { p: [40, 60], in: [-10, 5], out: [10, 5] }, wob)

  F('meshGrid', [0, 0], [100, 200]); F('meshGrid', [-10, 5], [30, 7]); F('meshGrid', [0, 0], [0, 0])
  F('warpAt', { k: 'mesh', lo, hi, grid: grid(lo, hi) }, sample)
  F('warpAt', { k: 'mesh', lo, hi, grid: moved }, sample)
  F('warpAt', { k: 'mesh', lo, hi, grid: grid2 }, sample)
  F('warpAt', { k: 'mesh', lo: [5, 5], hi: [5, 5], grid: grid([5, 5], [6, 7]) }, sample)
  F('warpAt', { k: 'mesh', lo: [5, 0], hi: [5, 9], grid: moved }, sample)

  for (const c of cs) {
    F('warpCubicAt', c, aff); F('warpCubicAt', c, bow); F('warpCubicAt', c, { k: 'quad' }, { pieces: 4 })
    F('warpCubicAt', c, wob, { step: 30 }); F('warpCubicAt', c, { k: 'sin' }, { step: 20, eps: 1e-4 })
  }
  F('warpCubicAt', cubics.arch!, bow, { pieces: 0 }); F('warpCubicAt', cubics.arch!, bow, { pieces: 2.5 }); F('warpCubicAt', cubics.arch!, bow, { step: 0 })
  F('warpCubicAt', cubics.arch!, bow, { step: 1000 }); F('warpCubicAt', cubics.arch!, bow, { pieces: 3, step: 10 }); F('warpCubicAt', line, bow, {})
  F('warpCubicsAt', [], aff); F('warpCubicsAt', cs, aff); F('warpCubicsAt', cs.slice(0, 3), wob, { pieces: 3 }); F('warpCubicsAt', [line, cubics.s!], bow, { step: 25 })
  F('warpContoursAt', [], aff); F('warpContoursAt', [rect, ring], aff); F('warpContoursAt', [rect, ring], bow, { pieces: 2 })
  F('warpContoursAt', [rect], wob, { step: 25 }); F('warpContoursAt', [[], ring], { k: 'sin' }, { step: 40 })

  F('warpAt', bow, sample)
  F('warpAt', { k: 'spine', dx: -15, y0: 10, y1: 90 }, sample)
  F('warpAt', { k: 'spine', dx: 0, y0: 0, y1: 100 }, sample)
  F('warpAt', { k: 'spine', dx: 7, y0: 50, y1: 50 }, sample)
  F('warpAt', { k: 'spine', dx: { w: 'sine', cycles: 2, low: -10, high: 10 }, y0: 0, y1: 200 }, sample)
  F('warpAt', { k: 'spine', dx: { w: 'saw', cycles: 3 }, y0: 20, y1: 180 }, sample)
  F('warpAt', { k: 'pinned', f: { k: 'quad' }, ys: [0, 200], reach: 40 }, sample)
  F('warpAt', { k: 'pinned', f: { k: 'sin' }, ys: [50], reach: 100 }, sample)
  F('warpAt', { k: 'pinned', f: { k: 'sin' }, ys: [], reach: 30 }, sample)
  F('warpAt', { k: 'pinned', f: { k: 'sin' }, ys: [0, 100], reach: 0 }, sample)
  F('warpAt', { k: 'pinned', f: { k: 'sin' }, ys: [0, 100], reach: -5 }, sample)
  F('warpAt', { k: 'pinned', f: bow, ys: [0, 100], reach: 30 }, sample)

  for (const w of [{ amplitude: 0 }, { amplitude: 12 }, { amplitude: 12, frequency: 20, octaves: 3, seed: 7 }, { amplitude: 5, frequency: 3.5, octaves: 2.4, seed: 1.5 },
    { amplitude: 9, frequency: 0, seed: -4 }, { amplitude: 9, octaves: 0 }, { amplitude: 3, frequency: 1e-9, seed: 123456 }, { amplitude: 4, octaves: -2, seed: 40000000 }]) {
    F('wavelength', w); F('wobbleStep', w)
    F('warpAt', { k: 'wobble', w }, sample)
    F('warpAt', { k: 'wobble', w }, [[-500.5, 321.1], [1234.5, -987.6], [0, 0], [125, 125], [3000, 3000]])
    F('wobbleWaveAt', { ...w, amplitude: 0.3 }, 800, undefined, [0, 0.1, 0.25, 0.5, 0.77, 1])
    F('wobbleWaveAt', { ...w, amplitude: 0.3 }, 800, true, [0, 0.1, 0.25, 0.5, 0.77, 1])
  }
  F('wavelength', { amplitude: 1, frequency: 1 }); F('wobbleStep', { amplitude: 1, octaves: 4 })
  F('wobbleWaveAt', { amplitude: 0.2, frequency: 8, seed: 2 }, 0, false, [0, 0.5, 1])

  const grid1: Pt[] = []
  for (let i = -20; i < 20; i++) grid1.push([i * 0.37 + 0.11, i * -0.53 + 3.3])
  for (const seed of [0, 1, 2, 3, 12345, -1, 0.5, 1e6, 7.9]) { F('noise2At', seed, grid1); F('noise2At', seed, [[-300.7, 300.2], [255.5, 256.5], [511.2, 1.5], [1e3, -1e3], [0, 0], [1, 1]]) }
})
