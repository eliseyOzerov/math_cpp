import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/profile.ts'
import type { Keys, Strength, Swing } from '../../../../../../apps/Fountain/fountain-ts/src/curves/profile.ts'
import { record } from './record.ts'

// Waves are functions, so they are tested by sampling at a list of x. A strength given to a wrapper is a number, a list
// of numbers or { w: 'sine' | 'cosine' | 'square' | 'triangle' | 'saw' | 'gradient' | 'steps', ...options }.
type Spec = number | number[] | ({ w: string, values?: number[], duty?: number } & Swing)
const strength = (s: Spec): Strength => {
  if (typeof s === 'number' || Array.isArray(s)) return s
  switch (s.w) {
    case 'sine': return m.sine(s)
    case 'cosine': return m.cosine(s)
    case 'square': return m.square(s)
    case 'triangle': return m.triangle(s)
    case 'saw': return m.saw(s)
    case 'gradient': return m.gradient(s.values!)
    default: return m.steps(s.values!)
  }
}
const lerpVec = (a: number[], b: number[], u: number) => a.map((x, i) => x + (b[i]! - x) * u)
const at = (w: m.Wave, xs: number[]) => xs.map(w)

const mod = {
  ...m,
  constantAt: (v: number, xs: number[]) => at(m.constant(v), xs),
  gradientAt: (vs: number[], xs: number[]) => at(m.gradient(vs), xs),
  stepsAt: (vs: number[], xs: number[]) => at(m.steps(vs), xs),
  sineAt: (o: Swing, xs: number[]) => at(m.sine(o), xs),
  cosineAt: (o: Swing, xs: number[]) => at(m.cosine(o), xs),
  squareAt: (o: Swing & { duty?: number }, xs: number[]) => at(m.square(o), xs),
  triangleAt: (o: Swing, xs: number[]) => at(m.triangle(o), xs),
  sawAt: (o: Swing, xs: number[]) => at(m.saw(o), xs),
  waveAt: (s: Spec, xs: number[]) => at(m.wave(strength(s)), xs),
  timesAt: (ss: Spec[], xs: number[]) => at(m.times(...ss.map(strength)), xs),
  plusAt: (ss: Spec[], xs: number[]) => at(m.plus(...ss.map(strength)), xs),
  keysAtVec: (k: Keys<number[]>, t: number, d: number[]) => m.keysAt(k, t, d, lerpVec),
  keysAtMixed: (k: Keys<number[]>, t: number, d: number[]) => m.keysAt(k, t, d, lerpVec),
  keysOfSpec: (s: Spec, samples?: number) => m.keysOf(strength(s), samples),
}

const xs = [-0.5, 0, 0.1, 0.25, 1 / 3, 0.5, 0.6, 0.75, 0.9, 1, 1.5]
const swings: Swing[] = [{}, { cycles: 3 }, { cycles: 2, phase: 0.25 }, { low: 0, high: 1 }, { cycles: 1.5, phase: -0.3, low: 2, high: 5 }]
const plain: Keys = [[0, 1], [0.5, 3], [1, 2]]
const handled: Keys = [[0, 0, { out: [0.3, 1] }], [0.5, 2, { in: [-0.2, 0.5], out: [0.1, -1] }], [1, 1, { in: [-0.4, 0] }]]
const tests = [-1, 0, 0.1, 0.25, 0.4, 0.5, 0.7, 1, 2]

export default record(mod, F => {
  F('constantAt', 3.5, xs)
  F('gradientAt', [2], xs); F('gradientAt', [0, 1], xs); F('gradientAt', [1, 5, 2, 8], xs); F('gradientAt', [1, 0, 1], xs)
  F('stepsAt', [4], xs); F('stepsAt', [1, 2, 3], xs); F('stepsAt', [0, 1], xs)
  for (const s of swings) { F('sineAt', s, xs); F('cosineAt', s, xs); F('triangleAt', s, xs); F('sawAt', s, xs); F('squareAt', s, xs) }
  F('sineAt', {}, xs); F('squareAt', { duty: 0.25 }, xs); F('squareAt', { duty: 0.8, cycles: 2, low: 0, high: 3 }, xs)
  F('waveAt', 2, xs); F('waveAt', [1, 3, 2], xs); F('waveAt', { w: 'sine', cycles: 2 }, xs)
  F('timesAt', [], xs); F('timesAt', [2, [0, 1], { w: 'sine', cycles: 2 }], xs); F('timesAt', [{ w: 'saw' }, { w: 'steps', values: [1, 0, 1] }], xs)
  F('plusAt', [], xs); F('plusAt', [1, [0, 1], { w: 'cosine', cycles: 2 }], xs); F('plusAt', [{ w: 'triangle' }, { w: 'gradient', values: [3, 1] }], xs)
  for (const u of [-1, 0, 0.2, 0.5, 0.9, 1, 3]) F('smoothstep', u)

  F('keysAt', [], 0.5); F('keysAt', [], 0.5, 7); F('keysAt', null, 0.5); F('keysAt', null, 0.5, 9)
  for (const t of tests) {
    F('keysAt', plain, t); F('keysAt', handled, t); F('keysAt', [[0.5, 4]], t, 1)
    F('keysAt', [[0, 0], [0, 5], [1, 1]], t); F('keysAt', m.easeKeys(0.2, 1, 0.8, 4), t); F('keysAt', m.bulge(0.2), t); F('keysAt', m.bulge(2, 1), t)
    F('keysAt', [[0, 0, { out: [5, 2] }], [1, 4, { in: [-5, 1] }]], t)
    F('keysAtVec', [[0, [0, 10]], [0.5, [4, 0]], [1, [2, 2]]], t, [9, 9])
    F('keysAtMixed', [[0, [1, 1], { out: [0.2, 1] }], [1, [3, 5]]], t, [0, 0])
  }
  F('keysAtVec', [], 0.5, [9, 9])

  F('sorted', [[1, 1], [0, 2], [0.5, 3], [0.5, 4]]); F('sorted', plain); F('sorted', []); F('sorted', null); F('sorted', handled.slice().reverse())
  F('spread', [5]); F('spread', [1, 2, 3]); F('spread', [0, 1]); F('spread', [])
  F('easeKeys', 0.2, 1, 0.8, 4); F('easeKeys', 0, 0, 1, 1)
  F('bulge', 0.4); F('bulge', 2, 1); F('bulge', 0.4, 0.8); F('bulge', 1)

  F('keysOf', null); F('keysOf', 3); F('keysOf', [2]); F('keysOf', [1, 4, 2]); F('keysOf', []); F('keysOf', plain); F('keysOf', handled)
  F('keysOfSpec', { w: 'sine', cycles: 2 }); F('keysOfSpec', { w: 'saw' }, 5); F('keysOfSpec', { w: 'steps', values: [1, 2] }, 9)
  F('keysTimes', null, null); F('keysTimes', plain, null); F('keysTimes', null, plain)
  F('keysTimes', plain, [[0, 2], [0.3, 2], [0.3, 0.5], [1, 0.5]]); F('keysTimes', handled, plain); F('keysTimes', [], plain)
})
