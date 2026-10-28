import * as m from '../../../../../../apps/Fountain/fountain-ts/src/ink/outline.ts'
import { ink } from '../../../../../../apps/Fountain/fountain-ts/src/ink/ink.ts'
import type { Cut, InkOptions, InkStroke } from '../../../../../../apps/Fountain/fountain-ts/src/ink/ink.ts'
import type { NibInput } from '../../../../../../apps/Fountain/fountain-ts/src/ink/nib.ts'
import { record } from './record.ts'
import { strokes } from './ink.ts'

// outline takes the ink of a stroke, which is too big to write into every case, so the cases give the stroke, the nib
// and the options, and the wrappers ink them first.
const mod = {
  ...m,
  inkOutline: (st: InkStroke, nib: NibInput, o: InkOptions, oo?: Parameters<typeof m.outline>[1]) => m.outline(ink(st, nib, o), oo),
  inkPlan: (st: InkStroke, nib: NibInput, o: InkOptions, tolerance?: number) => m.outlinePlan(ink(st, nib, o), tolerance == null ? undefined : { tolerance }),
  // Several nibs on one stroke, one plan for them all, and each outlined by it.
  sharedOutlines: (st: InkStroke, nibs: NibInput[], o: InkOptions) => {
    const inks = nibs.map(n => ink(st, n, o))
    const plan = m.sharedPlan(inks.map(i => m.outlinePlan(i)))
    return plan ? { plan, outlines: inks.map(i => m.outline(i, { plan })) } : null
  },
}

const { line, vline, diag, arch, sCurve, bowl, ring, vee, chevron, boxed, hook } = strokes
const cut = (mode: Cut['mode'], extra: Partial<Cut> = {}): Cut => ({ mode, ...extra })
const rect: NibInput = { w: 50, h: 24, angle: 30 }, round: NibInput = { w: 44, h: 44 }, square: NibInput = { w: 40, h: 40, rx: 0, ry: 0 }
const pen: NibInput = { weight: 40, contrast: 0.5, stress: 25 }
const edge: NibInput = { w: 60, h: 20, angle: -15, rx: [1, 0, 1, 0], ry: 0.5 }
const turning: NibInput = { w: 50, h: 20, follow: 1 }
const all = { line, vline, diag, arch, sCurve, bowl, ring, vee, chevron, boxed, hook }
const plan = (st: InkStroke, nib: NibInput, o: InkOptions = {}) => m.outlinePlan(ink(st, nib, o))
const plans = (...ps: m.OutlinePlan[]) => ps

export default record(mod, F => {
  // Each stroke under each kind of nib, with butt caps (trimmed square) and nib caps (the stamp's far side).
  for (const s of Object.values(all)) for (const n of [round, rect, square, edge]) {
    F('inkOutline', s, n, {})
    F('inkOutline', s, n, { caps: 'nib' })
  }
  for (const s of [line, arch, sCurve, bowl, ring, vee, boxed]) { F('inkOutline', s, pen, {}); F('inkOutline', s, pen, { caps: 'nib' }); F('inkOutline', s, turning, {}); F('inkOutline', s, turning, { caps: 'nib' }) }
  F('inkOutline', line, undefined, {}); F('inkOutline', arch, 30, {}); F('inkOutline', ring, 36, {}); F('inkOutline', sCurve, 50, { caps: 'nib' })
  // Cut ends: level, angled, flat, round, nib; with the nib turned so the rail stops short of the plane.
  for (const c of [cut('flat'), cut('nib'), cut('round'), cut('level'), cut('angle', { deg: 30 }), cut('angle', { deg: 120 }), cut('angle', { deg: 8 })]) {
    for (const s of [arch, sCurve, diag]) {
      F('inkOutline', { ...s, cuts: { start: c, end: c } }, rect, {})
      F('inkOutline', { ...s, cuts: { end: c } }, round, { caps: 'nib' })
    }
  }
  F('inkOutline', { ...arch, cuts: { start: cut('level'), end: cut('flat') } }, turning, {})
  F('inkOutline', { ...sCurve, cuts: { start: cut('angle', { deg: 45 }), end: cut('level') } }, pen, {})
  F('inkOutline', arch, rect, { levels: [0] }); F('inkOutline', arch, rect, { levels: [0, 300] }); F('inkOutline', vline, rect, { levels: [0, 300] })
  F('inkOutline', arch, rect, { levels: [0], cuts: false }); F('inkOutline', arch, round, { levels: [0], caps: 'nib' })
  // Taper, size, rotate, follow, offset, shapes.
  F('inkOutline', { ...arch, cuts: { start: cut('flat', { taper: -0.5 }), end: cut('flat', { taper: -0.5 }) } }, rect, {})
  F('inkOutline', { ...sCurve, cuts: { start: cut('nib', { taper: -0.7, reach: 100 }) } }, round, { caps: 'nib' })
  F('inkOutline', { ...arch, size: [[0, 1], [0.5, 1.5], [1, 0.6]] }, rect, {})
  F('inkOutline', { ...arch, size: [[0, 1], [0.5, 1.5], [1, 0.6]] }, round, { caps: 'nib' })
  F('inkOutline', { ...sCurve, rotate: [[0, 0], [1, 60]] }, rect, {})
  F('inkOutline', { ...arch, follow: [[0, 0], [1, 1]] }, rect, { caps: 'nib' })
  F('inkOutline', { ...arch, offset: [[0, -1], [1, 1]] }, rect, {})
  F('inkOutline', { ...sCurve, offset: [[0, 0.5]] }, round, { caps: 'nib' })
  F('inkOutline', { ...arch, shapes: [[0, { w: 1, h: 1, rx: 1, ry: 1 }], [1, { w: 2, h: 0.5, rx: 0, ry: 0 }]] }, rect, {})
  F('inkOutline', { ...bowl, shapes: [[0, { w: 1, h: 1, rx: [1, 0, 1, 0], ry: 1 }], [0.5, { w: 0.5, h: 1.5, rx: 0.5, ry: 0.5 }], [1, { w: 1, h: 1, rx: 1, ry: 1 }]] }, round, { caps: 'nib' })
  F('inkOutline', { ...ring, size: [[0, 1], [0.5, 0.5], [1, 1]] }, round, {})
  F('inkOutline', { ...ring, shapes: [[0, { w: 1, h: 1, rx: 1, ry: 1 }], [0.5, { w: 2, h: 0.5, rx: 0, ry: 0 }], [1, { w: 1, h: 1, rx: 1, ry: 1 }]] }, rect, {})
  F('inkOutline', { ...vee, size: [[0, 1], [1, 2]], rotate: [[0, 0], [1, 30]] }, rect, { caps: 'nib' })
  F('inkOutline', { ...arch, weight: 1.5 }, rect, {}); F('inkOutline', { ...ring, weight: 0.8 }, pen, {})
  // Spacing and stamps per span; pointed pens; round ends.
  F('inkOutline', arch, rect, { perSpan: 4 }); F('inkOutline', arch, rect, { spacing: 20 }); F('inkOutline', bowl, round, { spacing: 30, caps: 'nib' })
  F('inkOutline', arch, { w: 40, h: 20, perSpan: 8, perCorner: 3 }, {}); F('inkOutline', ring, { w: 40, h: 20, perSpan: 24 }, {})
  for (const s of [line, arch, sCurve, ring, vee, boxed, hook]) { F('inkOutline', s, rect, { model: 'pointed' }); F('inkOutline', s, pen, { model: 'pointed', caps: 'nib' }) }
  F('inkOutline', { ...arch, cuts: { start: cut('round'), end: cut('round') } }, rect, {})
  F('inkOutline', { ...arch, cuts: { start: cut('round'), end: cut('round') } }, rect, { model: 'pointed' })
  // Tolerance, and a plan given.
  for (const tolerance of [0.05, 0.3, 5, 100]) for (const s of [arch, bowl, vee, ring]) F('inkOutline', s, rect, { caps: 'nib' }, { tolerance })
  F('inkOutline', sCurve, round, {}, { tolerance: 0.01 }); F('inkOutline', sCurve, round, { caps: 'nib' }, { tolerance: 0.01 })
  const pArch = plan(arch, rect, { caps: 'nib' }), pBowl = plan(bowl, round, { caps: 'nib' }), pRing = plan(ring, rect), pVee = plan(vee, rect, { caps: 'nib' })
  F('inkOutline', arch, rect, { caps: 'nib' }, { plan: pArch })
  F('inkOutline', arch, round, { caps: 'nib' }, { plan: pArch })
  F('inkOutline', arch, round, { caps: 'nib' }, { plan: { ...pArch, left: [3], right: [4], start: 5, end: 1 } })
  F('inkOutline', arch, round, { caps: 'nib' }, { plan: { left: [2], right: [2], start: 2, end: 2 } })
  F('inkOutline', arch, round, {}, { plan: { left: [2], right: [3], start: 'line', end: 'line' } })
  F('inkOutline', bowl, rect, { caps: 'nib' }, { plan: pBowl }); F('inkOutline', bowl, rect, { caps: 'nib' }, { plan: { ...pBowl, left: pBowl.left.map(k => k + 1), start: 4 } })
  F('inkOutline', ring, round, {}, { plan: pRing }); F('inkOutline', ring, pen, {}, { plan: { ...pRing, left: [2, 2, 2, 2], right: [1, 1, 1, 3] } })
  F('inkOutline', vee, round, { caps: 'nib' }, { plan: pVee }); F('inkOutline', vee, square, { caps: 'nib' }, { plan: { ...pVee, left: pVee.left.map(() => 3) } })
  // outlinePlan.
  for (const s of Object.values(all)) { F('inkPlan', s, rect, {}); F('inkPlan', s, round, { caps: 'nib' }); F('inkPlan', s, edge, { caps: 'nib' }) }
  for (const t of [0.05, 0.5, 3, 50]) { F('inkPlan', arch, rect, { caps: 'nib' }, t); F('inkPlan', bowl, round, {}, t); F('inkPlan', ring, round, {}, t); F('inkPlan', hook, rect, { caps: 'nib' }, t) }
  F('inkPlan', { ...arch, cuts: { start: cut('level') } }, rect, {}); F('inkPlan', arch, pen, { model: 'pointed', caps: 'nib' })
  // sharedPlan and sharedOutlines: shareable inks of one stroke, and ones that can't share.
  for (const s of [arch, sCurve, bowl, vee, ring, hook, line]) {
    F('sharedOutlines', s, [round, rect, square], { caps: 'nib' })
    F('sharedOutlines', s, [rect, { ...rect, w: 70 }, { ...rect, angle: 50 }], {})
    F('sharedOutlines', s, [round, { w: 30, h: 30 }, { w: 60, h: 60 }, edge], { caps: 'nib' })
  }
  F('sharedOutlines', arch, [round], { caps: 'nib' }); F('sharedOutlines', arch, [rect, pen], {})
  const a = plan(arch, rect, { caps: 'nib' }), b = plan(arch, { ...rect, w: 90 }, { caps: 'nib' }), c = plan(arch, round)
  F('sharedPlan', plans(a, b)); F('sharedPlan', plans(a)); F('sharedPlan', plans(a, b, a))
  F('sharedPlan', plans(a, c)); F('sharedPlan', plans(c, a)); F('sharedPlan', plans(plan(arch, rect), plan(bowl, rect)))
  F('sharedPlan', plans(plan(bowl, rect, { caps: 'nib' }), plan(bowl, round, { caps: 'nib' }), plan(bowl, edge, { caps: 'nib' })))
  F('sharedPlan', plans(plan(ring, rect), plan(ring, round)))
  F('sharedPlan', plans({ left: [1, 2], right: [3, 1], start: 2, end: 'line' }, { left: [2, 1], right: [1, 4], start: 5, end: 'line' }))
  F('sharedPlan', plans({ left: [1, 2], right: [3, 1], start: 2, end: 'line' }, { left: [2, 1], right: [1, 4], start: 'line', end: 'line' }))
  F('sharedPlan', plans({ left: [1, 2], right: [3, 1], start: 2, end: 3, runs: { start: 'c', end: 'lcl' } }, { left: [2, 1], right: [1, 4], start: 5, end: 1, runs: { start: 'c', end: 'lcl' } }))
  F('sharedPlan', plans({ left: [1, 2], right: [3, 1], start: 2, end: 3, runs: { start: 'c', end: 'lcl' } }, { left: [2, 1], right: [1, 4], start: 5, end: 1, runs: { start: 'cc', end: 'lcl' } }))
  F('sharedPlan', plans({ left: [1], right: [1], start: 2, end: 2, runs: { start: 'c', end: 'c' } }, { left: [1], right: [1], start: 3, end: 2 }))
  F('sharedPlan', plans({ left: [1], right: [1], start: 2, end: 2 }, { left: [1], right: [2], start: 3, end: 2, runs: { start: 'c', end: 'c' } }))
  F('sharedPlan', plans({ left: [1], right: [1], start: 'line', end: 'line', runs: { start: 'l', end: 'l' } }, { left: [3], right: [1], start: 'line', end: 'line', runs: { start: 'c', end: 'c' } }))
  // planKey.
  for (const p of [a, b, c, pBowl, pRing, pVee, plan(hook, rect, { caps: 'nib' }), plan(line, round), plan(line, round, { caps: 'nib' })]) F('planKey', p)
  F('planKey', { left: [1, 2, 3], right: [3, 2, 1], start: 'line', end: 4 })
  F('planKey', { left: [1], right: [1], start: 2, end: 'line', runs: { start: 'lcl', end: 'l' } })
  F('planKey', { left: [1], right: [1], start: 2, end: 3, runs: { start: 'lcl', end: 'cl' } })
  F('planKey', { left: [], right: [], start: 'line', end: 'line', runs: { start: 'l', end: 'l' } })
})
