// Checks the wasm module's embind API against the TypeScript it is ported from: the same calls made on both sides
// must give the same values. The golden tests (`node build-wasm/curves_tests.js`) cover the arithmetic; this covers
// the bindings, that each name takes and returns what the TS does.
//
//   node tests/wasm/smoke.ts            after `emcmake cmake -B build-wasm && cmake --build build-wasm`
import assert from 'node:assert/strict'
import * as ts from '../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import * as tsMods from '../../../../../apps/Fountain/fountain-ts/src/curves/mods.ts'
import * as tsSvg from '../../../../../apps/Fountain/fountain-ts/src/curves/svg.ts'
import * as tsGeo from '../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'
import * as tsBezier from '../../../../../apps/Fountain/fountain-ts/src/curves/bezier.ts'
import * as tsBezn from '../../../../../apps/Fountain/fountain-ts/src/curves/bezn.ts'
import * as tsFit from '../../../../../apps/Fountain/fountain-ts/src/curves/fit.ts'
import * as tsBoolean from '../../../../../apps/Fountain/fountain-ts/src/curves/boolean.ts'
import * as tsCorners from '../../../../../apps/Fountain/fountain-ts/src/curves/corners.ts'
import * as tsNodes from '../../../../../apps/Fountain/fountain-ts/src/curves/nodes.ts'
import * as tsProfile from '../../../../../apps/Fountain/fountain-ts/src/curves/profile.ts'
import * as tsShape from '../../../../../apps/Fountain/fountain-ts/src/curves/shape.ts'
import * as tsWarp from '../../../../../apps/Fountain/fountain-ts/src/curves/warp.ts'
import * as tsAttach from '../../../../../apps/Fountain/fountain-ts/src/curves/attach.ts'
import * as tsBool from '../../../../../apps/Fountain/fountain-ts/src/curves/bool.ts'
import * as tsAnalysis from '../../../../../apps/Fountain/fountain-ts/src/curves/analysis.ts'
import * as tsOffset from '../../../../../apps/Fountain/fountain-ts/src/curves/offset.ts'
import * as tsIntersect from '../../../../../apps/Fountain/fountain-ts/src/curves/intersect.ts'
import * as tsMorph from '../../../../../apps/Fountain/fountain-ts/src/curves/morph.ts'
import * as tsSpline from '../../../../../apps/Fountain/fountain-ts/src/curves/spline.ts'
import * as tsSelection from '../../../../../apps/Fountain/fountain-ts/src/curves/selection.ts'
import createCurves from '../../build-wasm/curves.js'

const cv = await createCurves()
let checks = 0
const same = (name: string, wasm: unknown, js: unknown) => { assert.deepEqual(wasm, js, name); checks++ }

const pts: [number, number][] = [[0, 0], [100, 0], [100, 80], [30, 120]]
const t = ts.poly(pts[0]!, ...pts.slice(1)).close()
const w = cv.poly(pts[0], pts.slice(1)).close()

same('length', w.length, t.length)
same('at', w.at(0.3), t.at(0.3))
same('nodes', w.nodes, t.nodes)
same('segs', w.segs, t.segs)
same('bbox', w.bbox(), t.bbox())
same('crossY', w.crossY(40), t.crossY(40))
same('outline', cv.svgPath(w.outline(6)), tsSvg.svgPath(t.outline(6)))

const curved = (p: any) => p.quarter([60, 60]).arch([120, 0], 30)
const tc = curved(ts.from([0, 0])), wc = curved(cv.from([0, 0]))
same('quarter + arch', wc.segs, tc.segs)
same('project', wc.project([50, 50]), tc.project([50, 50]))

const ts1 = t.select([1, 2]).shift(10, 90).path, ws1 = w.select([1, 2]).shift(10, 90).path
same('select by nodes, shift', ws1.nodes, ts1.nodes)
const ts2 = t.select().soften(0.3).inflate(0.1).path, ws2 = w.select().soften(0.3).inflate(0.1).path
same('soften, inflate', ws2.segs, ts2.segs)

for (const m of tsMods.mods) {
  const v = tsMods.modDefaults(m)
  same(`modDefaults ${m.name}`, cv.modDefaults(m.name), v)
  same(`applyMod ${m.name}`, cv.applyMod(w.select(), m.name, v).path.segs, tsMods.applyMod(t.select(), m, v).path.segs)
  same(`modCode ${m.name}`, cv.modCode(m.name, v), tsMods.modCode(m, v))
}
const steps: { mod: string; values: Record<string, number | string> }[] = [{ mod: 'soften', values: { reach: 0.6, falloff: 'linear' } }, { mod: 'rotate', values: { deg: 15 } }]
same('chain', cv.chain(w.select(), steps).path.segs, tsMods.chain(t.select(), steps).path.segs)
same('chainCode', cv.chainCode(steps), tsMods.chainCode(steps))

const d = 'M0 0C10 20 30 20 40 0L40 40Z'
same('parseSvgPath', cv.parseSvgPath(d), tsSvg.parseSvgPath(d))

// The lower files' free functions. Paths made here are deleted at the end.
const owned: { delete(): void }[] = []
const own = <T extends { delete(): void }>(x: T) => { owned.push(x); return x }
const c: tsGeo.Cubic = [[0, 0], [20, 60], [80, 60], [100, 0]]
const cs: tsGeo.Contour[] = [t.segs as unknown as tsGeo.Contour], wcs = cs

same('dot', cv.dot([1, 2], [3, 4]), tsGeo.dot([1, 2], [3, 4]))
same('rotate', cv.rotate([3, 4], 0.7), tsGeo.rotate([3, 4], 0.7))
same('turn', cv.turn([1, 0], [0, 1]), tsGeo.turn([1, 0], [0, 1]))
same('bez, bezD', [cv.bez(c, 0.3), cv.bezD(c, 0.3)], [tsGeo.bez(c, 0.3), tsGeo.bezD(c, 0.3)])
same('split', cv.split(c, 0.4), tsGeo.split(c, 0.4))
same('nearest', cv.nearest(t.segs, [50, 50]), tsGeo.nearest(t.segs, [50, 50]))
same('nearSegment', cv.nearSegment([5, 1], [0, 0], [10, 0], 2), tsGeo.nearSegment([5, 1], [0, 0], [10, 0], 2))
same('arcLength', cv.arcLength(c, 0.5), tsGeo.arcLength(c, 0.5))
same('box', cv.box(t.segs), tsGeo.box(t.segs))
same('arcCubics', cv.arcCubics([0, 0], [100, 0], 90), tsGeo.arcCubics([0, 0], [100, 0], 90))
same('rotation, apply', cv.apply(cv.rotation(30, [10, 10]), [50, 0]), tsGeo.apply(tsGeo.rotation(30, [10, 10]), [50, 0]))
same('scaling', cv.scaling(2), tsGeo.scaling(2))
same('shift', cv.shift(5, 90), tsGeo.shift(5, 90))
same('bbox (bezier)', cv.bbox(c), tsBezier.bbox(c))
same('raise', cv.raise(c), tsBezn.raise(c))

const sample = Array.from({ length: 12 }, (_, i) => tsGeo.bez(c, i / 11))
same('fitCubic', cv.fitCubic(sample, [0.3, 1], [0.3, -1]), tsFit.fitCubic(sample, [0.3, 1], [0.3, -1]))
same('fitCubics', cv.fitCubics(sample, 0.5, { corner: 45 }), tsFit.fitCubics(sample, 0.5, { corner: 45 }))
same('cubicToQuads', cv.cubicToQuads(c, 0.5), tsFit.cubicToQuads(c, 0.5))
same('hull', cv.hull(sample), tsFit.hull(sample))
same('clipPolygon', cv.clipPolygon(pts, { at: [50, 0], n: [1, 0] }), tsFit.clipPolygon(pts, { at: [50, 0], n: [1, 0] }))

same('contourArea', cv.contourArea(cs[0]), tsBoolean.contourArea(cs[0]!))
same('orient', cv.orient(wcs), tsBoolean.orient(cs))
const polys = tsBoolean.flatten(cs, 4)
same('flatten', cv.flatten(wcs, 4), polys)
same('edges', cv.edges(polys), tsBoolean.edges(polys))
same('scanline', cv.scanline(tsBoolean.edges(polys), 40), tsBoolean.scanline(tsBoolean.edges(polys), 40))

same('pits, peaks', [cv.pits(wcs), cv.peaks(wcs, 30)], [tsCorners.pits(cs), tsCorners.peaks(cs, 30)])
same('roundCorners', cv.roundCorners(wcs, 8, { minDeg: 20 }), tsCorners.roundCorners(cs, 8, { minDeg: 20 }))
same('roundCorners by corner', cv.roundCorners(wcs, (k: any) => k.at * 3), tsCorners.roundCorners(cs, k => k.at * 3))
same('shapeCorners', cv.shapeCorners(wcs, () => 6, () => ({ angle: 30 })), tsCorners.shapeCorners(cs, () => 6, () => ({ angle: 30 })))
same('caps', [cv.capsTriangle(), cv.capsRect(0.3), cv.capsArc()], [tsCorners.caps.triangle(), tsCorners.caps.rect(0.3), tsCorners.caps.arc()])
const end = { p: [100, 0] as [number, number], t: [1, 0] as [number, number], r: 5 }
same('capped', cv.capped(wcs, end, { shape: cv.capsArc() }), tsCorners.capped(cs, end, { shape: tsCorners.caps.arc() }))

const ns = tsNodes.nodesOf(tc)
same('nodesOf', cv.nodesOf(wc), ns)
same('segmentsOf', cv.segmentsOf(ns, false), tsNodes.segmentsOf(ns, false))
same('pathOf (nodes)', own(cv.pathOf(ns, false)).segs, tsNodes.pathOf(ns, false).segs)
same('pointOn', cv.pointOn(ns, false, 1, 0.4), tsNodes.pointOn(ns, false, 1, 0.4))
same('nearestOn', cv.nearestOn(ns, false, [50, 50]), tsNodes.nearestOn(ns, false, [50, 50]))
same('crossingsOn', cv.crossingsOn(ns, false, 1, 30), tsNodes.crossingsOn(ns, false, 1, 30))
same('moldNodes', cv.moldNodes(ns, false, 1, 0.5, [70, 70]), tsNodes.moldNodes(ns, false, 1, 0.5, [70, 70]))
same('nodeKind', ns.map(n => cv.nodeKind(n)), ns.map(tsNodes.nodeKind))
same('withHandle', cv.withHandle(ns[1], 'out', [10, 5]), tsNodes.withHandle(ns[1]!, 'out', [10, 5]))
same('withNodeKind', cv.withNodeKind(ns[1], 'smooth'), tsNodes.withNodeKind(ns[1]!, 'smooth'))
same('insertNode', cv.insertNode(ns, 0, 0.5), tsNodes.insertNode(ns, 0, 0.5))
same('simplified', cv.simplified(ns, false, 1), tsNodes.simplified(ns, false, 1))
const lists = [ns.map(n => tsNodes.withNodeKind(n, 'symmetric'))]
const kl = tsNodes.kindLocks(lists)
same('kindLocks', cv.kindLocks(lists), kl)
const r = { s: 0, i: 1, side: 'in' as const }
same('lockOf', cv.lockOf(kl.locks, r), tsNodes.lockOf(kl.locks, r))
same('withLockedHandle', cv.withLockedHandle(lists, kl.locks, r, [-20, 4]), tsNodes.withLockedHandle(lists, kl.locks, r, [-20, 4]))
const r2 = [{ s: 0, i: 0, side: 'out' as const }, { s: 0, i: 2, side: 'in' as const }]
same('locked', cv.locked(kl.locks, r2, { angle: true }), tsNodes.locked(kl.locks, r2, { angle: true }))
same('unlocked', cv.unlocked(kl.locks, [r]), tsNodes.unlocked(kl.locks, [r]))
same('remapLocks', cv.remapLocks(kl.locks, 0, (i: number) => i === 2 ? undefined : i + 1), tsNodes.remapLocks(kl.locks, 0, i => i === 2 ? undefined : i + 1))
same('lockedKind', cv.lockedKind(kl.locks, ns[1], 0, 1), tsNodes.lockedKind(kl.locks, ns[1]!, 0, 1))
same('lockedKinds', cv.lockedKinds(kl.lists, kl.locks), tsNodes.lockedKinds(kl.lists, kl.locks))

const ks = tsProfile.easeKeys(0.2, 1, 0.8, 3)
same('easeKeys', cv.easeKeys(0.2, 1, 0.8, 3), ks)
same('keysAt', [cv.keysAt(ks, 0.5), cv.keysAt(null, 0.5, 7)], [tsProfile.keysAt(ks, 0.5), tsProfile.keysAt(null, 0.5, 7)])
same('keysOf', [cv.keysOf(2), cv.keysOf([1, 3, 2]), cv.keysOf(ks), cv.keysOf((x: number) => x * x, 5), cv.keysOf(undefined)],
  [tsProfile.keysOf(2), tsProfile.keysOf([1, 3, 2]), tsProfile.keysOf(ks), tsProfile.keysOf(x => x * x, 5), tsProfile.keysOf(undefined)])
same('keysTimes', [cv.keysTimes(ks, [[0, 2], [1, 1]]), cv.keysTimes(null, ks), cv.keysTimes(undefined, null)],
  [tsProfile.keysTimes(ks, [[0, 2], [1, 1]]), tsProfile.keysTimes(null, ks), tsProfile.keysTimes(undefined, null)])
same('sorted, spread', [cv.sorted([[0.5, 1], [0, 2]]), cv.spread([1, 2, 3])], [tsProfile.sorted([[0.5, 1], [0, 2]]), tsProfile.spread([1, 2, 3])])

same('soft', own(cv.soft(pts, { radius: 0.2, bulge: [0, 0.3, 0] }, [100, 100])).segs, tsShape.soft(pts, { radius: 0.2, bulge: [0, 0.3, 0] }, [100, 100]).segs)
same('oval', own(cv.oval(0, 0, 100, 60, { start: 'left' })).segs, tsShape.oval(0, 0, 100, 60, { start: 'left' }).segs)
same('lobe', own(cv.lobe([0, 0], [100, 0], { reach: 0.6, bulge: 0.2 }, [100, 100])).segs, tsShape.lobe([0, 0], [100, 0], { reach: 0.6, bulge: 0.2 }, [100, 100]).segs)

const grid = tsWarp.meshGrid([0, 0], [100, 120]).map(([x, y]) => [x + y / 10, y] as [number, number])
same('meshGrid', cv.meshGrid([0, 0], [100, 120]), tsWarp.meshGrid([0, 0], [100, 120]))
const wm = cv.mesh([0, 0], [100, 120], grid), tm = tsWarp.mesh([0, 0], [100, 120], grid)
same('mesh (a returned warp)', wm([30, 40]), tm([30, 40]))
same('pinned (a JS warp in, a warp out)', cv.pinned(tm, [0, 120], 20)([30, 10]), tsWarp.pinned(tm, [0, 120], 20)([30, 10]))
same('spine', cv.spine((y: number) => y / 4, 0, 120)([10, 60]), tsWarp.spine(y => y / 4, 0, 120)([10, 60]))
same('warpContours', cv.warpContours(wcs, wm, { step: 10 }), tsWarp.warpContours(cs, tm, { step: 10 }))
same('warpNode', cv.warpNode(ns[1], tm), tsWarp.warpNode(ns[1]!, tm))
const wob = { amplitude: 4, frequency: 12, octaves: 2, seed: 3 }
same('wavelength, wobbleStep', [cv.wavelength(wob), cv.wobbleStep(wob)], [tsWarp.wavelength(wob), tsWarp.wobbleStep(wob)])
same('wobble', cv.wobble(wob)([40, 50]), tsWarp.wobble(wob)([40, 50]))
same('wobbleWave', cv.wobbleWave(wob, 300, true)(0.3), tsWarp.wobbleWave(wob, 300, true)(0.3))

const tl = ts.line([0, 0], [10, 0]), wl = own(cv.line([0, 0], [10, 0]))
same('place', own(cv.place(wl, [{ point: [5, 5] }, { on: w, at: 0.5 }])).segs, tsAttach.place(tl, [{ point: [5, 5] }, { on: t, at: 0.5 }]).segs)
same('place, aligned', own(cv.place(wl, [{ point: [5, 5] }], { on: w, at: 0.2, to: 'normal', angle: 10 })).segs,
  tsAttach.place(tl, [{ point: [5, 5] }], { on: t, at: 0.2, to: 'normal', angle: 10 }).segs)

const sq = ts.poly([40, 40], [140, 40], [140, 100], [40, 100]).close(), wsq = own(cv.poly([40, 40], [[140, 40], [140, 100], [40, 100]]).close())
const tb = new tsBool.Bool('union', [t, sq]), wb = own(new cv.Bool('union', [w, wsq]))
same('Bool', [wb.op, wb.eps, wb.ok, wb.contours, wb.sources], [tb.op, tb.eps, tb.ok, tb.contours, tb.sources])
same('Bool corners', [wb.corners(), wb.corners(0, 1, { same: true })], [tb.corners(), tb.corners(0, 1, { same: true })])
same('Bool outward', wb.outward(0, 1), tb.outward(0, 1))
const tb2 = new tsBool.Bool('subtract', [tb, [[[[60, 60], [80, 60]], [[80, 60], [70, 80]], [[70, 80], [60, 60]]]]], 0.5)
const wb2 = own(new cv.Bool('subtract', [wb, [[[[60, 60], [80, 60]], [[80, 60], [70, 80]], [[70, 80], [60, 60]]]]], 0.5))
same('Bool of a Bool', wb2.contours, tb2.contours)
same('Bool.with', own(wb.with([wsq])).contours, tb.with([sq]).contours)

// The functions the TS uses only inside src/curves. A name two TS files share takes a cubic or a Path.
const wave = (f: (u: number) => number) => [0, 0.13, 0.5, 0.77, 1].map(f)
same('KAPPA, identity', [cv.KAPPA, cv.identity], [tsGeo.KAPPA, tsGeo.identity])
same('neg, fromAngle, mean', [cv.neg([1, -2]), cv.fromAngle(0.4), cv.mean(pts)], [tsGeo.neg([1, -2]), tsGeo.fromAngle(0.4), tsGeo.mean(pts)])
same('bezDD', cv.bezDD(c, 0.3), tsGeo.bezDD(c, 0.3))
same('meet', [cv.meet([0, 0], [1, 1], [10, 0], [-1, 1]), cv.meet([0, 0], [1, 0], [0, 1], [1, 0]), cv.meet([0, 0], [1, 1], [10, 0], [-1, 1], { inside: 0.1 })],
  [tsGeo.meet([0, 0], [1, 1], [10, 0], [-1, 1]), tsGeo.meet([0, 0], [1, 0], [0, 1], [1, 0]), tsGeo.meet([0, 0], [1, 1], [10, 0], [-1, 1], { inside: 0.1 })])
const ln: [[number, number], [number, number]] = [[0, 0], [30, 40]]
same('asCubic, reverse, endTangents', [cv.asCubic(ln), cv.reverse(c), cv.endTangents(ln), cv.endTangents(c)],
  [tsGeo.asCubic(ln), tsGeo.reverse(c), tsGeo.endTangents(ln), tsGeo.endTangents(c)])
same('trim, roots', [cv.trim(c, 0.2, 0.7), cv.roots(c, 1, 30), cv.roots(c, 0, 50, 8)], [tsGeo.trim(c, 0.2, 0.7), tsGeo.roots(c, 1, 30), tsGeo.roots(c, 0, 50, 8)])
same('tAtLength, pieceLength', [cv.tAtLength(c, 40), cv.tAtLength(c, 40, 200), cv.pieceLength(ln), cv.pieceLength(c)],
  [tsGeo.tAtLength(c, 40), tsGeo.tAtLength(c, 40, 200), tsGeo.pieceLength(ln), tsGeo.pieceLength(c)])
const gappy = [[[0, 0], [10, 0]], [[10.5, 0], [10, 10]], [[10, 10], [0, 0.2]]] as any
same('stitch (returns the contour)', cv.stitch(gappy), tsGeo.stitch(structuredClone(gappy)))
same('arcCubic', cv.arcCubic([0, 0], [1, 0], [50, 50], [0, 1]), tsGeo.arcCubic([0, 0], [1, 0], [50, 50], [0, 1]))
same('cutAt, splitEven', [cv.cutAt(c, [0.2, 0.5]), cv.cutAt(c, [0.5], [1, 2]), cv.splitEven(c, 3)],
  [tsGeo.cutAt(c, [0.2, 0.5]), tsGeo.cutAt(c, [0.5], [1, 2]), tsGeo.splitEven(c, 3)])
const run = tsGeo.runLengths(pts)
same('runLengths, pointAlong, hullLength', [cv.runLengths(pts), cv.pointAlong(pts, run, 150), cv.hullLength(c)],
  [run, tsGeo.pointAlong(pts, run, 150), tsGeo.hullLength(c)])
same('area, bounds, center', [cv.area(pts), cv.bounds(pts), cv.center(pts)], [tsGeo.area(pts), tsGeo.bounds(pts), tsGeo.center(pts)])
const b1: [[number, number], [number, number]] = [[0, 0], [10, 10]], b2: [[number, number], [number, number]] = [[11, 0], [20, 10]]
same('overlaps, within', [cv.overlaps(b1, b2), cv.overlaps(b1, b2, 2), cv.within(b1, [0.3, 0.6])], [tsGeo.overlaps(b1, b2), tsGeo.overlaps(b1, b2, 2), tsGeo.within(b1, [0.3, 0.6])])
same('toMetric, fromMetric', [cv.toMetric([3, 4], [2, 0.5]), cv.fromMetric([3, 4], [2, 0.5])], [tsGeo.toMetric([3, 4], [2, 0.5]), tsGeo.fromMetric([3, 4], [2, 0.5])])
const rot = tsGeo.rotation(30, [10, 10])
same('then_ (then), shear', [cv.then_(rot, cv.scaling(2)), cv.shear(20), cv.shear(20, 1), cv.shear(20, 0, [5, 5])],
  [tsGeo.then(rot, tsGeo.scaling(2)), tsGeo.shear(20), tsGeo.shear(20, 1), tsGeo.shear(20, 0, [5, 5])])
same('inMetric, partly', [cv.inMetric(rot, [1, 2]), cv.partly(rot, 0.3)], [tsGeo.inMetric(rot, [1, 2]), tsGeo.partly(rot, 0.3)])

same('cubicRoots, inUnit', [cv.cubicRoots(1, -6, 11, -6), cv.inUnit([-0.1, 0.5, 1.2]), cv.inUnit([1.05], 0.1)],
  [tsBezier.cubicRoots(1, -6, 11, -6), tsBezier.inUnit([-0.1, 0.5, 1.2]), tsBezier.inUnit([1.05], 0.1)])
same('coordRoots, cubicCoordRoots', [cv.coordRoots(0, 60, 60, 0), cv.coordRoots(0, 60, 60, 0, 30), cv.cubicCoordRoots(c, 1, 30)],
  [tsBezier.coordRoots(0, 60, 60, 0), tsBezier.coordRoots(0, 60, 60, 0, 30), tsBezier.cubicCoordRoots(c, 1, 30)])
same('extrema, inflections (cubic)', [cv.extrema(c), cv.inflections([[0, 0], [100, 80], [0, 80], [100, 0]])],
  [tsBezier.extrema(c), tsBezier.inflections([[0, 0], [100, 80], [0, 80], [100, 0]])])
same('align', [cv.align(c), cv.align(c, [20, 60]), cv.align(c, [20, 60], [80, 60])], [tsBezier.align(c), tsBezier.align(c, [20, 60]), tsBezier.align(c, [20, 60], [80, 60])])
same('tightBox, canonical', [cv.tightBox(c), cv.canonical(c), cv.canonical([[0, 0], [100, 80], [0, 80], [100, 0]], 1e-3)],
  [tsBezier.tightBox(c), tsBezier.canonical(c), tsBezier.canonical([[0, 0], [100, 80], [0, 80], [100, 0]], 1e-3)])
same('casteljau, lut, tangent', [cv.casteljau(c, 0.3), cv.lut(c), cv.lut(c, 5), cv.tangent(c, 0.3)],
  [tsBezier.casteljau(c, 0.3), tsBezier.lut(c), tsBezier.lut(c, 5), tsBezier.tangent(c, 0.3)])
same('normal, curvature, osculating (cubic)', [cv.normal(c, 0.3), cv.curvature(c, 0.3), cv.osculating(c, 0.3), cv.osculating([[0, 0], [1, 0], [2, 0], [3, 0]], 0.5)],
  [tsBezier.normal(c, 0.3), tsGeo.curvature(c, 0.3), tsBezier.osculating(c, 0.3), tsBezier.osculating([[0, 0], [1, 0], [2, 0], [3, 0]], 0.5)])
same('project, mold (cubic)', [cv.project(c, [50, 50]), cv.mold(c, 0.4, [40, 80])], [tsBezier.project(c, [50, 50]), tsBezier.mold(c, 0.4, [40, 80])])
same('quadThrough, cubicThrough', [cv.quadThrough([0, 0], [50, 40], [100, 0]), cv.quadThrough([0, 0], [50, 40], [100, 0], 0.3),
  cv.cubicThrough([0, 0], [50, 40], [100, 0]), cv.cubicThrough([0, 0], [50, 40], [100, 0], { t: 0.4, d1: 20, dir: [1, 0.2] })],
  [tsBezier.quadThrough([0, 0], [50, 40], [100, 0]), tsBezier.quadThrough([0, 0], [50, 40], [100, 0], 0.3),
   tsBezier.cubicThrough([0, 0], [50, 40], [100, 0]), tsBezier.cubicThrough([0, 0], [50, 40], [100, 0], { t: 0.4, d1: 20, dir: [1, 0.2] })])

same('project, extrema, inflections, bbox (path)', [cv.project(wc, [50, 50]), cv.extrema(wc), cv.inflections(wc), cv.bbox(wc)],
  [tsAnalysis.project(tc, [50, 50]), tsAnalysis.extrema(tc), tsAnalysis.inflections(tc), tsAnalysis.bbox(tc)])
same('curvature, normal, osculating (path)', [cv.curvature(wc, 0.3), cv.normal(wc, 0.3), cv.osculating(wc, 0.3)],
  [tsAnalysis.curvature(tc, 0.3), tsAnalysis.normal(tc, 0.3), tsAnalysis.osculating(tc, 0.3)])
same('mold (path)', own(cv.mold(wc, 0.4, [50, 90])).segs, tsAnalysis.mold(tc, 0.4, [50, 90]).segs)
same('side, turns', [(['left', 'right', 'top', 'bottom'] as const).map(k => cv.side(w, k)), cv.turns(w), cv.turns(w, 80)],
  [(['left', 'right', 'top', 'bottom'] as const).map(k => tsAnalysis.side(t, k)), tsAnalysis.turns(t), tsAnalysis.turns(t, 80)])
same('nearness', [wave(cv.nearness(w, [0.2, 0.6], 0.1)), wave(cv.nearness(w, [0.2], 0.3, 'linear'))],
  [wave(tsAnalysis.nearness(t, [0.2, 0.6], 0.1)), wave(tsAnalysis.nearness(t, [0.2], 0.3, 'linear'))])
const sa = own(cv.selectAt(w, 0.3, 0.1)), sb = own(cv.selectAt(w, 0.3, 0.2, { falloff: 'sharp', along: 'plane', cut: false }))
same('selectAt', [own(sa.shift(10, 90).path).segs, own(sb.shift(10, 90).path).segs],
  [tsAnalysis.selectAt(t, 0.3, 0.1).shift(10, 90).path.segs, tsAnalysis.selectAt(t, 0.3, 0.2, { falloff: 'sharp', along: 'plane', cut: false }).shift(10, 90).path.segs])
same('offset (path), outline (path)', [own(cv.offset(wc, 5)).segs, own(cv.offset(wc, (u: number) => 2 + u * 4)).segs, cv.outline(wc, 4), cv.outline(wc, 4, 8)],
  [tsAnalysis.offset(tc, 5).segs, tsAnalysis.offset(tc, u => 2 + u * 4).segs, tsAnalysis.outline(tc, 4), tsAnalysis.outline(tc, 4, 8)])
same('lineCrossings, circleCrossings, selfCrossings (path)', [cv.lineCrossings(wc, [0, 30], [200, 30]), cv.lineCrossings(wc, [0, 30], [10, 30], { segment: false }),
  cv.circleCrossings(wc, [60, 40], 30), cv.selfCrossings(wc)],
  [tsAnalysis.lineCrossings(tc, [0, 30], [200, 30]), tsAnalysis.lineCrossings(tc, [0, 30], [10, 30], { segment: false }),
   tsAnalysis.circleCrossings(tc, [60, 40], 30), tsAnalysis.selfCrossings(tc)])
same('arcs', [cv.arcs(c), cv.arcs(c, 0.1), cv.arcs(wc), cv.arcs(wc, 2)], [tsOffset.arcs(c), tsOffset.arcs(c, 0.1), tsAnalysis.arcs(tc), tsAnalysis.arcs(tc, 2)])

same('simple, reduce, scale', [cv.simple(c), cv.reduce(c), cv.scale(c, 5), cv.scale(c, (u: number) => 2 + 3 * u)],
  [tsOffset.simple(c), tsOffset.reduce(c), tsOffset.scale(c, 5), tsOffset.scale(c, u => 2 + 3 * u)])
same('offset, offsetAt (cubic)', [cv.offset(c, 5), cv.offsetAt(c, 0.3, 5)], [tsOffset.offset(c, 5), tsOffset.offsetAt(c, 0.3, 5)])
same('outline (cubic)', [cv.outline(c, 4), cv.outline(c, 4, 6), cv.outline(c, 4, 6, 2), cv.outline(c, 4, undefined, 2, 3)],
  [tsOffset.outline(c, 4), tsOffset.outline(c, 4, 6), tsOffset.outline(c, 4, 6, 2), tsOffset.outline(c, 4, undefined, 2, 3)])
same('outlineShapes', [cv.outlineShapes(c, 4), cv.outlineShapes(c, 4, 6, 2, 3)], [tsOffset.outlineShapes(c, 4), tsOffset.outlineShapes(c, 4, 6, 2, 3)])
const loop: tsGeo.Cubic[] = [[[0, 0], [100, 100], [100, 0], [0, 100]]]
same('lineCrossings, circleCrossings, selfCrossings (cubic)', [cv.lineCrossings(c, [0, 30], [100, 30]), cv.lineCrossings(c, [0, 30], [10, 30], { segment: false }),
  cv.circleCrossings(c, [50, 0], 50), cv.circleCrossings(c, [50, 0], 50, 16), cv.selfCrossings(loop), cv.selfCrossings(t.segs, true)],
  [tsIntersect.lineCrossings(c, [0, 30], [100, 30]), tsIntersect.lineCrossings(c, [0, 30], [10, 30], { segment: false }),
   tsIntersect.circleCrossings(c, [50, 0], 50), tsIntersect.circleCrossings(c, [50, 0], 50, 16), tsIntersect.selfCrossings(loop as any), tsIntersect.selfCrossings(t.segs, true)])

const q5: [number, number][] = [[0, 0], [20, 60], [50, 80], [80, 60], [100, 0]]
same('bezierAt, casteljauN, splitN, derivativeN', [cv.bezierAt(q5, 0.3), cv.casteljauN(q5, 0.3), cv.splitN(q5, 0.3), cv.derivativeN(q5)],
  [tsBezn.bezierAt(q5, 0.3), tsBezn.casteljauN(q5, 0.3), tsBezn.splitN(q5, 0.3), tsBezn.derivativeN(q5)])
same('lower, fitBezier', [cv.lower(q5), cv.fitBezier(sample), cv.fitBezier(sample, 4)], [tsBezn.lower(q5), tsBezn.fitBezier(sample), tsBezn.fitBezier(sample, 4)])
same('rationalAt, toCubics', [cv.rationalAt(q5, [1, 2, 1, 2, 1], 0.4), cv.toCubics(q5), cv.toCubics(q5, { weights: [1, 2, 1, 2, 1], tol: 0.1, samples: 16 })],
  [tsBezn.rationalAt(q5, [1, 2, 1, 2, 1], 0.4), tsBezn.toCubics(q5), tsBezn.toCubics(q5, { weights: [1, 2, 1, 2, 1], tol: 0.1, samples: 16 })])
same('bernstein, solve', [cv.bernstein(4, 0.3), cv.solve([[2, 1], [1, 3]], [3, 5])], [tsBezn.bernstein(4, 0.3), tsBezn.solve([[2, 1], [1, 3]], [3, 5])])

const x1: tsGeo.Cubic = [[0, 0], [30, 100], [70, 100], [100, 0]], x2: tsGeo.Cubic = [[0, 50], [30, -20], [70, 120], [100, 50]]
same('crossings, curveCrossings', [cv.crossings(x1, x2), cv.crossings(x1, x2, { tol: 0.1 }), cv.curveCrossings(x1, x2), cv.crossings(x1, x1, { shared: [[0, 0]] })],
  [tsBoolean.crossings(x1 as any, x2 as any), tsBoolean.crossings(x1 as any, x2 as any, { tol: 0.1 }), tsIntersect.curveCrossings(x1 as any, x2 as any), tsBoolean.crossings(x1 as any, x1 as any, { shared: [[0, 0]] })])
const scs: tsGeo.Contour[] = [sq.segs as unknown as tsGeo.Contour]
same('winding', [cv.winding(wcs, [50, 40]), cv.winding(wcs, [500, 40])], [tsBoolean.winding(cs, [50, 40]), tsBoolean.winding(cs, [500, 40])])
const both = (w: number[]) => w[0] !== 0 && w[1] !== 0
same('boolean, tryBoolean', [cv.boolean([wcs, scs], both), cv.boolean([wcs, scs], both, { eps: 0.5 }), cv.tryBoolean([wcs, scs], both), cv.tryBoolean([wcs, scs], both, { eps: 0.5 })],
  [tsBoolean.boolean([cs, scs], both), tsBoolean.boolean([cs, scs], both, { eps: 0.5 }), tsBoolean.tryBoolean([cs, scs], both), tsBoolean.tryBoolean([cs, scs], both, { eps: 0.5 })])
same('traceBoolean, union', [cv.traceBoolean([wcs, scs], both), cv.union([cs[0], scs[0]])], [tsBoolean.traceBoolean([cs, scs], both), tsBoolean.union([cs[0]!, scs[0]!])])
same('reverseContour', cv.reverseContour(cs[0]), tsBoolean.reverseContour(cs[0]!))
const neck: [number, number][] = [[0, 0], [100, 0], [100, 40], [55, 42], [100, 44], [100, 80], [0, 80], [0, 44], [45, 42], [0, 40]]
same('narrowest, polyGap', [cv.narrowest(neck, 3), cv.narrowest(pts, 1000), cv.polyGap(pts, [[150, 0], [200, 50]])],
  [tsBoolean.narrowest(neck, 3), tsBoolean.narrowest(pts, 1000), tsBoolean.polyGap(pts, [[150, 0], [200, 50]])])

same('corners', [cv.corners(wcs), cv.corners(wcs, 80)], [tsCorners.corners(cs), tsCorners.corners(cs, 80)])
const mk = (a: any, ta: any, b: any, tb: any): tsGeo.Piece[] => [[a, tsGeo.add(a, tsGeo.mul(ta, 3)), tsGeo.sub(b, tsGeo.mul(tb, 3)), b]]
same('replaceCorners', [cv.replaceCorners(wcs, () => 6, mk), cv.replaceCorners(wcs, (k: any) => k.at * 2 + 3, mk, { minDeg: 30, share: 0.3 })],
  [tsCorners.replaceCorners(cs, () => 6, mk as any), tsCorners.replaceCorners(cs, k => k.at * 2 + 3, mk as any, { minDeg: 30, share: 0.3 })])

same('segmentOf, shiftNodes', [cv.segmentOf(ns[0], ns[1]), cv.shiftNodes(ns, [5, 5]), cv.shiftNodes(ns, [5, 5], (i: number) => i > 0)],
  [tsNodes.segmentOf(ns[0]!, ns[1]!), tsNodes.shiftNodes(ns, [5, 5]), tsNodes.shiftNodes(ns, [5, 5], i => i > 0)])

same('falloffs', Object.fromEntries(Object.entries(cv.falloffs).map(([k, f]: any) => [k, wave(f)])),
  Object.fromEntries(Object.entries(tsSelection.falloffs).map(([k, f]) => [k, wave(f)])))
const segs = tc.segs
same('moveNodes', cv.moveNodes(segs, false, [1], (p: any, i: number) => [p[0] + i, p[1] + 10]), tsSelection.moveNodes(segs, false, [1], (p, i) => [p[0] + i, p[1] + 10]))
same('smoothNodes', [cv.smoothNodes(segs, false, [1]), cv.smoothNodes(segs, false, [1], 0.5), cv.smoothNodes(segs, false, [1], (i: number) => i / 2, [1, 2])],
  [tsSelection.smoothNodes(segs, false, [1]), tsSelection.smoothNodes(segs, false, [1], 0.5), tsSelection.smoothNodes(segs, false, [1], i => i / 2, [1, 2])])
same('inflation', cv.inflation([10, 0], [0, 0], 20), tsSelection.inflation([10, 0], [0, 0], 20))
same('fairNodes', [cv.fairNodes(t.segs, true, [1, 2]), cv.fairNodes(t.segs, true, [1, 2], 0.3), cv.fairNodes(t.segs, true, [1, 2], 0.3, [1, 2]), cv.fairNodes(t.segs, true, [1, 2], 0.3, [1, 1], [1, 2, 2, 1])],
  [tsSelection.fairNodes(t.segs, true, [1, 2]), tsSelection.fairNodes(t.segs, true, [1, 2], 0.3), tsSelection.fairNodes(t.segs, true, [1, 2], 0.3, [1, 2]), tsSelection.fairNodes(t.segs, true, [1, 2], 0.3, [1, 1], [1, 2, 2, 1])])

const sw = { cycles: 2, phase: 0.1, low: -1, high: 3 }
same('constant, gradient, steps', [wave(cv.constant(4)), wave(cv.gradient([1, 3, 2])), wave(cv.steps([1, 3, 2]))],
  [wave(tsProfile.constant(4)), wave(tsProfile.gradient([1, 3, 2])), wave(tsProfile.steps([1, 3, 2]))])
for (const k of ['sine', 'cosine', 'triangle', 'saw', 'square'] as const)
  same(k, [wave(cv[k]()), wave(cv[k](sw))], [wave(tsProfile[k]()), wave(tsProfile[k](sw))])
same('square duty', wave(cv.square({ duty: 0.3, cycles: 2 })), wave(tsProfile.square({ duty: 0.3, cycles: 2 })))
same('wave', [wave(cv.wave(2)), wave(cv.wave([1, 2])), wave(cv.wave((u: number) => u * u))], [wave(tsProfile.wave(2)), wave(tsProfile.wave([1, 2])), wave(tsProfile.wave(u => u * u))])
same('times, plus', [wave(cv.times()), wave(cv.times(2, [1, 3], (u: number) => u, 0.5)), wave(cv.plus()), wave(cv.plus(2, [1, 3], (u: number) => u))],
  [wave(tsProfile.times()), wave(tsProfile.times(2, [1, 3], u => u, 0.5)), wave(tsProfile.plus()), wave(tsProfile.plus(2, [1, 3], u => u))])
same('bulge', [cv.bulge(1.4), cv.bulge(1.4, 0.6)], [tsProfile.bulge(1.4), tsProfile.bulge(1.4, 0.6)])

same('bowed, fitBulges', [cv.bowed([0, 0], [100, 0]), cv.bowed([0, 0], [100, 0], 0.3), cv.fitBulges(pts, [0.2, 0.1, 0.3])],
  [tsShape.bowed([0, 0], [100, 0]), tsShape.bowed([0, 0], [100, 0], 0.3), tsShape.fitBulges(pts, [0.2, 0.1, 0.3])])
same('softPolyline', [cv.softPolyline(pts), cv.softPolyline(pts, { radius: 0.2, bulge: 0.1 }), cv.softPolyline(pts, { bulge: [0, 0.2, 0.1] })],
  [tsShape.softPolyline(pts), tsShape.softPolyline(pts, { radius: 0.2, bulge: 0.1 }), tsShape.softPolyline(pts, { bulge: [0, 0.2, 0.1] })])

const sp: [number, number][] = [[0, 0], [30, 50], [70, 40], [100, 90], [130, 20]]
same('catmullRom', [cv.catmullRom(sp), cv.catmullRom(sp, { alpha: 0, tension: 0.5, closed: true, metric: [1, 2] })],
  [tsSpline.catmullRom(sp), tsSpline.catmullRom(sp, { alpha: 0, tension: 0.5, closed: true, metric: [1, 2] })])
same('catmullRomPath', [own(cv.catmullRomPath(sp)).segs, own(cv.catmullRomPath(sp, { closed: true })).segs],
  [tsSpline.catmullRomPath(sp).segs, tsSpline.catmullRomPath(sp, { closed: true }).segs])
same('deBoor', [cv.deBoor(sp, 0.4), cv.deBoor(sp, 0.4, { degree: 2, weights: [1, 2, 1, 2, 1] })], [tsSpline.deBoor(sp, 0.4), tsSpline.deBoor(sp, 0.4, { degree: 2, weights: [1, 2, 1, 2, 1] })])
same('bspline', [cv.bspline(sp), cv.bspline(sp, { closed: true, tol: 0.1 })], [tsSpline.bspline(sp), tsSpline.bspline(sp, { closed: true, tol: 0.1 })])
same('bsplinePath', [own(cv.bsplinePath(sp)).segs, own(cv.bsplinePath(sp, { metric: [1, 2] })).segs], [tsSpline.bsplinePath(sp).segs, tsSpline.bsplinePath(sp, { metric: [1, 2] }).segs])

const al = cv.alike(w, wsq), tal = tsMorph.alike(t, sq)
same('alike', [al.from.segs, al.to.segs, al.matched], [tal.from.segs, tal.to.segs, tal.matched])
al.from.delete(); al.to.delete()
same('morph, between', [own(cv.morph(w, wsq, 0.3)).segs, own(cv.morph(w, wsq, 0.3, { eps: 1e-3 })).segs, own(cv.between(w, wsq, 0.6)).segs, own(cv.between(w, wsq, 0.6, 1e-3)).segs],
  [tsMorph.morph(t, sq, 0.3).segs, tsMorph.morph(t, sq, 0.3, { eps: 1e-3 }).segs, tsMorph.between(t, sq, 0.6).segs, tsMorph.between(t, sq, 0.6, 1e-3).segs])

same('jacobian', [cv.jacobian(tm, [30, 40]), cv.jacobian(tm, [30, 40], 0.1)], [tsWarp.jacobian(tm, [30, 40]), tsWarp.jacobian(tm, [30, 40], 0.1)])
same('warpCubic, warpCubics', [cv.warpCubic(c, tm), cv.warpCubic(c, tm, { pieces: 3 }), cv.warpCubics(t.segs, tm), cv.warpCubics(t.segs, tm, { step: 10, eps: 0.01 })],
  [tsWarp.warpCubic(c, tm), tsWarp.warpCubic(c, tm, { pieces: 3 }), tsWarp.warpCubics(t.segs, tm), tsWarp.warpCubics(t.segs, tm, { step: 10, eps: 0.01 })])
same('noise2', [cv.noise2(3)(0.3, 1.7), cv.noise2(5)(10.2, -4)], [tsWarp.noise2(3)(0.3, 1.7), tsWarp.noise2(5)(10.2, -4)])

same('attach', [own(cv.attach(wl, { on: w, at: 0.3 })).segs, own(cv.attach(wl, { on: w, at: 0.3 }, { on: wc, at: 0.6, from: 0.9 })).segs, own(cv.attach(wl, { on: w, at: 0.5, segs: [1, 2] }, undefined)).segs],
  [tsAttach.attach(tl, { on: t, at: 0.3 }).segs, tsAttach.attach(tl, { on: t, at: 0.3 }, { on: tc, at: 0.6, from: 0.9 }).segs, tsAttach.attach(tl, { on: t, at: 0.5, segs: [1, 2] }).segs])
same('target', [cv.target({ point: [3, 4] }), cv.target({ on: w, at: 0.4 }), cv.target({ cross: w, with: wsq })],
  [tsAttach.target({ point: [3, 4] }), tsAttach.target({ on: t, at: 0.4 }), tsAttach.target({ cross: t, with: sq })])

same('expr', cv.expr('a + 1', 3), tsMods.expr('a + 1', 3))
for (const m of tsMods.mods) {
  const tm = tsMods.mod(m.name)
  same(`mod ${m.name}`, cv.mod(m.name), { name: tm.name, group: tm.group, doc: tm.doc, params: tm.params })
}

for (const o of owned) o.delete()
w.delete(); wc.delete(); ws1.delete(); ws2.delete()
console.log(`wasm smoke: ${checks} checks passed`)
