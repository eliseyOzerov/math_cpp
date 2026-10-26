// Types for the emscripten/embind module (build-wasm/curves.js), written by hand to match src/wasm.cpp. Copy to
// build-wasm/curves.d.ts, where TS looks for them. Everything crosses as plain JS values (points [x, y], cubics four
// points, contours lists of pieces); Path, Selection, Bool and Pick are handles that are freed with `.delete()`.

export type Pt = [number, number]
export type Cubic = [Pt, Pt, Pt, Pt]
export type Line = [Pt, Pt]
/** Two points for a line, four for a cubic. */
export type Piece = Line | Cubic
export type Contour = readonly Piece[]
/** [low, high] corners. */
export type Box = [Pt, Pt]
/** [a, b, c, d, e, f], as SVG's matrix. */
export type Affine = readonly [number, number, number, number, number, number]
/** A share of a path's length, 0…1. */
export type Progress = number
export type Wave = (x: number) => number
export type Warp = (p: Pt) => Pt

export type NodeKind = 'corner' | 'smooth' | 'symmetric'
export type HandleSide = 'in' | 'out'
export type FalloffName = 'smooth' | 'linear' | 'sphere' | 'root' | 'sharp' | 'constant'
export type Falloff = FalloffName | Wave
/** A number is a segment index; a name or a number picks the end for atY and the like. */
export type Which = 'first' | 'last' | number
/** Segments `n`, or `[from, to]`. */
export type Segs = number | [number, number]
export type Way = 'left' | 'right' | 'top' | 'bottom'
export type Pivot = Pt | 'center'
export type Values = Partial<Record<string, number | string>>
/** A wave, a constant or a list of gradient values. */
export type Strength = number | number[] | Wave

export interface Node { p: Pt; in?: Pt; out?: Pt; kind?: NodeKind }
export interface HandleRef { s: number; i: number; side: HandleSide }
export interface HandleLock { handles: (HandleRef & { flip?: boolean })[]; angle?: boolean; length?: boolean }
/** [t, v] or [t, v, handles]. */
export type Key = [number, number] | [number, number, { in?: Pt; out?: Pt }]
export type Keys = Key[]

export interface Corner { contour: number; at: number; p: Pt; in: Pt; out: Pt; turn: number; outside: boolean }
export interface BoolCorner { contour: number; node: number; p: Pt; between: [number, number]; in: Pt; out: Pt; outward: Pt; convex: boolean }
export interface CornerOpts { minDeg?: number; share?: number }
export interface Cap { shape: readonly Piece[]; align?: number }
export interface EndFrame { p: Pt; t: Pt; r: number }
export type CornerShapeSpec = number | { angle: number; profile?: Piece[] }

export interface QuarterOptions { from?: 'h' | 'v'; tension?: number }
export interface SoftenOptions { falloff?: Falloff; along?: 'path' | 'plane' }
export interface HandlesOptions { angle?: number; turn?: number; length?: number; side?: 'in' | 'out' | 'both' }
export interface InflateOptions { align?: Pt; handles?: number; smooth?: number; strength?: number; along?: 'path' | 'ring' }
export interface ShearOptions { axis?: number; about?: Pivot }
export interface Place { x?: number; y?: number }

export interface Located { i: number; t: number }
export interface PathProjection { at: Progress; p: Pt; d: number }
export interface PathCrossing { p: Pt; t: Progress; u: Progress }
export interface Nearest { i: number; t: number; p: Pt; d: number }
export interface CubicProjection { t: number; p: Pt; d: number }
export interface Extrema { x: number[]; y: number[]; values: number[] }
export interface Osculating { center?: Pt; r: number; k: number }
export interface Arc { center?: Pt; r: number; start: number; end: number; ccw: boolean; from: Pt; to: Pt; t0: number; t1: number }
export interface PathArc extends Arc { at0: Progress; at1: Progress }
export interface LineCrossing { t: number; s: number; p: Pt }
export interface PathLineCrossing { at: Progress; s: number; p: Pt }
export interface CircleCrossing { t: number; p: Pt }
export interface PathCircleCrossing { at: Progress; p: Pt }
export interface SelfCrossing { a: { i: number; t: number }; b: { i: number; t: number }; p: Pt }
export interface PathSelfCrossing { a: Progress; b: Progress; p: Pt }
export interface Reduced { c: Cubic; t0: number; t1: number }
export interface Turn { at: Progress; p: Pt; turn: number; tip: Pt }
export interface Gap { d: number; a: Pt; b: Pt }
export interface Canonical { kind: string; at: Pt }
/** `from` and `to` are new Paths, each freed with `.delete()`. */
export interface Alike { from: Path; to: Path; matched: boolean[] }
export interface NodesNearest { seg: number; t: number; p: Pt; d: number }
export interface NodesCrossing { seg: number; t: number; p: Pt }
export interface Faired { segs: Cubic[]; nodes: number[]; weights: number[] }
export interface CubicFit { c: Cubic; err: number }

export interface ModParamNumber { min: number; max: number; step: number; value: number }
export interface ModParamChoice { options: string[]; value: string }
export interface ModInfo { name: string; group: string; doc: string; params: Record<string, ModParamNumber | ModParamChoice> }
export interface Step { mod: string; values?: Values }

export interface Swing { cycles?: number; phase?: number; low?: number; high?: number }
export interface SquareSwing extends Swing { duty?: number }
export interface Softness { radius?: number; bulge?: number | number[] }
export interface OvalOptions { tension?: number; start?: 'right' | 'top' | 'left' | 'bottom' }
export interface LobeOptions { reach?: number; nodes?: number; radius?: number; bulge?: number; axis?: number; reachFrom?: 'start' | 'end' }
export interface WarpOptions { pieces?: number; step?: number; eps?: number }
export interface Wobble { amplitude: number; frequency?: number; octaves?: number; seed?: number }
export interface CatmullRomOptions { alpha?: number; tension?: number; closed?: boolean; metric?: Pt }
export interface BSplineOptions { degree?: number; knots?: number[]; weights?: number[]; closed?: boolean; tol?: number; metric?: Pt }
export interface ToCubicsOptions { weights?: number[]; tol?: number; samples?: number }
export interface ThroughOptions { t?: number; d1?: number; dir?: Pt }
export interface SelectAtOptions { falloff?: Falloff; along?: 'path' | 'plane'; cut?: boolean }
export interface MeetOptions { flat?: number; inside?: number }
export interface FitOptions { corner?: number }

export type Target =
  | { point: Pt }
  | { cross: Path; with: Path; which?: Which }
  | { on: Path; at: Progress; segs?: Segs }
export type Pair = Target & { from?: Progress }
export interface Align { on: Path; at: Progress; segs?: Segs; to?: 'tangent' | 'normal'; angle?: number; from?: Progress }
export interface Mount { on: Path; at: Progress; from?: Progress; segs?: Segs }

/** A pick: node indices, or a function of each node's point, index and progress giving its weight (true is 1). */
export type NodePick = readonly number[] | ((p: Pt, i: number, at: Progress) => number | boolean)
/** A pick from nodeAt, cubicAt or section. */
export declare class Pick { delete(): void }
export type AnyPick = NodePick | Pick

export type Op = 'union' | 'intersect' | 'subtract' | 'exclude' | 'cut'
export type Operand = Path | Bool | Contour[]

/** The binding's closures made by warps and waves, freed with the function; not for direct use. */
export declare class WarpFn { call(p: Pt): Pt; delete(): void }
export declare class WaveFn { call(x: number): number; delete(): void }
export declare class Fn2 { call(x: number, y: number): number; delete(): void }

export declare class Path {
  readonly start: Pt
  readonly segs: Cubic[]
  readonly closed: boolean
  readonly metric: Pt
  readonly kinds: NodeKind[]
  readonly length: number
  readonly end: Pt
  readonly angle: number
  readonly nodeCount: number
  /** The nodes' points; nodesOf(path) gives them as Nodes. */
  readonly nodes: Pt[]
  readonly distances: number[]
  delete(): void

  measured(metric: Pt): Path
  kind(i: number): NodeKind
  withKind(kind: NodeKind, pick?: NodePick): Path
  handle(i: number, side: HandleSide, to: Pt): Path
  locate(at: Progress): Located
  progress(i: number, t: number): Progress
  line(to: Pt): Path
  curve(c1: Pt, c2: Pt, to: Pt): Path
  quarter(to: Pt, o?: QuarterOptions): Path
  arch(to: Pt, peak: number, tension?: number): Path
  join(next: Path): Path
  close(): Path
  at(at: Progress): Pt
  crossY(y: number): Pt[]
  crossX(x: number): Pt[]
  atY(y: number, which?: Which): Pt
  atX(x: number, which?: Which): Pt
  tAtY(y: number, which?: Which): Progress
  tAtX(x: number, which?: Which): Progress
  chordY(y: number): Path
  chordX(x: number): Path
  slice(p0: Progress, p1: Progress): Path
  aperture(offset: number, angle?: number): Path
  divide(n: number): Path
  direction(at: Progress): Pt
  crossings(other: Path): PathCrossing[]
  cut(ats: readonly Progress[]): Path
  node(i: number): Pt
  nodeProgress(i: number): Progress
  reverse(): Path
  startAt(r: number): Path
  select(pick?: AnyPick): Selection
  selectAt(at: Progress, reach: number): Selection
  attach(on: Path, at: Progress): Path
  project(q: Pt): PathProjection
  extrema(): number[]
  inflections(): number[]
  bbox(): Box
  curvature(at: Progress): number
  normal(at: Progress): Pt
  mold(at: Progress, to: Pt): Path
  offset(d: number): Path
  outline(d1: number, d2?: number): Contour[]
}

export declare class Selection {
  /** Each path's weights, by node. */
  readonly weights: number[][]
  readonly points: Pt[]
  readonly center: Pt
  readonly bounds: Box
  readonly paths: Path[]
  readonly path: Path
  delete(): void

  /** One selection (the TS takes several). */
  and(other: Selection): Selection
  soften(reach: number, o?: SoftenOptions): Selection
  weigh(k: number | readonly number[] | Wave): Selection
  invert(): Selection
  put(to: Place): Selection
  kind(kind: NodeKind): Selection
  handles(o: HandlesOptions): Selection
  inflate(r: number, o?: InflateOptions): Selection
  smooth(amount?: number): Selection
  fair(reach?: number): Selection
  rotate(deg: number, about?: Pivot): Selection
  shear(deg: number, o?: ShearOptions): Selection
  shift(d: number, axis?: number): Selection
  scale(k: number | Pt, about?: Pivot): Selection
  mirror(axis?: number, about?: Pivot): Selection
  select(pick?: AnyPick): Selection
}

export declare class Bool {
  /** `op` is a name; `operands` Paths, Bools or contour lists. */
  constructor(op: Op, operands: readonly Operand[], eps?: number)
  readonly op: Op
  readonly eps: number
  readonly ok: boolean
  readonly contours: Contour[]
  /** For each contour, the operands it came from. */
  readonly sources: number[][]
  delete(): void

  /** The operands taken as a list. */
  with(operands: readonly Operand[]): Bool
  corners(a?: number | null, c?: number | null, o?: { same?: boolean }): BoolCorner[]
  outward(contour: number, node: number): Pt
}

export interface CurvesModule {
  // constants
  KAPPA: number
  identity: Affine
  falloffs: Record<FalloffName, Wave>

  // classes
  Path: typeof Path
  Selection: typeof Selection
  Bool: typeof Bool
  Pick: typeof Pick
  WarpFn: typeof WarpFn
  WaveFn: typeof WaveFn
  Fn2: typeof Fn2

  // picks and paths
  nodeAt(indices: readonly number[]): Pick
  cubicAt(indices: readonly number[]): Pick
  section(from: Progress, to: Progress): Pick
  from(p: Pt): Path
  line(a: Pt, b: Pt): Path
  /** The rest of the points as a list, not spread. */
  poly(p: Pt, rest: readonly Pt[]): Path
  pathOf(segs: readonly Cubic[]): Path
  pathOf(segs: readonly Cubic[], closed: boolean): Path
  pathOf(nodes: readonly Node[], closed: boolean, metric?: Pt): Path
  group(of: (Path | Selection)[]): Selection

  // mods
  mods(): ModInfo[]
  mod(name: string): ModInfo | undefined
  modDefaults(name: string): Values
  applyMod(s: Selection, name: string, values?: Values): Selection
  modCode(name: string, values?: Values): string
  chain(s: Selection, steps: readonly Step[]): Selection
  chainCode(steps: readonly Step[]): string
  expr(code: string, value: unknown): { expr: string; value: unknown }

  // svg
  parseSvgPath(d: string): Contour[]
  svgPath(cs: readonly Contour[], digits?: number): string

  // geometry
  add(a: Pt, b: Pt): Pt
  sub(a: Pt, b: Pt): Pt
  mul(a: Pt, k: number): Pt
  neg(v: Pt): Pt
  dot(a: Pt, b: Pt): number
  cross(a: Pt, b: Pt): number
  len(v: Pt): number
  dist(a: Pt, b: Pt): number
  lerp(a: Pt, b: Pt, t: number): Pt
  norm(v: Pt): Pt
  rad(deg: number): number
  deg(rad: number): number
  clamp(v: number, a: number, b: number): number
  halfTurn(a: number): number
  turn(a: Pt, b: Pt): number
  rotate(v: Pt, t: number): Pt
  fromAngle(a: number): Pt
  mean(ps: readonly Pt[]): Pt
  bez(c: Cubic, t: number): Pt
  bezD(c: Cubic, t: number): Pt
  bezDD(c: Cubic, t: number): Pt
  split(c: Cubic, t: number): [Cubic, Cubic]
  nearSegment(p: Pt, a: Pt, b: Pt, tol: number, over?: number): boolean
  isLine(c: Cubic): boolean
  lineCubic(a: Pt, b: Pt): Cubic
  nearest(segs: readonly Cubic[], p: Pt, steps?: number): Nearest
  arcLength(c: Cubic, t?: number): number
  box(pieces: readonly Piece[], samples?: number): Box
  arcCubics(a: Pt, b: Pt, alpha: number): Cubic[]
  apply(m: Affine, p: Pt): Pt
  rotation(deg: number, about?: Pt): Affine
  scaling(kx: number, ky?: number, about?: Pt): Affine
  shift(d: number, axis?: number): Affine
  shear(d: number, axis?: number, about?: Pt): Affine
  /** `then` in the TS. */
  then_(m: Affine, n: Affine): Affine
  inMetric(m: Affine, metric: Pt): Affine
  partly(m: Affine, w: number): Affine
  meet(p: Pt, d: Pt, q: Pt, e: Pt, o?: MeetOptions): Pt | undefined
  asCubic(q: Piece): Cubic
  /** A piece reversed (the Path's reverse is a method). */
  reverse(q: Piece): Piece
  endTangents(q: Piece): [Pt, Pt]
  trim(c: Cubic, t0: number, t1: number): Cubic
  roots(c: Cubic, coord: number, v: number, steps?: number): number[]
  tAtLength(c: Cubic, s: number, total?: number | null): number
  pieceLength(q: Piece): number
  /** Returns the stitched contour instead of mutating. */
  stitch(c: Contour): Contour
  arcCubic(a: Pt, ta: Pt, b: Pt, tb: Pt): Cubic
  cutAt(c: Cubic, shares: readonly number[], metric?: Pt): Cubic[]
  splitEven(c: Cubic, n: number): Cubic[]
  runLengths(ps: readonly Pt[]): number[]
  pointAlong(ps: readonly Pt[], run: readonly number[], s: number): Pt
  hullLength(c: Cubic): number
  area(ps: readonly Pt[]): number
  bounds(ps: readonly Pt[]): Box
  overlaps(a: Box, b: Box, e?: number): boolean
  center(ps: readonly Pt[]): Pt
  within(b: Box, at: Pt): Pt
  toMetric(p: Pt, m: Pt): Pt
  fromMetric(p: Pt, m: Pt): Pt

  // bezier, and analysis: one name for a cubic and a path
  cubicRoots(a: number, b: number, c: number, d: number): number[]
  inUnit(ts: readonly number[], eps?: number): number[]
  coordRoots(p0: number, p1: number, p2: number, p3: number, v?: number): number[]
  cubicCoordRoots(c: Cubic, coord: number, v: number): number[]
  align(c: Cubic, a?: Pt, b?: Pt): Cubic
  tightBox(c: Cubic): Pt[]
  canonical(c: Cubic, eps?: number): Canonical
  casteljau(c: Cubic, t: number): Pt[]
  lut(c: Cubic, steps?: number): Pt[]
  tangent(c: Cubic, t: number): Pt
  quadThrough(s: Pt, b: Pt, e: Pt, t?: number | null): Cubic
  cubicThrough(s: Pt, b: Pt, e: Pt, o?: ThroughOptions): Cubic
  project(x: Path, p: Pt): PathProjection
  project(x: Cubic, p: Pt): CubicProjection
  extrema(x: Path): number[]
  extrema(x: Cubic): Extrema
  inflections(x: Path | Cubic): number[]
  bbox(x: Path | Cubic): Box
  curvature(x: Path | Cubic, t: number): number
  normal(x: Path | Cubic, t: number): Pt
  osculating(x: Path | Cubic, t: number): Osculating
  mold(x: Path, t: Progress, to: Pt): Path
  mold(x: Cubic, t: number, to: Pt): Cubic

  // analysis
  side(p: Path, way: Way): Progress
  turns(p: Path, minDeg?: number): Turn[]
  nearness(p: Path, ats: readonly Progress[], reach: number, falloff?: Falloff): Wave
  selectAt(p: Path, at: Progress, reach: number, o?: SelectAtOptions): Selection

  // offset
  simple(c: Cubic): boolean
  reduce(c: Cubic): Reduced[]
  scale(c: Cubic, d: number | Wave): Cubic
  offset(x: Path, d: number | Wave): Path
  offset(x: Cubic, d: number | Wave): Cubic[]
  offsetAt(c: Cubic, t: number, d: number): Pt
  outline(x: Path, d1: number, d2?: number): Contour[]
  outline(x: Cubic, d1: number, d2?: number | null, d3?: number | null, d4?: number | null): Piece[]
  outlineShapes(c: Cubic, d1: number, d2?: number | null, d3?: number | null, d4?: number | null): Contour[]
  arcs(x: Path, threshold?: number): PathArc[]
  arcs(x: Cubic, threshold?: number): Arc[]

  // intersect
  lineCrossings(x: Path, a: Pt, b: Pt, o?: { segment?: boolean }): PathLineCrossing[]
  lineCrossings(x: Cubic, a: Pt, b: Pt, o?: { segment?: boolean }): LineCrossing[]
  circleCrossings(x: Path, o: Pt, r: number): PathCircleCrossing[]
  circleCrossings(x: Cubic, o: Pt, r: number, samples?: number): CircleCrossing[]
  selfCrossings(x: Path): PathSelfCrossing[]
  selfCrossings(segs: readonly Cubic[], closed?: boolean): SelfCrossing[]
  crossings(a: Cubic, b: Cubic, o?: { tol?: number; shared?: [number, number][] }): [number, number][] | null
  curveCrossings(a: Cubic, b: Cubic, o?: { tol?: number; shared?: [number, number][] }): [number, number][] | null

  // bezier of any order
  raise(ps: readonly Pt[]): Pt[]
  bezierAt(ps: readonly Pt[], t: number): Pt
  casteljauN(ps: readonly Pt[], t: number): Pt[][]
  splitN(ps: readonly Pt[], t: number): [Pt[], Pt[]]
  derivativeN(ps: readonly Pt[]): Pt[]
  lower(ps: readonly Pt[]): Pt[]
  fitBezier(ps: readonly Pt[], order?: number): Pt[]
  rationalAt(ps: readonly Pt[], ws: readonly number[], t: number): Pt
  toCubics(ps: readonly Pt[], o?: ToCubicsOptions): Cubic[]
  bernstein(n: number, t: number): number[]
  solve(a: readonly (readonly number[])[], b: readonly number[]): number[]

  // fit
  fitCubic(ps: readonly Pt[], ta: Pt, tb: Pt): CubicFit
  fitCubics(ps: readonly Pt[], tol: number, o?: FitOptions): Cubic[]
  cubicToQuads(c: Cubic, tol: number): Pt[]
  hull(ps: readonly Pt[], sorted?: boolean): Pt[]
  mergeHulls(a: readonly Pt[], b: readonly Pt[]): Pt[]
  clipPolygon(poly: readonly Pt[], plane: { at: Pt; n: Pt }): Pt[]

  // boolean
  contourArea(c: Contour): number
  orient(cs: readonly Contour[]): Contour[]
  flatten(cs: readonly Contour[], step?: number, min?: number): Pt[][]
  edges(polys: readonly (readonly Pt[])[]): Line[]
  scanline(es: readonly Line[], y: number): [number, number][]
  winding(cs: readonly Contour[], p: Pt): number
  /** `inside` is given each group's winding numbers and says if the point is inside. */
  boolean(groups: readonly (readonly Contour[])[], inside: (winding: number[]) => boolean, o?: { eps?: number }): Contour[]
  tryBoolean(groups: readonly (readonly Contour[])[], inside: (winding: number[]) => boolean, o?: { eps?: number }): Contour[] | null
  traceBoolean(groups: readonly (readonly Contour[])[], inside: (winding: number[]) => boolean, o?: { eps?: number }): { contours: Contour[]; from: number[][] } | null
  union(cs: readonly Contour[]): Contour[]
  reverseContour(c: Contour): Contour
  narrowest(poly: readonly Pt[], apart: number): Gap | undefined
  polyGap(p: readonly Pt[], q: readonly Pt[]): Gap

  // corners
  corners(cs: readonly Contour[], minDeg?: number): Corner[]
  pits(cs: readonly Contour[], maxDeg?: number): Corner[]
  peaks(cs: readonly Contour[], minDeg?: number): Corner[]
  roundCorners(cs: readonly Contour[], r: number | ((k: Corner) => number), o?: CornerOpts): Contour[]
  shapeCorners(cs: readonly Contour[], size: (k: Corner) => number, shape: (k: Corner) => CornerShapeSpec): Contour[]
  replaceCorners(cs: readonly Contour[], reach: (k: Corner) => number, make: (a: Pt, ta: Pt, b: Pt, tb: Pt, k: Corner) => readonly Piece[], o?: CornerOpts): Contour[]
  capped(cs: readonly Contour[], end: EndFrame, cap: Cap): Contour[]
  capsTriangle(h?: number): Piece[]
  capsRect(h?: number): Piece[]
  capsArc(h?: number): Piece[]

  // nodes
  nodesOf(p: Path): Node[]
  segmentsOf(ns: readonly Node[], closed: boolean): Cubic[]
  segmentOf(a: Node, b: Node): Cubic
  pointOn(ns: readonly Node[], closed: boolean, seg: number, t: number): Pt
  nearestOn(ns: readonly Node[], closed: boolean, p: Pt): NodesNearest
  crossingsOn(ns: readonly Node[], closed: boolean, j: number, v: number): NodesCrossing[]
  moldNodes(ns: readonly Node[], closed: boolean, seg: number, t: number, to: Pt): Node[]
  nodeKind(n: Node): NodeKind
  withHandle(n: Node, side: HandleSide, to: Pt): Node
  withNodeKind(n: Node, kind: NodeKind): Node
  insertNode(ns: readonly Node[], seg: number, t: number): Node[]
  simplified(ns: readonly Node[], closed: boolean, tol: number): Node[]
  shiftNodes(ns: readonly Node[], d: Pt, pick?: (i: number) => boolean): Node[]
  lockOf(locks: readonly HandleLock[], ref: HandleRef): HandleLock | undefined
  withLockedHandle(lists: readonly (readonly Node[])[], locks: readonly HandleLock[], ref: HandleRef, to: Pt): Node[][]
  locked(locks: readonly HandleLock[], refs: readonly HandleRef[], how: { angle?: boolean; length?: boolean }): HandleLock[]
  unlocked(locks: readonly HandleLock[], refs: readonly HandleRef[]): HandleLock[]
  remapLocks(locks: readonly HandleLock[], s: number, to: (i: number) => number | null | undefined): HandleLock[]
  lockedKind(locks: readonly HandleLock[], n: Node, s: number, i: number): NodeKind
  lockedKinds(lists: readonly (readonly Node[])[], locks: readonly HandleLock[]): Node[][]
  kindLocks(lists: readonly (readonly Node[])[]): { lists: Node[][]; locks: HandleLock[] }

  // selection
  moveNodes(segs: readonly Cubic[], closed: boolean, ns: readonly number[], f: (p: Pt, i: number) => Pt): Cubic[]
  smoothNodes(segs: readonly Cubic[], closed: boolean, ns: readonly number[], amount?: number | ((i: number) => number), metric?: Pt): Cubic[]
  inflation(p: Pt, from: Pt, r: number): Pt
  fairNodes(segs: readonly Cubic[], closed: boolean, ns: readonly number[], reach?: number | null, metric?: Pt, weights?: readonly number[]): Faired

  // profile: waves and number keys
  smoothstep(u: number): number
  easeKeys(t0: number, a: number, t1: number, b: number): Keys
  keysAt(keys: Keys | null | undefined, t: number, dflt?: number): number
  /** Number, wave, number list or keys to keys; left out gives back what was passed. */
  keysOf(v: number | Wave | readonly number[] | Keys | null | undefined, samples?: number): Keys | null | undefined
  keysTimes(a: Keys | null | undefined, b: Keys | null | undefined): Keys | null | undefined
  sorted(keys: Keys | null | undefined): Keys | null
  spread(vs: readonly number[]): Keys
  constant(v: number): Wave
  gradient(vs: readonly number[]): Wave
  steps(vs: readonly number[]): Wave
  sine(o?: Swing): Wave
  cosine(o?: Swing): Wave
  square(o?: SquareSwing): Wave
  triangle(o?: Swing): Wave
  saw(o?: Swing): Wave
  wave(s: Strength): Wave
  /** Up to four strengths (the TS takes any number). */
  times(...s: readonly Strength[]): Wave
  /** Up to four strengths (the TS takes any number). */
  plus(...s: readonly Strength[]): Wave
  bulge(mid: number, ends?: number): Keys

  // shape
  soft(ps: readonly Pt[], o?: Softness, metric?: Pt): Path
  oval(x0: number, y0: number, x1: number, y1: number, o?: OvalOptions): Path
  lobe(a: Pt, b: Pt, o?: LobeOptions, metric?: Pt): Path
  bowed(a: Pt, b: Pt, bulge?: number): Cubic
  fitBulges(ps: readonly Pt[], bs: readonly number[]): number[]
  softPolyline(ps: readonly Pt[], o?: Softness): Cubic[]

  // spline
  catmullRom(ps: readonly Pt[], o?: CatmullRomOptions): Cubic[]
  catmullRomPath(ps: readonly Pt[], o?: CatmullRomOptions): Path
  deBoor(ps: readonly Pt[], u: number, o?: BSplineOptions): Pt
  bspline(ps: readonly Pt[], o?: BSplineOptions): Cubic[]
  bsplinePath(ps: readonly Pt[], o?: BSplineOptions): Path

  // morph
  alike(a: Path, b: Path, o?: { eps?: number }): Alike
  morph(a: Path, b: Path, k: number, o?: { eps?: number }): Path
  between(a: Path, b: Path, k: number, eps?: number): Path

  // warp: warps, waves and noise made here come back as JS functions
  meshGrid(lo: Pt, hi: Pt): Pt[]
  mesh(lo: Pt, hi: Pt, grid: readonly Pt[]): Warp
  spine(dx: number | Wave, y0: number, y1: number): Warp
  pinned(f: Warp, ys: readonly number[], reach: number): Warp
  warpContours(cs: readonly Contour[], f: Warp, o?: WarpOptions): Contour[]
  warpNode(n: Node, f: Warp, eps?: number): Node
  warpCubic(c: Cubic, f: Warp, o?: WarpOptions): Cubic[]
  warpCubics(cs: readonly Cubic[], f: Warp, o?: WarpOptions): Cubic[]
  jacobian(f: Warp, p: Pt, eps?: number): [Pt, Pt]
  wavelength(w: Wobble): number
  wobble(w: Wobble): Warp
  wobbleStep(w: Wobble): number
  wobbleWave(w: Wobble, length: number, closed?: boolean): Wave
  noise2(seed: number): (x: number, y: number) => number

  // attach
  place(p: Path, pairs: readonly Pair[], align?: Align | null): Path
  attach(p: Path, a: Mount, b?: Mount): Path
  target(t: Target): Pt

  // the C API (include/curves.h), through the runtime
  _malloc(bytes: number): number
  _free(ptr: number): void
  _cv_bez: Function
  _cv_split: Function
  _cv_arc_length: Function
  _cv_cubic_roots: Function
  _cv_extrema: Function
  _cv_bbox: Function
  _cv_inflections: Function
  _cv_canonical: Function
  _cv_project: Function
  _cv_mold: Function
  _cv_cubic_through: Function
  HEAPF64: Float64Array
  ccall(name: string, returnType: string | null, argTypes: readonly string[], args: readonly unknown[]): unknown
  cwrap(name: string, returnType: string | null, argTypes: readonly string[]): (...args: unknown[]) => unknown
}

export default function createCurves(opts?: object): Promise<CurvesModule>
