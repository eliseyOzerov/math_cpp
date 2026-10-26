// The C++ API for Swift: CurvesCxx (the headers through C++ interop, with swift/CurvesCxx/include/CurvesCxx.h for
// what Swift can't reach in them), re-exported, plus what Swift's import drops. A client target needs
// `swiftSettings: [.interoperabilityMode(.Cxx)]`.
//
// Swift imports a C++ default argument only when it is a primitive, a Pt or taken by value. The ones taken by `const&`
// (option structs, Pivot, FalloffSpec, optionals) are dropped, so each such function gets an overload here without
// them. Variants get inits for each alternative, closures included.
@_exported import CurvesCxx

// MARK: Variants

extension curves.Strength {
  public init(_ v: Double) { self = curves.cxx.strength(v) }
  public init(_ vs: [Double]) { self = curves.cxx.strength(curves.DoubleList(vs)) }
  public init(_ wave: @escaping (Double) -> Double) { self = curves.cxx.strength(curves.Wave(wave)) }
}

extension curves.Reach {
  public init(_ v: Double) { self = curves.cxx.reach(v) }
  public init(_ f: @escaping (Double) -> Double) { self = curves.cxx.reach(curves.Wave(f)) }
}

extension curves.NodePick {
  public init(_ nodes: [Int32]) { self = curves.cxx.nodePick(curves.IntList(nodes)) }
  /// A weight for each node: its point, index and progress along the path.
  public init(_ weight: @escaping (curves.Pt, Int32, Double) -> Double) {
    self = curves.cxx.nodePick(curves.NodeWeight(weight))
  }
}

extension curves.FalloffSpec {
  public init(_ name: curves.FalloffName) { self = curves.cxx.falloffSpec(name) }
  public init(_ f: @escaping (Double) -> Double) { self = curves.cxx.falloffSpec(curves.Falloff(f)) }
}

extension curves.KeysInput {
  public init(_ keys: curves.KeyList) { self = curves.cxx.keysInput(keys) }
  public init(_ v: Double) { self = curves.cxx.keysInput(v) }
  public init(_ vs: [Double]) { self = curves.cxx.keysInput(curves.DoubleList(vs)) }
  public init(_ wave: @escaping (Double) -> Double) { self = curves.cxx.keysInput(curves.Wave(wave)) }
}

extension curves.Which {
  public init(_ end: curves.End) { self = curves.cxx.which(end) }
  public init(_ i: Int32) { self = curves.cxx.which(i) }
}

extension curves.ParamValue {
  public init(_ v: Double) { self = curves.cxx.paramValue(v) }
  public init(_ s: String) { self = curves.cxx.paramValue(std.string(s)) }
  public var number: Double? { curves.cxx.isNumber(self) ? curves.cxx.number(self) : nil }
  public var text: String? { curves.cxx.isNumber(self) ? nil : String(curves.cxx.text(self)) }
}

extension curves.Target {
  public init(_ t: curves.TargetOn) { self = curves.cxx.target(t) }
  public init(_ t: curves.TargetPoint) { self = curves.cxx.target(t) }
  public init(_ t: curves.TargetCross) { self = curves.cxx.target(t) }
}

extension curves.Operand {
  public init(_ contours: curves.ContourList) { self = curves.cxx.operand(contours) }
  public init(_ path: curves.Path) { self = curves.cxx.operand(path) }
  public init(_ b: curves.Bool) { self = curves.cxx.operand(b) }
}

extension curves.Values {
  /// Values by parameter name: numbers or strings.
  public init(_ values: [String: Any]) {
    self.init()
    for (name, v) in values {
      if let x = v as? Double { curves.cxx.set(&self, std.string(name), x) }
      else if let x = v as? Int { curves.cxx.set(&self, std.string(name), Double(x)) }
      else if let s = v as? String { curves.cxx.set(&self, std.string(name), std.string(s)) }
    }
  }
}

extension curves.Cubic {
  public init(_ a: curves.Pt, _ b: curves.Pt, _ c: curves.Pt, _ d: curves.Pt) {
    self.init()
    self[0] = a; self[1] = b; self[2] = c; self[3] = d
  }
}

// MARK: Callbacks

extension curves.Selection {
  /// Moves each picked node by the warp `f` makes for it (its point, the path, its index).
  public func moveEach(_ f: @escaping (curves.Pt, curves.Path, Int32) -> curves.Warp) -> curves.Selection {
    curves.cxx.moveEach(self, curves.NodeWarp(f))
  }
}

// MARK: Default arguments

extension curves {
  public static func nearness(_ path: curves.Path, _ ats: curves.DoubleList, _ reach: Double) -> curves.Wave {
    curves.nearness(path, ats, reach, curves.FalloffSpec(.smooth))
  }
  public static func selectAt(_ path: curves.Path, _ at: Double, _ reach: Double) -> curves.Selection {
    curves.selectAt(path, at, reach, curves.SelectAtOptions())
  }
  public static func attach(_ p: curves.Path, _ a: curves.Mount) -> curves.Path {
    curves.attach(p, a, curves.OptMount())
  }
  public static func place(_ p: curves.Path, _ pairs: curves.PairList) -> curves.Path {
    curves.place(p, pairs, curves.OptAlign())
  }
  public static func cubicThrough(_ s: curves.Pt, _ b: curves.Pt, _ e: curves.Pt) -> curves.Cubic {
    curves.cubicThrough(s, b, e, curves.ThroughOptions())
  }
  public static func toCubics(_ pts: curves.PtList) -> curves.CubicList {
    curves.toCubics(pts, curves.ToCubicsOptions())
  }
  public static func applyMod(_ s: curves.Selection, _ m: curves.Mod) -> curves.Selection {
    curves.applyMod(s, m, curves.Values())
  }
  public static func modCode(_ m: curves.Mod) -> std.string { curves.modCode(m, curves.Values()) }
  public static func shiftNodes(_ nodes: curves.NodeList, _ d: curves.Pt) -> curves.NodeList {
    curves.shiftNodes(nodes, d, curves.IntTest())
  }
  public static func shiftNodes(_ nodes: curves.NodeList, _ d: curves.Pt, _ pick: @escaping (Int32) -> Swift.Bool)
    -> curves.NodeList
  {
    curves.shiftNodes(nodes, d, curves.IntTest(pick))
  }
  public static func fairNodes(
    _ segs: curves.CubicList, _ closed: Swift.Bool, _ nodes: curves.IntList, _ reach: Double = 0.5,
    _ metric: curves.Pt = curves.Pt(x: 1, y: 1)
  ) -> curves.Faired {
    curves.fairNodes(segs, closed, nodes, reach, metric, curves.OptDoubles())
  }
  public static func softPolyline(_ pts: curves.PtList) -> curves.CubicList {
    curves.softPolyline(pts, curves.Softness())
  }
  public static func soft(_ pts: curves.PtList) -> curves.Path { curves.soft(pts, curves.Softness()) }
  public static func oval(_ x0: Double, _ y0: Double, _ x1: Double, _ y1: Double) -> curves.Path {
    curves.oval(x0, y0, x1, y1, curves.OvalOptions())
  }
  public static func lobe(_ a: curves.Pt, _ b: curves.Pt) -> curves.Path { curves.lobe(a, b, curves.LobeOptions()) }
  public static func catmullRom(_ pts: curves.PtList) -> curves.CubicList {
    curves.catmullRom(pts, curves.CatmullRomOptions())
  }
  public static func catmullRomPath(_ pts: curves.PtList) -> curves.Path {
    curves.catmullRomPath(pts, curves.CatmullRomOptions())
  }
  public static func deBoor(_ pts: curves.PtList, _ u: Double) -> curves.Pt {
    curves.deBoor(pts, u, curves.BSplineOptions())
  }
  public static func bspline(_ pts: curves.PtList) -> curves.CubicList { curves.bspline(pts, curves.BSplineOptions()) }
  public static func bsplinePath(_ pts: curves.PtList) -> curves.Path {
    curves.bsplinePath(pts, curves.BSplineOptions())
  }
  public static func warpCubic(_ c: curves.Cubic, _ f: @escaping (curves.Pt) -> curves.Pt) -> curves.CubicList {
    curves.warpCubic(c, curves.Warp(f), curves.WarpOptions())
  }
  public static func warpCubics(_ cs: curves.CubicList, _ f: @escaping (curves.Pt) -> curves.Pt) -> curves.CubicList {
    curves.warpCubics(cs, curves.Warp(f), curves.WarpOptions())
  }
  public static func warpContours(_ cs: curves.ContourList, _ f: @escaping (curves.Pt) -> curves.Pt)
    -> curves.ContourList
  {
    curves.warpContours(cs, curves.Warp(f), curves.WarpOptions())
  }
  /// Contours with each corner rounded by its own radius.
  public static func roundCorners(_ cs: curves.ContourList, _ radius: @escaping (curves.Corner) -> Double)
    -> curves.ContourList
  {
    curves.roundCorners(cs, curves.CornerSize(radius), curves.CornerOpts())
  }
  /// The boolean of contour groups: inside where `inside` holds for the groups' windings.
  public static func boolean(
    _ groups: curves.ContourGroups, _ inside: @escaping (curves.IntList) -> Swift.Bool, _ eps: Double = 0.25
  ) -> curves.ContourList {
    curves.boolean(groups, curves.WindingRule(inside), eps)
  }
}

extension curves.Path {
  public func quarter(_ to: curves.Pt) -> curves.Path { quarter(to, curves.QuarterOptions()) }
  public func nearness(_ ats: curves.DoubleList, _ reach: Double) -> curves.Wave {
    nearness(ats, reach, curves.FalloffSpec(.smooth))
  }
  public func selectAt(_ at: Double, _ reach: Double) -> curves.Selection {
    selectAt(at, reach, curves.SelectAtOptions())
  }
  public func warp(_ f: @escaping (curves.Pt) -> curves.Pt) -> curves.Path { warp(curves.Warp(f), curves.WarpOptions()) }
}

extension curves.Bool {
  public func corners() -> curves.BoolCornerList {
    corners(curves.OptInt(), curves.OptInt(), curves.BoolCornersOptions())
  }
}

extension curves.Selection {
  public func soften(_ reach: Double) -> curves.Selection { soften(reach, curves.SoftenOptions()) }
  public func inflate(_ r: Double) -> curves.Selection { inflate(r, curves.InflateOptions()) }
  public func rotate(_ deg: Double) -> curves.Selection { rotate(deg, curves.Pivot()) }
  public func shear(_ deg: Double) -> curves.Selection { shear(deg, curves.ShearOptions()) }
  public func scale(_ k: Double) -> curves.Selection { scale(k, curves.Pivot()) }
  public func scale(_ k: curves.Pt) -> curves.Selection { scale(k, curves.Pivot()) }
  public func mirror(_ axis: Double = 90) -> curves.Selection { mirror(axis, curves.Pivot()) }
}
