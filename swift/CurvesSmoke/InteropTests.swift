// What the Curves overlay adds to the C++ import: dropped defaults, variants, vector overloads of span functions,
// closures as callbacks, mods and keys.
import Curves
import Testing

private func pt(_ x: Double, _ y: Double) -> curves.Pt { curves.Pt(x: x, y: y) }
private func near(_ a: curves.Pt, _ x: Double, _ y: Double) -> Swift.Bool { abs(a.x - x) < 1e-9 && abs(a.y - y) < 1e-9 }
private let corner = curves.from(pt(0, 0)).line(pt(100, 0)).line(pt(100, 100))

@Test func defaultsDropped() {
  let o = curves.oval(0, 0, 100, 50)
  let xs = Array(o.nodes()).map(\.x), ys = Array(o.nodes()).map(\.y)
  #expect(abs(xs.max()! - 100) < 1e-9 && abs(xs.min()!) < 1e-9)
  #expect(abs(ys.max()! - 50) < 1e-9 && abs(ys.min()!) < 1e-9)

  let line = curves.line(pt(0, 0), pt(100, 0))
  #expect(near(line.select().rotate(180).path().node(0), 100, 0))
  #expect(near(line.select().scale(2.0).path().node(0), -50, 0))
  #expect(near(line.select().mirror().path().node(0), 100, 0))
  #expect(curves.catmullRomPath([pt(0, 0), pt(50, 50), pt(100, 0)]).nodeCount() == 3)

  let up = curves.warpCubic(curves.Cubic(pt(0, 0), pt(10, 0), pt(20, 0), pt(30, 0))) { pt($0.x, $0.y + 5) }
  #expect(Array(up).allSatisfy { c in (0..<4).allSatisfy { abs(c[$0].y - 5) < 1e-9 } })
  #expect(near(line.warp { pt($0.x, $0.y + 5) }.node(1), 100, 5))
}

@Test func variants() {
  #expect(near(corner.select().weigh(curves.Strength(0.5)).shift(10, 90).path().node(2), 100, 105))
  let picked = corner.select(curves.NodePick([2])).shift(10, 90).path()
  #expect(near(picked.node(2), 100, 110) && near(picked.node(0), 0, 0))
  let weighed = corner.select(curves.NodePick { _, i, _ in i == 0 ? 1 : 0 }).shift(10, 90).path()
  #expect(near(weighed.node(0), 0, 10) && near(weighed.node(2), 100, 100))
  #expect(curves.ParamValue(3).number == 3 && curves.ParamValue("x").text == "x")
}

@Test func closures() {
  let moved = corner.select().moveEach { _, _, i in curves.Warp { q in pt(q.x + Double(i), q.y) } }.path()
  #expect(near(moved.node(2), 102, 100))
}

@Test func spanOverloads() {
  let a = curves.parseSvgPath(std.string("M0 0L10 0L10 10L0 10Z"))
  let b = curves.parseSvgPath(std.string("M5 5L15 5L15 15L5 15Z"))
  #expect(curves.corners(a).size() == 4)
  #expect(curves.winding(a, pt(5, 5)) != 0 && curves.winding(a, pt(20, 20)) == 0)
  let both = curves.ContourList(Array(a) + Array(b))
  let united = curves.unite(both)
  let union = curves.boolean([a, b]) { w in w[0] != 0 || w[1] != 0 }
  #expect(united.size() == 1 && String(curves.svgPath(united, 2)) == String(curves.svgPath(union, 2)))
  let cut = curves.boolean([a, b]) { w in w[0] != 0 && w[1] != 0 }
  #expect(String(curves.svgPath(cut, 2)).contains("10"))
  let rounded = curves.roundCorners(a) { _ in 2 }
  #expect(rounded.size() == 1 && rounded[0].size() > a[0].size())
  #expect(abs(curves.area(curves.PtList([pt(0, 0), pt(10, 0), pt(10, 10)]))) == 50)
}

@Test func modsAndKeys() {
  let rotate = curves.mod(std.string("rotate")).pointee
  let line = curves.line(pt(0, 0), pt(100, 0))
  #expect(near(curves.applyMod(line.select(), rotate, curves.Values(["deg": 180])).path().node(0), 100, 0))
  #expect(!String(curves.modCode(rotate)).isEmpty)
  #expect(curves.cxx.keysAt(curves.cxx.spread([0, 10]), 0.5) == 5)
}
