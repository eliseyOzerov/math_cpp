// The C++ API from Swift through C++ interop: a path built, measured and edited through a selection.
import Curves
import Testing

@Test func pathFromSwift() {
  let p = curves.from(curves.Pt(x: 0, y: 0)).line(curves.Pt(x: 100, y: 0)).line(curves.Pt(x: 100, y: 100))
  #expect(abs(p.length() - 200) < 1e-9)
  #expect(p.nodeCount() == 3)
  let mid = p.at(0.5)
  #expect(mid.x == 100 && mid.y == 0)
  let moved = p.select().shift(10, 90).path()
  #expect(moved.node(2).y == 110)
}

@Test func svgFromSwift() {
  let contours = curves.parseSvgPath(std.string("M0 0L10 0L10 10Z"))
  #expect(contours.size() == 1)
  #expect(String(curves.svgPath(contours, 2)) == "M0 0L10 0L10 10L0 0Z")
}
