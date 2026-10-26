// The C API from Swift, through the module map: a few calls checked against the golden values.
import CCurves
import Testing

@Test func cubicFromSwift() {
  let arch = cv_cubic(p: (cv_pt(x: 0, y: 0), cv_pt(x: 0, y: 100), cv_pt(x: 100, y: 100), cv_pt(x: 100, y: 0)))
  let mid = cv_bez(arch, 0.5)
  #expect(mid.x == 50 && mid.y == 75)
  var ts = (0.0, 0.0, 0.0, 0.0)
  let n = withUnsafeMutablePointer(to: &ts) { $0.withMemoryRebound(to: Double.self, capacity: 4) { cv_extrema(arch, $0) } }
  #expect(n == 1 && ts.0 == 0.5)
  #expect(cv_canonical(arch, 1e-6, nil) == CV_ARCH)
  var halves = [cv_cubic](repeating: cv_cubic(), count: 2)
  cv_split(arch, 0.5, &halves)
  #expect(halves[0].p.3.x == 50 && halves[1].p.0.y == 75)
}
