// MathFields with the modules it depends on and no others: a sphere's signed distance.
import MathFields
import MathGeometry
import MathLinear
import Testing

@Test func sphereDistanceThroughInterop() {
  let sphere = editor.Sphere(editor.Vector3(0, 0, 0), 1)
  #expect(editor.signedDistanceOf(sphere, editor.Vector3(3, 0, 0)) == 2)
}
