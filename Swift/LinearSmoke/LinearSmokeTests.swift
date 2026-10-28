// MathLinear on its own, through C++ interop: the product a host takes for vectors and matrices.
import MathLinear
import Testing

@Test func vectorLengthThroughInterop() {
  let vector = editor.Vector3(3, 4, 0)
  #expect(vector.length() == 5)
}
