// Curves the cases share: cubics of every kind and the awkward ones, a few points and parameters.
import type { Cubic, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'

export const cubics: Record<string, Cubic> = {
  arch: [[0, 0], [0, 100], [100, 100], [100, 0]],
  s: [[0, 0], [50, 100], [50, -100], [100, 0]],
  loop: [[0, 0], [150, 100], [-50, 100], [100, 0]],
  cusp: [[0, 0], [0, 100], [100, 100], [0, 75]],
  line: [[0, 0], [10, 10], [20, 20], [30, 30]],
  handleOnNode: [[0, 0], [0, 0], [100, 50], [100, 100]],
  skew: [[12.5, -3], [40, 80], [95, 60], [130, 10]],
  dot: [[5, 5], [5, 5], [5, 5], [5, 5]],
}
export const cs = Object.values(cubics)
export const pts: Pt[] = [[50, 50], [-20, 30], [130, 120], [0, 0]]
export const ts_ = [0, 0.25, 0.5, 0.8, 1]
