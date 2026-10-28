// Paths in cases: named by a spec, { start, segs, closed?, arrives?, metric?, kinds? }, and written back whole as
// { start, segs, closed, arrives, metric, kinds } (kinds read for every node). tests/paths.hpp reads and writes them.
import * as m from '../../../../../../apps/Fountain/fountain-ts/src/curves/path.ts'
import type { Cubic, Pt } from '../../../../../../apps/Fountain/fountain-ts/src/curves/geometry.ts'

export type Spec = { start: Pt, segs: Cubic[], closed?: boolean, arrives?: m.Heading, metric?: Pt, kinds?: (m.NodeKind | null)[] }
export const P = (s: Spec) => new m.Path(s.start, s.segs, s.closed ?? false, s.arrives, s.metric ?? [1, 1], s.kinds?.map(k => k ?? undefined))
export const ser = (p: m.Path) => ({ start: p.start, segs: p.segs, closed: p.closed, arrives: p.arrives ?? null, metric: p.metric, kinds: p.kinds })
/** A spec of segments that join. */
export const spec = (segs: Cubic[], closed = false, metric: Pt = [1, 1]): Spec => ({ start: segs[0]![0], segs, closed, metric })
