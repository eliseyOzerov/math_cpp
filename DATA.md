# graphics_cpp data

How the library's values are saved as JSON: drawings, the nib along them and the settings of the moves on them, so
an app can keep them in files and hand them across an FFI. Only generic curve data is here. An app's own formats are
made of these and documented by the app (Fountain's fonts: `docs/format.md` in Fountain).

The JSON is the one the TS side (`fountain-ts/src/curves`) writes today. A codec for it (`include/curves/json.hpp`)
is still to be written. FountainInk's reader (`fi::json`) is where it starts.

## Conventions

- Field names are the TS names, which the C++ structs share.
- Enums are their names as strings (`"smooth"`, `"flat"`).
- An optional field is left out when it isn't set. Nothing is written as `null`.
- A flag is `true` or left out, never `false`.
- Numbers are written shortest round-trip, so a value read back is the same double.
- Lengths are in the units of the drawing they belong to (font units in Fountain), unless said otherwise.
- Functions (waves, warps, callbacks) are not data. Wherever the C++ takes a wave, the JSON takes keys.

## Points and curves

| Type | JSON | C++ |
|---|---|---|
| Pt | `[x, y]` | `Pt` |
| Piece | `[p0, p1]` for a line, `[p0, c1, c2, p3]` for a cubic | `Piece` |
| Contour | `[Piece, …]`, closed | `Contour` |

## Nodes

A drawing is lists of nodes, and each list is drawn as a path (nodes.hpp).

| Type | JSON | Notes |
|---|---|---|
| Node | `{ "p": Pt, "in"?: Pt, "out"?: Pt, "kind"?: "corner" \| "smooth" \| "symmetric" }` | `in` and `out` are offsets from `p`. An open list's first node has no `in` and its last no `out`. `kind` is read when loading and turned into handle locks (`kindLocks`), so saved drawings hold locks rather than kinds. |
| NodeList | `{ "nodes": [Node, …], "closed"?: true }` | At least two nodes, or one when closed. |
| Rest | `{ "s": int, "i": int, "t": number, "lock"?: true }` | A node resting on list `s`: on its segment `i` (from node `i` to the next) at parameter `t`. The node stays on that spot of the curve as the host's nodes move. With `lock`, moving the node bends the host through it (`moldNodes`). Without it, the node slides to the nearest place (`nearestOn`). A rest whose host is gone is dropped. |
| HandleRef | `{ "s": int, "i": int, "side": "in" \| "out" }` | |
| HandleLock | `{ "handles": [HandleRef & { "flip"?: true }, …], "angle"?: true, "length"?: true }` | Handles held together, across lists too. |

Indices (`s`, `i`) count from 0 over the lists of one drawing, in its order.

`Rest` lives in fountain-ts as `Link` in `src/script/drawn.ts` today. It holds nothing about fonts, so it moves to
nodes.hpp as part of the port.

## Values along a path

| Type | JSON | Notes |
|---|---|---|
| Keys&lt;V&gt; | `[[t, V], [t, V, { "in"?: Pt, "out"?: Pt }], …]` | `t` is a share 0…1 along the path. The third element holds the key's handles, each `(dt, dv)` from it. |
| Strength | `number \| [number, …] \| Keys` | One value everywhere, values spread evenly (a gradient or steps), or keys. The C++ also takes a Wave, which isn't data, so a wave is saved as keys (`keysOf`). |

## Nib

| Type | JSON | Notes |
|---|---|---|
| Radius | `number \| [number, number, number, number]` | One radius for every corner, or one per corner counter-clockwise from the top right. 0 is square, 1 round. |
| Nib | `{ "w"?, "h"?, "rx"?: Radius, "ry"?: Radius, "angle"?, "follow"?, "perCorner"?, "perSpan"? }` | Fields left out take `Nib`'s defaults. |
| StemPen | `{ "weight": number, "contrast"?, "stress"? }` | A pen leaving stems `weight` wide. |
| NibInput | `number \| Nib \| StemPen` | A bare number is a round nib that wide. A StemPen is told apart by its `weight`. |
| Shape | `{ "w", "h", "rx": Radius, "ry": Radius }` | |
| ShapesInput | `Keys<Shape> \| [{ "w"?, "h"?, "rx"?, "ry"? }, …]` | Partial shapes are spread evenly along the path. |
| NodeNib | `{ "scale"?, "angle"?, "w"?, "h"?, "rx"?: Radius, "ry"?: Radius }` | A node's own nib on top of the drawing's. `scale` multiplies its size and `angle` adds degrees to it. `w` and `h` are factors. A field left out is taken from the nodes on either side that set it. |
| NibLock | `{ "nodes": [{ "s", "i" }, …], "shape"?: true, "angle"?: true, "scale"?: true }` | Nodes whose nibs keep one shape, rotation and/or scale. The editor keeps them alike, so the ink reads plain node nibs. |

`NodeNib`, `NibLock` and how node nibs turn into the stroke's keys (`nibKeys`) live in fountain-ts
`src/script/drawn.ts` today. They're generic and move to nib.hpp as part of the port.

## Ends, corners and noise

| Type | JSON | Notes |
|---|---|---|
| Cut | `{ "mode": "auto" \| "flat" \| "nib" \| "round" \| "level" \| "angle", "deg"?, "taper"?, "reach"? }` | |
| Cap | `{ "shape": [Piece, …], "align"? }` | An open curve across an end. |
| CornerShape | `{ "angle": number, "profile"?: [Piece, …] }` | |
| Wobble | `{ "amplitude": number, "frequency"?, "octaves"?, "seed"? }` | |
