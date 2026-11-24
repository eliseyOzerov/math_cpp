# Third-Party Notices

## Pomax Bezier Work

The 2D curve work in `Curves` (ported from graphics_cpp) builds on the Bézier curve work by Pomax:

- A Primer on Bezier Curves: https://pomax.github.io/bezierinfo/
- Bezier.js API: https://pomax.github.io/bezierjs/
- Bezier.js source repository: https://github.com/Pomax/bezierjs

Bezier.js license notice:

```text
Copyright (c) 2023 Pomax

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
```

## mapbox/earcut

`Curves/Earcut.cpp` is a C++ port of mapbox/earcut 3.0.2 (https://github.com/mapbox/earcut), the polygon triangulation by ear clipping, written to return the same indices as the JavaScript.

earcut license notice:

```text
ISC License

Copyright (c) 2016, Mapbox

Permission to use, copy, modify, and/or distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright notice
and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS
OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF
THIS SOFTWARE.
```

## fdlibm (Sun Microsystems), via FreeBSD msun

`Numerics/DeterministicMath.cpp` ports the double-precision `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `exp`, `expm1`, `log`, `log1p` and `sinh` of fdlibm 5.3 as FreeBSD's msun (https://github.com/freebsd/freebsd-src/tree/main/lib/msun/src) ships them: `e_rem_pio2.c`, `k_rem_pio2.c`, `k_sin.c`, `k_cos.c`, `k_tan.c`, `s_sin.c`, `s_cos.c`, `s_tan.c`, `e_asin.c`, `e_acos.c`, `s_atan.c`, `e_atan2.c`, `e_exp.c`, `k_exp.c`, `s_expm1.c`, `e_log.c`, `s_log1p.c` and `e_sinh.c`. The algorithms and constants are the originals; the code is rewritten as C++20 (bit access through `std::bit_cast`, one NaN, no floating-point flag handling). The results are bit-identical to the C sources compiled with `-ffp-contract=off`. Its `hypot` is our own.

fdlibm notice (preserved as the licence requires):

```text
Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.

Developed at SunSoft, a Sun Microsystems, Inc. business.
Permission to use, copy, modify, and distribute this
software is freely granted, provided that this notice
is preserved.
```

`e_exp.c` carries the same notice with "Copyright (C) 2004 by Sun Microsystems, Inc."
