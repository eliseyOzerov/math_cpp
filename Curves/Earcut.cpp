// Port of mapbox/earcut 3.0.2 (Vladimir Agafonkin, ISC), function by function, so it reads
// next to the JavaScript. JavaScript's `null` is -1; its linked-list nodes live in one pool
// and link by index. Every number is a double and every expression keeps earcut's order.
#include "Earcut.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace editor {

namespace {

constexpr int none = -1;
constexpr int dim = 2;

struct Node {
    int i;          // vertex index in coordinates array
    double x, y;    // vertex coordinates
    int prev = none;  // previous and next vertex nodes in a polygon ring
    int next = none;
    int z = 0;      // z-order curve value
    int prevZ = none;  // previous and next nodes in z-order
    int nextZ = none;
    bool steiner = false;  // indicates whether this is a steiner point
};

/// JavaScript truthiness of a number: false for 0 and NaN.
bool truthy(double value) { return value != 0 && !std::isnan(value); }

/// JavaScript's `value | 0`: truncate, then wrap to 32 bits.
int32_t toInt32(double value) {
    if (!std::isfinite(value)) return 0;
    double wrapped = std::fmod(std::trunc(value), 4294967296.0);
    if (wrapped < 0) wrapped += 4294967296.0;
    return static_cast<int32_t>(static_cast<uint32_t>(wrapped));
}

int sign(double num) { return num > 0 ? 1 : num < 0 ? -1 : 0; }

double signedArea(std::span<const double> data, int start, int end) {
    double sum = 0;
    for (int i = start, j = end - dim; i < end; i += dim) {
        sum += (data[j] - data[i]) * (data[i + 1] + data[j + 1]);
        j = i;
    }
    return sum;
}

// z-order of a point given coords and inverse of the longer side of data bbox
int zOrder(double x, double y, double minX, double minY, double invSize) {
    // coords are transformed into non-negative 15-bit integer range
    uint32_t xBits = static_cast<uint32_t>(toInt32((x - minX) * invSize));
    uint32_t yBits = static_cast<uint32_t>(toInt32((y - minY) * invSize));

    xBits = (xBits | (xBits << 8)) & 0x00FF00FF;
    xBits = (xBits | (xBits << 4)) & 0x0F0F0F0F;
    xBits = (xBits | (xBits << 2)) & 0x33333333;
    xBits = (xBits | (xBits << 1)) & 0x55555555;

    yBits = (yBits | (yBits << 8)) & 0x00FF00FF;
    yBits = (yBits | (yBits << 4)) & 0x0F0F0F0F;
    yBits = (yBits | (yBits << 2)) & 0x33333333;
    yBits = (yBits | (yBits << 1)) & 0x55555555;

    return static_cast<int32_t>(xBits | (yBits << 1));
}

// check if a point lies within a convex triangle
bool pointInTriangle(double ax, double ay, double bx, double by, double cx, double cy, double px, double py) {
    return (cx - px) * (ay - py) >= (ax - px) * (cy - py) &&
           (ax - px) * (by - py) >= (bx - px) * (ay - py) &&
           (bx - px) * (cy - py) >= (cx - px) * (by - py);
}

// check if a point lies within a convex triangle but false if its equal to the first point of the triangle
bool pointInTriangleExceptFirst(double ax, double ay, double bx, double by, double cx, double cy, double px, double py) {
    return !(ax == px && ay == py) && pointInTriangle(ax, ay, bx, by, cx, cy, px, py);
}

// Math.min / Math.max of three numbers, as written (no NaN handling needed for finite input)
double minOf(double a, double b, double c) { return std::min(std::min(a, b), c); }
double maxOf(double a, double b, double c) { return std::max(std::max(a, b), c); }

// The pool of nodes and every earcut function that touches them.
class Earcut {
public:
    std::vector<Node> pool;
    std::vector<int> triangles;
    std::span<const double> data;

    Node& at(int node) { return pool[static_cast<size_t>(node)]; }

    int createNode(int i, double x, double y) {
        Node node;
        node.i = i;
        node.x = x;
        node.y = y;
        pool.push_back(node);
        return static_cast<int>(pool.size()) - 1;
    }

    // create a node and optionally link it with previous one (in a circular doubly linked list)
    int insertNode(int i, double x, double y, int last) {
        int p = createNode(i, x, y);

        if (last == none) {
            at(p).prev = p;
            at(p).next = p;
        } else {
            at(p).next = at(last).next;
            at(p).prev = last;
            at(at(last).next).prev = p;
            at(last).next = p;
        }
        return p;
    }

    void removeNode(int p) {
        at(at(p).next).prev = at(p).prev;
        at(at(p).prev).next = at(p).next;

        if (at(p).prevZ != none) at(at(p).prevZ).nextZ = at(p).nextZ;
        if (at(p).nextZ != none) at(at(p).nextZ).prevZ = at(p).prevZ;
    }

    // signed area of a triangle
    double area(int p, int q, int r) {
        const Node& P = at(p);
        const Node& Q = at(q);
        const Node& R = at(r);
        return (Q.y - P.y) * (R.x - Q.x) - (Q.x - P.x) * (R.y - Q.y);
    }

    // check if two points are equal
    bool equals(int p1, int p2) { return at(p1).x == at(p2).x && at(p1).y == at(p2).y; }

    // for collinear points p, q, r, check if point q lies on segment pr
    bool onSegment(int p, int q, int r) {
        const Node& P = at(p);
        const Node& Q = at(q);
        const Node& R = at(r);
        return Q.x <= std::max(P.x, R.x) && Q.x >= std::min(P.x, R.x) && Q.y <= std::max(P.y, R.y) && Q.y >= std::min(P.y, R.y);
    }

    // check if two segments intersect
    bool intersects(int p1, int q1, int p2, int q2) {
        int o1 = sign(area(p1, q1, p2));
        int o2 = sign(area(p1, q1, q2));
        int o3 = sign(area(p2, q2, p1));
        int o4 = sign(area(p2, q2, q1));

        if (o1 != o2 && o3 != o4) return true;  // general case

        if (o1 == 0 && onSegment(p1, p2, q1)) return true;  // p1, q1 and p2 are collinear and p2 lies on p1q1
        if (o2 == 0 && onSegment(p1, q2, q1)) return true;  // p1, q1 and q2 are collinear and q2 lies on p1q1
        if (o3 == 0 && onSegment(p2, p1, q2)) return true;  // p2, q2 and p1 are collinear and p1 lies on p2q2
        if (o4 == 0 && onSegment(p2, q1, q2)) return true;  // p2, q2 and q1 are collinear and q1 lies on p2q2

        return false;
    }

    // check if a polygon diagonal intersects any polygon segments
    bool intersectsPolygon(int a, int b) {
        int p = a;
        do {
            if (at(p).i != at(a).i && at(at(p).next).i != at(a).i && at(p).i != at(b).i && at(at(p).next).i != at(b).i &&
                    intersects(p, at(p).next, a, b)) return true;
            p = at(p).next;
        } while (p != a);

        return false;
    }

    // check if a polygon diagonal is locally inside the polygon
    bool locallyInside(int a, int b) {
        int aPrev = at(a).prev;
        int aNext = at(a).next;
        return area(aPrev, a, aNext) < 0 ?
            area(a, b, aNext) >= 0 && area(a, aPrev, b) >= 0 :
            area(a, b, aPrev) < 0 || area(a, aNext, b) < 0;
    }

    // check if the middle point of a polygon diagonal is inside the polygon
    bool middleInside(int a, int b) {
        int p = a;
        bool inside = false;
        double px = (at(a).x + at(b).x) / 2;
        double py = (at(a).y + at(b).y) / 2;
        do {
            const Node& P = at(p);
            const Node& N = at(P.next);
            if (((P.y > py) != (N.y > py)) && N.y != P.y &&
                    (px < (N.x - P.x) * (py - P.y) / (N.y - P.y) + P.x))
                inside = !inside;
            p = P.next;
        } while (p != a);

        return inside;
    }

    // check if a diagonal between two polygon nodes is valid (lies in polygon interior)
    bool isValidDiagonal(int a, int b) {
        int aNext = at(a).next;
        int aPrev = at(a).prev;
        int bPrev = at(b).prev;
        return at(aNext).i != at(b).i && at(aPrev).i != at(b).i && !intersectsPolygon(a, b) &&  // doesn't intersect other edges
               ((locallyInside(a, b) && locallyInside(b, a) && middleInside(a, b) &&  // locally visible
                 (truthy(area(aPrev, a, bPrev)) || truthy(area(a, bPrev, b)))) ||  // does not create opposite-facing sectors
                (equals(a, b) && area(aPrev, a, aNext) > 0 && area(bPrev, b, at(b).next) > 0));  // special zero-length case
    }

    // link two polygon vertices with a bridge; if the vertices belong to the same ring, it splits polygon into two;
    // if one belongs to the outer ring and another to a hole, it merges it into a single ring
    int splitPolygon(int a, int b) {
        int a2 = createNode(at(a).i, at(a).x, at(a).y);
        int b2 = createNode(at(b).i, at(b).x, at(b).y);
        int an = at(a).next;
        int bp = at(b).prev;

        at(a).next = b;
        at(b).prev = a;

        at(a2).next = an;
        at(an).prev = a2;

        at(b2).next = a2;
        at(a2).prev = b2;

        at(bp).next = b2;
        at(b2).prev = bp;

        return b2;
    }

    // create a circular doubly linked list from polygon points in the specified winding order
    int linkedList(int start, int end, bool clockwise) {
        int last = none;

        if (clockwise == (signedArea(data, start, end) > 0)) {
            for (int i = start; i < end; i += dim) last = insertNode(i / dim, data[i], data[i + 1], last);
        } else {
            for (int i = end - dim; i >= start; i -= dim) last = insertNode(i / dim, data[i], data[i + 1], last);
        }

        if (last != none && equals(last, at(last).next)) {
            removeNode(last);
            last = at(last).next;
        }

        return last;
    }

    // eliminate colinear or duplicate points
    int filterPoints(int start, int end = none) {
        if (start == none) return start;
        if (end == none) end = start;

        int p = start;
        bool again;
        do {
            again = false;

            if (!at(p).steiner && (equals(p, at(p).next) || area(at(p).prev, p, at(p).next) == 0)) {
                removeNode(p);
                p = end = at(p).prev;
                if (p == at(p).next) break;
                again = true;

            } else {
                p = at(p).next;
            }
        } while (again || p != end);

        return end;
    }

    // check whether a polygon node forms a valid ear with adjacent nodes
    bool isEar(int ear) {
        int a = at(ear).prev;
        int b = ear;
        int c = at(ear).next;

        if (area(a, b, c) >= 0) return false;  // reflex, can't be an ear

        // now make sure we don't have other points inside the potential ear
        double ax = at(a).x, bx = at(b).x, cx = at(c).x, ay = at(a).y, by = at(b).y, cy = at(c).y;

        // triangle bbox
        double x0 = minOf(ax, bx, cx),
               y0 = minOf(ay, by, cy),
               x1 = maxOf(ax, bx, cx),
               y1 = maxOf(ay, by, cy);

        int p = at(c).next;
        while (p != a) {
            const Node& P = at(p);
            if (P.x >= x0 && P.x <= x1 && P.y >= y0 && P.y <= y1 &&
                pointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, P.x, P.y) &&
                area(P.prev, p, P.next) >= 0) return false;
            p = P.next;
        }

        return true;
    }

    bool isEarHashed(int ear, double minX, double minY, double invSize) {
        int a = at(ear).prev;
        int b = ear;
        int c = at(ear).next;

        if (area(a, b, c) >= 0) return false;  // reflex, can't be an ear

        double ax = at(a).x, bx = at(b).x, cx = at(c).x, ay = at(a).y, by = at(b).y, cy = at(c).y;

        // triangle bbox
        double x0 = minOf(ax, bx, cx),
               y0 = minOf(ay, by, cy),
               x1 = maxOf(ax, bx, cx),
               y1 = maxOf(ay, by, cy);

        // z-order range for the current triangle bbox;
        int minZ = zOrder(x0, y0, minX, minY, invSize);
        int maxZ = zOrder(x1, y1, minX, minY, invSize);

        int p = at(ear).prevZ;
        int n = at(ear).nextZ;

        // look for points inside the triangle in both directions
        while (p != none && at(p).z >= minZ && n != none && at(n).z <= maxZ) {
            if (at(p).x >= x0 && at(p).x <= x1 && at(p).y >= y0 && at(p).y <= y1 && p != a && p != c &&
                pointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, at(p).x, at(p).y) && area(at(p).prev, p, at(p).next) >= 0) return false;
            p = at(p).prevZ;

            if (at(n).x >= x0 && at(n).x <= x1 && at(n).y >= y0 && at(n).y <= y1 && n != a && n != c &&
                pointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, at(n).x, at(n).y) && area(at(n).prev, n, at(n).next) >= 0) return false;
            n = at(n).nextZ;
        }

        // look for remaining points in decreasing z-order
        while (p != none && at(p).z >= minZ) {
            if (at(p).x >= x0 && at(p).x <= x1 && at(p).y >= y0 && at(p).y <= y1 && p != a && p != c &&
                pointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, at(p).x, at(p).y) && area(at(p).prev, p, at(p).next) >= 0) return false;
            p = at(p).prevZ;
        }

        // look for remaining points in increasing z-order
        while (n != none && at(n).z <= maxZ) {
            if (at(n).x >= x0 && at(n).x <= x1 && at(n).y >= y0 && at(n).y <= y1 && n != a && n != c &&
                pointInTriangleExceptFirst(ax, ay, bx, by, cx, cy, at(n).x, at(n).y) && area(at(n).prev, n, at(n).next) >= 0) return false;
            n = at(n).nextZ;
        }

        return true;
    }

    // go through all polygon nodes and cure small local self-intersections
    int cureLocalIntersections(int start) {
        int p = start;
        do {
            int a = at(p).prev;
            int b = at(at(p).next).next;

            if (!equals(a, b) && intersects(a, p, at(p).next, b) && locallyInside(a, b) && locallyInside(b, a)) {

                triangles.push_back(at(a).i);
                triangles.push_back(at(p).i);
                triangles.push_back(at(b).i);

                // remove two nodes involved
                removeNode(p);
                removeNode(at(p).next);

                p = start = b;
            }
            p = at(p).next;
        } while (p != start);

        return filterPoints(p);
    }

    // try splitting polygon into two and triangulate them independently
    void splitEarcut(int start, double minX, double minY, double invSize) {
        // look for a valid diagonal that divides the polygon into two
        int a = start;
        do {
            int b = at(at(a).next).next;
            while (b != at(a).prev) {
                if (at(a).i != at(b).i && isValidDiagonal(a, b)) {
                    // split the polygon in two by the diagonal
                    int c = splitPolygon(a, b);

                    // filter colinear points around the cuts
                    a = filterPoints(a, at(a).next);
                    c = filterPoints(c, at(c).next);

                    // run earcut on each half
                    earcutLinked(a, minX, minY, invSize, 0);
                    earcutLinked(c, minX, minY, invSize, 0);
                    return;
                }
                b = at(b).next;
            }
            a = at(a).next;
        } while (a != start);
    }

    // main ear slicing loop which triangulates a polygon (given as a linked list)
    void earcutLinked(int ear, double minX, double minY, double invSize, int pass) {
        if (ear == none) return;

        // interlink polygon nodes in z-order
        if (!pass && truthy(invSize)) indexCurve(ear, minX, minY, invSize);

        int stop = ear;

        // iterate through ears, slicing them one by one
        while (at(ear).prev != at(ear).next) {
            int prev = at(ear).prev;
            int next = at(ear).next;

            if (truthy(invSize) ? isEarHashed(ear, minX, minY, invSize) : isEar(ear)) {
                triangles.push_back(at(prev).i);  // cut off the triangle
                triangles.push_back(at(ear).i);
                triangles.push_back(at(next).i);

                removeNode(ear);

                // skipping the next vertex leads to less sliver triangles
                ear = at(next).next;
                stop = at(next).next;

                continue;
            }

            ear = next;

            // if we looped through the whole remaining polygon and can't find any more ears
            if (ear == stop) {
                // try filtering points and slicing again
                if (!pass) {
                    earcutLinked(filterPoints(ear), minX, minY, invSize, 1);

                // if this didn't work, try curing all small self-intersections locally
                } else if (pass == 1) {
                    ear = cureLocalIntersections(filterPoints(ear));
                    earcutLinked(ear, minX, minY, invSize, 2);

                // as a last resort, try splitting the remaining polygon into two
                } else if (pass == 2) {
                    splitEarcut(ear, minX, minY, invSize);
                }

                break;
            }
        }
    }

    // find the leftmost node of a polygon ring
    int getLeftmost(int start) {
        int p = start;
        int leftmost = start;
        do {
            if (at(p).x < at(leftmost).x || (at(p).x == at(leftmost).x && at(p).y < at(leftmost).y)) leftmost = p;
            p = at(p).next;
        } while (p != start);

        return leftmost;
    }

    // whether sector in vertex m contains sector in vertex p in the same coordinates
    bool sectorContainsSector(int m, int p) {
        return area(at(m).prev, m, at(p).prev) < 0 && area(at(p).next, m, at(m).next) < 0;
    }

    // David Eberly's algorithm for finding a bridge between hole and outer polygon
    int findHoleBridge(int hole, int outerNode) {
        int p = outerNode;
        double hx = at(hole).x;
        double hy = at(hole).y;
        double qx = -std::numeric_limits<double>::infinity();
        int m = none;

        // find a segment intersected by a ray from the hole's leftmost point to the left;
        // segment's endpoint with lesser x will be potential connection point
        // unless they intersect at a vertex, then choose the vertex
        if (equals(hole, p)) return p;
        do {
            int pNext = at(p).next;
            if (equals(hole, pNext)) return pNext;
            else if (hy <= at(p).y && hy >= at(pNext).y && at(pNext).y != at(p).y) {
                double x = at(p).x + (hy - at(p).y) * (at(pNext).x - at(p).x) / (at(pNext).y - at(p).y);
                if (x <= hx && x > qx) {
                    qx = x;
                    m = at(p).x < at(pNext).x ? p : pNext;
                    if (x == hx) return m;  // hole touches outer segment; pick leftmost endpoint
                }
            }
            p = pNext;
        } while (p != outerNode);

        if (m == none) return none;

        // look for points inside the triangle of hole point, segment intersection and endpoint;
        // if there are no points found, we have a valid connection;
        // otherwise choose the point of the minimum angle with the ray as connection point

        int stop = m;
        double mx = at(m).x;
        double my = at(m).y;
        double tanMin = std::numeric_limits<double>::infinity();

        p = m;

        do {
            if (hx >= at(p).x && at(p).x >= mx && hx != at(p).x &&
                    pointInTriangle(hy < my ? hx : qx, hy, mx, my, hy < my ? qx : hx, hy, at(p).x, at(p).y)) {

                double tan = std::fabs(hy - at(p).y) / (hx - at(p).x);  // tangential

                if (locallyInside(p, hole) &&
                    (tan < tanMin || (tan == tanMin && (at(p).x > at(m).x || (at(p).x == at(m).x && sectorContainsSector(m, p)))))) {
                    m = p;
                    tanMin = tan;
                }
            }

            p = at(p).next;
        } while (p != stop);

        return m;
    }

    // find a bridge between vertices that connects hole with an outer ring and link it
    int eliminateHole(int hole, int outerNode) {
        int bridge = findHoleBridge(hole, outerNode);
        if (bridge == none) {
            return outerNode;
        }

        int bridgeReverse = splitPolygon(bridge, hole);

        // filter collinear points around the cuts
        filterPoints(bridgeReverse, at(bridgeReverse).next);
        return filterPoints(bridge, at(bridge).next);
    }

    bool compareXYSlopeLess(int a, int b) {
        double result = at(a).x - at(b).x;
        // when the left-most point of 2 holes meet at a vertex, sort the holes counterclockwise so that when we find
        // the bridge to the outer shell is always the point that they meet at.
        if (result == 0) {
            result = at(a).y - at(b).y;
            if (result == 0) {
                double aSlope = (at(at(a).next).y - at(a).y) / (at(at(a).next).x - at(a).x);
                double bSlope = (at(at(b).next).y - at(b).y) / (at(at(b).next).x - at(b).x);
                result = aSlope - bSlope;
            }
        }
        return result < 0;  // a NaN result counts as a tie, as in JavaScript's sort
    }

    // link every hole into the outer loop, producing a single-ring polygon without holes
    int eliminateHoles(std::span<const int> holeIndices, int outerNode) {
        std::vector<int> queue;

        for (size_t i = 0, len = holeIndices.size(); i < len; i++) {
            int start = holeIndices[i] * dim;
            int end = i < len - 1 ? holeIndices[i + 1] * dim : static_cast<int>(data.size());
            int list = linkedList(start, end, false);
            if (list == at(list).next) at(list).steiner = true;
            queue.push_back(getLeftmost(list));
        }

        std::stable_sort(queue.begin(), queue.end(), [this](int a, int b) { return compareXYSlopeLess(a, b); });

        // process holes from left to right
        for (size_t i = 0; i < queue.size(); i++) {
            outerNode = eliminateHole(queue[i], outerNode);
        }

        return outerNode;
    }

    // Simon Tatham's linked list merge sort algorithm
    // http://www.chiark.greenend.org.uk/~sgtatham/algorithms/listsort.html
    int sortLinked(int list) {
        int numMerges;
        int inSize = 1;

        do {
            int p = list;
            int e;
            list = none;
            int tail = none;
            numMerges = 0;

            while (p != none) {
                numMerges++;
                int q = p;
                int pSize = 0;
                for (int i = 0; i < inSize; i++) {
                    pSize++;
                    q = at(q).nextZ;
                    if (q == none) break;
                }
                int qSize = inSize;

                while (pSize > 0 || (qSize > 0 && q != none)) {

                    if (pSize != 0 && (qSize == 0 || q == none || at(p).z <= at(q).z)) {
                        e = p;
                        p = at(p).nextZ;
                        pSize--;
                    } else {
                        e = q;
                        q = at(q).nextZ;
                        qSize--;
                    }

                    if (tail != none) at(tail).nextZ = e;
                    else list = e;

                    at(e).prevZ = tail;
                    tail = e;
                }

                p = q;
            }

            at(tail).nextZ = none;
            inSize *= 2;

        } while (numMerges > 1);

        return list;
    }

    // interlink polygon nodes in z-order
    void indexCurve(int start, double minX, double minY, double invSize) {
        int p = start;
        do {
            if (at(p).z == 0) at(p).z = zOrder(at(p).x, at(p).y, minX, minY, invSize);
            at(p).prevZ = at(p).prev;
            at(p).nextZ = at(p).next;
            p = at(p).next;
        } while (p != start);

        at(at(p).prevZ).nextZ = none;
        at(p).prevZ = none;

        sortLinked(p);
    }

    std::vector<int> earcut(std::span<const int> holeIndices) {
        bool hasHoles = !holeIndices.empty();
        int outerLen = hasHoles ? holeIndices[0] * dim : static_cast<int>(data.size());
        int outerNode = linkedList(0, outerLen, true);

        if (outerNode == none || at(outerNode).next == at(outerNode).prev) return triangles;

        double minX = 0, minY = 0, invSize = 0;

        if (hasHoles) outerNode = eliminateHoles(holeIndices, outerNode);

        // if the shape is not too simple, we'll use z-order curve hash later; calculate polygon bbox
        if (static_cast<int>(data.size()) > 80 * dim) {
            minX = data[0];
            minY = data[1];
            double maxX = minX;
            double maxY = minY;

            for (int i = dim; i < outerLen; i += dim) {
                double x = data[i];
                double y = data[i + 1];
                if (x < minX) minX = x;
                if (y < minY) minY = y;
                if (x > maxX) maxX = x;
                if (y > maxY) maxY = y;
            }

            // minX, minY and invSize are later used to transform coords into integers for z-order calculation
            invSize = std::max(maxX - minX, maxY - minY);
            invSize = invSize != 0 ? 32767 / invSize : 0;
        }

        earcutLinked(outerNode, minX, minY, invSize, 0);

        return triangles;
    }
};

}  // namespace

std::vector<int> earcutTriangles(std::span<const double> coordinates, std::span<const int> holeStarts) {
    Earcut state;
    state.data = coordinates;
    state.pool.reserve(coordinates.size() / dim + 16);
    return state.earcut(holeStarts);
}

}  // namespace editor
