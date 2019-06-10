#ifndef UWB_VMATH_H
#define UWB_VMATH_H

#include "math.h"  //	C Math Library

/** Either a vector or a point in 3D space */
struct Coord3D {
    double x;
    double y;
    double z;
};
typedef struct Coord3D Coord3D;

/** @return Difference of two vectors : A - B */
inline Coord3D vdiff(const Coord3D A, const Coord3D B) {
    Coord3D v;
    v.x = A.x - B.x;
    v.y = A.y - B.y;
    v.z = A.z - B.z;
    return v;
}

/** @return Sum of two vectors : A + B */
inline Coord3D vsum(const Coord3D A, const Coord3D B) {
    Coord3D v;
    v.x = A.x + B.x;
    v.y = A.y + B.y;
    v.z = A.z + B.z;
    return v;
}

/** @return Multiplication of a vector A by a number n */
inline Coord3D vmul(const Coord3D A, const double n) {
    Coord3D v;
    v.x = A.x * n;
    v.y = A.y * n;
    v.z = A.z * n;
    return v;
}

/** @return Division of a vector A by a number n */
inline Coord3D vdiv(const Coord3D A, const double n) {
    Coord3D v;
    v.x = A.x / n;
    v.y = A.y / n;
    v.z = A.z / n;
    return v;
}

/** @return Scalar Product of two vectors : A (dot) B */
inline double dot(const Coord3D A, const Coord3D B) {
    return A.x * B.x + A.y * B.y + A.z * B.z;
}

/** @return Euclidean norm of vector */
inline double vnorm(const Coord3D A) {
    return sqrt(dot(A, A));
}

/** @return Euclidean distance between points v1 and v2 */
inline double vdist(const Coord3D v1, const Coord3D v2) {
    return vnorm(vdiff(v1, v2));
}

/** @return Cross Product of two vectors : A X B
 *  | v.x   v.y   v.z |
 *  | A.x   A.y   A.z |
 *  | B.x   B.y   B.z |
 */
inline Coord3D cross(const Coord3D A, const Coord3D B) {
    Coord3D v;
    v.x = A.y * B.z - A.z * B.y;
    v.y = A.z * B.x - A.x * B.z;
    v.z = A.x * B.y - A.y * B.x;
    return v;
}

/** @return Unit vector in the direction of a-b */
inline Coord3D unit_vector(Coord3D a, Coord3D b) {
    Coord3D a_b = vdiff(b, a);
    double norm = vnorm(a_b);
    return vdiv(a_b, norm);
}

#endif //UWB_VMATH_H
