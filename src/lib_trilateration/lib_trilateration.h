#ifndef UWB_TRILATERATION_H
#define UWB_TRILATERATION_H

#include "stdio.h"
#include "uwb_vmath.h"


using namespace std;

// Exposed C functions for C# wrapper through P/Invoke
// Usually done with C++/CLI wrapper then C# ref, but cannot be done outside of Windows

namespace lib_trilateration {
    /** Determine the solution(s) of the algorithm with the given inputs - Anchors positions and distances
     * @return Return TRIL_3SPHERES if it is performed using 3 spheres and return
     * @return TRIL_4SPHERES if it is performed using 4 spheres
     * @return Return negative number for other errors
     * @return For TRIL_3SPHERES, there are two solutions: result1 and result2
     * @return For TRIL_4SPHERES, there is only one solution: best_solution
    */
    int trilateration(Coord3D *const result1,
                      Coord3D *const result2,
                      Coord3D *const best_solution,
                      const Coord3D p1, const double r1,
                      const Coord3D p2, const double r2,
                      const Coord3D p3, const double r3,
                      const Coord3D p4, const double r4);

    Coord3D trilateration_4_spheres(const Coord3D result1_in, const Coord3D result2_in, const Coord3D p4,
                                    const double r4);

    /* Location Functions */
    /**
     * @return Whether p1 and p2 are concentric or not
     */
    bool concentric(const Coord3D p1, const Coord3D p2);

    /** @return Determine the Geometric Dilution Of Precision rate between 0 and 1
     * Lower GDOP rate means better precision of intersection
     */
    double gdoprate(const Coord3D tag, const Coord3D p1, const Coord3D p2, const Coord3D p3);

    /** Determine the intersection between a line and a sphere
     * Intersecting the fourth sphere s4 with radius of r4, with a segment p1-p2
     * @return zero if successful, negative error otherwise.
     * @param mu1 & mu2 are constant to find points of intersection.
    */
    int sphereline(const Coord3D p1, const Coord3D p2, const Coord3D s4, double r4, double *const mu1,
                   double *const mu2);

    /**
     * This function calls trilateration to get the best solution.

     * If any three spheres does not produce valid solution,
     * then each distance is increased to ensure intersection to happen.

     * @param solution1 One of the two solutions with TRIL_3SPHERES
     * @param solution2 One of the two solutions with TRIL_3SPHERES
     * @param best_solution The unique solution with TRIL_4SPHERES
     * @param nosolution_count the number of failed attempt before intersection is found
     by increasing the sphere diameter.
     * @param combination
     * @return the trilateration mode, either TRIL_3SPHERES or TRIL_4SPHERES or an error value
     */
    int manager_3D(Coord3D *const solution1, Coord3D *const solution2, Coord3D *const best_solution,
                   int *const nosolution_count, double *const best_3derror, double *const best_gdoprate,
                   Coord3D p1, double r1, Coord3D p2, double r2, Coord3D p3, double r3, Coord3D p4, double r4,
                   int *combination);

    /**
     * Defines the policy to chose between the two solutions that might be obtained
     * @param best_solution Chosen final best solution
     * @param use4thAnchor Whether a 4th anchor is used or not
     */
    extern "C" int get_location(Coord3D *best_solution, const bool use4thAnchor, Coord3D *anchorArray, int *distanceArray);
};

#endif
