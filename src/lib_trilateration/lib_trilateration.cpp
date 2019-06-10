// -------------------------------------------------------------------------------------------------------------------
//
//  File: lib_trilateration.cpp
//
//  Based on algorithm written in 2016 by (c) Decawave Ltd, Dublin, Ireland and released as part of TREK1000 kit.
//
// -------------------------------------------------------------------------------------------------------------------

#include <cmath>

#include "lib_trilateration.h"
//Largest non negative number considered zero
#define        MAXZERO                                  0.001
//Integer values for the output of the trilateration function
#define        TRIL_3SPHERES                            3
#define        TRIL_4SPHERES                            4
//Errors < 0
#define        ERR_TRIL_CONCENTRIC                     -1
#define        ERR_TRIL_COLINEAR_2SOLUTIONS            -2
#define        ERR_TRIL_SQRTNEGNUMB                    -3
//Errors not used
//#define		ERR_TRIL_NOINTERSECTION_SPHERE4			-4
//#define		ERR_TRIL_NEEDMORESPHERE					-5
#include <iostream>


namespace lib_trilateration {
    inline bool concentric(const Coord3D p1, const Coord3D p2) {
        return vdist(p1, p2) <= MAXZERO;
    }

    double gdoprate(const Coord3D tag, const Coord3D p1, const Coord3D p2, const Coord3D p3) {
        Coord3D u1 = unit_vector(p1, tag);
        Coord3D u2 = unit_vector(p2, tag);
        Coord3D u3 = unit_vector(p3, tag);

        double gdop1 = fabs(dot(u1, u2));
        double gdop2 = fabs(dot(u2, u3));
        double gdop3 = fabs(dot(u3, u1));

        /* Return the worst GDOP */
        double result;
        if (gdop1 < gdop2) result = gdop2; else result = gdop1;
        if (result < gdop3) result = gdop3;

        return result;
    }

    int sphereline(const Coord3D p1, const Coord3D p2, const Coord3D s4, double r4, double *const mu1,
                   double *const mu2) {
        double a, b, c;
        Coord3D p1_2;

        /* Segment p12 */
        p1_2.x = p2.x - p1.x;
        p1_2.y = p2.y - p1.y;
        p1_2.z = p2.z - p1.z;

        /* Quadratic Equation : a²µ + bµ + c = 0 */
        /* a = (p2.x - p1.x)² + (p2.y - p1.y)² + (p2.z - p1.z)² */
        a = p1_2.x * p1_2.x + p1_2.y * p1_2.y + p1_2.z * p1_2.z;

        b = 2 * (p1_2.x * (p1.x - s4.x) + p1_2.y * (p1.y - s4.y) + p1_2.z * (p1.z - s4.z));
        /* c = s4².x + s4².y + s4².z + p1².x + p1².y + p1².z
                - 2(s4.x * p1.x + s4.y * p1.y + s4.z * p1.z) - r4² */
        c = s4.x * s4.x + s4.y * s4.y + s4.z * s4.z;
        c += p1.x * p1.x + p1.y * p1.y + p1.z * p1.z;
        c -= 2 * (s4.x * p1.x + s4.y * p1.y + s4.z * p1.z);
        c -= r4 * r4;

        // Second degree equation solving
        double delta = b * b - 4 * a * c;

        /* No intersection : return -1 */
        if (fabs(a) == 0 || delta < 0) {
            *mu1 = 0;
            *mu2 = 0;
            return -1;
        }

        *mu1 = (-b + sqrt(delta)) / (2 * a);
        *mu2 = (-b - sqrt(delta)) / (2 * a);

        return 0;
    }

    Coord3D trilateration_4_spheres(const Coord3D result1_in, const Coord3D result2_in, const Coord3D p4,
                                    const double r4) {
        Coord3D result1 = result1_in;
        Coord3D result2 = result2_in;

        /* If result2 is closest to the sphere 4 than result1, we
           want the result1 as the nearest point to the centre of sphere 4.
           We just switch the two variables result1 and result2. */
        if (vdist(result1, p4) > vdist(result2, p4)) {
            Coord3D temp = result1;
            result1 = result2;
            result2 = temp;
        }

        int count4 = 0;
        double r4_temp = r4;
        int result = 1; // 0 when sphereline is successful
        /* Intersecting result1-result2 vector with sphere 4 */
        /* We stay in the loop if :
              - We find no intersections
              - We increment the radius of sphere 4 till 1m */
        double mu1, mu2;
        while (result && count4 < 10) {
            result = sphereline(result1, result2, p4, r4_temp, &mu1, &mu2);
            r4_temp += 0.1; // + 10cm
            count4++;
        }

        /* If no solution found with sphereline, we take the closest solution to the sphere 4 as the best solution.*/
        if (result) return result1;

        /* New relative coordinate system with result1 as the origin */
        /*
           res1_res2 = result2-result1
           n_res1_2 = |result2 - result1|
           ux = (result2 - result1) / |result2 - result1|
           R_point => Point of intersection relative to origin result1
           O_point => Point of intersection relative to origin p1
        */
        if ((mu1 < 0 && mu2 > 1) || (mu2 < 0 && mu1 > 1)) {
            /* result1-result2 line segment is inside sphere 4 with no intersection */
            double mu;
            if (mu1 > mu2) mu = mu1; else mu = mu2;
            Coord3D ux = unit_vector(result1, result2);// Unit vector ux with respect to result1 (new coordinate system)
            /* Points to the intersection */
            Coord3D R_point = vmul(ux, mu * vdist(result1, result2));

            /* We come back to our original coordinate system with p1 as the origin*/
            Coord3D O_point = vsum(result1, R_point);
            /* Vector O_point-result2 with 50-50 error correction on the length of op_res2 */
            Coord3D op_res2 = vmul(vdiff(result2, O_point), 0.5);
            return vsum(O_point, op_res2);
        } else {
            double mu;
            if (mu1 < 0 && mu2 < 0) {
                /* result1-result2 line segment is outside sphere 4 with no intersection */
                if (fabs(mu1) <= fabs(mu2)) mu = mu1; else mu = mu2;
                /* 50-50 error correction for mu */
                mu = 0.5 * mu;
            } else if ((mu1 > 0 && mu1 < 1) != (mu2 > 0 && mu2 < 1)) {
                /* If one mu is between 0 to 1 and the other is not */

                /* result1-result2 line segment intersects sphere 4 at one point */
                if (mu1 >= 0 && mu1 <= 1) mu = mu1; else mu = mu2;
                /* Add or subtract with 0.5*mu to distribute error equally onto every sphere */
                if (mu <= 0.5) mu -= 0.5 * mu; else mu -= 0.5 * (1 - mu);
            } else if (mu1 == mu2) {
                /* If both mu1 and mu2 are between 0 and 1, and mu1 = mu2 */
                /* result1-result2 line segment is tangential to sphere 4 at one point */
                mu = mu1;
                /* Add or subtract with 0.5*mu to distribute error equally onto every sphere */
                if (mu <= 0.25) mu -= 0.5 * mu;
                else if (mu <= 0.5) mu -= 0.5 * (0.5 - mu);
                else if (mu <= 0.75) mu -= 0.5 * (mu - 0.5);
                else mu -= 0.5 * (1 - mu);
            } else {
                /* If both mu1 and mu2 are between 0 and 1 */
                /* result1-result2 line segment intersects sphere 4 at two points */
                mu = mu1 + mu2;
                /* 50-50 error correction for mu */
                mu = 0.5 * mu;
            }
            Coord3D ux = unit_vector(result1, result2);

            /* Points to the intersection */
            Coord3D R_point = vmul(ux, mu * vdist(result1, result2));

            /* We come back to our original coordinate system with p1 as the origin*/
            Coord3D O_point = vsum(result1, R_point);
            return O_point;
        }

    }

    int trilateration(Coord3D *const result1,
                      Coord3D *const result2,
                      Coord3D *const best_solution,
                      const Coord3D p1, const double r1,
                      const Coord3D p2, const double r2,
                      const Coord3D p3, const double r3,
                      const Coord3D p4, const double r4) {

        /****************************** FINDING TWO POINTS FROM THE FIRST THREE SPHERES ************************/

        /*------------------------------- CONCENTRIC CHECK FOR p1 p2 p3 ---------------------------------------*/
        // if there are at least 2 concentric spheres within the first 3 spheres
        // then stop the calculation, drop it with error -1 : ERR_TRIL_CONCENTRIC

        if (concentric(p1, p3) || concentric(p2, p3) || concentric(p1, p2)) {
            return ERR_TRIL_CONCENTRIC;
        }

        /*------------------------------- END OF CONCENTRIC CHECK FOR p1 p2 p3 --------------------------------*/

        /* ex = (p2 - p1) / |p2 - p1| */
        Coord3D ex = unit_vector(p1,
                                 p2); // Unit vector ex with respect to p1 (origin of the relative coordinate system)
        /* p1_3 = p3 - p1, p1_3_X = ex * (ex (dot) (p3 - p1)) */
        Coord3D p1_3 = vdiff(p3, p1); // vector p13
        double i = dot(ex, p1_3);      // Scalar of p13 on the ex direction
        Coord3D p1_3_X = vmul(ex, i); // X component of the vector p13 -> p13.x
        /* p1_3_Y = (p1_3 - p1_3_X), t = |p1_3 - p1_3_X| */
        Coord3D p1_3_Y = vdiff(p1_3, p1_3_X); // Vector p13.y
        double t = vnorm(p1_3_Y);              // Norm of the vector p13.y
        Coord3D ey = vdiv(p1_3_Y, t); // unit vector ey with respect to p1 (origin of the relative coordinate system)


        /* if j=0, that means that the centres of the three spheres are on the same axis (co-linear) */
        /* NB: t <= MAXZERO implies j = 0.0 */
        double j;
        if (t > MAXZERO) {

            /* j = ey (dot) (p3 - p1) */
            j = dot(ey, p1_3);     // Scalar p1_3 on the ey direction
        } else
            j = 0.0;

        bool are_spheres_colinear = fabs(j) <= MAXZERO;

        /*------------------------------ FIND SOLUTIONS FOR COLINEAR SPHERES ----------------------------------*/
        /* This section is also used if the user puts wrong values of positions for the three anchors.
           Errors in configurations or physically misplaced */

        if (are_spheres_colinear) {
            /* Co-linear with one intersection co_Lin_Int */
            /* Is point p1 + (r1 along the axis) the intersection ? */
            Coord3D co_Lin_Int;
            co_Lin_Int = vsum(p1, vmul(ex, r1));
            if (fabs(vdist(p2, co_Lin_Int) - r2) <= MAXZERO &&
                fabs(vdist(p3, co_Lin_Int) - r3) <= MAXZERO) {
                /* If the condition is TRUE, co_Lin_Int is the only intersection point. */
                *result1 = co_Lin_Int;
                *result2 = co_Lin_Int;
                return TRIL_3SPHERES;
            }

            /* Is point p1 - (r1 along the axis) the intersection ? */
            co_Lin_Int = vsum(p1, vmul(ex, -r1));
            if (fabs(vdist(p2, co_Lin_Int) - r2) <= MAXZERO &&
                fabs(vdist(p3, co_Lin_Int) - r3) <= MAXZERO) {
                /* If the condition is TRUE, co_Lin_Int is the only intersection point. */
                *result1 = co_Lin_Int;
                *result2 = co_Lin_Int;
                return TRIL_3SPHERES;
            }
            /* If any of the conditions above are not TRUE, it means that
            p1, p2 and p3 are co-linear with more than ONE solution */
            return ERR_TRIL_COLINEAR_2SOLUTIONS;
        }
        /*------------------------------END OF FIND SOLUTIONS FOR COLINEAR SPHERES----------------------------*/

        /*------------------------------FIND RESULT1 AND RESULT2 WITH EQUATIONS-------------------------------*/
        /* ex = (p2 - p1) / |p2 - p1| */
        /* ey = (p1_3_Y) / |p1_3_Y| */
        /* ez = ex X ey */
        Coord3D ez = cross(ex, ey); // Unit vector ez with respect to p1 (origin of the relative coordinate system)
        double d = vdist(p1, p2); // norm of the vector p12
        double x = (r1 * r1 - r2 * r2) / (2 * d) + d / 2;
        double y = (r1 * r1 - r3 * r3 + i * i) / (2 * j) + j / 2 - x * i / j;
        double z_squared = r1 * r1 - x * x - y * y;
        double z;
        if (z_squared < -MAXZERO) {
            /* The solution is invalid, square root of negative number */
            return ERR_TRIL_SQRTNEGNUMB;
        } else if (z_squared > 0.0) {
            z = sqrt(z_squared);
        } else {
            /* Approximation */
            z = 0.0;
        }
        /* Solutions for any coordinate system */
        /* Temporary variable temp */
        /* temp = p1 + x ex + y ey */
        Coord3D temp = vsum(p1, vmul(ex, x));
        temp = vsum(temp, vmul(ey, y));
        /* result1 = p1 + x ex + y ey + z ez */
        *result1 = vsum(temp, vmul(ez, z));

        /* result2 = p1 + x ex + y ey - z ez */
        *result2 = vsum(temp, vmul(ez, -z));

        /*------------------------------ END OF FIND RESULT1 AND RESULT2 WITH EQUATIONS ------------------------*/
        /******************************* END OF FINDING TWO POINTS FROM THE FIRST THREE SPHERES ****************/

        /******************************* FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE *******************/

        if (concentric(p1, p4) || concentric(p2, p4) || concentric(p3, p4)) {
            return TRIL_3SPHERES;
        }


        // If sphere 4 is not concentric to any spheres, then we can obtain the best solution
        *best_solution = trilateration_4_spheres(*result1, *result2, p4, r4);
        return TRIL_4SPHERES;

        /********************************** END OF FINDING ONE SOLUTION BY INTRODUCING ONE MORE SPHERE *********/
    }

    int manager_3D(Coord3D *const solution1, Coord3D *const solution2, Coord3D *const best_solution,
                   int *const nosolution_count, double *const best_3derror, double *const best_gdoprate,
                   Coord3D p1, double r1, Coord3D p2, double r2, Coord3D p3, double r3, Coord3D p4, double r4,
                   int *combination) {

        int combination_counter = 4; /* Four spheres combination */

        *best_gdoprate = 1; /* put the worst gdoprate init */
        double gdoprate_compare1 = 1;
        double gdoprate_compare2 = 1;

        int result;

        do {
            Coord3D o1, o2, solution;
            bool success;
            int overlook_count = 0;
            double ovr_r1 = r1;
            double ovr_r2 = r2;
            double ovr_r3 = r3;
            double ovr_r4 = r4;

            do {
                result = trilateration(&o1, &o2, &solution, p1, ovr_r1, p2, ovr_r2, p3, ovr_r3, p4, ovr_r4);
                success = result == TRIL_3SPHERES || result == TRIL_4SPHERES;

                if (result != TRIL_3SPHERES && result != TRIL_4SPHERES && result != ERR_TRIL_CONCENTRIC) {
                    ovr_r1 += 0.05;
                    ovr_r2 += 0.05;
                    ovr_r3 += 0.05;
                    ovr_r4 += 0.05;
                    overlook_count++;
                }
            } while (!success && (overlook_count <= 5) && result != ERR_TRIL_CONCENTRIC);


            if (success) {
                switch (result) {
                    case TRIL_3SPHERES:
                        *solution1 = o1;
                        *solution2 = o2;
                        *nosolution_count = overlook_count;

                        combination_counter = 0;
                        break;

                    case TRIL_4SPHERES:
                        /* calculate the new gdop */
                        gdoprate_compare1 = gdoprate(solution, p1, p2, p3);

                        /* compare and swap with the better result */
                        if (gdoprate_compare1 <= gdoprate_compare2) {

                            *solution1 = o1;
                            *solution2 = o2;
                            *best_solution = solution;
                            *nosolution_count = overlook_count;
                            *best_3derror = sqrt((vdist(solution, p1) - r1) * (vdist(solution, p1) - r1) +
                                                 (vdist(solution, p2) - r2) * (vdist(solution, p2) - r2) +
                                                 (vdist(solution, p3) - r3) * (vdist(solution, p3) - r3) +
                                                 (vdist(solution, p4) - r4) * (vdist(solution, p4) - r4));
                            *best_gdoprate = gdoprate_compare1;

                            /* save the previous result */
                            gdoprate_compare2 = gdoprate_compare1;

                            *combination = 5 - combination_counter;

                            Coord3D ptemp = p1;
                            p1 = p2;
                            p2 = p3;
                            p3 = p4;
                            p4 = ptemp;
                            double rtemp = r1;
                            r1 = r2;
                            r2 = r3;
                            r3 = r4;
                            r4 = rtemp;
                            combination_counter--;
                        }
                        break;

                    default:
                        break;
                }
            } else {
                combination_counter = 0;
            }

        } while (combination_counter);

        return result;
    }

    int get_location(Coord3D *best_solution, const bool use4thAnchor, Coord3D *anchorArray, int *distanceArray) {
        Coord3D p1 = anchorArray[0];
        Coord3D p2 = anchorArray[1];
        Coord3D p3 = anchorArray[2];

        double r1 = (double) distanceArray[0] / 1000.0;
        double r2 = (double) distanceArray[1] / 1000.0;
        double r3 = (double) distanceArray[2] / 1000.0;

        Coord3D o1, o2;
        int error, combination;
        double best_3derror, best_gdoprate;

        // TODO Use fourth anchor if possible ?
        int result = manager_3D(&o1, &o2, best_solution, &error, &best_3derror, &best_gdoprate,
                                p1, r1, p2, r2, p3, r3, p1, r1, &combination);

        // TODO guessing if TRIL4SPHERES has been used could be done here
        // TODO The best solution with 4 spheres would not be used..
        if (result > 0) {
            //if have 4 ranging results, then use 4th anchor to pick solution closest to it
            if (use4thAnchor) {
                Coord3D p4 = anchorArray[3];
                double r4 = (double) distanceArray[3] / 1000.0;

                double dist1 = vdist(o1, p4);
                double dist2 = vdist(o2, p4);

                /* find the distance closest to received range measurement from 4th anchor */
                double diff1 = fabs(r4 - dist1);
                double diff2 = fabs(r4 - dist2);

                /* pick the closest match to the 4th anchor range */
                if (diff1 < diff2) {
                    *best_solution = o1;
                } else {
                    *best_solution = o2;
                }
            } else {
                // assume tag is below the anchors (1, 2, and 3)
                // FIXME This only checks one anchor. Perfect solution would be to check if below the plane instead.
                if (o1.z < p1.z) {
                    *best_solution = o1;
                } else {
                    *best_solution = o2;
                }
            }
        }

        return result;
    }

}

