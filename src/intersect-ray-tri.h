/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#pragma once

#include <glm.hpp>

#include "eps.h"
#include "math.h"

namespace fuzzybools
{

    // https://www.iquilezles.org/www/articles/intersectors/intersectors.htm
    static bool intersect_ray_triangle(
        const glm::dvec3& origin, const glm::dvec3& end, const glm::dvec3& A, const glm::dvec3& B, const glm::dvec3& C, glm::dvec3& out, double& t, bool infiniteLength = false
    ) {
        double EPS = EPS_SMALL;

        glm::dvec3 dir = end - origin;
        glm::dvec3 E1 = B - A;
        glm::dvec3 E2 = C - A;
        glm::dvec3 ROV0 = origin - A;
        glm::dvec3 N = glm::cross(E1, E2);
        glm::dvec3 Q = glm::cross(ROV0, dir);
        double d = dot(dir, N);
        if (std::fabs(d) < EPS)
        {
            // we don't properly handle this case:
            // !!!!!!!!!!!!!! edge AB on plane DEF where B is outside DEF
            return false;
        }
        double det = 1.0 / d;
        double u = det * glm::dot(E2, (Q * -1.0));
        double v = det * glm::dot(E1, Q);
        double w = 1 - u - v;
        t = det * glm::dot(ROV0, (N * -1.0));

        // TODO: this fuzz is quite serious
        if (t > 1 - EPS && t < 1 + EPS && !infiniteLength)
        {
            t = 1;
        }

        // TODO: this fuzz is quite serious
        if (t < EPS && t > 0 - EPS && !infiniteLength)
        {
            t = 0;
        }

        if (infiniteLength && t < EPS && t > -EPS)
        {
            t = 0;
        }

        //log(t, u, v, det);
        bool result = (t >= 0.0 - EPS && u >= 0.0 - EPS && v >= 0.0 - EPS && (u + v) <= 1.0 + EPS && (t <= 1 + EPS || infiniteLength));
        if (result)
        {
            glm::dvec3 vec = origin + (dir * t);
#ifdef _DEBUG
            if (!infiniteLength)
            {
                glm::dvec3 bary(w, u, v);
                glm::dvec3 vecBary = bary.x * A + bary.y * B + bary.z * C;
                if (!equals(vec, vecBary, EPS_SMALL))
                {
                    printf("bad bary conversion\n");
                }
                glm::dvec3 reconstr = ToBary(A, B, C, vec);
                if (!equals(bary, reconstr, EPS_SMALL))
                {
                    printf("bad bary conversion\n");
                }
            }
#endif
            out = vec;
            return true;
        }
        else
        {
            return false;
        }
    }
}