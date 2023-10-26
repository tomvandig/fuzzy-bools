#pragma once

#include <glm/glm.hpp>
#include <algorithm>

#include "eps.h"

namespace fuzzybools
{
	struct AABB
	{
		uint32_t index;
		glm::dvec3 min = glm::dvec3(DBL_MAX, DBL_MAX, DBL_MAX);
		glm::dvec3 max = glm::dvec3(-DBL_MAX, -DBL_MAX, -DBL_MAX);
		glm::dvec3 center = glm::dvec3();

		bool intersects(const AABB& other) const
		{
			double eps = EPS_BIG;
			return (max.x + eps >= other.min.x && other.max.x + eps >= min.x &&
				max.y + eps >= other.min.y && other.max.y + eps >= min.y &&
				max.z + eps >= other.min.z && other.max.z + eps >= min.z);
		}

		bool contains(const glm::dvec3& pos) const
		{
			double eps = EPS_BIG;
			return pos.x + eps >= min.x && pos.x - eps <= max.x &&
				pos.y + eps >= min.y && pos.y - eps <= max.y &&
				pos.z + eps >= min.z && pos.z - eps <= max.z;
		}

		void merge(const AABB& other)
		{
			min = glm::min(min, other.min);
			max = glm::max(max, other.max);
		}

		void merge(const glm::dvec3& other)
		{
			min = glm::min(min, other);
			max = glm::max(max, other);
		}

		// https://gamedev.stackexchange.com/questions/18436/most-efficient-aabb-vs-ray-collision-algorithms
		bool Intersect(const glm::dvec3& origin, const glm::dvec3& dir) const
		{// r.dir is unit direction vector of ray
			glm::dvec3 dirfrac;
			dirfrac.x = 1.0f / dir.x;
			dirfrac.y = 1.0f / dir.y;
			dirfrac.z = 1.0f / dir.z;
			// lb is the corner of AABB with minimal coordinates - left bottom, rt is maximal corner
			// r.org is origin of ray
			double t1 = (min.x - origin.x) * dirfrac.x;
			double t2 = (max.x - origin.x) * dirfrac.x;
			double t3 = (min.y - origin.y) * dirfrac.y;
			double t4 = (max.y - origin.y) * dirfrac.y;
			double t5 = (min.z - origin.z) * dirfrac.z;
			double t6 = (max.z - origin.z) * dirfrac.z;

			double tmin = std::max(std::max(std::min(t1, t2), std::min(t3, t4)), std::min(t5, t6));
			double tmax = std::min(std::min(std::max(t1, t2), std::max(t3, t4)), std::max(t5, t6));

			// if tmax < 0, ray (line) is intersecting AABB, but the whole AABB is behind us
			if (tmax < -EPS_BIG)
			{
				//t = tmax;
				return false;
			}

			// if tmin > tmax, ray doesn't intersect AABB
			if (tmin > tmax + EPS_BIG)
			{
				//t = tmax;
				return false;
			}

			//t = tmin;
			return true;
		}
	};
}