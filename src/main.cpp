
#include "shared-position.h"
#include "clip-mesh.h"

int main()
{
	fuzzybools::Geometry geom1;
	fuzzybools::Geometry geom2;

	fuzzybools::SharedPosition sp;
	sp.AddGeometry(geom1, true);
	sp.AddGeometry(geom2, false);

	auto bvh1 = fuzzybools::MakeBVH(geom1);
	auto bvh2 = fuzzybools::MakeBVH(geom2);

	auto geom = sp.Normalize();

	fuzzybools::boolJoin(geom, bvh1, bvh2);
}