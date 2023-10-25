# fuzzy-bools

*Warning: this is pre-alpha software, awaiting a proper benchmark. There is no guarantee of stability.*

![Rabbits.](/demo/rabbits.png)

Fuzzy bools is a 3D boolean lib specialized for the CAD boolean domain. Its main use is inside web-ifc, where it solves problems with coplanar surfaces and tolerances that other libraries overlook because they are not relevant to most domains.

## Usage

This is a header only lib with two dependencies: `glm` and `cdt`, both can be found checked in to this repo under `/deps`.

The main entry point of the library can be used like this:

```C++
#include "fuzzy-bools.h"

fuzzybools::Geometry A;
fuzzybools::Geometry B;

// fill A and B with your geometry data
// An easy way to do this is with Geometry::AddFace()

// subtract A from B
auto result = fuzzybools::Subtract(A, B);

// result now contains a triangulated mesh
```