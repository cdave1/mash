#ifndef _MATH_SCRAPS_TRIANGLE_H_
#define _MATH_SCRAPS_TRIANGLE_H_

#include <Vertex.h>

struct Triangle {
	Triangle () {
        vertices[0].Reset();
        vertices[1].Reset();
        vertices[2].Reset();
    }

    Vertex vertices[3];
};



#endif
