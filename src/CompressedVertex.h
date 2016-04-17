#ifndef _MATH_SCRAPS_COMPRESSED_VERTEX_H_
#define _MATH_SCRAPS_COMPRESSED_VERTEX_H_

#include <Vec2.h>
#include <Vec3.h>
#include <Vertex.h>
#include <Color4f.h>
#include <Color4ub.h>

//
// TODO: MOAR COMPRESHUN.
//
struct CompressedVertex {
	CompressedVertex () {
        origin.Set(0, 0, 0);
        color.Setf(1.0f, 1.0f, 1.0f, 1.0f);
        normal.Set(0, 0, 0);
        texCoord.Set(0, 0);
    }


    CompressedVertex(const Vertex &vertex) {
        origin.Set(vertex.origin);
        color.Setf(vertex.color[0], vertex.color[1], vertex.color[2], vertex.color[3]);
        normal.Set(vertex.normal);
        texCoord.Set(vertex.texCoord);
    }


    void Reset() {
        origin.Set(0, 0, 0);
        color.Setf(1.0f, 1.0f, 1.0f, 1.0f);
        normal.Set(0, 0, 0);
        texCoord.Set(0, 0);
    }


    Vec3 origin;
    Color4ub color;
    Vec3 normal;
    Vec2 texCoord;
};

#endif
