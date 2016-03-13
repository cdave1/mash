AM_CPPFLAGS = -I$(top_srcdir)/include -I$(top_srcdir) -I$(top_srcdir)/include/math_scraps.h

AM_CFLAGS = $(GLOBAL_CFLAGS) -I$(top_srcdir)/include/math_scraps.h -I$(top_srcdir)/include/common.h

lib_LTLIBRARIES = libmathscraps.la
libmathscraps_la_LDFLAGS = $(AM_LDFLAGS) -version-info $(LIBMATHSCRAPS_VERSION) -no-undefined
libmathscraps_la_SOURCES = \
		AABB.h \
		Collision3.h \
		Color4f.h \
		Color4ub.h \
		CompressedVertex.h \
		Constants.h \
		Frustum.h \
		Functions.h \
		Geometry.h \
		Logger.h \
		Logger.cpp \
		Matrix.cpp
		Matrix.h \
		Plane.h \
		Quaternion.h \
		Size2.h \
		Size3.h \
		Size4.h \
		Triangle.h \
		Vec2.h \
		Vec3.h \
		Vec4.h \
		Vertex.h
