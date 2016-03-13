AM_CPPFLAGS = -I$(top_srcdir)/include -I$(top_srcdir)/src -I$(top_srcdir) -I$(top_srcdir)/include/math_scraps.h

AM_CFLAGS = $(GLOBAL_CFLAGS) -I$(top_srcdir)/include/math_scraps.h -I$(top_srcdir)/include/common.h

bin_PROGRAMS = test

test_SOURCES = main.cpp test.cpp
test_CFLAGS = -I$(top_srcdir) $(AM_CFLAGS)
test_LDFLAGS = $(AM_LDFLAGS)
test_LDADD = $(top_builddir)/src/libmathscraps.la
