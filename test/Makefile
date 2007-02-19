#	$LAAS: Makefile,v 1.1 2007/02/19 16:18:05 fcamps Exp $
#
# Copyright (c) 2005 CNRS/LAAS
#
# Permission to use, copy, modify, and distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

prefix=		/usr/local/openrobots
exec_prefix=	${prefix}
srcdir=		.
builddir=	.
top_srcdir=	..
top_builddir=	..

BINDIR=		${exec_prefix}/bin
LIBDIR=		${exec_prefix}/lib

MKDEP=		/usr/local/bin/mkdep
CC=		gcc
CXX=		g++
LIBTOOL=	$(SHELL) $(top_builddir)/libtool
INSTALL=	/usr/bin/install -c
MKINSTALLDIRS=	/bin/sh ${top_srcdir}/mkinstalldirs

CPPFLAGS+=	-I${top_srcdir} 

ifeq (${CC},gcc)
CFLAGS+= 	-Wall
endif

SRCS=		test.cpp

OBJS=		$(SRCS:%.cpp=%.lo)


vpath %.c $(srcdir)

PROG=		MTITest

all: $(PROG)

$(PROG): $(OBJS)
	$(LIBTOOL) --mode=link $(CXX) -o $@ $(OBJS) -rpath $(LIBDIR) \
	-L../src -lMTI -lrt

%.lo: %.cpp
	$(LIBTOOL) --mode=compile $(CXX) -c $(CPPFLAGS) $(CFLAGS) $<

ifeq ($(findstring clean,$(MAKECMDGOALS)),)
depend DEPENDANCES: $(SRCS)
	$(MKDEP) -c$(CXX) -oDEPENDANCES -t.lo $(CPPFLAGS) $?
endif

clean:
	$(LIBTOOL) --mode=clean rm -f $(OBJS) $(PROG)

distclean: clean
	rm -f DEPENDANCES Makefile

install: $(PROG)
	$(MKINSTALLDIRS) $(BINDIR)
	$(LIBTOOL) --mode=install $(INSTALL) $(PROG) $(BINDIR)

include DEPENDANCES
