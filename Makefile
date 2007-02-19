#	$LAAS: Makefile,v 1.1 2007/02/19 16:18:04 fcamps Exp $

prefix=		/usr/local/openrobots
exec_prefix=	${prefix}
top_srcdir=	.

INSTALL=	/usr/bin/install -c
MKINSTALLDIRS=	/bin/sh ./mkinstalldirs
LIBDIR=		${exec_prefix}/lib

SUBDIRS=	MTI src test

all:
	@for d in $(SUBDIRS); do \
		$(MAKE) -C $$d all || exit 2; \
	done

depend:
	@for d in $(SUBDIRS); do \
		$(MAKE) -C $$d depend || exit 2; \
	done

install:: MTI.pc
	$(MKINSTALLDIRS) $(LIBDIR)/pkgconfig
	$(INSTALL) MTI.pc $(LIBDIR)/pkgconfig

install::
	@for d in $(SUBDIRS); do \
		$(MAKE) -C $$d install ;\
	done

clean:
	@for d in $(SUBDIRS); do \
		$(MAKE) -C $$d clean ;\
	done

distclean: clean
	@for d in $(SUBDIRS); do \
		$(MAKE) -C $$d distclean ;\
	done
	rm -f Makefile libtool config.log config.status MTI.pc
