AC_REVISION($LAAS: configure.ac,v 1.4 2008/07/17 16:16:47 adegroot Exp $)

AC_INIT(MTI-clients,0.2)
AC_PREREQ(2.59)

AC_CANONICAL_HOST
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_PROG_INSTALL

dnl check for LAAS mkdep
AC_PATH_PROG(MKDEP, mkdep, no, $prefix/bin:$exec_prefix/bin:$PATH)

AC_OUTPUT(Makefile
	MTI/Makefile
	src/Makefile
	test/Makefile
	MTI-clients.pc)
