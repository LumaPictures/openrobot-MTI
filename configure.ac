AC_REVISION($LAAS: configure.ac,v 1.5 2009/08/05 11:23:44 adegroot Exp $)

AC_INIT(MTI-clients,0.4)
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
