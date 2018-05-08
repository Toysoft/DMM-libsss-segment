AC_INIT([libsss-segment],[0.1],[martin@hirte.info])
AM_INIT_AUTOMAKE([foreign])

AC_CONFIG_SRCDIR([src/libsss-segment.c])

m4_ifdef([AM_SILENT_RULES], [AM_SILENT_RULES([yes])])
AC_USE_SYSTEM_EXTENSIONS
AC_SYS_LARGEFILE

dnl find a compiler
AC_PROG_CC
AC_PROG_CC_STDC

dnl check if the compiler supports '-c' and '-o' options
AM_PROG_CC_C_O

# Checks for header files.
AC_CHECK_HEADERS([stdio.h stdlib.h stdint.h inttypes.h string.h])

# Checks for typedefs, structures, and compiler characteristics.
AC_TYPE_SIZE_T

LT_INIT

AC_CONFIG_FILES([
Makefile
src/Makefile
])
AC_OUTPUT
