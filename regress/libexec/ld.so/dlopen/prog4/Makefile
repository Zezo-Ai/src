# $OpenBSD: Makefile,v 1.4 2025/01/21 19:14:28 anton Exp $

.include <bsd.obj.mk>

PROG=prog4

SRCS=main.c

AA_DIR=${.CURDIR}/../libaa
AA_OBJDIR!=    if [ -d $(AA_DIR)/${__objdir} ]; then \
                        echo "$(AA_DIR)/${__objdir}"; \
		else \
			echo "$(AA_DIR)"; \
		fi

AC_DIR=${.CURDIR}/../libac
AC_OBJDIR!=    if [ -d $(AC_DIR)/${__objdir} ]; then \
                        echo "$(AC_DIR)/${__objdir}"; \
		else \
			echo "$(AC_DIR)"; \
		fi



LDADD=
#LDADD+=-laa
#LDADD+=-lac
LDFLAGS=-L$(AA_OBJDIR) -L$(AC_OBJDIR)
#LDFLAGS+= -Wl,-rpath,$(AA_OBJDIR)  -Wl,-rpath,$(AC_OBJDIR) 
# This intentionally leaves out AA_OBJDIR from -rpath
LDFLAGS+= -Wl,-rpath,$(AC_OBJDIR) 
NOMAN=
CC=${CXX}

.include <bsd.regress.mk>
