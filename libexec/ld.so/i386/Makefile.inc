#	$OpenBSD: Makefile.inc,v 1.4 2025/05/08 13:57:23 deraadt Exp $

CFLAGS += -fPIC
AFLAGS += -fpic
LD_SCRIPT = ${.CURDIR}/${MACHINE_CPU}/ld.script
STACK_PROTECTOR=

RELATIVE_RELOC=R_386_RELATIVE
