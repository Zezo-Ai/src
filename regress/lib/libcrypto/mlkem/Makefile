#       $OpenBSD: Makefile,v 1.4 2024/12/19 23:45:09 tb Exp $

PROGS +=        mlkem_unittest
PROGS +=        mlkem768_nist_keygen_tests
PROGS +=        mlkem768_keygen_tests
PROGS +=        mlkem768_nist_decap_tests
PROGS +=        mlkem768_decap_tests
PROGS +=        mlkem768_encap_tests
PROGS +=        mlkem768_iteration_test
PROGS +=        mlkem1024_nist_keygen_tests
PROGS +=        mlkem1024_keygen_tests
PROGS +=        mlkem1024_nist_decap_tests
PROGS +=        mlkem1024_decap_tests
PROGS +=        mlkem1024_encap_tests
PROGS +=        mlkem1024_iteration_test

# Link test programs with mlkem_tests_util.c and use custom target
.for p in ${PROGS}
SRCS_$p +=      $p.c mlkem_tests_util.c

REGRESS_TARGETS += run-$p
run-$p: $p
	./$p ${.CURDIR}/$p.txt
.endfor

LDADD =         ${CRYPTO_INT}
DPADD =         ${LIBCRYPTO}

CFLAGS +=       -DLIBRESSL_INTERNAL -Wall
CFLAGS +=       -I${.CURDIR}/../../../../lib/libcrypto/bytestring
CFLAGS +=       -I${.CURDIR}/../../../../lib/libcrypto/mlkem
CFLAGS +=       -I${.CURDIR}/../../../../lib/libcrypto/sha
CFLAGS +=       -DLIBRESSL_INTERNAL

.include <bsd.regress.mk>
