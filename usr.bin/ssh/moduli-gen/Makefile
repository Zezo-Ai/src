#	$OpenBSD: Makefile,v 1.7 2025/06/16 09:09:42 dtucker Exp $

.include <bsd.own.mk>

# The larger ones will take many days, so if you're going to regen them run
# it in a tmux session or something.  The checkpoints should make it safe
# to stop and restart.
# The sizes match those in dh.c:dh_estimate() plus some historic sizes.
DHSIZE=2048 3072 4096 6144 7680 8192

.for bits in ${DHSIZE}
MODULI_PARTS+=moduli.${bits}

all:	${MODULI_PARTS}

moduli.${bits}:
	sh ${.CURDIR}/moduli-gen.sh ${.CURDIR} ${.CURDIR}/work ${bits}
.endfor

update-moduli:	${MODULI_PARTS}
	( \
		echo -n '#    $$Open'; echo 'BSD$$'; \
		echo '# Time Type Tests Tries Size Generator Modulus'; \
		for i in ${MODULI_PARTS}; do head -100 $$i; done \
	) > ${BSDSRCDIR}/etc/moduli

clean:
.for bits in ${DHSIZE}
	rm -f ${.OBJDIR}/moduli.${bits}.sieved.gz
.endfor
