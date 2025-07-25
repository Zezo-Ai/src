#	$OpenBSD: Makefile,v 1.140 2025/07/04 07:52:17 djm Exp $

OPENSSL?=	yes

# Unit tests require OpenSSL.
.if !defined(SKIP_UNIT) && ${OPENSSL:L} == yes
SUBDIR=		unittests
.endif
SUBDIR+=	misc

REGRESS_SETUP_ONCE=misc	# For sk-dummy.so

# Key conversion operations are not supported when built w/out OpenSSL.
.if !defined(LTESTS_FROM) && ${OPENSSL:L} != no
REGRESS_TARGETS=	t1 t2 t3 t4 t5 t7 t9 t10 t11 t12
.endif

LTESTS= 	connect \
		proxy-connect \
		sshfp-connect \
		connect-privsep \
		connect-uri \
		proto-version \
		proto-mismatch \
		exit-status \
		exit-status-signal \
		envpass \
		transfer \
		banner \
		rekey \
		dhgex \
		stderr-data \
		stderr-after-eof \
		broken-pipe \
		try-ciphers \
		yes-head \
		login-timeout \
		agent \
		agent-getpeereid \
		agent-timeout \
		agent-ptrace \
		agent-subprocess \
		keyscan \
		keygen-change \
		keygen-comment \
		keygen-convert \
		keygen-knownhosts \
		keygen-moduli \
		keygen-sshfp \
		key-options \
		scp \
		scp3 \
		scp-uri \
		sftp \
		sftp-chroot \
		sftp-cmds \
		sftp-badcmds \
		sftp-batch \
		sftp-glob \
		sftp-perm \
		sftp-resume \
		sftp-uri \
		reconfigure \
		dynamic-forward \
		forwarding \
		multiplex \
		reexec \
		brokenkeys \
		sshcfgparse \
		cfgparse \
		cfgmatch \
		cfgmatchlisten \
		percent \
		addrmatch \
		localcommand \
		forcecommand \
		portnum \
		keytype \
		kextype \
		cert-hostkey \
		cert-userkey \
		host-expand \
		keys-command \
		forward-control \
		integrity \
		krl \
		multipubkey \
		limit-keytype \
		hostkey-agent \
		hostkey-rotate \
		principals-command \
		cert-file \
		cfginclude \
		servcfginclude \
		allow-deny-users \
		authinfo \
		sshsig \
		knownhosts \
		knownhosts-command \
		agent-restrict \
		hostbased \
		password \
		channel-timeout \
		connection-timeout \
		match-subsystem \
		agent-pkcs11-restrict \
		agent-pkcs11-cert \
		penalty \
		penalty-expire \
		connect-bigconf

INTEROP_TESTS=	putty-transfer putty-ciphers putty-kex conch-ciphers
INTEROP_TESTS+=	dropbear-ciphers dropbear-kex dropbear-server
#INTEROP_TESTS+=ssh-com ssh-com-client ssh-com-keygen ssh-com-sftp

EXTRA_TESTS=	agent-pkcs11
#EXTRA_TESTS+= 	cipher-speed

USERNAME!=	id -un
CLEANFILES+=	*.core actual agent-key.* authorized_keys_${USERNAME} \
		authorized_keys_${USERNAME}.* authorized_principals_${USERNAME} \
		banner.in banner.out cert_host_key* cert_user_key* \
		copy.1 copy.2 data ed25519-agent ed25519-agent* \
		ed25519-agent.pub empty.in expect failed-regress.log \
		failed-ssh.log failed-sshd.log hkr.* host.ecdsa-sha2-nistp256 \
		host.ecdsa-sha2-nistp384 host.ecdsa-sha2-nistp521 \
		host.ssh-ed25519 host.ssh-rsa \
		host_* host_ca_key* host_krl_* host_revoked_* key.* \
		key.ecdsa-* key.ed25519-512 key.ed25519-512.pub \
		key.rsa-* keys-command-args kh.* known_hosts askpass \
		known_hosts-cert known_hosts.* krl-* ls.copy modpipe \
		netcat pidfile putty.rsa2 ready regress.log remote_pid \
		revoked-* rsa rsa-agent rsa-agent.pub rsa.pub rsa_ssh2_cr.prv \
		rsa_ssh2_crnl.prv scp-ssh-wrapper.exe \
		scp-ssh-wrapper.scp setuid-allowed sftp-server.log \
		sftp-server.sh sftp.log ssh-log-wrapper.sh ssh.log \
		ssh-agent.log ssh-add.log slow-sftp-server.sh \
		ssh-rsa_oldfmt knownhosts_command \
		ssh_config ssh_config.* ssh_proxy ssh_proxy_bak \
		ssh_proxy_* sshd.log sshd_config sshd_config.* \
		sshd_proxy sshd_proxy.* sshd_proxy_bak sshd_proxy_orig \
		t10.out t10.out.pub t12.out t12.out.pub t2.out t3.out \
		t6.out1 t6.out2 t7.out t7.out.pub t8.out t8.out.pub \
		t9.out t9.out.pub \
		timestamp testdata user_*key* user_ca* user_key*

# Enable all malloc(3) randomisations and checks
TEST_ENV=      "MALLOC_OPTIONS=CFGJRSUX"

t1:
	ssh-keygen -if ${.CURDIR}/rsa_ssh2.prv | diff - ${.CURDIR}/rsa_openssh.prv
	tr '\n' '\r' <${.CURDIR}/rsa_ssh2.prv > ${.OBJDIR}/rsa_ssh2_cr.prv
	ssh-keygen -if ${.OBJDIR}/rsa_ssh2_cr.prv | diff - ${.CURDIR}/rsa_openssh.prv
	awk '{print $$0 "\r"}' ${.CURDIR}/rsa_ssh2.prv > ${.OBJDIR}/rsa_ssh2_crnl.prv
	ssh-keygen -if ${.OBJDIR}/rsa_ssh2_crnl.prv | diff - ${.CURDIR}/rsa_openssh.prv

t2:
	cat ${.CURDIR}/rsa_openssh.prv > t2.out
	chmod 600 t2.out
	ssh-keygen -yf t2.out | diff - ${.CURDIR}/rsa_openssh.pub

t3:
	ssh-keygen -ef ${.CURDIR}/rsa_openssh.pub |\
		ssh-keygen -if /dev/stdin |\
		diff - ${.CURDIR}/rsa_openssh.pub

t4:
	ssh-keygen -E md5 -lf ${.CURDIR}/rsa_openssh.pub |\
		awk '{print $$2}' | diff - ${.CURDIR}/t4.ok

t5:
	ssh-keygen -Bf ${.CURDIR}/rsa_openssh.pub |\
		awk '{print $$2}' | diff - ${.CURDIR}/t5.ok

t7.out:
	ssh-keygen -q -t rsa -N '' -f $@ ; \

t7: t7.out
	ssh-keygen -lf t7.out > /dev/null
	ssh-keygen -Bf t7.out > /dev/null

t9.out:
	ssh-keygen -q -t ecdsa -N '' -f $@

t9: t9.out
	ssh-keygen -lf t9.out > /dev/null
	ssh-keygen -Bf t9.out > /dev/null

t10.out:
	ssh-keygen -q -t ed25519 -N '' -f $@

t10: t10.out
	ssh-keygen -lf t10.out > /dev/null
	ssh-keygen -Bf t10.out > /dev/null

t11:
	ssh-keygen -E sha256 -lf ${.CURDIR}/rsa_openssh.pub |\
		awk '{print $$2}' | diff - ${.CURDIR}/t11.ok

t12.out:
	ssh-keygen -q -t ed25519 -N '' -C 'test-comment-1234' -f $@

t12: t12.out
	ssh-keygen -lf t12.out.pub | grep -q test-comment-1234

modpipe: modpipe.c

timestamp: timestamp.c

t-integrity: modpipe

.for t in ${LTESTS} ${INTEROP_TESTS} ${EXTRA_TESTS}
t-${t}: timestamp
	env SUDO="${SUDO}" ${TEST_ENV} \
	    sh ${.CURDIR}/test-exec.sh ${.OBJDIR} ${.CURDIR}/${t}.sh
.endfor

.undef LTESTS_STARTED
.for t in ${LTESTS}
.if defined(LTESTS_FROM) && ${LTESTS_FROM} == t-${t}
LTESTS_STARTED=yes
.endif
.if !defined(LTESTS_FROM) || defined(LTESTS_STARTED)
REGRESS_TARGETS+=t-${t}
.endif
.endfor

# Not run by default
.if make(interop)

.for t in ${INTEROP_TESTS}
INTEROP_TARGETS+=t-${t}
.endfor

REGRESS_TARGETS=${INTEROP_TARGETS}
SUBDIR=

interop: regress

.endif

# Not run by default
.if make(extra)

.for t in ${EXTRA_TESTS}
EXTRA_TARGETS+=t-${t}
.endfor

REGRESS_TARGETS=${EXTRA_TARGETS}
SUBDIR=

extra: regress

.endif

.for s in ${SUBDIR}
CLEAN_SUBDIR+=c-${s}
c-${s}:
	${MAKE} -C ${.CURDIR}/${s} clean
.endfor

clean: ${CLEAN_SUBDIR}
	rm -f ${CLEANFILES}
	rm -rf .putty
	rm -rf .dropbear

.include <bsd.regress.mk>
