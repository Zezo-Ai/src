#include <sys/mman.h>
#include <sys/param.h>

#include <err.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PCI_MMIO_BAR_BASE	0xF0000000ULL

static void
handler(int sig, siginfo_t *sip, void *ctx)
{
	printf("signo %d, code %d, errno %d\n", sip->si_signo, sip->si_code,
	    sip->si_errno);
	if (sig != SIGBUS)
		errx(1, "expected SIGBUS: %d", sig);
	if (sip->si_code != BUS_ADRERR)
		errx(1, "expected BUS_ADRERR: %d", sip->si_code);

	exit(0);
}

int
main(int argc, char **argv)
{
	struct sigaction	sa;
	int	 		 fd;
	char			*p;
	volatile char		 v;

	memset(&sa, 0, sizeof(sa));
	sa.sa_sigaction = handler;
	sa.sa_flags = SA_SIGINFO;
	if (sigaction(SIGBUS, &sa, NULL) == -1)
		err(2, "sigaction");

	fd = open("/dev/xf86", O_RDWR);
	if (fd == -1)
		err(2, "open");

	p = mmap(NULL, sizeof(v), PROT_READ, 0, fd, PCI_MMIO_BAR_BASE);
	if (p == MAP_FAILED)
		err(2, "mmap");

	v = *p;

	errx(1, "expected signal");
}
