/*	$OpenBSD: test___fpending.c,v 1.1 2025/05/25 00:20:54 yasuoka Exp $	*/

/*
 * Copyright (c) 2025 YASUOKA Masahiko <yasuoka@yasuoka.net>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <assert.h>
#include <stdio.h>
#include <stdio_ext.h>
#include <stdlib.h>

/* we use assert() */
#undef	NDEBUG

#define	TMPFILENAME	"test___fpending.tmp"

void test___fpending0(void);

void
test___fpending0(void)
{
	FILE	*fp;
	int	 r;
	size_t	 s;

	fp = fopen(TMPFILENAME, "w");
	assert(fp != NULL);
	r = fputs("Hello world", fp);
	assert(r >= 0);
	s = __fpending(fp);
	assert(s > 0);		/* assume buffered */
	r = fflush(fp);
	assert(r == 0);
	s = __fpending(fp);
	assert(s == 0);		/* buffer must be 0 */
	r = fclose(fp);
	assert(r == 0);
}

int
main(int argc, char *argv[])
{
	test___fpending0();

	exit(0);
}
