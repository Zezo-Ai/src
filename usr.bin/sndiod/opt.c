/*	$OpenBSD: opt.c,v 1.15 2025/06/20 07:14:38 ratchov Exp $	*/
/*
 * Copyright (c) 2008-2011 Alexandre Ratchov <alex@caoua.org>
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
#include <string.h>

#include "dev.h"
#include "midi.h"
#include "opt.h"
#include "sysex.h"
#include "utils.h"

struct opt *opt_list;

void opt_midi_imsg(void *, unsigned char *, int);
void opt_midi_omsg(void *, unsigned char *, int);
void opt_midi_fill(void *, int);
void opt_midi_exit(void *);

struct midiops opt_midiops = {
	opt_midi_imsg,
	opt_midi_omsg,
	opt_midi_fill,
	opt_midi_exit
};

struct app *
opt_mkapp(struct opt *o, char *who)
{
	char *p;
	char name[APP_NAMEMAX];
	unsigned int i, ser, bestser, bestidx, inuse;
	struct app *a;
	struct slot *s;

	/*
	 * create a valid control name (lowcase, remove [^a-z], truncate)
	 */
	for (i = 0, p = who; ; p++) {
		if (i == APP_NAMEMAX - 1 || *p == '\0') {
			name[i] = '\0';
			break;
		} else if (*p >= 'A' && *p <= 'Z') {
			name[i++] = *p + 'a' - 'A';
		} else if (*p >= 'a' && *p <= 'z')
			name[i++] = *p;
	}
	if (i == 0)
		strlcpy(name, "noname", APP_NAMEMAX);

	/*
	 * return the app with this name (if any)
	 */
	for (i = 0, a = o->app_array; i < OPT_NAPP; i++, a++) {
		if (strcmp(a->name, name) == 0)
			return a;
	}

	/*
	 * build a bitmap of app structures currently in use
	 */
	inuse = 0;
	for (i = 0, s = slot_array; i < DEV_NSLOT; i++, s++) {
		if (s->app != NULL && s->ops != NULL)
			inuse |= 1 << (s->app - o->app_array);
	}

	if (inuse == (1 << OPT_NAPP) - 1) {
		logx(1, "%s: too many programs", name);
		return NULL;
	}

	/*
	 * recycle the oldest free structure
	 */

	o->app_serial++;
	bestser = 0;
	bestidx = OPT_NAPP;
	for (i = 0, a = o->app_array; i < OPT_NAPP; i++, a++) {
		if (inuse & (1 << i))
			continue;
		ser = o->app_serial - a->serial;
		if (ser > bestser) {
			bestser = ser;
			bestidx = i;
		}
	}

	a = o->app_array + bestidx;

	ctl_del(CTL_APP_LEVEL, o, a);

	strlcpy(a->name, name, sizeof(a->name));
	a->serial = o->app_serial;
	a->vol = MIDI_MAXCTL;
	ctl_new(CTL_APP_LEVEL, o, a,
	    CTL_NUM, "", "app", a->name, -1, "level",
	    NULL, -1, 127, a->vol);
	opt_midi_appdesc(o, a);
	opt_midi_vol(o, a);

	return a;
}

void
opt_appvol(struct opt *o, struct app *a, int vol)
{
	struct slot *s;
	int i;

	a->vol = vol;

	for (i = 0, s = slot_array; i < DEV_NSLOT; i++, s++) {
		if (s->app != a || s->opt != o)
			continue;
		s->mix.vol = MIDI_TO_ADATA(vol);
#ifdef DEBUG
		logx(3, "%s/%s: setting volume %u", o->name, a->name, vol);
#endif
	}
}

void
opt_midi_imsg(void *arg, unsigned char *msg, int len)
{
#ifdef DEBUG
	struct opt *o = arg;

	logx(0, "%s: can't receive midi messages", o->name);
	panic();
#endif
}

void
opt_midi_omsg(void *arg, unsigned char *msg, int len)
{
	struct opt *o = arg;
	struct sysex *x;
	unsigned int fps, chan;

	if ((msg[0] & MIDI_CMDMASK) == MIDI_CTL && msg[1] == MIDI_CTL_VOL) {
		chan = msg[0] & MIDI_CHANMASK;
		if (chan >= OPT_NAPP)
			return;
		opt_appvol(o, o->app_array + chan, msg[2]);
		ctl_onval(CTL_APP_LEVEL, o, o->app_array + chan, msg[2]);
		return;
	}
	x = (struct sysex *)msg;
	if (x->start != SYSEX_START)
		return;
	if (len < SYSEX_SIZE(empty))
		return;
	switch (x->type) {
	case SYSEX_TYPE_RT:
		if (x->id0 == SYSEX_CONTROL && x->id1 == SYSEX_MASTER) {
			if (len == SYSEX_SIZE(master)) {
				dev_master(o->dev, x->u.master.coarse);
				if (o->dev->master_enabled) {
					ctl_onval(CTL_DEV_MASTER, o->dev, NULL,
					   x->u.master.coarse);
				}
			}
			return;
		}
		if (x->id0 != SYSEX_MMC)
			return;
		switch (x->id1) {
		case SYSEX_MMC_STOP:
			if (len != SYSEX_SIZE(stop))
				return;
			if (o->mtc == NULL)
				return;
			mtc_setdev(o->mtc, o->dev);
			logx(2, "%s: mmc stop", o->name);
			mtc_stop(o->mtc);
			break;
		case SYSEX_MMC_START:
			if (len != SYSEX_SIZE(start))
				return;
			if (o->mtc == NULL)
				return;
			mtc_setdev(o->mtc, o->dev);
			logx(2, "%s: mmc start", o->name);
			mtc_start(o->mtc);
			break;
		case SYSEX_MMC_LOC:
			if (len != SYSEX_SIZE(loc) ||
			    x->u.loc.len != SYSEX_MMC_LOC_LEN ||
			    x->u.loc.cmd != SYSEX_MMC_LOC_CMD)
				return;
			if (o->mtc == NULL)
				return;
			mtc_setdev(o->mtc, o->dev);
			switch (x->u.loc.hr >> 5) {
			case MTC_FPS_24:
				fps = 24;
				break;
			case MTC_FPS_25:
				fps = 25;
				break;
			case MTC_FPS_30:
				fps = 30;
				break;
			default:
				mtc_stop(o->mtc);
				return;
			}
			mtc_loc(o->mtc,
			    (x->u.loc.hr & 0x1f) * 3600 * MTC_SEC +
			     x->u.loc.min * 60 * MTC_SEC +
			     x->u.loc.sec * MTC_SEC +
			     x->u.loc.fr * (MTC_SEC / fps));
			break;
		}
		break;
	case SYSEX_TYPE_EDU:
		if (x->id0 != SYSEX_AUCAT || x->id1 != SYSEX_AUCAT_DUMPREQ)
			return;
		if (len != SYSEX_SIZE(dumpreq))
			return;
		opt_midi_dump(o);
		break;
	}
}

void
opt_midi_fill(void *arg, int count)
{
	/* nothing to do */
}

void
opt_midi_exit(void *arg)
{
	struct opt *o = arg;

	logx(1, "%s: midi end point died", o->name);
}

/*
 * send a volume change MIDI message
 */
void
opt_midi_vol(struct opt *o, struct app *a)
{
	unsigned char msg[3];

	msg[0] = MIDI_CTL | (a - o->app_array);
	msg[1] = MIDI_CTL_VOL;
	msg[2] = a->vol;
	midi_send(o->midi, msg, sizeof(msg));
}

/*
 * send a sndiod-specific slot description MIDI message
 */
void
opt_midi_appdesc(struct opt *o, struct app *a)
{
	struct sysex x;

	memset(&x, 0, sizeof(struct sysex));
	x.start = SYSEX_START;
	x.type = SYSEX_TYPE_EDU;
	x.dev = SYSEX_DEV_ANY;
	x.id0 = SYSEX_AUCAT;
	x.id1 = SYSEX_AUCAT_SLOTDESC;
	strlcpy(x.u.slotdesc.name, a->name, SYSEX_NAMELEN);
	x.u.slotdesc.chan = (a - o->app_array);
	x.u.slotdesc.end = SYSEX_END;
	midi_send(o->midi, (unsigned char *)&x, SYSEX_SIZE(slotdesc));
}

/*
 * send a MIDI dump: master volume, state of MIDI channels
 */
void
opt_midi_dump(struct opt *o)
{
	struct sysex x;
	struct app *a;
	int i;

	dev_midi_master(o->dev);
	for (i = 0, a = o->app_array; i < OPT_NAPP; i++, a++) {
		opt_midi_appdesc(o, a);
		opt_midi_vol(o, a);
	}
	x.start = SYSEX_START;
	x.type = SYSEX_TYPE_EDU;
	x.dev = SYSEX_DEV_ANY;
	x.id0 = SYSEX_AUCAT;
	x.id1 = SYSEX_AUCAT_DUMPEND;
	x.u.dumpend.end = SYSEX_END;
	midi_send(o->midi, (unsigned char *)&x, SYSEX_SIZE(dumpend));
}

/*
 * create a new audio sub-device "configuration"
 */
struct opt *
opt_new(struct dev *d, char *name,
    int pmin, int pmax, int rmin, int rmax,
    int maxweight, int mmc, int dup, unsigned int mode)
{
	struct dev *a;
	struct opt *o, **po;
	char str[64];
	unsigned int len, num;
	char c;

	if (name == NULL) {
		name = d->name;
		len = strlen(name);
	} else {
		for (len = 0; name[len] != '\0'; len++) {
			if (len == OPT_NAMEMAX) {
				logx(0, "%s: too long", name);
				return NULL;
			}
			c = name[len];
			if ((c < 'a' || c > 'z') &&
			    (c < 'A' || c > 'Z')) {
				logx(0, "%s: only alphabetic chars allowed", name);
				return NULL;
			}
		}
	}
	num = 0;
	for (po = &opt_list; *po != NULL; po = &(*po)->next)
		num++;
	if (num >= OPT_NMAX) {
		logx(0, "%s: too many opts", name);
		return NULL;
	}

	if (opt_byname(name)) {
		logx(1, "%s: already defined", name);
		return NULL;
	}

	if (mmc) {
		if (mtc_array[0].dev != NULL && mtc_array[0].dev != d) {
			logx(0, "%s: MTC already setup for another device", name);
			return NULL;
		}
		mtc_array[0].dev = d;
		logx(2, "%s: initial MTC source, controlled by MMC", d->path);
	}

	if (strcmp(d->name, name) == 0)
		a = d;
	else {
		/* circulate to the first "alternate" device (greatest num) */
		for (a = d; a->alt_next->num > a->num; a = a->alt_next)
			;
	}

	o = xmalloc(sizeof(struct opt));
	o->num = num;
	o->alt_first = o->dev = a;
	o->refcnt = 0;

	/*
	 * XXX: below, we allocate a midi input buffer, since we don't
	 *	receive raw midi data, so no need to allocate a input
	 *	ibuf.  Possibly set imsg & fill callbacks to NULL and
	 *	use this to in midi_new() to check if buffers need to be
	 *	allocated
	 */
	o->midi = midi_new(&opt_midiops, o, MODE_MIDIIN | MODE_MIDIOUT);
	midi_tag(o->midi, o->num);

	if (mode & MODE_PLAY) {
		o->pmin = pmin;
		o->pmax = pmax;
	}
	if (mode & MODE_RECMASK) {
		o->rmin = rmin;
		o->rmax = rmax;
	}
	o->maxweight = maxweight;
	o->mtc = mmc ? &mtc_array[0] : NULL;
	o->dup = dup;
	o->mode = mode;
	memcpy(o->name, name, len + 1);
	o->next = *po;
	*po = o;

	logx(2, "%s: %s%s, vol = %d", o->name, (chans_fmt(str, sizeof(str),
	    o->mode, o->pmin, o->pmax, o->rmin, o->rmax), str),
	    (o->dup) ? ", dup" : "", o->maxweight);

	return o;
}

struct opt *
opt_byname(char *name)
{
	struct opt *o;

	for (o = opt_list; o != NULL; o = o->next) {
		if (strcmp(name, o->name) == 0)
			return o;
	}
	return NULL;
}

struct opt *
opt_bynum(int num)
{
	struct opt *o;

	for (o = opt_list; o != NULL; o = o->next) {
		if (o->num == num)
			return o;
	}
	return NULL;
}

void
opt_del(struct opt *o)
{
	struct opt **po;

	for (po = &opt_list; *po != o; po = &(*po)->next) {
#ifdef DEBUG
		if (*po == NULL) {
			logx(0, "%s: not on list", __func__);
			panic();
		}
#endif
	}
	midi_del(o->midi);
	*po = o->next;
	xfree(o);
}

void
opt_init(struct opt *o)
{
}

void
opt_done(struct opt *o)
{
	struct dev *d;

	if (o->refcnt != 0) {
		// XXX: all clients are already kicked, so this never happens
		logx(0, "%s: still has refs", o->name);
	}
	for (d = dev_list; d != NULL; d = d->next)
		ctl_del(CTL_OPT_DEV, o, d);
}

/*
 * Set opt's device, and (if necessary) move clients to
 * to the new device
 */
int
opt_setdev(struct opt *o, struct dev *ndev)
{
	struct dev *odev;
	struct ctl *c;
	struct ctlslot *p;
	struct slot *s;
	int i;

	if (!dev_ref(ndev))
		return 0;

	odev = o->dev;
	if (odev == ndev) {
		dev_unref(ndev);
		return 1;
	}

	/* check if clients can use new device */
	for (i = 0, s = slot_array; i < DEV_NSLOT; i++, s++) {
		if (s->opt != o)
			continue;
		if (s->ops != NULL && !dev_iscompat(odev, ndev)) {
			dev_unref(ndev);
			return 0;
		}
	}

	/*
	 * if we're using MMC, move all opts to the new device, mtc_setdev()
	 * will call us back
	 *
	 * XXX: move this to the end to avoid the recursion
	 */
	if (o->mtc != NULL && o->mtc->dev != ndev) {
		mtc_setdev(o->mtc, ndev);
		dev_unref(ndev);
		return 1;
	}

	c = ctl_find(CTL_OPT_DEV, o, o->dev);
	if (c != NULL)
		c->curval = 0;

	/* detach clients from old device */
	for (i = 0, s = slot_array; i < DEV_NSLOT; i++, s++) {
		if (s->opt != o)
			continue;

		if (s->pstate == SLOT_RUN || s->pstate == SLOT_STOP)
			slot_detach(s);
	}

	o->dev = ndev;

	if (o->refcnt > 0) {
		dev_unref(odev);
		dev_ref(o->dev);
	}

	c = ctl_find(CTL_OPT_DEV, o, o->dev);
	if (c != NULL) {
		c->curval = 1;
		c->val_mask = ~0;
	}

	/* attach clients to new device */
	for (i = 0, s = slot_array; i < DEV_NSLOT; i++, s++) {
		if (s->opt != o)
			continue;

		if (s->pstate == SLOT_RUN || s->pstate == SLOT_STOP) {
			slot_initconv(s);
			slot_attach(s);
		}
	}

	/* move controlling clients to new device */
	for (p = ctlslot_array, i = 0; i < DEV_NCTLSLOT; i++, p++) {
		if (p->ops == NULL)
			continue;
		if (p->opt == o)
			ctlslot_update(p);
	}

	dev_unref(ndev);
	return 1;
}

/*
 * Get a reference to opt's device
 */
struct dev *
opt_ref(struct opt *o)
{
	struct dev *d;

	if (o->refcnt == 0) {
		if (strcmp(o->name, o->dev->name) == 0) {
			if (!dev_ref(o->dev))
				return NULL;
		} else {
			/* find first working one */
			d = o->alt_first;
			while (1) {
				if (dev_ref(d))
					break;
				d = d->alt_next;
				if (d == o->alt_first)
					return NULL;
			}

			/* if device changed, move everything to the new one */
			if (d != o->dev)
				opt_setdev(o, d);

			/* create server.device control */
			for (d = dev_list; d != NULL; d = d->next) {
				d->refcnt++;
				if (d->pstate == DEV_CFG)
					dev_open(d);
				ctl_new(CTL_OPT_DEV, o, d,
				    CTL_SEL, dev_getdisplay(d),
				    o->name, "server", -1, "device",
				    d->name, -1, 1, o->dev == d);
			}
		}
	}

	o->refcnt++;
	return o->dev;
}

/*
 * Release opt's device
 */
void
opt_unref(struct opt *o)
{
	struct dev *d;

	o->refcnt--;
	if (o->refcnt == 0) {
		/* delete server.device control */
		for (d = dev_list; d != NULL; d = d->next) {
			if (ctl_del(CTL_OPT_DEV, o, d))
				dev_unref(d);
		}
		dev_unref(o->dev);
	}
}
