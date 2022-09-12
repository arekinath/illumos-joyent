#include <sys/cdefs.h>

#include <sys/time.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <endian.h>
#include <port.h>
#include <errno.h>
#include <assert.h>
#include <unistd.h>
#include <limits.h>
#include <libnvpair.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/uio.h>

#include "usb_emul.h"
#include "console.h"
#include "bhyvegc.h"
#include "mevent.h"
#include "config.h"
#include "debug.h"

#define	dprintf(fmt, arg...)	do { \
	time_t t; \
	struct tm dtm; \
	time(&t); \
	localtime_r(&t, &dtm); \
	EPRINTLN("[%04d-%02d-%02d %02d:%02d:%02d] urelay: " fmt, \
	    dtm.tm_year + 1900, dtm.tm_mon + 1, dtm.tm_mday, dtm.tm_hour, \
	    dtm.tm_min, dtm.tm_sec, ##arg); \
	} while (0)

enum urelay_msg_type {
	URELAY_STATUS		= 0x80,
	URELAY_CTRL 		= 0x01,
	URELAY_CTRL_RESP	= 0x81,
	URELAY_DATA		= 0x02,
	URELAY_DATA_RESP	= 0x82,
	URELAY_RESET		= 0x03,
	URELAY_INIT		= 0x04,
	URELAY_INTR		= 0x05,
};

typedef struct { uint16_t v; } uint16be_t;
typedef struct { uint32_t v; } uint32be_t;

inline uint16_t
from_be16(uint16be_t val)
{
	return (be16toh(val.v));
}

inline uint32_t
from_be32(uint32be_t val)
{
	return (be32toh(val.v));
}

inline uint16be_t
to_be16(uint16_t v)
{
	return ((uint16be_t){ .v = htobe16(v) });
}

inline uint32be_t
to_be32(uint32_t v)
{
	return ((uint32be_t){ .v = htobe32(v) });
}

struct urelay_msg_hdr {
	/* total length of everything after ur_msg_len */
	uint16be_t		ur_msg_len;
	uint32be_t		ur_seq;
	uint8_t			ur_msg_type; /* urelay_msg_type */
	/* followed by a struct urelay_msg_* */
	/* and then any additional data */
} __packed;

struct urelay_msg_status {
	uint8_t			ume_errno;
} __packed;

struct urelay_msg_ctrl {
	uint8_t			umc_req;
	uint8_t			umc_req_type;
	uint16be_t		umc_value;
	uint16be_t		umc_index;
	uint16be_t		umc_length;
} __packed;

struct urelay_msg_ctrl_resp {
	uint8_t			umcr_errcode;
	uint32be_t		umcr_rc;
	uint32be_t		umcr_blen;
	uint32be_t		umcr_bdone;
} __packed;

enum urelay_msg_dir {
	URELAY_DIR_IN 	= 0x11,
	URELAY_DIR_OUT	= 0x22,
};

struct urelay_msg_data {
	uint8_t			umd_dir;
	uint8_t			umd_ep;
	uint32be_t		umd_rem;
} __packed;

struct urelay_msg_data_resp {
	uint8_t			umdr_errcode;
	uint32be_t		umdr_rc;
	uint32be_t		umdr_blen;
	uint32be_t		umdr_bdone;
} __packed;

struct urelay_msg_reset {
} __packed;

struct urelay_msg_init {
	char			umi_vm_name[256];
} __packed;

struct urelay_msg_intr {
	uint8_t			umint_dir;
	uint8_t			umint_ep;
} __packed;

enum urelay_io_req_state {
	IOREQ_INIT	= 0,
	IOREQ_WAIT,
	IOREQ_DONE
};

struct urelay_io_req {
	pthread_mutex_t		 uir_mtx;
	enum urelay_io_req_state uir_state;
	pthread_cond_t		 uir_state_change;

	struct urelay_io_req	*uir_next;
	struct urelay_io_req	*uir_prev;

	void			*uir_data;
	size_t			 uir_len;
};

struct urelay_softc {
	struct usb_hci		*urs_hci;
	pthread_mutex_t		 urs_mtx;
	pthread_t		 urs_io_thread;
	pthread_t		 urs_srvreq_thread;
	int			 urs_eport;

	struct urelay_io_req	*urs_reqs;
	struct urelay_io_req	*urs_reqs_tail;

	struct urelay_io_req	*urs_srvreqs;
	struct urelay_io_req	*urs_srvreqs_tail;
	pthread_cond_t		 urs_srvreqs_nonempty;

	struct urelay_io_req	*urs_resps;
	struct urelay_io_req	*urs_resps_tail;

	char			 urs_vm_name[256];
	char			 urs_sockpath[PATH_MAX];
	struct urelay_io_req	*urs_waiting;
};

static int
msg_new(enum urelay_msg_type type, size_t *plen, struct urelay_msg_hdr **phdr,
    void **pbody, void **pdata, size_t dlen)
{
	struct urelay_msg_hdr *hdr;
	size_t len;

	len = sizeof (*hdr) + dlen;
	switch (type) {
	case URELAY_STATUS:
		len += sizeof (struct urelay_msg_status);
		break;
	case URELAY_CTRL:
		len += sizeof (struct urelay_msg_ctrl);
		break;
	case URELAY_CTRL_RESP:
		len += sizeof (struct urelay_msg_ctrl_resp);
		break;
	case URELAY_DATA:
		len += sizeof (struct urelay_msg_data);
		break;
	case URELAY_DATA_RESP:
		len += sizeof (struct urelay_msg_data_resp);
		break;
	case URELAY_RESET:
		len += sizeof (struct urelay_msg_reset);
		break;
	case URELAY_INIT:
		len += sizeof (struct urelay_msg_init);
		break;
	case URELAY_INTR:
		len += sizeof (struct urelay_msg_intr);
		break;
	}
	*plen = len;

	hdr = calloc(1, len);
	if (hdr == NULL)
		return (errno);

	hdr->ur_msg_len = to_be16(len - sizeof (uint16be_t));
	hdr->ur_msg_type = (uint8_t)type;

	*phdr = hdr;
	*pbody = (hdr + 1);
	switch (type) {
	case URELAY_STATUS:
		*pdata = ((struct urelay_msg_status *)(*pbody)) + 1;
		break;
	case URELAY_CTRL:
		*pdata = ((struct urelay_msg_ctrl *)(*pbody)) + 1;
		break;
	case URELAY_CTRL_RESP:
		*pdata = ((struct urelay_msg_ctrl_resp *)(*pbody)) + 1;
		break;
	case URELAY_DATA:
		*pdata = ((struct urelay_msg_data *)(*pbody)) + 1;
		break;
	case URELAY_DATA_RESP:
		*pdata = ((struct urelay_msg_data_resp *)(*pbody)) + 1;
		break;
	case URELAY_RESET:
		*pdata = ((struct urelay_msg_reset *)(*pbody)) + 1;
		break;
	case URELAY_INIT:
		*pdata = ((struct urelay_msg_init *)(*pbody)) + 1;
		break;
	case URELAY_INTR:
		*pdata = ((struct urelay_msg_intr *)(*pbody)) + 1;
		break;
	}

	return (0);
}

static int
ioreq_from_buf(void *buf, size_t len, struct urelay_io_req **preq)
{
	struct urelay_io_req *req;
	int rc;

	req = calloc(1, sizeof (*req));
	if (req == NULL)
		return (errno);

	if ((rc = pthread_mutex_init(&req->uir_mtx, NULL)) ||
	    (rc = pthread_cond_init(&req->uir_state_change, NULL))) {
		free(req);
		return (rc);
	}

	req->uir_data = buf;
	req->uir_len = len;

	*preq = req;

	return (0);
}

static int
ioreq_new(enum urelay_msg_type type, struct urelay_io_req **preq,
    struct urelay_msg_hdr **phdr, void **pbody, const void *idata, size_t dlen)
{
	int rc;
	struct urelay_io_req *req;
	struct urelay_msg_hdr *hdr;
	void *body, *data;
	size_t len;

	rc = msg_new(type, &len, &hdr, &body, &data, dlen);
	if (rc != 0)
		return (rc);

	req = calloc(1, sizeof (*req));
	if (req == NULL) {
		free(hdr);
		return (errno);
	}
	if ((rc = pthread_mutex_init(&req->uir_mtx, NULL)) ||
	    (rc = pthread_cond_init(&req->uir_state_change, NULL))) {
		free(hdr);
		free(req);
		return (rc);
	}

	req->uir_data = hdr;
	req->uir_len = len;

	if (idata != NULL)
		bcopy(idata, data, dlen);

	*preq = req;
	*phdr = hdr;
	*pbody = body;

	return (0);
}

static int
ioreq_enqueue(struct urelay_softc *sc, struct urelay_io_req *req)
{
	int rc;

	pthread_mutex_lock(&sc->urs_mtx);
	if (sc->urs_reqs_tail == NULL) {
		sc->urs_reqs = req;
	} else {
		req->uir_prev = sc->urs_reqs_tail;
		sc->urs_reqs_tail->uir_next = req;
	}
	sc->urs_reqs_tail = req;
	rc = port_send(sc->urs_eport, 1, sc);
	pthread_mutex_unlock(&sc->urs_mtx);

	return (rc);
}

static int
ioreq_enqueue_resp(struct urelay_softc *sc, struct urelay_io_req *req)
{
	int rc;

	pthread_mutex_lock(&sc->urs_mtx);
	if (sc->urs_resps_tail == NULL) {
		sc->urs_resps = req;
	} else {
		req->uir_prev = sc->urs_resps_tail;
		sc->urs_resps_tail->uir_next = req;
	}
	sc->urs_resps_tail = req;
	rc = port_send(sc->urs_eport, 1, sc);
	pthread_mutex_unlock(&sc->urs_mtx);

	return (rc);
}

static void
ioreq_enqueue_srv(struct urelay_softc *sc, struct urelay_io_req *req)
{
	pthread_mutex_lock(&sc->urs_mtx);
	if (sc->urs_srvreqs_tail == NULL) {
		sc->urs_srvreqs = req;
	} else {
		req->uir_prev = sc->urs_srvreqs_tail;
		sc->urs_srvreqs_tail->uir_next = req;
	}
	sc->urs_srvreqs_tail = req;
	pthread_cond_broadcast(&sc->urs_srvreqs_nonempty);
	pthread_mutex_unlock(&sc->urs_mtx);
}

static void
ioreq_waiting(struct urelay_softc *sc, struct urelay_io_req *req)
{
	pthread_mutex_lock(&sc->urs_mtx);
	pthread_mutex_lock(&req->uir_mtx);
	req->uir_state = IOREQ_WAIT;
	req->uir_next = sc->urs_waiting;
	sc->urs_waiting = req;
	pthread_cond_broadcast(&req->uir_state_change);
	pthread_mutex_unlock(&req->uir_mtx);
	pthread_mutex_unlock(&sc->urs_mtx);
}

static void
ioreq_complete(struct urelay_io_req *req)
{
	pthread_mutex_lock(&req->uir_mtx);
	req->uir_state = IOREQ_DONE;
	pthread_cond_broadcast(&req->uir_state_change);
	pthread_mutex_unlock(&req->uir_mtx);
}

static void
ioreq_requeue(struct urelay_softc *sc)
{
	struct urelay_io_req *req;

	pthread_mutex_lock(&sc->urs_mtx);
	while ((req = sc->urs_waiting) != NULL) {
		pthread_mutex_lock(&req->uir_mtx);
		sc->urs_waiting = req->uir_next;
		if (req->uir_next != NULL)
			req->uir_next->uir_prev = NULL;
		req->uir_next = NULL;

		if (sc->urs_reqs_tail == NULL) {
			sc->urs_reqs = req;
		} else {
			req->uir_prev = sc->urs_reqs_tail;
			sc->urs_reqs_tail->uir_next = req;
		}
		sc->urs_reqs_tail = req;

		req->uir_state = IOREQ_INIT;
		pthread_cond_broadcast(&req->uir_state_change);
		pthread_mutex_unlock(&req->uir_mtx);
	}
	pthread_mutex_unlock(&sc->urs_mtx);
}

static struct urelay_io_req *
ioreq_find(struct urelay_softc *sc, uint32_t seq)
{
	struct urelay_msg_hdr *hdr;
	struct urelay_io_req *req;
	uint found = 0;

	pthread_mutex_lock(&sc->urs_mtx);
	for (req = sc->urs_waiting; req != NULL; req = req->uir_next) {
		hdr = (struct urelay_msg_hdr *)req->uir_data;
		if (from_be32(hdr->ur_seq) == seq) {
			found = 1;
			if (req->uir_prev != NULL)
				req->uir_prev->uir_next = req->uir_next;
			if (req->uir_next != NULL)
				req->uir_next->uir_prev = req->uir_prev;
			if (req == sc->urs_waiting)
				sc->urs_waiting = req->uir_next;
			req->uir_next = NULL;
			req->uir_prev = NULL;
			break;
		}
	}
	pthread_mutex_unlock(&sc->urs_mtx);

	if (found)
		return (req);

	return (NULL);
}

static void
ioreq_wait(struct urelay_io_req *req)
{
	pthread_mutex_lock(&req->uir_mtx);
	while (req->uir_state != IOREQ_DONE)
		pthread_cond_wait(&req->uir_state_change, &req->uir_mtx);
	pthread_mutex_unlock(&req->uir_mtx);
}

static void
ioreq_free(struct urelay_io_req *req)
{
	if (req == NULL)
		return;
	pthread_mutex_lock(&req->uir_mtx);
	assert(req->uir_state == IOREQ_DONE || req->uir_state == IOREQ_INIT);
	assert(req->uir_next == NULL);
	pthread_mutex_unlock(&req->uir_mtx);
	pthread_cond_destroy(&req->uir_state_change);
	pthread_mutex_destroy(&req->uir_mtx);
	free(req->uir_data);
	free(req);
}

static struct urelay_io_req *
ioreq_take(struct urelay_softc *sc)
{
	struct urelay_io_req *req;
	pthread_mutex_lock(&sc->urs_mtx);
	req = sc->urs_reqs;
	if (req != NULL) {
		sc->urs_reqs = req->uir_next;
		if (sc->urs_reqs != NULL)
			sc->urs_reqs->uir_prev = NULL;
		if (sc->urs_reqs_tail == req)
			sc->urs_reqs_tail = NULL;
		req->uir_next = NULL;
	}
	pthread_mutex_unlock(&sc->urs_mtx);
	return (req);
}

static struct urelay_io_req *
ioreq_take_resp(struct urelay_softc *sc)
{
	struct urelay_io_req *req;
	pthread_mutex_lock(&sc->urs_mtx);
	req = sc->urs_resps;
	if (req != NULL) {
		sc->urs_resps = req->uir_next;
		if (sc->urs_resps != NULL)
			sc->urs_resps->uir_prev = NULL;
		if (sc->urs_resps_tail == req)
			sc->urs_resps_tail = NULL;
		req->uir_next = NULL;
	}
	pthread_mutex_unlock(&sc->urs_mtx);
	return (req);
}

static struct urelay_io_req *
ioreq_take_srv(struct urelay_softc *sc)
{
	struct urelay_io_req *req;
	req = sc->urs_srvreqs;
	if (req != NULL) {
		sc->urs_srvreqs = req->uir_next;
		if (sc->urs_srvreqs != NULL)
			sc->urs_srvreqs->uir_prev = NULL;
		if (sc->urs_srvreqs_tail == req)
			sc->urs_srvreqs_tail = NULL;
		req->uir_next = NULL;
	}
	return (req);
}

static int
handle_server_request(struct urelay_softc *sc, struct urelay_io_req *req)
{
	struct urelay_msg_hdr *hdr = req->uir_data;
	struct urelay_msg_intr *intr;
	struct urelay_msg_status *st;
	int epctx;
	size_t len = req->uir_len;

	switch (hdr->ur_msg_type) {
	case URELAY_INTR:
		VERIFY3U(len, >=, sizeof (*hdr) + sizeof (*intr));
		VERIFY3U(len, >=, sizeof (*hdr) + sizeof (*st));
		intr = (struct urelay_msg_intr *)(hdr + 1);
		epctx = intr->umint_ep;
		switch (intr->umint_dir) {
		case URELAY_DIR_IN:
			epctx |= UE_DIR_IN;
			break;
		case URELAY_DIR_OUT:
			epctx |= UE_DIR_OUT;
			break;
		default:
			VERIFY(0);
		}
		sc->urs_hci->hci_intr(sc->urs_hci, epctx);

		len = sizeof (*hdr) + sizeof (*st);
		hdr->ur_msg_len = to_be16(len - sizeof (uint16be_t));
		hdr->ur_msg_type = URELAY_STATUS;
		st = (struct urelay_msg_status *)(hdr + 1);
		st->ume_errno = 0;

		req->uir_len = len;
		return (0);
	default:
		return (ENOTSUP);
	}
}

static void *
urelay_srvreq_thread(void *arg)
{
	struct urelay_softc *sc = arg;
	struct urelay_io_req *req;
	int rc;

	pthread_mutex_lock(&sc->urs_mtx);
	while (1) {
		while (sc->urs_srvreqs == NULL) {
			pthread_cond_wait(&sc->urs_srvreqs_nonempty,
			    &sc->urs_mtx);
		}

		while ((req = ioreq_take_srv(sc)) != NULL) {
			pthread_mutex_lock(&req->uir_mtx);
			req->uir_state = IOREQ_WAIT;
			pthread_mutex_unlock(&req->uir_mtx);

			pthread_mutex_unlock(&sc->urs_mtx);

			rc = handle_server_request(sc, req);
			if (rc == 0) {
				rc = ioreq_enqueue_resp(sc, req);
				assert(rc == 0);
			}

			pthread_mutex_lock(&sc->urs_mtx);
		}
	}

	return (NULL);
}

static void *
urelay_io_thread(void *arg)
{
	struct urelay_softc *sc = arg;
	int sock;
	struct sockaddr_un un;
	u_long sleepfor = 1000000UL;
	const u_long sleepmax = 10000000UL;
	uint32_t our_seq;
	port_event_t ev;
	int rc;
	ssize_t done;
	size_t len;
	struct urelay_io_req *req;
	struct iovec iov[2];
	struct urelay_msg_hdr ihdr;
	struct urelay_msg_init init;
	struct urelay_msg_status st;
	struct urelay_msg_hdr *hdr;

	while (1) {
		our_seq = 0;

		sock = socket(AF_UNIX, SOCK_STREAM, 0);
		if (sock < 0) {
			dprintf("socket: %d: %s", errno, strerror(errno));
			goto delay;
		}

		dprintf("connect(%s)", sc->urs_sockpath);
		bzero(&un, sizeof (un));
		un.sun_family = AF_UNIX;
		strlcpy(un.sun_path, sc->urs_sockpath, sizeof (un.sun_path));
		if (connect(sock, (struct sockaddr *)&un, sizeof (un))) {
			dprintf("connect(%s): %d: %s", sc->urs_sockpath,
			    errno, strerror(errno));
			goto delay;
		}

		bzero(&ihdr, sizeof (ihdr));
		ihdr.ur_msg_len = to_be16(sizeof (ihdr) + sizeof (init) -
		    sizeof (uint16be_t));
		ihdr.ur_seq = to_be32(our_seq);
		ihdr.ur_msg_type = URELAY_INIT;

		bzero(&init, sizeof (init));
		strlcpy(init.umi_vm_name, sc->urs_vm_name,
		    sizeof (init.umi_vm_name));

		iov[0].iov_base = (caddr_t)&ihdr;
		iov[0].iov_len = sizeof (ihdr);
		iov[1].iov_base = (caddr_t)&init;
		iov[1].iov_len = sizeof (init);

		done = writev(sock, iov, 2);
		if (done < 0) {
			dprintf("write init: %d: %s", errno, strerror(errno));
			goto delay;
		}
		if ((size_t)done < sizeof (ihdr) + sizeof (init)) {
			dprintf("write init: short write");
			goto delay;
		}

		iov[0].iov_base = (caddr_t)&ihdr;
		iov[0].iov_len = sizeof (ihdr);
		iov[1].iov_base = (caddr_t)&st;
		iov[1].iov_len = sizeof (st);

		done = readv(sock, iov, 2);
		if (done < 0) {
			dprintf("read init st: %d: %s", errno, strerror(errno));
			goto delay;
		}
		if ((size_t)done < sizeof (ihdr) + sizeof (st)) {
			dprintf("read init st: short read");
			goto delay;
		}

		if (from_be16(ihdr.ur_msg_len) + sizeof (uint16be_t)
		    != sizeof (ihdr) + sizeof (st)) {
			dprintf("read init st: wrong len");
			goto delay;
		}

		if (from_be32(ihdr.ur_seq) != our_seq) {
			dprintf("read init st: seq mismatch");
			goto delay;
		}

		if (ihdr.ur_msg_type != URELAY_STATUS) {
			dprintf("read init st: type mismatch");
			goto delay;
		}

		if (st.ume_errno != 0) {
			dprintf("urelay init: server returned %d: %s",
			    st.ume_errno, strerror(st.ume_errno));
			goto delay;
		}

		dprintf("init complete");
		++our_seq;

		rc = port_associate(sc->urs_eport, PORT_SOURCE_FD, sock,
		    POLLIN, NULL);
		if (rc) {
			dprintf("port_associate: %d: %s", errno,
			    strerror(errno));
			goto delay;
		}

		while (1) {
			while ((req = ioreq_take(sc)) != NULL) {
				dprintf("sending %u", our_seq);

				hdr = (struct urelay_msg_hdr *)req->uir_data;
				hdr->ur_seq = to_be32(our_seq++);

				done = write(sock, req->uir_data, req->uir_len);
				if (done < 0) {
					dprintf("write req: %d: %s", errno,
					    strerror(errno));
					(void)ioreq_enqueue(sc, req);
					goto delay;
				}
				if ((size_t)done < req->uir_len) {
					dprintf("write req short: %zd / %zu",
					    done, req->uir_len);
					(void)ioreq_enqueue(sc, req);
					goto delay;
				}

				ioreq_waiting(sc, req);
			}

			while ((req = ioreq_take_resp(sc)) != NULL) {
				done = write(sock, req->uir_data, req->uir_len);
				if (done < 0) {
					dprintf("write resp: %d: %s", errno,
					    strerror(errno));
					(void)ioreq_enqueue_resp(sc, req);
					goto delay;
				}
				if ((size_t)done < req->uir_len) {
					dprintf("write resp short: %zd / %zu",
					    done, req->uir_len);
					(void)ioreq_enqueue_resp(sc, req);
					goto delay;
				}
				ioreq_complete(req);
				ioreq_free(req);
			}

			bzero(&ev, sizeof (ev));
			rc = port_get(sc->urs_eport, &ev, NULL);
			if (rc) {
				dprintf("port_get: %d: %s", errno,
				    strerror(errno));
				goto delay;
			}

			/* This is a wakeup for the outgoing queue. */
			if (ev.portev_source == PORT_SOURCE_USER) {
				dprintf("woke due to user event");
				continue;
			}

			dprintf("reading control socket");
			/* Otherwise we've got data waiting on the socket. */
			done = read(sock, &ihdr, sizeof (ihdr));
			if (done < 0) {
				dprintf("read req hdr: %d: %s", errno,
				    strerror(errno));
				goto delay;
			}
			if ((size_t)done < sizeof (ihdr)) {
				dprintf("read req hdr: short read");
				goto delay;
			}

			len = from_be16(ihdr.ur_msg_len);
			len += sizeof (uint16be_t);

			/*
			 * Check if it's an incoming new request (e.g. intr)
			 * rather than a response to something we sent
			 */
			if (!(ihdr.ur_msg_type & 0x80)) {
				hdr = malloc(len);
				VERIFY(hdr != NULL);
			} else {
				req = ioreq_find(sc, from_be32(ihdr.ur_seq));
				if (req == NULL) {
					dprintf("read req hdr: bad seq %u",
					    (uint)from_be32(ihdr.ur_seq));
					goto delay;
				}

				if (req->uir_len < len) {
					free(req->uir_data);
					req->uir_data = malloc(len);
				}
				req->uir_len = len;

				hdr = req->uir_data;

				dprintf("receiving %u (len = %zu)",
				    (uint)from_be32(ihdr.ur_seq),
				    len);
			}
			bcopy(&ihdr, hdr, sizeof (*hdr));

			len -= sizeof (ihdr);

			done = read(sock, hdr + 1, len);
			if (done < 0) {
				dprintf("read req body: %d: %s", errno,
				    strerror(errno));
				goto delay;
			}
			if ((size_t)done < len) {
				dprintf("read req body: short read");
				goto delay;
			}

			if (!(ihdr.ur_msg_type & 0x80)) {
				len += sizeof (ihdr);
				rc = ioreq_from_buf(hdr, len, &req);
				assert(rc == 0);
				ioreq_enqueue_srv(sc, req);
			} else {
				ioreq_complete(req);
			}

			rc = port_associate(sc->urs_eport, PORT_SOURCE_FD,
			    sock, POLLIN, NULL);
			if (rc) {
				dprintf("port_associate: %d: %s", errno,
				    strerror(errno));
				goto delay;
			}
		}

delay:
		if (sock >= 0) {
			dprintf("closing control socket");
			close(sock);
		}
		sock = -1;
		ioreq_requeue(sc);
		usleep(sleepfor);
		sleepfor *= 2;
		if (sleepfor > sleepmax)
			sleepfor = sleepmax;
	}

	return (NULL);
}

static void *
urelay_init(struct usb_hci *hci, nvlist_t *nvl)
{
	struct urelay_softc *sc;
	const char *v;
	int rc;

	sc = calloc(1, sizeof (struct urelay_softc));
	VERIFY(sc != NULL);
	sc->urs_hci = hci;

	pthread_mutex_init(&sc->urs_mtx, NULL);
	pthread_cond_init(&sc->urs_srvreqs_nonempty, NULL);

	sc->urs_eport = port_create();
	if (sc->urs_eport < 0) {
		dprintf("port_create: %d: %s", errno, strerror(errno));
		pthread_mutex_destroy(&sc->urs_mtx);
		free(sc);
		return (NULL);
	}

	v = get_config_value_node(nvl, "name");
	if (v == NULL)
		v = get_config_value("name");
	if (v != NULL)
		strlcpy(sc->urs_vm_name, v, sizeof (sc->urs_vm_name));

	strlcpy(sc->urs_sockpath, "/var/run/usb-relay.sock",
	    sizeof (sc->urs_sockpath));
	v = get_config_value_node(nvl, "socket");
	if (v == NULL)
		v = get_config_value("urelay_socket");
	if (v != NULL)
		strlcpy(sc->urs_sockpath, v, sizeof (sc->urs_sockpath));

	rc = pthread_create(&sc->urs_io_thread, NULL, urelay_io_thread,
	    sc);
	if (rc) {
		dprintf("pthread_create: %d: %s", rc, strerror(rc));
		pthread_cond_destroy(&sc->urs_srvreqs_nonempty);
		pthread_mutex_destroy(&sc->urs_mtx);
		free(sc);
		return (NULL);
	}
	pthread_setname_np(sc->urs_io_thread, "urelay io");

	rc = pthread_create(&sc->urs_srvreq_thread, NULL, urelay_srvreq_thread,
	    sc);
	if (rc) {
		dprintf("pthread_create: %d: %s", rc, strerror(rc));
		pthread_cond_destroy(&sc->urs_srvreqs_nonempty);
		pthread_mutex_destroy(&sc->urs_mtx);
		free(sc);
		return (NULL);
	}
	pthread_setname_np(sc->urs_srvreq_thread, "urelay srv reqs");

	return (sc);
}

static int
urelay_request(void *scarg, struct usb_data_xfer *xfer)
{
	struct urelay_softc *sc = scarg;
	int err;
	struct usb_data_xfer_block *data;
	uint8_t *udata;
	uint idx, i;
	int rc;
	size_t len;
	struct urelay_io_req *req;
	struct urelay_msg_hdr *hdr;
	struct urelay_msg_ctrl_resp *resp;
	struct urelay_msg_ctrl *ctrl;

	err = USB_ERR_NORMAL_COMPLETION;

	data = NULL;
	udata = NULL;
	idx = xfer->head;
	for (i = 0; i < xfer->ndata; i++) {
		xfer->data[idx].bdone = 0;
		if (data == NULL && USB_DATA_OK(xfer,i)) {
			data = &xfer->data[idx];
			udata = data->buf;
		}

		xfer->data[idx].processed = 1;
		idx = (idx + 1) % USB_MAX_XFER_BLOCKS;
	}

	if (!xfer->ureq)
		return (err);

	dprintf("urelay_request: req = 0x%02x, req_type = 0x%02x",
	    xfer->ureq->bRequest, xfer->ureq->bmRequestType);

	rc = ioreq_new(URELAY_CTRL, &req, &hdr, (void **)&ctrl, NULL, 0);
	if (rc) {
		dprintf("ioreq_new failed: %d: %s", rc, strerror(rc));
		return (err);
	}

	ctrl->umc_req = xfer->ureq->bRequest;
	ctrl->umc_req_type = xfer->ureq->bmRequestType;
	ctrl->umc_value = to_be16(UGETW(xfer->ureq->wValue));
	ctrl->umc_index = to_be16(UGETW(xfer->ureq->wIndex));
	ctrl->umc_length = to_be16(UGETW(xfer->ureq->wLength));

	rc = ioreq_enqueue(sc, req);
	if (rc) {
		dprintf("ioreq_enqueue failed: %d: %s", rc, strerror(rc));
		goto out;
	}

	ioreq_wait(req);

	hdr = req->uir_data;
	assert(req->uir_len >= sizeof (*hdr));
	if (hdr->ur_msg_type != URELAY_CTRL_RESP) {
		dprintf("unexpected type %02x response to URELAY_CTRL",
		    hdr->ur_msg_type);
		goto out;
	}
	resp = (struct urelay_msg_ctrl_resp *)(hdr + 1);
	if (req->uir_len < sizeof (*hdr) + sizeof (*resp)) {
		dprintf("short response to URELAY_CTRL");
		goto out;
	}

	if (data != NULL) {
		len = req->uir_len - (sizeof (*hdr) + sizeof (*resp));
		if (len > 0)
			bcopy(resp + 1, udata, len);

		data->blen = from_be32(resp->umcr_blen);
		data->bdone = from_be32(resp->umcr_bdone);
		data->processed = 1;
		USB_DATA_SET_ERRCODE(data, resp->umcr_errcode);
	}

	err = from_be32(resp->umcr_rc);
	dprintf("urelay_request: err = %d, bdone = %d", err,
	    (data != NULL) ? data->bdone : 0);

out:
	ioreq_free(req);
	return (err);
}

static int
urelay_data_handler(void *scarg, struct usb_data_xfer *xfer, int dir,
    int epctx)
{
	struct urelay_softc *sc = scarg;
	int err;
	struct usb_data_xfer_block *data;
	uint8_t *udata;
	uint idx, i;
	int rc;
	size_t len, iolen;
	struct urelay_io_req *req;
	struct urelay_msg_hdr *hdr;
	struct urelay_msg_data_resp *resp;
	struct urelay_msg_data *dreq;

	err = USB_ERR_NORMAL_COMPLETION;

	data = NULL;
	udata = NULL;
	idx = xfer->head;
	for (i = 0; i < xfer->ndata; i++) {
		data = &xfer->data[idx];
		if (data->buf != NULL && data->blen != 0) {
			break;
		} else {
			data->processed = 1;
			data = NULL;
		}
		idx = (idx + 1) % USB_MAX_XFER_BLOCKS;
	}

	if (data == NULL)
		return (err);

	udata = data->buf;
	len = data->blen;

	dprintf("urelay_data_handler: dir = %d, epctx = %d, len = %zu", dir,
	    epctx, len);

	iolen = 0;
	if (dir == USB_XFER_OUT)
		iolen = len;

	rc = ioreq_new(URELAY_DATA, &req, &hdr, (void **)&dreq, udata, iolen);
	if (rc) {
		dprintf("ioreq_new failed: %d: %s", rc, strerror(rc));
		return (err);
	}

	dreq->umd_ep = epctx;
	dreq->umd_rem = to_be32(len);
	switch (dir) {
	case USB_XFER_OUT:
		dreq->umd_dir = URELAY_DIR_OUT;
		break;
	case USB_XFER_IN:
		dreq->umd_dir = URELAY_DIR_IN;
		break;
	}

	rc = ioreq_enqueue(sc, req);
	if (rc) {
		dprintf("ioreq_enqueue failed: %d: %s", rc, strerror(rc));
		goto out;
	}

	ioreq_wait(req);

	hdr = req->uir_data;
	assert(req->uir_len >= sizeof (*hdr));
	if (hdr->ur_msg_type != URELAY_DATA_RESP) {
		dprintf("unexpected type %02x response to URELAY_DATA",
		    hdr->ur_msg_type);
		goto out;
	}
	resp = (struct urelay_msg_data_resp *)(hdr + 1);
	if (req->uir_len < sizeof (*hdr) + sizeof (*resp)) {
		dprintf("short response to URELAY_DATA");
		goto out;
	}

	len = req->uir_len - (sizeof (*hdr) + sizeof (*resp));
	if (len > 0)
		bcopy(resp + 1, udata, len);
	dprintf("ulrelay_data_handler: %zu copied", len);

	data->blen = from_be32(resp->umdr_blen);
	data->bdone = from_be32(resp->umdr_bdone);
	data->processed = 1;
	USB_DATA_SET_ERRCODE(data, resp->umdr_errcode);
	err = from_be32(resp->umdr_rc);
	dprintf("urelay_data_handler: err = %d, bdone = %d", err, data->bdone);

out:
	ioreq_free(req);
	return (err);
}

static int
urelay_reset(void *scarg)
{
	struct urelay_softc *sc = scarg;
	struct urelay_io_req *req;
	struct urelay_msg_hdr *hdr;
	struct urelay_msg_status *st;
	struct urelay_msg_reset *rst;
	int rc;

	rc = ioreq_new(URELAY_RESET, &req, &hdr, (void **)&rst, NULL, 0);
	if (rc) {
		dprintf("ioreq_new failed: %d: %s", rc, strerror(rc));
		return (0);
	}
	rc = ioreq_enqueue(sc, req);
	if (rc) {
		dprintf("ioreq_enqueue failed: %d: %s", rc, strerror(rc));
		return (0);
	}

	dprintf("enqueue reset req");

	ioreq_wait(req);

	dprintf("got response to reset req");

	hdr = req->uir_data;
	assert(req->uir_len >= sizeof (*hdr));
	if (hdr->ur_msg_type != URELAY_STATUS) {
		dprintf("unexpected type %02x response to URELAY_RESET",
		    hdr->ur_msg_type);
		goto out;
	}
	st = (struct urelay_msg_status *)(hdr + 1);
	if (req->uir_len < sizeof (*hdr) + sizeof (*st)) {
		dprintf("short response to URELAY_RESET");
		goto out;
	}
	if (st->ume_errno != 0) {
		dprintf("URELAY_RESET got %d: %s", st->ume_errno,
		    strerror(st->ume_errno));
	}

out:
	ioreq_free(req);
	return (0);
}

static int
urelay_remove(void *scarg)
{
	return (0);
}

static int
urelay_stop(void *scarg)
{
	return (0);
}

struct usb_devemu ue_relay = {
	.ue_emu =	"relay",
	.ue_usbver =	3,
	.ue_usbspeed =	USB_SPEED_HIGH,
	.ue_init =	urelay_init,
	.ue_request =	urelay_request,
	.ue_data =	urelay_data_handler,
	.ue_reset =	urelay_reset,
	.ue_remove =	urelay_remove,
	.ue_stop =	urelay_stop
};
USB_EMUL_SET(ue_relay);
