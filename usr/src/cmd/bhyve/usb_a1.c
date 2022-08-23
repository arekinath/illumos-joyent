#include <sys/cdefs.h>

#include <sys/time.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <errno.h>
#include <endian.h>
#include <ucred.h>

#include <sys/socket.h>
#include <sys/un.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "usb_emul.h"
#include "console.h"
#include "bhyvegc.h"
#include "mevent.h"
#include "config.h"
#include "debug.h"

enum {
	UMSTR_LANG,
	UMSTR_MANUFACTURER,
	UMSTR_PRODUCT,
	UMSTR_SERIAL,
	UMSTR_CONFIG,
	UMSTR_MAX
};

static const char *a1_desc_strings[] = {
	"\x09\x04",
	"COMP3301",
	"A1 Device",
	"01",
	"USB device for A1",
};

#define	UDESC_A1_CLASS	0xF0

struct a1_class_descriptor {
	uint8_t	bLength;
	uint8_t bDescriptorType;
	uint8_t bNumSlots;
};

struct a1_config_desc {
	struct usb_config_descriptor		confd;
	struct usb_interface_descriptor		ifcd;
	struct a1_class_descriptor		a1cd;
	struct usb_endpoint_descriptor		endpd0;
	struct usb_endpoint_descriptor		endpd1;
	struct usb_endpoint_descriptor		endpd2;
	struct usb_endpoint_ss_comp_descriptor	sscompd;
} __packed;

#define HSETW(ptr, val)   ptr = { (uint8_t)(val), (uint8_t)((val) >> 8) }
#define	MSETW(ptr, val)	ptr = { (uint8_t)(val), (uint8_t)((val) >> 8) }

#define	dprintf(fmt, arg...)	do { \
    time_t t; \
    struct tm dtm; \
    time(&t); \
    localtime_r(&t, &dtm); \
    EPRINTLN("[%04d-%02d-%02d %02d:%02d:%02d] a1_usb: " fmt, \
    	dtm.tm_year + 1900, dtm.tm_mon + 1, dtm.tm_mday, dtm.tm_hour, \
    	dtm.tm_min, dtm.tm_sec, ##arg); \
    } while (0)

static struct usb_device_descriptor a1_dev_desc = {
	.bLength = sizeof(a1_dev_desc),
	.bDescriptorType = UDESC_DEVICE,
	MSETW(.bcdUSB, UD_USB_3_0),
	.bMaxPacketSize = 8,			/* max packet size */
	MSETW(.idVendor, 0x3301),		/* vendor */
	MSETW(.idProduct, 0x0002),		/* product */
	MSETW(.bcdDevice, 0),			/* device version */
	.iManufacturer = UMSTR_MANUFACTURER,
	.iProduct = UMSTR_PRODUCT,
	.iSerialNumber = UMSTR_SERIAL,
	.bNumConfigurations = 1,
};

static struct a1_config_desc a1_confd = {
	.confd = {
		.bLength = sizeof(a1_confd.confd),
		.bDescriptorType = UDESC_CONFIG,
		MSETW(.wTotalLength, sizeof(a1_confd)),
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = UMSTR_CONFIG,
		.bmAttributes = UC_BUS_POWERED | UC_REMOTE_WAKEUP,
		.bMaxPower = 0,
	},
	.ifcd = {
		.bLength = sizeof(a1_confd.ifcd),
		.bDescriptorType = UDESC_INTERFACE,
		.bNumEndpoints = 3,
		.bInterfaceClass = UICLASS_VENDOR,
		.bInterfaceSubClass = 0x31,
		.bInterfaceProtocol = 0x01,
	},
	.a1cd = {
		.bLength = sizeof(a1_confd.a1cd),
		.bDescriptorType = UDESC_A1_CLASS,
		.bNumSlots = 8,
	},
	.endpd0 = {
		.bLength = sizeof(a1_confd.endpd0),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_OUT | 1,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = 128,
		.bInterval = 0xA,
	},
	.endpd1 = {
		.bLength = sizeof(a1_confd.endpd1),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN | 2,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = 128,
		.bInterval = 0xA,
	},
	.endpd2 = {
		.bLength = sizeof(a1_confd.endpd2),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN | 3,
		.bmAttributes = UE_INTERRUPT,
		.wMaxPacketSize[0] = 16,
		.bInterval = 0xFF,
	},
	.sscompd = {
		.bLength = sizeof(a1_confd.sscompd),
		.bDescriptorType = UDESC_ENDPOINT_SS_COMP,
		.bMaxBurst = 0,
		.bmAttributes = 0,
		MSETW(.wBytesPerInterval, 0),
	},
};

struct a1_bos_desc {
	struct usb_bos_descriptor		bosd;
	struct usb_devcap_ss_descriptor		usbssd;
} __packed;


struct a1_bos_desc a1_bosd = {
	.bosd = {
		.bLength = sizeof(a1_bosd.bosd),
		.bDescriptorType = UDESC_BOS,
		HSETW(.wTotalLength, sizeof(a1_bosd)),
		.bNumDeviceCaps = 1,
	},
	.usbssd = {
		.bLength = sizeof(a1_bosd.usbssd),
		.bDescriptorType = UDESC_DEVICE_CAPABILITY,
		.bDevCapabilityType = 3,
		.bmAttributes = 0,
		HSETW(.wSpeedsSupported, 0x08),
		.bFunctionalitySupport = 3,
		.bU1DevExitLat = 0xa,   /* dummy - not used */
		.wU2DevExitLat = { 0x20, 0x00 },
	}
};

enum a1_ctrl_command_type {
	A1_CTRL_GET		= 0x01,
	A1_CTRL_SET_SLOTS 	= 0x02,
	A1_CTRL_SET_P_ERROR 	= 0x03,
	A1_CTRL_SET_P_HANG 	= 0x04,
	A1_CTRL_SET_SPEED	= 0x05,
	A1_CTRL_RESET		= 0x06,
};

struct a1_ctrl_command {
	uint8_t		acc_cmd;	/* a1_ctrl_command_type */
	uint8_t		acc_val;
};

enum a1_ctrl_status {
	A1_CTRL_ST_OK		= 0,
	A1_CTRL_ST_STALLED	= 1<<1,
	A1_CTRL_ST_INT_ACTIVE	= 1<<2,
	A1_CTRL_ST_BUSY		= 1<<3,
};

struct a1_ctrl_reply {
	uint8_t		acc_status;	/* a1_ctrl_status */
	/* config */
	uint8_t		acc_slots;
	uint8_t		acc_p_error;
	uint8_t		acc_p_hang;
	uint8_t		acc_speed;
	/* stats */
	uint32_t	acc_ncmd;
	uint32_t	acc_ncomplete;
	uint32_t	acc_nerror;
};

enum a1_slot_state {
	SLOT_EMPTY,
	SLOT_BUFFERING,
	SLOT_RUNNING,
	SLOT_COMPLETE,
};

enum a1_operations {
	OP_ENCRYPT	= 0x01,
	OP_DECRYPT	= 0x02
};

struct a1_command {
	uint8_t		ac_slot;		/* slot index, from zero */
	uint8_t		ac_operation;		/* OP_ENCRYPT, OP_DECRYPT, ... */
	uint16_t	ac_length;		/* length of just the data field */
};

enum a1_status {
	STATUS_OK		= 1,
	STATUS_ERROR		= 2
};

struct a1_completion {
	uint8_t		ac_slot;
	uint8_t		ac_status;		/* STATUS_OK, STATUS_ERROR, ... */
	uint16_t	ac_length;
};

struct a1_interrupt {
	uint32_t	progress25;
	uint32_t	progress50;
	uint32_t	progress75;
	uint32_t	completion;
};

struct a1_slot {
	uint			 asl_idx;
	enum a1_slot_state	 asl_state;
	struct a1_command	 asl_chdr;
	size_t			 asl_bufsize;
	char			*asl_buf;
	size_t			 asl_len;
	size_t			 asl_read;	/* bytes of asl_buf read in */
	size_t			 asl_progress;	/* encryption progress */
	hrtime_t		 asl_lastbump;
	size_t			 asl_write;	/* bytes of asl_rhdr + asl_buf written out */
	struct a1_completion	 asl_rhdr;
	uint8_t			 asl_errp;
	uint8_t			 asl_errat;
};

struct a1_iq_ent {
	struct a1_iq_ent	*iq_next;
	struct a1_interrupt	 iq_intr;
};

struct a1_cq_ent {
	struct a1_cq_ent	*cq_next;
	struct a1_slot		*cq_slot;
};

struct a1_softc {
	struct usb_hci			*as_hci;

	pthread_mutex_t	 		 as_mtx;

	/* Descriptors (copied since they can vary per device) */
	struct usb_device_descriptor	 as_devd;
	struct a1_config_desc		 as_confd;

	/* Active configuration */
	uint				 as_slots;
	uint				 as_speed;
	uint				 as_phang;
	uint				 as_perror;

	/* Slot state */
	struct a1_slot			 as_slot[32];

	/* Slot currently buffering bulk-outs */
	struct a1_slot			*as_bslot;

	/* Contents of next interrupt transfer, and whether it's ready */
	struct a1_interrupt		 as_intr;
	uint				 as_intr_dirty;

	/* Completion queue, to be collected on bulk-in EP */
	struct a1_cq_ent		*as_cq;
	struct a1_cq_ent		*as_cq_tail;

	/* Main timer for advancing progress */
	struct mevent			*as_timer;

	/* Listening socket for control commands */
	char				 as_sockpath[PATH_MAX];
	struct mevent			*as_accept;
	int				 as_listen_fd;

	/* Active control client */
	int				 as_ctrl_fd;
	ucred_t				*as_ctrl_peer;
	struct mevent			*as_ctrl_r;
	struct mevent			*as_ctrl_w;

	/* Command/reply buffers being sent/received to control client */
	union {
		struct a1_ctrl_command	 as_ctrl_cmd;
		uint8_t			 as_ctrl_buf[sizeof(struct a1_ctrl_command)];
	};
	union {
		struct a1_ctrl_reply	 as_ctrl_rep;
		uint8_t			 as_ctrl_rbuf[sizeof(struct a1_ctrl_reply)];
	};
	uint				 as_ctrl_pos;
	uint				 as_ctrl_rpos;

	uint32_t			 as_ncmd;
	uint32_t			 as_ncomplete;
	uint32_t			 as_nerror;
};

static void
a1_timer(int fd, enum ev_type typ, void *cookie)
{
	struct a1_softc *sc = cookie;
	uint i;
	struct a1_slot *slot;
	struct a1_cq_ent *cqe;
	uint32_t mask;
	uint dirty = 0;

	pthread_mutex_lock(&sc->as_mtx);
	for (i = 0; i < sc->as_slots; ++i) {
		slot = &sc->as_slot[i];
		mask = 1UL << i;
		if (slot->asl_state != SLOT_RUNNING)
			continue;

		slot->asl_progress += sc->as_speed;

		if (slot->asl_errp == 0) {
			slot->asl_errp = arc4random_uniform(100);
			slot->asl_errat = arc4random_uniform(100);
		}
		if (slot->asl_errp < sc->as_perror &&
		    slot->asl_progress >= slot->asl_errat) {
			dprintf("slot %u hit error", slot->asl_idx);
			sc->as_intr.completion |= mask;
			sc->as_intr_dirty = (dirty = 1);

			slot->asl_state = SLOT_COMPLETE;

			slot->asl_rhdr.ac_slot = i;
			slot->asl_rhdr.ac_status = STATUS_ERROR;
			slot->asl_rhdr.ac_length = 0;

			slot->asl_len = 0;

			cqe = calloc(1, sizeof (*cqe));
			cqe->cq_slot = slot;
			if (sc->as_cq_tail == NULL) {
				sc->as_cq = cqe;
			} else {
				sc->as_cq_tail->cq_next = cqe;
			}
			sc->as_cq_tail = cqe;

			sc->as_nerror++;
			continue;
		}

		if (slot->asl_progress > 25 &&
		    !(sc->as_intr.progress25 & mask)) {
		    	dprintf("slot %u at 25%%", slot->asl_idx);
			sc->as_intr.progress25 |= mask;
			sc->as_intr_dirty = (dirty = 1);
		}
		if (slot->asl_progress > 50 &&
		    !(sc->as_intr.progress50 & mask)) {
		    	dprintf("slot %u at 50%%", slot->asl_idx);
			sc->as_intr.progress50 |= mask;
			sc->as_intr_dirty = (dirty = 1);
		}
		if (slot->asl_progress > 75 &&
		    !(sc->as_intr.progress75 & mask)) {
		    	dprintf("slot %u at 75%%", slot->asl_idx);
			sc->as_intr.progress75 |= mask;
			sc->as_intr_dirty = (dirty = 1);
		}
		if (slot->asl_progress >= 100) {
			dprintf("slot %u at 100%%", slot->asl_idx);
			sc->as_intr.completion |= mask;
			sc->as_intr_dirty = (dirty = 1);

			slot->asl_state = SLOT_COMPLETE;

			slot->asl_rhdr.ac_slot = i;
			slot->asl_rhdr.ac_status = STATUS_OK;
			slot->asl_rhdr.ac_length = htole16(slot->asl_len);

			cqe = calloc(1, sizeof (*cqe));
			cqe->cq_slot = slot;
			if (sc->as_cq_tail == NULL) {
				sc->as_cq = cqe;
			} else {
				sc->as_cq_tail->cq_next = cqe;
			}
			sc->as_cq_tail = cqe;

			sc->as_ncomplete++;
		}
	}
	pthread_mutex_unlock(&sc->as_mtx);

	if (dirty == 1)
		sc->as_hci->hci_intr(sc->as_hci, UE_DIR_IN | 3);
}

static void
a1_ctrl_close(struct a1_softc *sc)
{
	dprintf("closed ctrl");

	if (sc->as_ctrl_w != NULL)
		mevent_delete(sc->as_ctrl_w);
	sc->as_ctrl_w = NULL;

	if (sc->as_ctrl_r != NULL) {
		mevent_delete_close(sc->as_ctrl_r);
	} else if (sc->as_ctrl_fd != -1) {
		close(sc->as_ctrl_fd);
	}

	sc->as_ctrl_fd = -1;

	ucred_free(sc->as_ctrl_peer);
	sc->as_ctrl_peer = NULL;

	sc->as_ctrl_pos = 0;
	sc->as_ctrl_rpos = 0;
}

static void a1_reset_unlocked(struct a1_softc *sc);

static void
a1_ctrl_cmd(struct a1_softc *sc, const struct a1_ctrl_command *cmd,
    struct a1_ctrl_reply *rep)
{
	/*
	 * We're going to touch parts of the a1_softc that are also used by
	 * the VM i/o exit threads now, so take the lock.
	 */
	pthread_mutex_lock(&sc->as_mtx);
	bzero(rep, sizeof (*rep));

	switch (cmd->acc_cmd) {
	case A1_CTRL_GET:
		dprintf("ctrl get");
		break;
	case A1_CTRL_RESET:
		a1_reset_unlocked(sc);
		break;
	case A1_CTRL_SET_SLOTS:
		if (cmd->acc_val < 1 || cmd->acc_val > 32) {
			dprintf("tried to set invalid nslots = %u",
			    cmd->acc_val);
			break;
		}
		sc->as_slots = cmd->acc_val;
		sc->as_confd.a1cd.bNumSlots = cmd->acc_val;
		a1_reset_unlocked(sc);
		break;
	case A1_CTRL_SET_P_ERROR:
		sc->as_perror = cmd->acc_val;
		break;
	case A1_CTRL_SET_SPEED:
		sc->as_speed = cmd->acc_val;
		break;
	default:
		dprintf("unknown control command: %u", cmd->acc_cmd);
		break;
	}

	rep->acc_slots = sc->as_slots;
	rep->acc_p_error = sc->as_perror;
	rep->acc_speed = sc->as_speed;

	rep->acc_ncmd = sc->as_ncmd;
	rep->acc_ncomplete = sc->as_ncomplete;
	rep->acc_nerror = sc->as_nerror;

	pthread_mutex_unlock(&sc->as_mtx);
}

static void
a1_ctrl_read(int fd, enum ev_type typ, void *cookie)
{
	struct a1_softc *sc = cookie;
	ssize_t nread;
	size_t rem, pos;
	uint8_t *p;

	while (1) {
		pos = sc->as_ctrl_pos;
		rem = sizeof (sc->as_ctrl_buf) - pos;
		p = &sc->as_ctrl_buf[sc->as_ctrl_pos];

		nread = read(fd, p, rem);

		if (nread < 0) {
			if (errno == EAGAIN)
				break;
			a1_ctrl_close(sc);
			return;
		} else if (nread == 0) {
			a1_ctrl_close(sc);
			return;
		} else {
			sc->as_ctrl_pos += nread;
			if (sc->as_ctrl_pos >= sizeof (sc->as_ctrl_buf)) {
				sc->as_ctrl_pos = 0;
				sc->as_ctrl_rpos = 0;
				a1_ctrl_cmd(sc, &sc->as_ctrl_cmd, &sc->as_ctrl_rep);
				mevent_enable(sc->as_ctrl_w);
			}
		}
	}

	mevent_enable(sc->as_ctrl_r);
}

static void
a1_ctrl_write(int fd, enum ev_type typ, void *cookie)
{
	struct a1_softc *sc = cookie;
	ssize_t nwrote;
	size_t rem, pos;
	uint8_t *p;

	pos = sc->as_ctrl_rpos;
	rem = sizeof (sc->as_ctrl_rbuf) - pos;
	p = &sc->as_ctrl_rbuf[sc->as_ctrl_rpos];

	nwrote = write(fd, p, rem);

	if (nwrote < 0) {
		switch (errno) {
		case EAGAIN:
			break;
		default:
			a1_ctrl_close(sc);
			return;
		}
	} else {
		sc->as_ctrl_rpos += nwrote;
		if (sc->as_ctrl_rpos >= sizeof (sc->as_ctrl_rbuf)) {
			mevent_disable(sc->as_ctrl_w);
			return;
		}
	}

	mevent_enable(sc->as_ctrl_w);
}

static void
a1_accept(int fd, enum ev_type typ, void *cookie)
{
	struct a1_softc *sc = cookie;
	ucred_t *ucred = NULL;
	int cfd;

	cfd = accept(fd, NULL, NULL);
	if (cfd < 0)
		return;

	if (getpeerucred(cfd, &ucred)) {
		dprintf("ctrl sock: failed to get peer ucred: %d (%s)",
		    errno, strerror(errno));
		close(cfd);
		return;
	}

	if (sc->as_ctrl_fd != -1) {
		dprintf("control socket already open, closing old one");
		a1_ctrl_close(sc);
	}

	dprintf("control connection from pid %u, euid %u",
	    ucred_getpid(ucred), ucred_geteuid(ucred));
	sc->as_ctrl_fd = cfd;
	sc->as_ctrl_peer = ucred;
	sc->as_ctrl_r = mevent_add(cfd, EVF_READ, a1_ctrl_read, sc);
	sc->as_ctrl_w = mevent_add_disabled(cfd, EVF_WRITE, a1_ctrl_write, sc);
}

static void *
a1_init(struct usb_hci *hci, nvlist_t *nvl)
{
	struct a1_softc *sc;
	const char *v;
	struct sockaddr_un un;
	uint i;

	sc = calloc(1, sizeof (struct a1_softc));
	VERIFY(sc != NULL);
	sc->as_hci = hci;

	strlcpy(sc->as_sockpath, "/var/run/a1-ctrl-", sizeof (sc->as_sockpath));
	v = get_config_value("name");
	if (v != NULL)
		strlcat(sc->as_sockpath, v, sizeof (sc->as_sockpath));
	strlcat(sc->as_sockpath, ".sock", sizeof (sc->as_sockpath));

	v = get_config_value_node(nvl, "socket");
	if (v != NULL)
		strlcpy(sc->as_sockpath, v, sizeof (sc->as_sockpath));

	(void) unlink(sc->as_sockpath);
	sc->as_listen_fd = socket(AF_UNIX, SOCK_STREAM | SOCK_NONBLOCK, 0);
	if (sc->as_listen_fd < 0) {
		dprintf("failed to open listen socket: %s",
		    strerror(errno));
		free(sc);
		return (NULL);
	}

	bzero(&un, sizeof (un));
	un.sun_family = AF_UNIX;
	strlcpy(un.sun_path, sc->as_sockpath, sizeof (un.sun_path));
	if (bind(sc->as_listen_fd, (struct sockaddr *)&un, sizeof (un))) {
		dprintf("failed to bind listen socket '%s': %s",
		    sc->as_sockpath, strerror(errno));
		close(sc->as_listen_fd);
		free(sc);
		return (NULL);
	}

	if (listen(sc->as_listen_fd, 1)) {
		dprintf("failed to listen on socket '%s': %s",
		    sc->as_sockpath, strerror(errno));
		close(sc->as_listen_fd);
		free(sc);
		return (NULL);
	}

	pthread_mutex_init(&sc->as_mtx, NULL);

	bcopy(&a1_dev_desc, &sc->as_devd, sizeof (a1_dev_desc));
	bcopy(&a1_confd, &sc->as_confd, sizeof (a1_confd));

	/* randomise the product ID */
	do {
		arc4random_buf(&sc->as_devd.idProduct,
		    sizeof (sc->as_devd.idProduct));
	} while (sc->as_devd.idProduct[0] < 2);

	sc->as_timer = mevent_add(100, EVF_TIMER, a1_timer, sc);

	sc->as_accept = mevent_add(sc->as_listen_fd, EVF_READ, a1_accept, sc);

	sc->as_ctrl_fd = -1;

	sc->as_slots = 8;
	sc->as_confd.a1cd.bNumSlots = 8;
	sc->as_speed = 3;
	sc->as_phang = 0;
	sc->as_perror = 0;
	for (i = 0; i < 32; ++i) {
		sc->as_slot[i].asl_state = SLOT_EMPTY;
		sc->as_slot[i].asl_idx = i;
	}

	return (sc);
}

static void
a1_reset_unlocked(struct a1_softc *sc)
{
	struct a1_cq_ent *cqe;
	struct a1_slot *slot;
	uint i;

	dprintf("reset");

	sc->as_bslot = NULL;
	sc->as_intr_dirty = 0;
	while (sc->as_cq != NULL) {
		cqe = sc->as_cq;
		sc->as_cq = cqe->cq_next;
		free(cqe);
	}
	sc->as_cq = NULL;
	sc->as_cq_tail = NULL;

	for (i = 0; i < 32; ++i) {
		slot = &sc->as_slot[i];
		slot->asl_state = SLOT_EMPTY;
		slot->asl_len = 0;
		slot->asl_read = 0;
		slot->asl_progress = 0;
		slot->asl_write = 0;
		slot->asl_errp = 0;
		slot->asl_errat = 0;
	}

	bzero(&sc->as_intr, sizeof (sc->as_intr));
}

static int
a1_reset(void *scarg)
{
	struct a1_softc *sc = scarg;

	pthread_mutex_lock(&sc->as_mtx);
	a1_reset_unlocked(sc);
	pthread_mutex_unlock(&sc->as_mtx);

	return (0);
}

#define	UREQ(x,y)	((x) | ((y) << 8))

static int
a1_request(void *scarg, struct usb_data_xfer *xfer)
{
	struct a1_softc *sc = scarg;
	int err;
	struct usb_data_xfer_block *data;
	uint8_t *udata;
	uint idx, i;
	size_t len, slen;
	uint16_t value, index;
	const char *str;

	(void)sc;

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
		goto done;

	value = UGETW(xfer->ureq->wValue);
	index = UGETW(xfer->ureq->wIndex);
	len = UGETW(xfer->ureq->wLength);

	switch (UREQ(xfer->ureq->bRequest, xfer->ureq->bmRequestType)) {
	case UREQ(UR_GET_CONFIG, UT_READ_DEVICE):
		if (!data)
			break;

		*udata = sc->as_confd.confd.bConfigurationValue;
		data->blen = len > 0 ? len - 1 : 0;
		if (data->blen > 0)
			err = USB_ERR_SHORT_XFER;
		data->bdone += 1;
		break;

	case UREQ(UR_GET_DESCRIPTOR, UT_READ_DEVICE):
		if (!data)
			break;

		switch (value >> 8) {
		case UDESC_DEVICE:
			if ((value & 0xFF) != 0) {
				err = USB_ERR_STALLED;
				goto done;
			}
			if (len > sizeof(a1_dev_desc)) {
				data->blen = len - sizeof(a1_dev_desc);
				len = sizeof(a1_dev_desc);
			} else
				data->blen = 0;
			memcpy(data->buf, &sc->as_devd, len);
			data->bdone += len;
			break;

		case UDESC_CONFIG:
			if ((value & 0xFF) != 0) {
				err = USB_ERR_STALLED;
				goto done;
			}
			if (len > sizeof(a1_confd)) {
				data->blen = len - sizeof(a1_confd);
				len = sizeof(a1_confd);
			} else
				data->blen = 0;

			memcpy(data->buf, &sc->as_confd, len);
			data->bdone += len;
			break;

		case UDESC_STRING:
			str = NULL;
			if ((value & 0xFF) < UMSTR_MAX)
				str = a1_desc_strings[value & 0xFF];
			else
				goto done;

			if ((value & 0xFF) == UMSTR_LANG) {
				udata[0] = 4;
				udata[1] = UDESC_STRING;
				data->blen = len - 2;
				len -= 2;
				data->bdone += 2;

				if (len >= 2) {
					udata[2] = str[0];
					udata[3] = str[1];
					data->blen -= 2;
					data->bdone += 2;
				} else
					data->blen = 0;

				goto done;
			}

			slen = 2 + strlen(str) * 2;
			udata[0] = slen;
			udata[1] = UDESC_STRING;

			if (len > slen) {
				data->blen = len - slen;
				len = slen;
			} else
				data->blen = 0;
			for (i = 2; i < len; i += 2) {
				udata[i] = *str++;
				udata[i+1] = '\0';
			}
			data->bdone += slen;

			break;

		case UDESC_BOS:
			if (len > sizeof(a1_bosd)) {
				data->blen = len - sizeof(a1_bosd);
				len = sizeof(a1_bosd);
			} else
				data->blen = 0;
			memcpy(udata, &a1_bosd, len);
			data->bdone += len;
			break;

		default:
			err = USB_ERR_STALLED;
			goto done;
		}
		if (data->blen > 0)
			err = USB_ERR_SHORT_XFER;
		break;

	case UREQ(UR_GET_DESCRIPTOR, UT_READ_INTERFACE):
		if (!data)
			break;

		switch (value >> 8) {
		default:
			err = USB_ERR_STALLED;
			goto done;
		}
		/*if (data->blen > 0)
			err = USB_ERR_SHORT_XFER;*/
		break;

	case UREQ(UR_GET_INTERFACE, UT_READ_INTERFACE):
		if (index != 0) {
			err = USB_ERR_STALLED;
			goto done;
		}

		if (!data)
			break;

		if (len > 0) {
			*udata = 0;
			data->blen = len - 1;
		}
		if (data->blen > 0)
			err = USB_ERR_SHORT_XFER;
		data->bdone += 1;
		break;

	case UREQ(UR_GET_STATUS, UT_READ_DEVICE):
		if (data != NULL && len > 1) {
			USETW(udata, 0);
			data->blen = len - 2;
			data->bdone += 2;
		}

		if (data->blen > 0)
			err = USB_ERR_SHORT_XFER;
		break;

	case UREQ(UR_GET_STATUS, UT_READ_INTERFACE):
	case UREQ(UR_GET_STATUS, UT_READ_ENDPOINT):
		if (data != NULL && len > 1) {
			USETW(udata, 0);
			data->blen = len - 2;
			data->bdone += 2;
		}
		if (data->blen > 0)
			err = USB_ERR_SHORT_XFER;
		break;

	case UREQ(UR_SET_ADDRESS, UT_WRITE_DEVICE):
		/* XXX Controller should've handled this */
		break;

	case UREQ(UR_SET_CONFIG, UT_WRITE_DEVICE):
		dprintf("resetting on config change");
		(void) a1_reset(sc);
		break;

	case UREQ(UR_SET_DESCRIPTOR, UT_WRITE_DEVICE):
		break;


	case UREQ(UR_CLEAR_FEATURE, UT_WRITE_DEVICE):
		break;

	case UREQ(UR_SET_FEATURE, UT_WRITE_DEVICE):
		break;

	case UREQ(UR_CLEAR_FEATURE, UT_WRITE_INTERFACE):
	case UREQ(UR_CLEAR_FEATURE, UT_WRITE_ENDPOINT):
	case UREQ(UR_SET_FEATURE, UT_WRITE_INTERFACE):
	case UREQ(UR_SET_FEATURE, UT_WRITE_ENDPOINT):
		err = USB_ERR_STALLED;
		goto done;

	case UREQ(UR_SET_INTERFACE, UT_WRITE_INTERFACE):
		break;

	case UREQ(UR_ISOCH_DELAY, UT_WRITE_DEVICE):
		break;

	case UREQ(UR_SET_SEL, 0):
		break;

	case UREQ(UR_SYNCH_FRAME, UT_WRITE_ENDPOINT):
		break;

	default:
		err = USB_ERR_STALLED;
		break;
	}

done:
	return (err);
}

static void
rot13(uint8_t *buf, size_t len)
{
	uint i;
	for (i = 0; i < len; ++i) {
		if (buf[i] >= 'a' && buf[i] <= 'z') {
			buf[i] += 13;
			if (buf[i] > 'z')
				buf[i] -= 'z' + 1 - 'a';
		} else if (buf[i] >= 'A' && buf[i] <= 'Z') {
			buf[i] += 13;
			if (buf[i] > 'Z')
				buf[i] -= 'Z' + 1 - 'A';
		}
	}
}

static int
a1_data_handler(void *scarg, struct usb_data_xfer *xfer, int dir,
     int epctx)
{
	struct a1_softc *sc = scarg;
	struct usb_data_xfer_block *data;
	int err;
	char *udata;
	uint i, idx;
	size_t len, slen;
	struct a1_slot *slot;
	struct a1_command *cmd;
	struct a1_cq_ent *cqe;

	(void)sc;

	udata = NULL;
	err = USB_ERR_NORMAL_COMPLETION;

	data = NULL;
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
		goto done;

	udata = data->buf;
	len = data->blen;

	if (udata == NULL) {
		err = USB_ERR_NOMEM;
		goto done;
	}

	dprintf("transfer for ep %d (%zu bytes)", epctx, len);

	if (dir == USB_XFER_OUT && epctx == 1) {
		pthread_mutex_lock(&sc->as_mtx);
		/* Is this the start of a new command? */
		if (sc->as_bslot == NULL) {
			if (len < sizeof (struct a1_command)) {
				dprintf("short xfer");
				err = USB_ERR_SHORT_XFER;
				pthread_mutex_unlock(&sc->as_mtx);
				goto done;
			}
			cmd = (struct a1_command *)udata;
			if (cmd->ac_slot >= sc->as_slots) {
				dprintf("slot # too high (%u)",
				    cmd->ac_slot);
				USB_DATA_SET_ERRCODE(&xfer->data[xfer->head],
				    USB_NAK);
				err = USB_ERR_CANCELLED;
				pthread_mutex_unlock(&sc->as_mtx);
				goto done;
			}
			slot = &sc->as_slot[cmd->ac_slot];
			if (slot->asl_state != SLOT_EMPTY) {
				dprintf("slot already busy (%u)",
				    cmd->ac_slot);
				USB_DATA_SET_ERRCODE(&xfer->data[xfer->head],
				    USB_NAK);
				err = USB_ERR_CANCELLED;
				pthread_mutex_unlock(&sc->as_mtx);
				goto done;
			}
			slot->asl_state = SLOT_BUFFERING;
			sc->as_bslot = slot;
			slen = le16toh(cmd->ac_length);
			if (slot->asl_bufsize < slen) {
				slot->asl_bufsize = slen;
				free(slot->asl_buf);
				slot->asl_buf = malloc(slen);
			}
			slot->asl_len = slen;
			bcopy(udata, &slot->asl_chdr, sizeof (struct a1_command));
			udata += sizeof (struct a1_command);
			len -= sizeof (struct a1_command);
			dprintf("new cmd on slot %u: op = %u, length = %u",
			    slot->asl_idx, cmd->ac_operation, slen);
		} else {
			slot = sc->as_bslot;
		}
		if (slot->asl_state != SLOT_BUFFERING) {
			dprintf("slot state mismatch");
			USB_DATA_SET_ERRCODE(data, USB_NAK);
			err = USB_ERR_CANCELLED;
			pthread_mutex_unlock(&sc->as_mtx);
			goto done;
		}
		slen = slot->asl_len - slot->asl_read;
		if (len >= slen)
			len = slen;
		data->processed = 1;
		data->bdone = len;
		data->blen = 0;
		bcopy(udata, &slot->asl_buf[slot->asl_read], len);
		slot->asl_read += len;
		dprintf("slot %u: read %zu out of %zu", slot->asl_idx,
		    slot->asl_read, slot->asl_len);
		if (slot->asl_read >= slot->asl_len) {
			dprintf("starting slot %u", slot->asl_idx);
			sc->as_ncmd++;
			rot13((uint8_t *)slot->asl_buf, slot->asl_len);
			slot->asl_state = SLOT_RUNNING;
			slot->asl_lastbump = gethrtime();
			sc->as_bslot = NULL;
		}
		pthread_mutex_unlock(&sc->as_mtx);

	} else if (dir == USB_XFER_IN && epctx == 2) {
		pthread_mutex_lock(&sc->as_mtx);
		cqe = sc->as_cq;
		if (cqe == NULL) {
			dprintf("read from bulk in with no completion");
			USB_DATA_SET_ERRCODE(&xfer->data[xfer->head], USB_NAK);
			err = USB_ERR_CANCELLED;
			pthread_mutex_unlock(&sc->as_mtx);
			goto done;
		}
		slot = cqe->cq_slot;
		if (slot->asl_write == 0) {
			dprintf("read completion hdr on slot %u", slot->asl_idx);
			if (len < sizeof (struct a1_completion)) {
				err = USB_ERR_SHORT_XFER;
				pthread_mutex_unlock(&sc->as_mtx);
				goto done;
			}
			bcopy(&slot->asl_rhdr, udata, sizeof (slot->asl_rhdr));
			data->processed = 1;
			data->bdone += sizeof (slot->asl_rhdr);
			data->blen -= sizeof (slot->asl_rhdr);
			len -= sizeof (slot->asl_rhdr);
			slot->asl_write += sizeof (slot->asl_rhdr);
		}
		slen = slot->asl_len + sizeof (slot->asl_rhdr) -
		    slot->asl_write;
		if (len > slen)
			len = slen;
		bcopy(slot->asl_buf, udata, len);
		data->processed = 1;
		data->bdone += len;
		data->blen -= len;
		slot->asl_write += len;

		if (slen - len == 0) {
			dprintf("finished reading completion on slot %u", slot->asl_idx);
			slot->asl_state = SLOT_EMPTY;
			slot->asl_read = 0;
			slot->asl_len = 0;
			slot->asl_progress = 0;
			slot->asl_write = 0;
			slot->asl_errp = 0;
			slot->asl_errat = 0;

			sc->as_intr.progress25 &= ~(1 << slot->asl_idx);
			sc->as_intr.progress50 &= ~(1 << slot->asl_idx);
			sc->as_intr.progress75 &= ~(1 << slot->asl_idx);
			sc->as_intr.completion &= ~(1 << slot->asl_idx);

			sc->as_cq = cqe->cq_next;
			if (sc->as_cq_tail == cqe)
				sc->as_cq_tail = NULL;
			free(cqe);
		}
		pthread_mutex_unlock(&sc->as_mtx);

	} else if (dir == USB_XFER_IN && epctx == 3) {
		pthread_mutex_lock(&sc->as_mtx);
		if (!sc->as_intr_dirty) {
			dprintf("no interrupt waiting, NAK");
			USB_DATA_SET_ERRCODE(&xfer->data[xfer->head], USB_NAK);
			err = USB_ERR_CANCELLED;
			pthread_mutex_unlock(&sc->as_mtx);
			goto done;
		}
		if (len > sizeof (sc->as_intr))
			len = sizeof (sc->as_intr);
		if (len < sizeof (sc->as_intr)) {
			err = USB_ERR_SHORT_XFER;
			pthread_mutex_unlock(&sc->as_mtx);
			goto done;
		}
		bcopy((const void *)&sc->as_intr, udata, len);
		data->processed = 1;
		data->bdone += len;
		data->blen -= len;
		sc->as_intr_dirty = 0;
		pthread_mutex_unlock(&sc->as_mtx);

	} else {
		USB_DATA_SET_ERRCODE(data, USB_STALL);
		err = USB_ERR_STALLED;
	}

done:
	return (err);
}

static int
a1_remove(void *scarg)
{
	return (0);
}

static int
a1_stop(void *scarg)
{

	return (0);
}

struct usb_devemu ue_a1 = {
	.ue_emu =	"a1",
	.ue_usbver =	3,
	.ue_usbspeed =	USB_SPEED_HIGH,
	.ue_init =	a1_init,
	.ue_request =	a1_request,
	.ue_data =	a1_data_handler,
	.ue_reset =	a1_reset,
	.ue_remove =	a1_remove,
	.ue_stop =	a1_stop
};
USB_EMUL_SET(ue_a1);
