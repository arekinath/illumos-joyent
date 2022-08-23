#include <sys/cdefs.h>

#include <sys/time.h>

#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include "usb_emul.h"
#include "console.h"
#include "bhyvegc.h"
#include "debug.h"

enum {
	UMSTR_LANG,
	UMSTR_MANUFACTURER,
	UMSTR_PRODUCT,
	UMSTR_SERIAL,
	UMSTR_CONFIG,
	UMSTR_MAX
};

static const char *udummy_desc_strings[] = {
	"\x09\x04",
	"COMP3301",
	"Dummy Device",
	"01",
	"Dummy USB device for pracs",
};

struct udummy_config_desc {
	struct usb_config_descriptor		confd;
	struct usb_interface_descriptor		ifcd;
	struct usb_endpoint_descriptor		endpd0;
	struct usb_endpoint_descriptor		endpd1;
	struct usb_endpoint_ss_comp_descriptor	sscompd;
} __packed;

#define HSETW(ptr, val)   ptr = { (uint8_t)(val), (uint8_t)((val) >> 8) }
#define	MSETW(ptr, val)	ptr = { (uint8_t)(val), (uint8_t)((val) >> 8) }

static struct usb_device_descriptor udummy_dev_desc = {
	.bLength = sizeof(udummy_dev_desc),
	.bDescriptorType = UDESC_DEVICE,
	MSETW(.bcdUSB, UD_USB_3_0),
	.bMaxPacketSize = 8,			/* max packet size */
	MSETW(.idVendor, 0x3301),		/* vendor */
	MSETW(.idProduct, 0x0001),		/* product */
	MSETW(.bcdDevice, 0),			/* device version */
	.iManufacturer = UMSTR_MANUFACTURER,
	.iProduct = UMSTR_PRODUCT,
	.iSerialNumber = UMSTR_SERIAL,
	.bNumConfigurations = 1,
};

static struct udummy_config_desc udummy_confd = {
	.confd = {
		.bLength = sizeof(udummy_confd.confd),
		.bDescriptorType = UDESC_CONFIG,
		.wTotalLength[0] = sizeof(udummy_confd),
		.bNumInterface = 1,
		.bConfigurationValue = 1,
		.iConfiguration = UMSTR_CONFIG,
		.bmAttributes = UC_BUS_POWERED | UC_REMOTE_WAKEUP,
		.bMaxPower = 0,
	},
	.ifcd = {
		.bLength = sizeof(udummy_confd.ifcd),
		.bDescriptorType = UDESC_INTERFACE,
		.bNumEndpoints = 2,
		.bInterfaceClass = UICLASS_VENDOR,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
	},
	.endpd0 = {
		.bLength = sizeof(udummy_confd.endpd0),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_OUT | 1,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = 64,
		.bInterval = 0xA,
	},
	.endpd1 = {
		.bLength = sizeof(udummy_confd.endpd1),
		.bDescriptorType = UDESC_ENDPOINT,
		.bEndpointAddress = UE_DIR_IN | 2,
		.bmAttributes = UE_BULK,
		.wMaxPacketSize[0] = 64,
		.bInterval = 0xA,
	},
	.sscompd = {
		.bLength = sizeof(udummy_confd.sscompd),
		.bDescriptorType = UDESC_ENDPOINT_SS_COMP,
		.bMaxBurst = 0,
		.bmAttributes = 0,
		MSETW(.wBytesPerInterval, 0),
	},
};

struct udummy_bos_desc {
	struct usb_bos_descriptor		bosd;
	struct usb_devcap_ss_descriptor		usbssd;
} __packed;


struct udummy_bos_desc udummy_bosd = {
	.bosd = {
		.bLength = sizeof(udummy_bosd.bosd),
		.bDescriptorType = UDESC_BOS,
		HSETW(.wTotalLength, sizeof(udummy_bosd)),
		.bNumDeviceCaps = 1,
	},
	.usbssd = {
		.bLength = sizeof(udummy_bosd.usbssd),
		.bDescriptorType = UDESC_DEVICE_CAPABILITY,
		.bDevCapabilityType = 3,
		.bmAttributes = 0,
		HSETW(.wSpeedsSupported, 0x08),
		.bFunctionalitySupport = 3,
		.bU1DevExitLat = 0xa,   /* dummy - not used */
		.wU2DevExitLat = { 0x20, 0x00 },
	}
};

struct udummy_softc {
	struct usb_hci	*uds_hci;
	pthread_mutex_t	 uds_mtx;
	char		 uds_buf[64];
};

static void *
udummy_init(struct usb_hci *hci, nvlist_t *nvl)
{
	struct udummy_softc *sc;

	sc = calloc(1, sizeof (struct udummy_softc));
	VERIFY(sc != NULL);
	sc->uds_hci = hci;

	pthread_mutex_init(&sc->uds_mtx, NULL);

	return (sc);
}

#define	UREQ(x,y)	((x) | ((y) << 8))

static int
udummy_request(void *scarg, struct usb_data_xfer *xfer)
{
	struct udummy_softc *sc = scarg;
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

		*udata = udummy_confd.confd.bConfigurationValue;
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
			if (len > sizeof(udummy_dev_desc)) {
				data->blen = len - sizeof(udummy_dev_desc);
				len = sizeof(udummy_dev_desc);
			} else
				data->blen = 0;
			memcpy(data->buf, &udummy_dev_desc, len);
			data->bdone += len;
			break;

		case UDESC_CONFIG:
			if ((value & 0xFF) != 0) {
				err = USB_ERR_STALLED;
				goto done;
			}
			if (len > sizeof(udummy_confd)) {
				data->blen = len - sizeof(udummy_confd);
				len = sizeof(udummy_confd);
			} else
				data->blen = 0;

			memcpy(data->buf, &udummy_confd, len);
			data->bdone += len;
			break;

		case UDESC_STRING:
			str = NULL;
			if ((value & 0xFF) < UMSTR_MAX)
				str = udummy_desc_strings[value & 0xFF];
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
			if (len > sizeof(udummy_bosd)) {
				data->blen = len - sizeof(udummy_bosd);
				len = sizeof(udummy_bosd);
			} else
				data->blen = 0;
			memcpy(udata, &udummy_bosd, len);
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
udummy_data_handler(void *scarg, struct usb_data_xfer *xfer, int dir,
     int epctx)
{
	struct udummy_softc *sc = scarg;
	struct usb_data_xfer_block *data;
	int err;
	char *udata;
	uint i, idx;
	size_t len, slen;

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

	if (dir == USB_XFER_OUT && epctx == 1) {
		pthread_mutex_lock(&sc->uds_mtx);
		slen = strnlen(udata, sizeof (sc->uds_buf));
		if (slen > sizeof (sc->uds_buf) || slen == 0) {
			err = USB_ERR_SHORT_XFER;
			pthread_mutex_unlock(&sc->uds_mtx);
			goto done;
		}
		data->processed = 1;
		data->bdone = len;
		data->blen = 0;
		bzero(sc->uds_buf, sizeof (sc->uds_buf));
		bcopy(udata, sc->uds_buf, slen);
		rot13((uint8_t *)sc->uds_buf, sizeof (sc->uds_buf));
		pthread_mutex_unlock(&sc->uds_mtx);
	} else if (dir == USB_XFER_IN && epctx == 2) {
		pthread_mutex_lock(&sc->uds_mtx);
		if (len > sizeof (sc->uds_buf))
			len = sizeof (sc->uds_buf);
		bcopy(sc->uds_buf, udata, len);
		data->processed = 1;
		data->bdone += len;
		data->blen -= len;
		pthread_mutex_unlock(&sc->uds_mtx);
	} else {
		USB_DATA_SET_ERRCODE(data, USB_STALL);
		err = USB_ERR_STALLED;
	}

done:
	return (err);
}

static int
udummy_reset(void *scarg)
{
	struct udummy_softc *sc = scarg;

	(void)sc;

	return (0);
}

static int
udummy_remove(void *scarg)
{
	return (0);
}

static int
udummy_stop(void *scarg)
{

	return (0);
}

struct usb_devemu ue_dummy = {
	.ue_emu =	"dummy",
	.ue_usbver =	3,
	.ue_usbspeed =	USB_SPEED_HIGH,
	.ue_init =	udummy_init,
	.ue_request =	udummy_request,
	.ue_data =	udummy_data_handler,
	.ue_reset =	udummy_reset,
	.ue_remove =	udummy_remove,
	.ue_stop =	udummy_stop
};
USB_EMUL_SET(ue_dummy);
