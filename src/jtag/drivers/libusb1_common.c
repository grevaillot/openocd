/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 *                                                                         *
 *   Copyright (C) 2011 by Mauro Gamba <maurillo71@gmail.com>              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "log.h"
#include "libusb1_common.h"
#include "unicode.h"

static struct libusb_context *jtag_libusb_context; /**< Libusb context **/
static libusb_device **devs; /**< The usb device list **/

static bool jtag_libusb_match(struct libusb_device_descriptor *dev_desc,
		const uint16_t vids[], const uint16_t pids[])
{
	for (unsigned i = 0; vids[i]; i++) {
		if (dev_desc->idVendor == vids[i] &&
			dev_desc->idProduct == pids[i]) {
			return true;
		}
	}
	return false;
}
/* 
   Convert the serial number descriptor string from the device descriptor (possibly in wrong format with
   old devices) into a 24 characters string (hexadecimal representation of the 12-bytes STM32 unique ID)
*/
unsigned long compute_serial_str(unsigned char *descriptor,  char *str)
{
	int i;
	int len = descriptor[0];

	if (len == 26) {
		/* Size value returned by the "old" ST-Link DFU;
		In that case the DFU returns 12 8-bits values, separated by 12 zeros;
		This is a bad encoding of Unicode characters, that we workaround here ...
		*/
		for (i = 0; i < 12; i++) {
			/* Useful values reside at sNDescriptor[2],[4] ... [24]
			Each 8-bits value will be expressed on 2 hexadecimal digits in sNString
			*/
			sprintf((char *) &(str[i * 2]), "%02hX", (unsigned short)descriptor[2 * (i + 1)]);

		}
	} else if (len == 50) {
          /*
            Size value returned by the new ST-Link 
            In that case the DFU returns 24 Unicode characters;
            Simply convert in 1-byte character string
          */
            for (i = 0; i < 24; i++) {
                  str[i] = descriptor[2 * (i + 1)];
		}
	}
        /* In any cases add a NULL terminating character */
      str[24]='\0';

      return ERROR_OK;
}

/* Returns true if the string descriptor indexed by str_index in device matches string */
static bool serial_descriptor_equal(libusb_device_handle *device, uint8_t str_index,
				    const uint8_t *serial_utf8, bool print_mismatch)
{
	int retval;
	bool matched;
	unsigned char tbuf[255];
	uint16_t langid;
	uint8_t desc_utf16le[2+256*2+1];  /* Max size of string in UTF16 (+ 2 byte length) */
	uint8_t desc_utf8[256 + 1];

	if (str_index == 0)
		return false;

	/* Asking for the zero'th index is special - it returns a string
	 * descriptor that contains all the language IDs supported by the device.
	 * Typically there aren't many - often only one. The language IDs are 16
	 * bit numbers, and they start at the third byte in the descriptor. See
	 * USB 2.0 specification section 9.6.7 for more information.
	 * Note from libusb 1.0 sources (descriptor.c) */
	retval = libusb_get_string_descriptor(device, 0, 0, tbuf, sizeof(tbuf));
	if (retval < 4) {
		LOG_ERROR("libusb_get_descriptor() failed to obtain language id with %d",
				retval);
		return false;
	}

	langid = tbuf[2] | (tbuf[3] << 8);

	/* libusb1's libusb_get_string_descriptor_ascii() replaces non ASCII
	 * characters with '?' (0x3f). So use libusb_get_string_descriptor() instead.
	 * Non ASCII characters in USB serials are found in the wild on
	 * ST-Link and STM32 Discovery boards, which have serials like
	 * "Q\377j\006I\207PS(H\t\207".
	 * */

	memset(desc_utf16le, 0, sizeof(desc_utf16le));
	retval = libusb_get_string_descriptor(device, str_index, langid,
			desc_utf16le, sizeof(desc_utf16le) - 1);
	if (retval < 0) {
		LOG_ERROR("libusb_get_string_descriptor() failed with %d", retval);
		return false;
	}

	if (retval < 2
			|| desc_utf16le[1] != LIBUSB_DT_STRING
			|| desc_utf16le[0] > retval) {
		LOG_ERROR("libusb_get_string_descriptor() string descriptor "
				"validation failed");
		return false;
	}

	/* USB string descriptors are stored in UTF-16LE encoding.  Conversion to
	 * UTF-8 allow comparison with user entered serial number.
	 */
	if (utf16le_to_utf8(&desc_utf16le[2], retval - 2, desc_utf8, sizeof(desc_utf8)) < 0) {
		LOG_DEBUG("Invalid serial number utf-16le encoding");
		return false;
	}

        char serial_text2[256*4+1]; /* Max 256 byte descriptor formatted */
        char serial_text[256*4+1]; /* Max 256 byte descriptor formatted */
        compute_serial_str(desc_utf16le, serial_text2);
        utf8_to_text(serial_utf8, serial_text, sizeof(serial_text));
      
	matched = strcmp((const char *)serial_text,
			 (const char *)serial_text2) == 0;
	if (!matched) {

		/* as \xHH chars */
		char descriptor_text[256*4+1];

		utf8_to_text(serial_utf8, serial_text, sizeof(serial_text));
		utf8_to_text(desc_utf8, descriptor_text, sizeof(descriptor_text));

		if (print_mismatch)
			LOG_ERROR("Device serial number '%s' doesn't match requested "
				  "serial '%s'",
				  serial_text2,
				  serial_text);
	}

	return matched;
}

static struct jtag_libusb_device_handle *iterate_devs(const uint16_t vids[], const uint16_t pids[],
						      const char *serial_utf8, bool print_mismatch)
{
	struct jtag_libusb_device_handle *libusb_handle = NULL;
	int errCode;

	for (size_t idx = 0; devs[idx] != NULL; idx++) {
		struct libusb_device_descriptor dev_desc;

		if (libusb_get_device_descriptor(devs[idx], &dev_desc) != 0)
			continue;

		if (!jtag_libusb_match(&dev_desc, vids, pids))
			continue;

		errCode = libusb_open(devs[idx], &libusb_handle);

		if (errCode) {
			LOG_ERROR("libusb_open() failed with %s",
					libusb_error_name(errCode));
			continue;
		}

		/* Device must be open to use libusb_get_string_descriptor. */
		if (print_mismatch) {
			/* Device must be open to use libusb_get_string_descriptor. */
			if (serial_utf8 != NULL &&
					!serial_descriptor_equal(libusb_handle, dev_desc.iSerialNumber,
									(const uint8_t *)serial_utf8, print_mismatch)) {
				libusb_close(libusb_handle);
				continue;
			}
		}

		/* Success. */
		return libusb_handle;
	}
	return NULL;
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *serial_utf8,
		struct jtag_libusb_device_handle **out)
{
	if (libusb_init(&jtag_libusb_context) < 0)
		return -ENODEV;

	ssize_t cnt = libusb_get_device_list(jtag_libusb_context, &devs);
	if (cnt < 0) {
	  LOG_ERROR("libusb_get_device_list() failed with %s", serial_utf8); 
		return -ENODEV;
	}

	*out = iterate_devs(vids, pids, serial_utf8, false);

	if (serial_utf8 && *out == NULL)
		iterate_devs(vids, pids, serial_utf8, true);

	if (devs[0] != NULL)
		libusb_free_device_list(devs, 1);

	return *out != NULL ? 0 : -ENODEV;
}

void jtag_libusb_close(jtag_libusb_device_handle *dev)
{
	/* Close device */
	libusb_close(dev);

	libusb_exit(jtag_libusb_context);
}

int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev, uint8_t requestType,
		uint8_t request, uint16_t wValue, uint16_t wIndex, char *bytes,
		uint16_t size, unsigned int timeout)
{
	int transferred = 0;

	transferred = libusb_control_transfer(dev, requestType, request, wValue, wIndex,
				(unsigned char *)bytes, size, timeout);

	if (transferred < 0)
		transferred = 0;

	return transferred;
}

int jtag_libusb_bulk_write(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	int transferred = 0;

	libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
			     &transferred, timeout);
	return transferred;
}

int jtag_libusb_bulk_read(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	int transferred = 0;

	libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
			     &transferred, timeout);
	return transferred;
}

int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	int retCode = -99;

	struct libusb_config_descriptor *config = NULL;
	int current_config = -1;

	retCode = libusb_get_configuration(devh, &current_config);
	if (retCode != 0)
		return retCode;

	retCode = libusb_get_config_descriptor(udev, configuration, &config);
	if (retCode != 0 || config == NULL)
		return retCode;

	/* Only change the configuration if it is not already set to the
	   same one. Otherwise this issues a lightweight reset and hangs
	   LPC-Link2 with JLink firmware. */
	if (current_config != config->bConfigurationValue)
		retCode = libusb_set_configuration(devh, config->bConfigurationValue);

	libusb_free_config_descriptor(config);

	return retCode;
}

int jtag_libusb_choose_interface(struct jtag_libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	const struct libusb_interface *inter;
	const struct libusb_interface_descriptor *interdesc;
	const struct libusb_endpoint_descriptor *epdesc;
	struct libusb_config_descriptor *config;

	*usb_read_ep = *usb_write_ep = 0;

	libusb_get_config_descriptor(udev, 0, &config);
	for (int i = 0; i < (int)config->bNumInterfaces; i++) {
		inter = &config->interface[i];

		interdesc = &inter->altsetting[0];
		for (int k = 0;
		     k < (int)interdesc->bNumEndpoints; k++) {
			if ((bclass > 0 && interdesc->bInterfaceClass != bclass) ||
			    (subclass > 0 && interdesc->bInterfaceSubClass != subclass) ||
			    (protocol > 0 && interdesc->bInterfaceProtocol != protocol))
				continue;

			epdesc = &interdesc->endpoint[k];

			uint8_t epnum = epdesc->bEndpointAddress;
			bool is_input = epnum & 0x80;
			LOG_DEBUG("usb ep %s %02x",
				  is_input ? "in" : "out", epnum);

			if (is_input)
				*usb_read_ep = epnum;
			else
				*usb_write_ep = epnum;

			if (*usb_read_ep && *usb_write_ep) {
				LOG_DEBUG("Claiming interface %d", (int)interdesc->bInterfaceNumber);
				libusb_claim_interface(devh, (int)interdesc->bInterfaceNumber);
				libusb_free_config_descriptor(config);
				return ERROR_OK;
			}
		}
	}
	libusb_free_config_descriptor(config);

	return ERROR_FAIL;
}

int jtag_libusb_get_pid(struct jtag_libusb_device_handle *devh, uint16_t *pid)
{
	struct jtag_libusb_device *dev = jtag_libusb_get_device(devh);
	struct libusb_device_descriptor dev_desc;

	if (libusb_get_device_descriptor(dev, &dev_desc) == 0) {
		*pid = dev_desc.idProduct;

		return ERROR_OK;
	}

	return ERROR_FAIL;
}
