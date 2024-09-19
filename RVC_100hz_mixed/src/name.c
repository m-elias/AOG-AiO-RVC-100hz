//https://medium.com/@j0hnm4r5/changing-teensy-serial-name-and-port-name-2ca76552d26d
// Only Product Name shows up in properties->Details->Bus reported device description

// https://www.thewindowsclub.com/how-to-rename-hardware-in-device-manager-of-windows
// can manually edit device names

/*
#include <usb_names.h>

#define MANUFACTURER_NAME                       \
	{                                             \
		'A', 'g', 'O', 'p', 'e', 'n', 'G', 'P', 'S' \
	}
#define MANUFACTURER_NAME_LEN 9

#define PRODUCT_NAME                                                \
	{                                                                 \
		'A', 'i', 'O', 'v', '5', '.', '0', '-', 'P', 'r', 'o', 't', 'o' \
	}
#define PRODUCT_NAME_LEN 13

#define SERIAL_NUMBER   \
	{                     \
		'5', '.', '0', 'a'  \
	}
#define SERIAL_NUMBER_LEN 4

struct usb_string_descriptor_struct usb_string_manufacturer_name = {
	2 + MANUFACTURER_NAME_LEN * 2,
	3,
	MANUFACTURER_NAME};

struct usb_string_descriptor_struct usb_string_product_name = {
	2 + PRODUCT_NAME_LEN * 2,
	3,
	PRODUCT_NAME};

struct usb_string_descriptor_struct usb_string_serial_number = {
	2 + SERIAL_NUMBER_LEN * 2,
	3,
	SERIAL_NUMBER};

*/