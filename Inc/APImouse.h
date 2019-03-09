/*
 * APImouse.h
 *
 *  Created on: 08.03.2019
 *      Author: Awnar
 */

#ifndef APIMOUSE_H_
#define APIMOUSE_H_

#include "usb_device.h"

	extern USBD_HandleTypeDef hUsbDeviceFS;

	struct mouseHID_t {
		uint8_t buttons;
		int8_t x;
		int8_t y;
		int8_t wheel;
	} mouseHID;

			void APImouse(){
				mouseHID.buttons = 0;
				mouseHID.x = 0;
				mouseHID.y = 0;
				mouseHID.wheel = 0;
			}

			void send(){
				USBD_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
				HAL_Delay(1000);
			}

#endif /* APIMOUSE_H_ */
