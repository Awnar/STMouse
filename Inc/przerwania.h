/*
#include "APImouse.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
	   	{
	   		mouseHID.x=-10;
	   		USBD_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
	   		//HAL_Delay(500);
	   	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET)
	   	{
	   		mouseHID.x=10;
	   		USBD_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
	   		//HAL_Delay(500);
	   	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_SET)
	   	{
	   		mouseHID.y=-10;
	   		USBD_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
	   		//HAL_Delay(500);
	   	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)
	   	{
	   		mouseHID.y=10;
	   		USBD_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
	   		//HAL_Delay(500);
	   	}



	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET)
	   	{
	   		mouseHID.buttons=1;
	   		USBD_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
	   		//HAL_Delay(500);
	   	}
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
	   	{
			mouseHID.buttons=1;
	   		USBD_HID_SendReport(&hUsbDeviceFS, &mouseHID, sizeof(struct mouseHID_t));
	   		//HAL_Delay(500);
	   	}
}
