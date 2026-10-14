/**
  ******************************************************************************
  * @file    mx_usbx_device_cdc_acm.h
  * @brief   USBX Device CDC ACM applicative header file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025-2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the mx_usbx_license.md file
  * in the same directory as the generated code.
  * If no mx_usbx_license.md file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef MX_USBX_DEVICE_CDC_ACM_H
#define MX_USBX_DEVICE_CDC_ACM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "ux_api.h"
#include "ux_device_class_cdc_acm.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions prototypes ---------------------------------------------*/
VOID usbd_cdc_acm_activate(VOID *cdc_acm_instance);
VOID usbd_cdc_acm_deactivate(VOID *cdc_acm_instance);
VOID usbd_cdc_acm_ParameterChange(VOID *cdc_acm_instance);
/* Private defines -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif
#endif  /* MX_USBX_DEVICE_CDC_ACM_H */