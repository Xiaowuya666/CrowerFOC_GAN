/**
  ******************************************************************************
  * @file    mx_usbx_device.c
  * @brief   USBX Device applicative source file
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

/* Includes ------------------------------------------------------------------*/
#include "mx_usbx_device.h"
#include "ux_dcd_stm32.h"
#include "mx_usbx_device_cdc_acm.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/**
  * @brief  Application USBX Device Initialization.
  */
UINT app_usbx_device_init(VOID)
{
  UINT status = UX_SUCCESS;
  USB_DESCRIPTOR                    usbd_desc;
  ULONG                             report_desc_len;
  UX_SLAVE_CLASS_CDC_ACM_PARAMETER  cdc_acm_parameter;
  report_desc_len = sizeof(cdc_acm_parameter);
  usb_device_descriptor_register_class(USBD_CLASS_TYPE_CDC_ACM, cdc_acm_interface, &report_desc_len);

  usb_device_descriptor_get_framework(&usbd_desc);

  /* Install the device portion of USBX */
  status = ux_device_stack_initialize(usbd_desc.device_high_speed.framework,
                                      usbd_desc.device_high_speed.framework_length,
                                      usbd_desc.device_full_speed.framework,
                                      usbd_desc.device_full_speed.framework_length,
                                      usbd_desc.string.framework,
                                      usbd_desc.string.framework_length,
                                      usbd_desc.languageid.framework,
                                      usbd_desc.languageid.framework_length,
                                      UX_NULL);

  if (status != UX_SUCCESS)
  {
    return status;
  }
  /* Initialize the cdc acm class parameters for the device */
  cdc_acm_parameter.ux_slave_class_cdc_acm_instance_activate   = usbd_cdc_acm_activate;
  cdc_acm_parameter.ux_slave_class_cdc_acm_instance_deactivate = usbd_cdc_acm_deactivate;
  cdc_acm_parameter.ux_slave_class_cdc_acm_parameter_change    = usbd_cdc_acm_ParameterChange;

  /* Initialize the device cdc acm class */
  status = ux_device_stack_class_register(_ux_system_slave_class_cdc_acm_name,
                                          ux_device_class_cdc_acm_entry,
                                          0x01,
                                          cdc_acm_interface->interface_numbers,
                                          &cdc_acm_parameter);

  if (status != UX_SUCCESS)
  {
    return status;
  }
  return UX_SUCCESS;
}

/**
  * @brief  Application USBX Device De-Initialization.
  * @retval none
  */
UINT app_usbx_device_deinit(VOID)
{
  UINT status = UX_SUCCESS;


  /* Unregister CDC ACM class. */
  status = ux_device_stack_class_unregister(_ux_system_slave_class_cdc_acm_name, ux_device_class_cdc_acm_entry);
  if (status != UX_SUCCESS)
  {
    return status;
  }

  /* The code below is required for uninstalling the device portion of USBX.  */
  status = ux_device_stack_uninitialize();

  if (status != UX_SUCCESS)
  {
    return status;
  }
  return UX_SUCCESS;
}
/**
  * @brief  Application USBX Device Process.
  * @retval none
  */
UINT app_usbx_device_process(VOID)
{
  UINT return_status = UX_SUCCESS;
  return_status = ux_device_stack_tasks_run();
  return return_status;
}
