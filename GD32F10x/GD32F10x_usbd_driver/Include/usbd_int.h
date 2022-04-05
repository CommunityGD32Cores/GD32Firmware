/*!
    \file  usbd_int.h
    \brief USB device interrupt handler header file
*/

/*
    Copyright (C) 2017 GigaDevice

    2014-12-26, V1.0.0, firmware for GD32F10x
    2017-06-20, V2.0.0, firmware for GD32F10x
*/

#ifndef USBD_INT_H
#define USBD_INT_H

#include "usbd_core.h"
#include "usbd_std.h"
#include "usbd_pwr.h"

extern usbd_core_handle_struct usb_device_dev;

typedef struct
{
    uint8_t (*SOF) (usbd_core_handle_struct *pudev); /*!< SOF ISR callback */
}usbd_int_cb_struct;

extern usbd_int_cb_struct *usbd_int_fops;

/* function declarations */
/* USB device interrupt service routine */
void  usbd_isr (void);

#endif /* USBD_INT_H */
