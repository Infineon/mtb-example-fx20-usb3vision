/***************************************************************************//**
* \file fx3g2_descriptors.c
* \version 1.0
*
* Defines USB descriptors used by the FX10 firmware application.
*
*******************************************************************************
* \copyright
* (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "cy_pdl.h"
#include "cy_usb_u3v_device.h"
#include "cy_usb_common.h"
#include "cy_usbhs_cal_drv.h"
#include "cy_usb_usbd.h"
#include "cy_usb_app.h"


/* U3V Device0 should be enabled by default. */
#define U3V_DEV0_ENABLE                 (1)

#define CY_FX_USB_VID                   (0x04B4)
#define CY_FX_USB_PID                   (0x2021)

/* Set to 1 to enable Usb3Vision Device Event Interface (DEI)
 * TODO: Actual implementation is not available at present.
 */
#define ENABLE_U3V_EVT_INTERFACE0        (0)
#define U3V_INTERFACE_NUM                (2)
#define TOTAL_U3V_INTERFACES_FOR_DEV0    (U3V_INTERFACE_NUM * U3V_DEV0_ENABLE \
                                                + ENABLE_U3V_EVT_INTERFACE0)

/* Interface numbers*/
#define U3V_DEV0_DCINTF_INDEX            (0)
#define U3V_DEV0_EVTNTF_INDEX            (1 * ENABLE_U3V_EVT_INTERFACE0)
#define U3V_DEV0_DSINTF_INDEX            (U3V_DEV0_EVTNTF_INDEX + 1)

/* Minimum length of USB-SS Configuration Descriptor for U3V device. */
#define USB_CFGDSCR_BASE_LENGTH          (9u)
#define U3V_SS_CFGDSCR_BASE_LENGTH       (85u)

/* Additional length of USB-SS Device Event Interface descriptor. */
#define U3V_SS_EVTINTF_DSCR_LENGTH       (22u)

/* Additional length of USB-SS HID interface descriptor. */
#define SS_HID_INTF_LENGTH               (9u)

/* Minimum length of USB-HS Configuration Descriptor for U3V device. */
#define U3V_HS_CFGDSCR_BASE_LENGTH       (67u)

/* Additional length of USB-SS Device Event Interface descriptor. */
#define U3V_HS_EVTINTF_DSCR_LENGTH       (16u)

/* Additional length of USB-HS HID interface descriptor. */
#define HS_HID_INTF_LENGTH               (9u)

/* Total number of interfaces supported by the SS device. */
#define USBSS_NUM_INTERFACES             (TOTAL_U3V_INTERFACES_FOR_DEV0)

/* Total number of interfaces supported by the HS device. */
#define USBHS_NUM_INTERFACES             (TOTAL_U3V_INTERFACES_FOR_DEV0)

/* USB-SS configuration descriptor length. */
#define USBSS_CFG_DSCR_LENGTH   (                                  \
        USB_CFGDSCR_BASE_LENGTH +                                  \
        (U3V_DEV0_ENABLE * U3V_SS_CFGDSCR_BASE_LENGTH) +           \
        (ENABLE_U3V_EVT_INTERFACE0 * U3V_SS_EVTINTF_DSCR_LENGTH) )

/* USB-HS configuration descriptor length. */
#define USBHS_CFG_DSCR_LENGTH   (                                   \
        USB_CFGDSCR_BASE_LENGTH +                                   \
        (U3V_DEV0_ENABLE * U3V_HS_CFGDSCR_BASE_LENGTH) +            \
        (ENABLE_U3V_EVT_INTERFACE0 * U3V_HS_EVTINTF_DSCR_LENGTH) )

/* Standard device descriptor for full speed (FS) */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFSDeviceDscr[32] __attribute__ ((aligned (32)));
const uint8_t FullSpeedDeviceDescr[] =
{
    0x12,                           /* Descriptor size */
    0x01,                           /* Device descriptor type */
    0x10,0x02,                      /* USB 2.1 */
    0x00,                           /* Device class: Defined in interface */
    0x00,                           /* Device Sub-class:  Defined in interface */
    0x00,                           /* Device protocol: Defined in interface */
    0x40,                           /* Maxpacket size for EP0 */
    CY_USB_GET_LSB(CY_FX_USB_VID),  /* Vendor ID */
    CY_USB_GET_MSB(CY_FX_USB_VID),
    CY_USB_GET_LSB(CY_FX_USB_PID),  /* Product ID */
    CY_USB_GET_MSB(CY_FX_USB_PID),
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x09,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Standard device descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSB20DeviceDscr[18] __attribute__ ((aligned (32)));
const uint8_t Usb2DeviceDscr[] =
{
    0x12,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_DEVICE,        /* Device descriptor type */
    0x00,0x02,                      /* USB 2.00 (TODO: Avoiding LPM-L1 support at present). */
    0xEF,                           /* Device class */
    0x02,                           /* Device sub-class */
    0x01,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    CY_USB_GET_LSB(CY_FX_USB_VID),  /* Vendor ID */
    CY_USB_GET_MSB(CY_FX_USB_VID),
    CY_USB_GET_LSB(CY_FX_USB_PID),  /* Product ID */
    CY_USB_GET_MSB(CY_FX_USB_PID),
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x09,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

USB3_DESC_ATTRIBUTES uint8_t CyFxUSB30DeviceDscr[18] __attribute__ ((aligned (32)));
const uint8_t Usb3DeviceDscr[] =
{
    0x12,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_DEVICE,        /* Device descriptor type */
    0x20,0x03,                      /* USB 3.2*/
    0xEF,                           /* Device class */
    0x02,                           /* Device Sub-class */
    0x01,                           /* Device protocol */
    0x09,                           /* Maxpacket size for EP0 : 2^9 */
    CY_USB_GET_LSB(CY_FX_USB_VID),  /* Vendor ID */
    CY_USB_GET_MSB(CY_FX_USB_VID),
    CY_USB_GET_LSB(CY_FX_USB_PID),  /* Product ID */
    CY_USB_GET_MSB(CY_FX_USB_PID),
    0x00,0x00,                      /* Device release number */
    0x01,                           /* Manufacture string index */
    0x02,                           /* Product string index */
    0x09,                           /* Serial number string index */
    0x01                            /* Number of configurations */
};

/* Binary device object store descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBBOSDscr[64] __attribute__ ((aligned (32)));
const uint8_t BosDescriptor[] =
{
    0x05,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_BOS,           /* Device descriptor type */
    0x16,0x00,                      /* Length of this descriptor and all sub descriptors */
    0x02,                           /* Number of device capability descriptors */

    /* USB 2.0 Extension */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_DEVICE_CAP,    /* Device capability type descriptor */
    CY_USB_USB2_EXTN_CAPB_TYPE,     /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed Device Capability */
    0x0A,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_DEVICE_CAP,    /* Device capability type descriptor */
    CY_USB_SS_CAPB_TYPE,            /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features  */
    0x0E,0x00,                      /* Speeds supported by the device : SS, HS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 device exit latency */
    0xFF,0x07                       /* U2 device exit latency */
};

/* Binary Object Store (BOS) Descriptor to be used in SuperSpeedPlus connection. */
USB3_DESC_ATTRIBUTES uint8_t CyFxBOSDscr_Gen2[] =
{
    0x05,                           /* Descriptor size */
    0x0F,                           /* BOS descriptor. */
    0x2D,0x00,                      /* Length of this descriptor and all sub-descriptors */
    0x04,                           /* Number of device capability descriptors */

    /* USB 2.0 extension */
    0x07,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x02,                           /* USB 2.0 extension capability type */
    0x1E,0x64,0x00,0x00,            /* Supported device level features: LPM support, BESL supported,
                                       Baseline BESL=400 us, Deep BESL=1000 us. */

    /* SuperSpeed device capability */
    0x0A,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x03,                           /* SuperSpeed device capability type */
    0x00,                           /* Supported device level features: Not LTM capable.  */
    0x0E,0x00,                      /* Speeds supported by the device : SS Gen1, HS and FS */
    0x03,                           /* Functionality support */
    0x0A,                           /* U1 Device Exit latency */
    0xFF,0x07,                      /* U2 Device Exit latency */

    /* SuperSpeedPlus USB device capability */
    0x14,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0A,                           /* SuperSpeedPlus Device capability */
    0x00,                           /* Reserved */
    0x01,0x00,0x00,0x00,            /* SSAC=1, SSIC=0 */
    0x00,0x11,                      /* SSID=0, Min. RX Lane = 1, Min. Tx Lane = 1 */
    0x00,0x00,                      /* Reserved */
    0x30,0x40,0x0A,0x00,            /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Rx), LP=1(SSPlus), LSM=10 */
    0xB0,0x40,0x0A,0x00,             /* SSID=0, LSE=3(Gb/s), ST=0(Symmetric Tx), LP=1(SSPlus), LSM=10 */

    /* PTM device capability*/
    0x03,                           /* Descriptor size */
    0x10,                           /* Device capability type descriptor */
    0x0B                            /* Precision Time Measurement (PTM) Capability Descriptor */
};

/* Full Speed Configuration Descriptor*/
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFSConfigDscr[64] __attribute__ ((aligned (32)));
const uint8_t FullSpeedConfigDescr[] =
{
    /* Configuration descriptor */
    0x09,                               /* Descriptor size */
    CY_USB_DSCR_TYPE_CFG,               /* Configuration descriptor type */
    0x12,0x00,                          /* Total length of the descriptor. */
    0x01,                               /* Number of interfaces */
    0x01,                               /* Configuration number */
    0x09,                               /* Configuration string index */
    0x80,                               /* Config characteristics - bus powered */
    0x32,                               /* Max power consumption of device (in 2mA unit) : 100mA */

    /* interface descriptor */
    0x09,                           /* Descriptor size */
    0x04,                           /* Interface descriptor type */
    0x00,                           /* Interface number */
    0x00,                           /* Alternate setting number */
    0x00,                           /* Number of end points */
    0xFF,                           /* Interface class: Vendor defined */
    0x00,                           /* Interface sub class: Vendor defined */
    0x00,                           /* Interface protocol: Vendor defined */
    0x00                            /* Interface descriptor string index */
};

/* Standard High Speed Configuration Descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBHSConfigDscr[512] __attribute__ ((aligned (32)));
const uint8_t HighSpeedConfigDescr[] =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_CFG,           /* Configuration descriptor type */
    CY_USB_GET_LSB(USBHS_CFG_DSCR_LENGTH),
    CY_USB_GET_MSB(USBHS_CFG_DSCR_LENGTH), /* Total config descriptor length. */
    USBHS_NUM_INTERFACES,              /* Number of interfaces */ 
    0x01,                           /* Configuration number */
    0x08,                           /* Configuration string index */
    0x80,                           /* Config characteristics - bus powered */
    0xC8,                           /* Max power consumption of device (in 2mA unit) : 400mA */

    /* Interface association descriptor */
    0x08,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF_ASSOC,    /* Interface association descr type */
    U3V_DEV0_DCINTF_INDEX,          /* I/f number of first video control i/f */
    U3V_INTERFACE_NUM,              /* Number of U3V related interfaces. */
    0xEF,                           /* Miscellaneous Class */
    0x05,                           /* USB3Vision Class */
    0x00,                           /* Protocol : not used */
    0x03,                           /* String desc index for interface */

    /* Interface descriptor 0*/
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF,          /* Interface descriptor type */
    U3V_DEV0_DCINTF_INDEX,          /* Interface number */
    0x00,                           /* Alternate setting number */
    0x02,                           /* Number of end points */
    0xEF,                           /* Miscellaneous class */
    0x05,                           /* USB3Vision sub class */
    0x00,                           /* Interface protocol code */
    0x00,                           /* Interface descriptor string index */

    /* Class specific device info descriptor */
    0x14,                           /* Descriptor Size */
    0x24,                           /* Descriptor type = U3V_INTERFACE */
    0x01,                           /* Descriptor Subtype = U3V_DEVICEINFO */
    0x00, 0x00, 0x01, 0x00,         /* GenCP Version */
    0x00, 0x00, 0x01, 0x00,         /* U3V Version */
    0x04,                           /* U3V GUID String index */
    0x01,                           /* Vendor Name String index */
    0x02,                           /* Model Name String index */
    0x05,                           /* Family Name String index */
    0x06,                           /* Device Version String index */
    0x01,                           /* Manufacturer info string index */
    0x09,                           /* Serial Number string index */
    0x02,                           /* User Defined Name string index */
    0x0C,                           /* USB Speed Support - SS and HS supported */

    /* endpoint descriptor : Command */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    CY_U3V_EP_DCI_CMD,              /* Endpoint address and description */
    CY_USB_EP_BULK,                 /* Interrupt end point type */
    0x00,0x02,                      /* Max packet size = 512 bytes */
    0x00,                           /* Servicing interval : NA */

    /* endpoint descriptor : Response */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    0x80 | CY_U3V_EP_DCI_RSP,       /* Endpoint address and description */
    CY_USB_EP_BULK,                 /* Interrupt end point type */
    0x00,0x02,                      /* Max packet size = 512 bytes */
    0x00,                           /* Servicing interval : NA */

#if ENABLE_U3V_EVT_INTERFACE0
    /* Device Event Interface Descriptor */
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF,          /* Interface Descriptor Type */
    U3V_DEV0_EVTNTF_INDEX,                           /* Interface Number */
    0x00,                           /* Alternate Setting */
    0x01,                           /* Number of Endpoints */
    0xEF,                           /* Miscellaneous Class */
    0x05,                           /* USB3 Vision */
    0x01,                           /* Device Events */
    0x00,                           /* Interface String index */

    /* Endpoint descriptor */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    CY_FX_EP_DEI_EVENT,             /* bEndpoint Address */
    CY_USB_EP_BULK,                 /* bmAttributes */
    0x00,0x02,                      /* wMaxPacketSize */
    0x00,                           /* bInterval */
#endif /* ENABLE_U3V_EVT_INTERFACE0 */

    /* Device Streaming Interface Descriptor */
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF,          /* Interface Descriptor Type */
    U3V_DEV0_DSINTF_INDEX,               /* Interface Number */
    0x00,                           /* Alternate Setting */
    0x01,                           /* Number of Endpoints */
    0xEF,                           /* Miscellaneous Class */
    0x05,                           /* USB3 Vision */
    0x02,                           /* Device Streaming */
    0x00,                           /* Interface String index */

    /* Endpoint descriptor for streaming video data */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    0x80 | CY_U3V_EP_DSI_STREAM,    /* Endpoint address and description */
    CY_USB_EP_BULK,                 /* Bulk End Point */
    0x00, 0x02,                     /* Maximum packet size. */
    0x00,                            /* Servicing interval for data transfers */
};

/* Standard super speed configuration descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBSSConfigDscr[1024] __attribute__ ((aligned (32)));
const uint8_t SuperSpeedConfigDescr[] =
{
    /* Configuration descriptor */
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_CFG,           /* Configuration descriptor type */
    CY_USB_GET_LSB(USBSS_CFG_DSCR_LENGTH),
    CY_USB_GET_MSB(USBSS_CFG_DSCR_LENGTH), /* Total config descriptor length. */
    USBSS_NUM_INTERFACES,              /* Number of interfaces */  
    0x01,                           /* Configuration number */
    0x00,                           /* Configuration string index */
    0x80,                           /* Config characteristics - Bus powered */
    0x32,                           /* Max power consumption of device (in 8mA unit) : 400mA */

    /* Interface association descriptor */
    0x08,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF_ASSOC,    /* Interface association descr type */
    U3V_DEV0_DCINTF_INDEX,          /* I/f number of first VideoControl i/f */
    U3V_INTERFACE_NUM,              /* Number of U3V related interfaces. */
    0xEF,                           /* Miscellaneous Class */
    0x05,                           /* USB3 Vision */
    0x00,                           /* Protocol : Not used */
    0x03,                           /* String desc index for interface */

    /* Device control interface descriptor */
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF,          /* Interface descriptor type */
    U3V_DEV0_DCINTF_INDEX,          /* Interface number [R-10cd: The default Device Control interface MUST be the first IAD interface]*/
    0x00,                           /* Alternate setting number, [R-13cd: MUST NOT have alternate Device Control interfaces] */
    0x02,                           /* Number of end points [R-11cd:  default Device Control interface requires two endpoints]*/
    0xEF,                           /* Miscellaneous class */
    0x05,                           /* USB3 Vision */
    0x00,                           /* Protocol : Device Control */
    0x00,                           /* Interface descriptor string index */
    /* Class specific device info descriptor */
    0x14,                           /* Descriptor Size */
    0x24,                           /* Descriptor type = U3V_INTERFACE */
    0x01,                           /* Descriptor Subtype = U3V_DEVICEINFO */
    DWORD_MSB_BYTES(GENCP_VERSION), /* GenCP Version */
    DWORD_LSB_BYTES(GENCP_VERSION), /* GenCP Version */
    0x00, 0x00, 0x01, 0x00,         /* U3V Version */
    0x04,                           /* U3V GUID String index */
    0x01,                           /* Vendor Name String index */
    0x02,                           /* Model Name String index */
    0x05,                           /* Family Name String index */
    0x06,                           /* Device Version String index */
    0x01,                           /* Manufacturer info string index */
    0x09,                           /* Serial Number string index */
    0x02,                           /* User Defined Name string index */
    0x1C,                           /* USB Speed Support for U3V- GEN2,GEN1 and HS supported */

    /* Device control status endpoint descriptor, [R-27cd]  */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    CY_U3V_EP_DCI_CMD,              /* Endpoint address and description */
    CY_USB_EP_BULK,                 /* Bulk end point type */
    0x00,0x04,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval */

    /* Super speed endpoint companion descriptor */
    0x06,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_SS_ENDP_COMP,  /* SS endpoint companion descriptor type */
    0x00,                           /* Max no. of packets in a Burst : 1 */
    0x00,                           /* Mult: Max number of packets : 1 */
    0x00, 0x00,                     /* Bytes per interval : N/A */

    /* Video control status endpoint descriptor */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    0x80 | CY_U3V_EP_DCI_RSP,       /* Endpoint address and description */
    CY_USB_EP_BULK,                 /* Bulk end point type */
    0x00,0x04,                      /* Max packet size = 1024 bytes */
    0x00,                           /* Servicing interval */

    /* Super speed endpoint companion descriptor */
    0x06,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_SS_ENDP_COMP,  /* SS endpoint companion descriptor type */
    0x00,                           /* Max no. of packets in a Burst : 1 */
    0x00,                           /* Mult: Max number of packets : 1 */
    0x00, 0x00,                     /* Bytes per interval : N/A */

#if ENABLE_U3V_EVT_INTERFACE0
    /* Device Event Interface Descriptor */
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF,          /* Interface Descriptor Type */
    U3V_DEV0_EVTNTF_INDEX,          /* Interface Number */
    0x00,                           /* Alternate Setting */
    0x01,                           /* Number of Endpoints */
    0xEF,                           /* Miscellaneous Class */
    0x05,                           /* USB3 Vision */
    0x01,                           /* Device Events */
    0x00,                           /* Interface String index */

    /* Endpoint descriptor */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    CY_FX_EP_DEI_EVENT,             /* bEndpoint Address */
    CY_USB_EP_BULK,                 /* bmAttributes */
    0x00, 0x04,                     /* wMaxPacketSize */
    0x00,                           /* bInterval */

    /* Super speed endpoint companion descriptor */
    0x06,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_SS_ENDP_COMP,  /* SS endpoint companion descriptor type */
    0x00,                           /* Max no. of packets in a Burst : 1 */
    0x00,                           /* Mult: Max number of packets : 1 */
    0x00, 0x00,                     /* Bytes per interval : N/A */
#endif /* ENABLE_U3V_EVT_INTERFACE0 */

    /* Device Streaming Interface Descriptor */
    0x09,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_INTF,          /* Interface Descriptor Type */
    U3V_DEV0_DSINTF_INDEX,          /* Interface Number */
    0x00,                           /* Alternate Setting [R-25cd]*/
    0x01,                           /* Number of Endpoints [CR-24cd]*/
    0xEF,                           /* Miscellaneous Class [CR-23cd]*/
    0x05,                           /* USB3 Vision [CR-23cd]*/
    0x02,                           /* Device Streaming [CR-23cd]*/
    0x00,                           /* Interface String index */

    /* Endpoint descriptor for streaming video data */
    0x07,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_ENDP,          /* Endpoint descriptor type */
    0x80 | CY_U3V_EP_DSI_STREAM,    /* Endpoint address and description */
    CY_USB_EP_BULK,                 /* Bulk End Point */
    0x00, 0x04,                     /* Maximum packet size. */
    0x00,                           /* Servicing interval for data transfers */

    /* Super speed endpoint companion descriptor */
    0x06,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_SS_ENDP_COMP,  /* SS endpoint companion descriptor type */
    (CY_U3V_STREAM_EP_BURST-1),     /* Max no. of packets in a Burst : 16 */
    0x00,                           /* Mult.: Max number of packets : 1 */
    0x00,0x00,                       /* Field Valid only for Periodic Endpoints */
};


/* Standard device qualifier descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBDeviceQualDscr[16] __attribute__ ((aligned (32)));
const uint8_t DeviceQualDescriptor[]  =
{
    0x0A,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_DEVICE_QUALIFIER,    /* Device qualifier descriptor type */
    0x00,0x02,                      /* USB 2.0 */
    0xEF,                           /* Device class */
    0x02,                           /* Device sub-class */
    0x01,                           /* Device protocol */
    0x40,                           /* Maxpacket size for EP0 : 64 bytes */
    0x01,                           /* Number of configurations */
    0x00                            /* Reserved */
};

/* Standard language ID string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBStringLangIDDscr[16] __attribute__ ((aligned (32)));
const uint8_t LangStringDescr[] =
{
    0x04,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_STR,           /* Device descriptor type */
    0x09,0x04                       /* Language ID supported */
};

/* Standard manufacturer string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBManufactureDscr[32] __attribute__ ((aligned (32)));
const uint8_t MfgStringDescr[] =
{
    0x12,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_STR,           /* Device descriptor type */
    'I',0x00,
    'n',0x00,
    'f',0x00,
    'i',0x00,
    'n',0x00,
    'e',0x00,
    'o',0x00,
    'n',0x00
};

/* Standard product string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBProductDscr[16] __attribute__ ((aligned (32)));
const uint8_t ProdStringDescr[] =
{
    0x0A,                           /* Descriptor Size */
    CY_USB_DSCR_TYPE_STR,           /* Device descriptor type */
    'F',0x00,
    'X',0x00,
    '2',0x00,
    '0',0x00,
};

/* Function string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFunctionDscr[64] __attribute__ ((aligned (32)));
const uint8_t  FunctionDscr[] =
{
    0x26,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_STR,           /* Device descriptor type */
    'U',0x00,
    'S',0x00,
    'B',0x00,
    '3',0x00,
    ' ',0x00,
    'V',0x00,
    'i',0x00,
    's',0x00,
    'i',0x00,
    'o',0x00,
    'n',0x00,
    ' ',0x00,
    'D',0x00,
    'e',0x00,
    'v',0x00,
    'i',0x00,
    'c',0x00,
    'e',0x00
};

/* Standard GUID string descriptor */
/* R-1cd: 04B4 + 8 unique numer ID*/
/* CR-2cd: Add U3V before GUID on device/box*/
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBGUIDDscr[32] __attribute__ ((aligned (32)));
const uint8_t GUIDDscr[] =
{
    0x1A,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_STR,           /* Device descriptor type */
    '0',0x00,
    '4',0x00,
    'B',0x00,
    '4',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '1',0x00
};

/* Serial Number string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBSNumberDscr[32] __attribute__ ((aligned (32)));
const uint8_t SNumberDscr[] =
{
    0x1A,                           /* Descriptor size */
    CY_USB_DSCR_TYPE_STR,           /* Device descriptor type */
    'S',0x00,
    'N',0x00,
    ':',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '0',0x00,
    '1',0x00
};

/* Family Name String Descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFamilyNameDscr[16] __attribute__ ((aligned (32)));
const uint8_t FamilyNameDscr[] =
{
    0x0E,
    CY_USB_DSCR_TYPE_STR,
    'E',0x00,
    'Z',0x00,
    '-',0x00,
    'U',0x00,
    'S',0x00,
    'B',0x00
};

/* Device Version String Descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBDeviceVerDscr[16] __attribute__ ((aligned (32)));
const uint8_t DeviceVerDscr[] =
{
    0x0C,
    CY_USB_DSCR_TYPE_STR,
    'R',0x00,
    'E',0x00,
    'V',0x00,
    '-',0x00,
    'C',0x00
};

/* SS Configuration string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBConfigSSDscr[16] __attribute__ ((aligned (32)));
const uint8_t SSDscr[] =
{
    0x10,                               /* Descriptor Size */
    CY_USB_DSCR_TYPE_STR,               /* Device descriptor type */
    'U', 0x00,                          /* Super-Speed Configuration Descriptor */
    'S', 0x00,
    'B', 0x00,
    '-', 0x00,
    '3', 0x00,
    '.', 0x00,
    '2', 0x00
};
/*  HS Configuration string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBConfigHSDscr[16]  __attribute__ ((aligned (32)));
 const uint8_t HSDscr[] =
{
    0x10,                               /* Descriptor Size */
    CY_USB_DSCR_TYPE_STR,               /* Device descriptor type */
    'U', 0x00,                          /* High-Speed Configuration Descriptor */
    'S', 0x00,
    'B', 0x00,
    '-', 0x00,
    '2', 0x00,
    '.', 0x00,
    '1', 0x00
};

/* FS Not Supported string descriptor */
USB3_DESC_ATTRIBUTES uint8_t CyFxUSBFSDscr[64] __attribute__ ((aligned (32)));
const uint8_t FullSpeedStringDescr[] =
{
    0x22,        /* Descriptor Size */
    0x03,        /* Device descriptor type */
    'F',0x00,
    'S',0x00,
    ' ',0x00,
    'N',0x00,
    'O',0x00,
    'T',0x00,
    ' ',0x00,
    'S',0x00,
    'U',0x00,
    'P',0x00,
    'P',0x00,
    'O',0x00,
    'R',0x00,
    'T',0x00,
    'E',0x00,
    'D',0x00
};


void CopyDescriptorsToHBRam (void)
{
    memcpy (CyFxUSB30DeviceDscr, Usb3DeviceDscr, sizeof(Usb3DeviceDscr));
    memcpy (CyFxUSB20DeviceDscr, Usb2DeviceDscr, sizeof(Usb2DeviceDscr));
    memcpy (CyFxUSBBOSDscr, BosDescriptor, sizeof(BosDescriptor));
    memcpy (CyFxUSBDeviceQualDscr, DeviceQualDescriptor, sizeof(DeviceQualDescriptor));
    memcpy (CyFxUSBSSConfigDscr, SuperSpeedConfigDescr, sizeof(SuperSpeedConfigDescr));
    memcpy (CyFxUSBHSConfigDscr, HighSpeedConfigDescr, sizeof(HighSpeedConfigDescr));
    memcpy (CyFxUSBFSDeviceDscr, FullSpeedDeviceDescr, sizeof(FullSpeedDeviceDescr));
    memcpy (CyFxUSBFSConfigDscr, FullSpeedConfigDescr, sizeof(FullSpeedConfigDescr));
    memcpy (CyFxUSBStringLangIDDscr, LangStringDescr, sizeof(LangStringDescr));
    memcpy (CyFxUSBManufactureDscr, MfgStringDescr, sizeof(MfgStringDescr));
    memcpy (CyFxUSBProductDscr, ProdStringDescr, sizeof(ProdStringDescr));
    memcpy (CyFxUSBFSDscr, FullSpeedStringDescr, sizeof(FullSpeedStringDescr));
    memcpy (CyFxUSBConfigHSDscr, HSDscr, sizeof(HSDscr));
    memcpy (CyFxUSBConfigSSDscr, SSDscr, sizeof(SSDscr));
    memcpy (CyFxUSBDeviceVerDscr, DeviceVerDscr, sizeof(DeviceVerDscr));
    memcpy (CyFxUSBFamilyNameDscr, FamilyNameDscr, sizeof(FamilyNameDscr));
    memcpy (CyFxUSBGUIDDscr, GUIDDscr, sizeof(GUIDDscr));
    memcpy (CyFxUSBSNumberDscr, SNumberDscr, sizeof(SNumberDscr));
    memcpy (CyFxUSBFunctionDscr, FunctionDscr, sizeof(FunctionDscr));
}

void CyApp_RegisterUsbDescriptors(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_en_usb_speed_t usbSpeed)
{
    /* Can be moved so that copy is only done once. */
    CopyDescriptorsToHBRam();

    if ((pAppCtxt != NULL) && (pAppCtxt->pUsbdCtxt != NULL))  {

        /* Register USB descriptors with the stack for USB >=HS */
        if ((usbSpeed >= CY_USBD_USB_DEV_HS) )
        {
            DBG_APP_INFO("Register HS/SS Descriptor\r\n");
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB20DeviceDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSB30DeviceDscr);

            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBHSConfigDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBSSConfigDscr);

            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxUSBProductDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 3, (uint8_t *)CyFxUSBFunctionDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 4, (uint8_t *)CyFxUSBGUIDDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 5, (uint8_t *)CyFxUSBFamilyNameDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 6, (uint8_t *)CyFxUSBDeviceVerDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 7, (uint8_t *)CyFxUSBConfigSSDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 8, (uint8_t *)CyFxUSBConfigHSDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 9, (uint8_t *)CyFxUSBSNumberDscr);

            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_DEVICE_QUAL_DSCR, 0, (uint8_t *)CyFxUSBDeviceQualDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr);

            if (usbSpeed >= CY_USBD_USB_DEV_SS_GEN2) {
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxBOSDscr_Gen2);
            } else {
                Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_SS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr);
            }
        }
        else {
            /* FOR USB FS */
            DBG_APP_INFO("Register FS Descriptor\r\n");
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_DEVICE_DSCR, 0, (uint8_t *)CyFxUSBFSDeviceDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_FS_CONFIG_DSCR, 0, (uint8_t *)CyFxUSBFSConfigDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 0, (uint8_t *)CyFxUSBStringLangIDDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 1, (uint8_t *)CyFxUSBManufactureDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 2, (uint8_t *)CyFxUSBProductDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 3, (uint8_t *)CyFxUSBFunctionDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 4, (uint8_t *)CyFxUSBGUIDDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 5, (uint8_t *)CyFxUSBFamilyNameDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 6, (uint8_t *)CyFxUSBDeviceVerDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 7, (uint8_t *)CyFxUSBConfigSSDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 8, (uint8_t *)CyFxUSBConfigHSDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_STRING_DSCR, 9, (uint8_t *)CyFxUSBFSDscr);
            Cy_USBD_SetDscr(pAppCtxt->pUsbdCtxt, CY_USB_SET_HS_BOS_DSCR, 0, (uint8_t *)CyFxUSBBOSDscr);
        }
    }
}
/* [ ] */


