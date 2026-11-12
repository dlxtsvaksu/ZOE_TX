#include "UsbHid.h"
#include "usb.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_hid.h"
#include "Func.h"
#include "Schedule.h"

/* HID report: X/Y/Rz are signed 16-bit, Z (throttle) is unsigned 16-bit,
   followed by 8 buttons.  The report descriptor uses a separate logical
   range for Z so throttle stays unipolar (0..65535). */
#define USB_HID_REPORT_SIZE   9U
#define USB_HID_AXIS_CENTER   32768U
#define USB_HID_AXIS_MAX      65535U

USBD_HandleTypeDef hUsbDeviceFS;

/**
  * @brief  Initialize the USB device stack and start the HID joystick.
  */
void USB_HID_Init(void)
{
    if (USBD_Init(&hUsbDeviceFS, &HID_Desc, 0U) != USBD_OK)
    {
        Error_Handler();
    }

    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_HID) != USBD_OK)
    {
        Error_Handler();
    }

    USBD_Start(&hUsbDeviceFS);
}

/**
  * @brief  Map a CRSF channel value (172..1811, center 992) to 0..65535.
  *         The center maps exactly to 32768 so Windows game controllers
  *         report a centered stick without drift.
  */
static uint16_t USB_HID_AxisMap(uint16_t value)
{
    if (value < (uint16_t)crsf_mid)
    {
        if (value <= (uint16_t)crsf_min)
        {
            return 0U;
        }

        return (uint16_t)(((uint32_t)(value - (uint16_t)crsf_min) * USB_HID_AXIS_CENTER) /
                          (uint16_t)(crsf_mid - crsf_min));
    }

    if (value >= (uint16_t)crsf_max)
    {
        return USB_HID_AXIS_MAX;
    }

    return (uint16_t)(USB_HID_AXIS_CENTER +
                      (((uint32_t)(value - (uint16_t)crsf_mid) * (USB_HID_AXIS_MAX - USB_HID_AXIS_CENTER)) /
                       (uint16_t)(crsf_max - crsf_mid)));
}

/* Store one centered axis in the signed 16-bit HID range.  USB_HID_AxisMap()
   still produces 0..65535 (center 32768); subtracting 32768 converts it to
   -32768..32767.  The unsigned subtraction intentionally wraps below center. */
static void USB_HID_PutSignedAxis(uint8_t *dst, uint16_t value)
{
    uint16_t raw = (uint16_t)(value - USB_HID_AXIS_CENTER);

    dst[0] = (uint8_t)(raw & 0xFFU);
    dst[1] = (uint8_t)(raw >> 8);
}

/* Store the throttle axis unchanged: the descriptor declares Z as 0..65535,
   so full physical travel maps to 0..100 in unipolar simulators. */
static void USB_HID_PutUnsignedAxis(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)(value >> 8);
}

/* Switch_Read() writes exactly crsf_min / crsf_mid / crsf_max.  For the
   3-position switches (SC/SD) the middle position leaves both related buttons
   released. */
static uint8_t USB_HID_SwitchHigh(uint16_t value)
{
    return (value == (uint16_t)crsf_max) ? 1U : 0U;
}

static uint8_t USB_HID_SwitchLow(uint16_t value)
{
    return (value == (uint16_t)crsf_min) ? 1U : 0U;
}

/**
  * @brief  Build and send one 9-byte joystick report.  Call every 5 ms.
  */
void USB_HID_Task(void)
{
    static uint8_t report[USB_HID_REPORT_SIZE];
    uint16_t x;
    uint16_t y;
    uint16_t z;
    uint16_t rz;
    uint8_t  buttons = 0U;

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
    {
        return;
    }

    if (pwr_latched == 0U)
    {
        /* Powered down: keep the host side neutral (throttle low). */
        x  = USB_HID_AXIS_CENTER;
        y  = USB_HID_AXIS_CENTER;
        z  = 0U; /* unsigned throttle minimum */
        rz = USB_HID_AXIS_CENTER;
    }
    else
    {
        x  = USB_HID_AxisMap(aetr[0]);
        y  = USB_HID_AxisMap(aetr[1]);
        z  = USB_HID_AxisMap(aetr[2]);
        rz = USB_HID_AxisMap(aetr[3]);

        /* B1..B8:
           SA, SB, SC-low, SC-high, SD-low, SD-high, SE, SF
           SC/SD are 3-position: middle releases both of their buttons.
           TRIM_SET / TRIM_RESET are deliberately not mapped. */
        if (USB_HID_SwitchHigh(Switch_State[SA])) buttons |= (uint8_t)(1U << 0);
        if (USB_HID_SwitchHigh(Switch_State[SB])) buttons |= (uint8_t)(1U << 1);
        if (USB_HID_SwitchLow(Switch_State[SC]))  buttons |= (uint8_t)(1U << 2);
        if (USB_HID_SwitchHigh(Switch_State[SC])) buttons |= (uint8_t)(1U << 3);
        if (USB_HID_SwitchLow(Switch_State[SD]))  buttons |= (uint8_t)(1U << 4);
        if (USB_HID_SwitchHigh(Switch_State[SD])) buttons |= (uint8_t)(1U << 5);
        if (USB_HID_SwitchHigh(Switch_State[SE])) buttons |= (uint8_t)(1U << 6);
        if (USB_HID_SwitchHigh(Switch_State[SF])) buttons |= (uint8_t)(1U << 7);
    }

    report[8] = buttons;

    USB_HID_PutSignedAxis(&report[0], x);   /* X  */
    USB_HID_PutSignedAxis(&report[2], y);   /* Y  */
    USB_HID_PutUnsignedAxis(&report[4], z); /* Z: throttle */
    USB_HID_PutSignedAxis(&report[6], rz);  /* Rz */

    (void)USBD_HID_SendReport(&hUsbDeviceFS, report, USB_HID_REPORT_SIZE);
}
