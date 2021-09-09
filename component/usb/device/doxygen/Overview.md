@details
# Introduction  {#Introduction}
The USB device stack is composed of the USB controller driver only, which consists of the common controller driver and the controller (like: xHCI in Kinetis) driver.
The device class driver and the USB framework to handle the standard enumeration and request defined by USB specification 2.0 are moved to the
application layer. These two parts are example-specific to reduce the footprint of the examples.
@note The xHCI represents either EHCI or KHCI, not the XHCI for USB 3.0.

@n In the USB Device stack, there are two different USB applications. One is the lite version and the other is similar to the examples in the previous USB stack.
@n The whole architecture and components of USB stack are shown below:

@image html usb_device_stack_architecture.jpg
@image latex usb_device_stack_architecture.jpg "USB device stack architecture" width=\textwidth

@n For the lite version application, the code size is smaller than the non-lite version.
However, an obvious drawback of the new architecture is that customers need to use the controller driver API to implement the standard enumeration process,
the class-specific process, and the customer-specific functionality.

@n The device stack initialization sequence for the lite version application is as follows:
    1. Initialize the Pin Mux, USB clock, and so on. If the SoC has a USB KHCI-dedicated RAM, the
	RAM memory needs to be clear after the KHCI clock is enabled.
	When the demo uses USB EHCI IP, the USB KHCI dedicated-RAM can't be used and the memory can't be accessed.
    2. Initialize the USB device stack by calling the API #USB_DeviceInit.
    3. When the device task is enabled, create the USB device task by using the device handle, returned from #USB_DeviceInit, as the task parameter when
	the environment is an RTOS.
    4. Install the USB ISR.
    5. Enable the USB interrupt and the interrupt priority.
    6. Start the USB device by calling the #USB_DeviceRun.

@image html usb_device_initialization_lite.jpg
@image latex usb_device_initialization_lite.jpg "USB device initialization for lite version"


@n To assist customers with less concerns about the footprint and focus on ease of use of the USB stack, a generic usb_ch9 implementation
is provided and the specified class driver, such as HID class driver, CDC class driver, and so on. This implementation is more generic, it can be reused
in different examples and the APIs are easier to use. However, some callback functions need to be implemented and the code size is larger.
@n The device stack initialization sequence for non-lite version application is as follows:
    1. Initialize the Pin Mux, USB clock, and so on. If the SOC has the USB KHCI-dedicated RAM,
	the RAM memory needs to be clear after the KHCI clock is enabled.
	When the demo uses USB EHCI IP, the USB KHCI-dedicated RAM can't be used and the memory can't be accessed.
    @note The  USB_GLOBAL, USB_BDT, and USB_RAM_ADDRESS_ALIGNMENT(n) are only used for USB device stack.
    The USB device global variables are put into the section m_usb_global or m_usb_bdt by using the MACRO USB_GLOBAL and USB_BDT.
    In this way, the USB device global variables can be linked into USB dedicated RAM by changing the linker file.
    This feature can only be enabled when the USB dedicated RAM is not less than 2 K Bytes.
    2. Initialize the USB device stack by calling the API #USB_DeviceClassInit. Initialize each application.
    3. Get each class handle from the usb_device_class_config_struct_t::classHandle.
    4. When the device task is enabled, create the USB device task by using the device handle, returned from #USB_DeviceClassInit, as the task parameter when the
	environment is RTOS.
    5. Install the USB ISR.
    6. Enable the USB interrupt and the interrupt priority.
    7. Start the USB device by calling the #USB_DeviceRun.

@image html usb_device_initialization_none_lite.jpg
@image latex usb_device_initialization_none_lite.jpg "USB device initialization for non-lite version"

@n To support different RTOSes with the same code base, the OSA is used inside the USB stack to wrap the differences between RTOSes.
@note The OSA should not be used in the USB application. As a result, from the USB application's viewpoint, the OSA is invisible.

# USB Device Callback Work Flow {#USBDeviceCallbackWorkingFlow}

The device callback is registered when the #USB_DeviceInit function is called.
@n The following events should be processed in this callback function:
- kUsbDeviceEventBusReset
@n When the application receives this event, the device has received a BUS RESET signal.
In the event, the control pipe should be initialized. See the work flow. The parameter eventParam is not used.
- kUsbDeviceEventSetConfiguration
@n When the application receives this event, the host has sent a set configuration request.
The configuration value can be received from the parameter eventParam. In the event, the application configuration can be set.
Initialize each interface in the current configuration by using zero as an alternate setting.
- kUsbDeviceEventSetInterface
@n When the application receives this event, the host sent a set alternate setting request of an interface.
The interface and alternate setting value can be received from the parameter eventParam.
The eventParam points to a uint16_t variable. The high 8-bit is interface value and the low 8-bit is alternate setting.
In the event, the application changes the alternate setting of this interface if the new alternate setting is not equal to the current setting.
@n Normally, change the steps as follows:
@n 1. Cancel all transfers of the current alternate setting in this interface.
@n 2. De-initialize all pipes of the current alternate setting in this interface.
@n 3. Initialize all pipes of the new alternate setting in this interface.
@n 4. Prime the transfers of the new setting.
@n For example,
@code
uint16_t*   temp16 = (uint16_t*)eventParam;
uint8_t       interface = (uint8_t)((*temp16&0xFF00)>>0x08);
currentAlternateSetting[interface] = (uint8_t)(*temp16&0x00FF);
@endcode
@n The device callback event work flow:

@image html usb_device_callback_working_flow.jpg
@image latex usb_device_callback_working_flow.jpg "USB device callback working flow" width=\textwidth

# USB Device Class-Specific Request Work Flow {#USBDeviceClassSpecificRequestFlowWorkingFlow}

The class sepcific request can be classified into two types according to whether these is the data stage in a setup transfer.
The section describes class specific request with data stage only. For the class-specific request without data stage, the case is quite simple, we don't describe here.
Depend on the data direction, there are two cases, host wants to send data to device and host wants to get data from device.

## USB Device Class-Specific Request with Data Sent from Host

@image html usb_device_class_specific_request_with_data_sent_from_host.jpg
@image latex usb_device_class_specific_request_with_data_sent_from_host.jpg "USB Device Class-Specific Request with Data Sent from Host" width=\textwidth

## USB Device Class-Specific Request with Data Sent to Host

@image html usb_device_class_specific_request_with_data_sent_to_host.jpg
@image latex usb_device_class_specific_request_with_data_sent_to_host.jpg "USB Device Class-Specific Request with Data Sent to Host" width=\textwidth

