@details
# Introduction  {#Introduction}
The USB host stack is composed of USB class drivers. The USB class drivers include the USB common host driver and USB controller driver, which consists of the xHCI driver.
Note that the xHCI represents either the EHCI or the KHCI, not the XHCI for USB 3.0.
@n To support different RTOSes with the same code base, the OSA is used inside the USB stack to wrap the differences between RTOSes.
Note that the OSA is not supported for use in the USB application. Therefore, from the USB application’s view point, the OSA is invisible.
@n The USB host stack must work with a dedicated application in which the following tasks should be done:
    - Configure the USB clock
    - Initialize/configure the USB host stack
    - Choose the proper configuration when one peripheral is connected on callback event received and decide if one peripheral could be supported by this application
    - Initialize class
    - Choose the proper interface setting and configure the peripheral if needed
    - Initialize the transfer request
    - Handle the transfer result through the callback
@n The architecture and components of the USB host stack are shown below:

@image html host_stack_architecture.jpg
@image latex host_stack_architecture.jpg "Host stack architecture"


The interface between the KHCI/EHCI Driver and the Common Controller driver is internal and is simplified in this document.

# USB Host Initialization flow {#USBHostInitFlow}

The host stack works as follows:

@image html host_stack_whole_flow.jpg
@image latex host_stack_whole_flow.jpg "Host stack work flow"


The host stack initialization work flow is as follows:

@image html host_stack_initialization_flow.jpg
@image latex host_stack_initialization_flow.jpg "Host stack initialization flow"
<UL>
   <LI>If the platform uses a GPIO to control the VBUS, initialize the GPIO and the output high.
   <LI>Initialize the USB host clock.
  <LI>Call the USB_HostInit to initialize the USB host stack.
 <LI>Set the USB interrupt priority and enable the interrupt.
   <LI>Create the host task with the task API USB_HostkhciTaskFunction or the USB_HostEhciTaskFunction. Create an application task if necessary.
   </UL>

# USB Host peripheral attach/detach flow {#USBHostEventFlow}
The peripheral attach/detach/unsupported event notifies the application through the callback function that it is registered by the USB_HostInit.
@n The peripheral attach/detach flow is as follows:

@image html host_stack_attach_flow.jpg
@image latex host_stack_attach_flow.jpg "Host stack attach flow"


The parameters of the callback contain the device handle, configuration handle, and event code.
The key point is the configuration handle. All interface information within this configuration is included.
The application should make use of the information to decide if this configuration is supported.
Note that, if the application returns kStatus_USB_NotSupported, the USB host stack checks the next configuration descriptor of the peripheral
until the application returns the kStatus_USB_Success or all configuration descriptors are checked.
If there is no supported configuration found in the peripheral descriptor,
the kUSB_HostEventNotSupported event is notified to the application through a callback function registered by the usb_host_init.

There are four events in the callback. See the host_event_t:
- kUSB_HostEventAttach for attaching the peripheral
- kUSB_HostEventDetach for attaching the unsupported peripheral
- kUSB_HostEventEnumerationDone for a supported peripheral enumeration
- kUSB_HostEventNotSupported for detaching the peripheral

For example:
- Use case 1: The device has one configuration and is supported by the host application. The event flow is as follows:
    - (1) kUSB_HostEventAttach event; An application chooses the configuration and returns the kStatus_USB_Success.
    - (2) kUSB_HostEventEnumerationDone event; An application starts to initialize the class and run.

- Use case 2: The device has two configurations and is not supported by the host application. The event flow is as follows:
    - (1) kUSB_HostEventAttach event; An application chooses the first configuration and returns the kStatus_USB_NotSupported.
    - (2) kUSB_HostEventAttach event; An application chooses the second configuration and returns the kStatus_USB_NotSupported.
    - (3) kUSB_HostEventNotSupported event; An application prints the device not supported information.
