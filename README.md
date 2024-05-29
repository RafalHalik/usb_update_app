# ZEPHYR usb update app

The main purpose of this app is to enable usb firmware updates using mcuboot during run time.

# HOW IT WORKS

A ported usb host stack from MCUXpresso will detect and enumerate the usb device this will signal to another thread
that a usb device is attached and ready to be read. From this point a statemachine takes care of mounting fatfs
to allow access to the contence of the usb drive. Once fatfs is setup the statemachine will read the data in chunks of
2048 bytes. (Stored in an array size 512 of uint32_t values) Each chunk is read and then written before reading another
chunk of data. 

When the entire file is read then a very basic check is performed to see if the header is intact, the new image version number is checked. The basic check passes if the new image version number is non 0.

After checks pass the device will display on screen that the update is complete and wait till the user removes the USB device
before performing a REBOOT this then hands over controll to mcuboot which in turn decides if the image written to slot1 is valid.
Depending on what the user configured mcuboot to check it will boot into the new image or load the "old" image back into memory.

This is because it is set by default to use a swap image algorithm rather than an overwrite however this is also user configurable. 

# TESTS 

- Does the device processor heavy tasks still comeplete the USB update?

    Running LVGL music demo puts a lot of strain on the processor so this was used. When usb device is 
    attached screen changes to update promptly and usb update completes without issues, repeated this 
    several times and usb update completed every time.


- Can you unplug usb device and plug back in and usb update still procced?  

    Repeatedly removed the usb device minimum of 5 times at different stages of the update process.
    When USB was plugged back in the device was detected and started the USB update process every time.

    ***NOTE this does restart the update process, it will not pick up from where it left off last time.


- Will app wait till usb device is removed before restarting?

    USB device left in slot for 10 minutes no restart performed until usb was removed.