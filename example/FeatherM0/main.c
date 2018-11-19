#include <atmel_start.h>
#include <stdio.h>

#define MESSAGE_SIZE		(64)    // USB message size (64 byte endpoints, multiple of 64 is fastest)
#define SAMPLE_PERIOD_MS    (1000)  // sample period for this test program

int main(void)
{
	/*** Initialize MCU, drivers and middleware - always call this first ****/
	atmel_start_init();
	/*************************************************************************/

	char    message[MESSAGE_SIZE];	// message buffer to send over USB
	int32_t err;                    // error return value

	// wait till USB serial is connected and asserts DTR signal
    while(!usb_dtr()){
        gpio_toggle_pin_level(LED_BUILTIN);
        delay_ms(250);
    }

	for(;;) {
        if(usb_state() == USB_Configured) {
		    /* Turn on LED if the DTR signal is set (serial terminal open on host) */
		    gpio_set_pin_level(LED_BUILTIN, usb_dtr());

            for(int i = 0; i < err; ++i) {
                sprintf(message, "");
                usb_write(message, strlen(message));
            }

		    delay_ms(SAMPLE_PERIOD_MS);
        }
	}
}