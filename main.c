#include <stdio.h>
#include "pico/stdlib.h"

void main(void)
{
	stdio_init_all();

	while (1) {
		printf("Hello, World!\n");
		sleep_ms(1000);
	}
}
