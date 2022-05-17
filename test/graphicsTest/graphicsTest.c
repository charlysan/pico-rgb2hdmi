//System defined includes
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/vreg.h"

//Library related includes
#include "dvi.h"
#include "dvi_serialiser.h"
#include "keyboard.h"
#include "graphics.h"

//HW Configuration includes
#include "common_configs.h"
#include "math.h"

// System config definitions
// TMDS bit clock 252 MHz
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define REFRESH_RATE 50

#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

// --------- Global register start --------- 
struct dvi_inst dvi0;
uint16_t framebuf[FRAME_WIDTH * FRAME_HEIGHT];
uint gpio_pins[3] = { KEYBOARD_PIN_UP, KEYBOARD_PIN_DOWN, KEYBOARD_PIN_ACTION };
const uint LED_PIN = PICO_DEFAULT_LED_PIN;
bool blink = true;

static const graphic_ctx_t graphic_ctx = {
	.width = FRAME_WIDTH,
	.height = FRAME_HEIGHT,
	.video_buffer = framebuf,
	.bppx = rgb_16,
	.parent = NULL
};

// --------- Global register end --------- 

void __not_in_flash_func(core1_main)() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	dvi_start(&dvi0);
	dvi_scanbuf_main_16bpp(&dvi0);
	__builtin_unreachable();
}

static inline void core1_scanline_callback() {
	uint16_t *bufptr;
	while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr));
	static uint scanline = 2;
	bufptr = &framebuf[FRAME_WIDTH * scanline];
	queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
	scanline = (scanline + 1) % FRAME_HEIGHT;
}

void on_keyboard_event(keyboard_status_t keys) {
    printf("Keyboard event received \n");
}

int __not_in_flash_func(main)() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);

	// Run system at TMDS bit clock
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

    stdio_init_all();
	gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

	printf("Initializing keyboard\n");
	keyboard_initialize(gpio_pins, 3, KEYBOARD_REFRESH_RATE_MS, KEYBOARD_REPEAT_RATE_MS, on_keyboard_event);

	printf("Configuring DVI\n");

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi0.scanline_callback = core1_scanline_callback;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	// Once we've given core 1 the framebuffer, it will just keep on displaying
	// it without any intervention from core 0
	for (int n=0; n < FRAME_WIDTH * FRAME_HEIGHT; n++) {
		framebuf[n] = 0x0000;
	}

	//Prepare for the first time the two initial lines
	uint16_t *bufptr = framebuf;
	queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
	bufptr += FRAME_WIDTH;
	queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

	printf("Core 1 start\n");
	multicore_launch_core1(core1_main);

	printf("Start rendering\n");

	// for (int y = 0; y < FRAME_HEIGHT; ++y) {
	// 	for (int x = 0; x < FRAME_WIDTH; ++x) {
	// 		int red = x * 32 / FRAME_WIDTH;
	// 		int green = y * 64 / FRAME_HEIGHT;
	// 		int blue = 31 - (x * 32) / FRAME_WIDTH;
	// 		framebuf[y * FRAME_WIDTH + x] = (x%8>0)&&(y%8>0) ? blue<<11 |green<<5 | red : 0xFFFF;
	// 	}
	// }
	uint x, y, a;
	uint sizex = 160;
	uint sizey = 120;
	uint color1 = 0b0000011111100000;
	uint color2 = 0b1111100000011111;

	for (a = 0; a < 16; a++) {
		x = sizex + sizex/2 * sin(2*M_PI*a/16);
		y = sizey + sizey/2 * cos(2*M_PI*a/16);
		draw_circle(&graphic_ctx, x, y, 16, color1);
	}
	x = 159;
	y = 119;
	fill_rect(&graphic_ctx, x - 16, y - 16, x + 16, y + 16, color2);

	while (1)
	{
		sleep_ms(1000);
	}
	__builtin_unreachable();
}

