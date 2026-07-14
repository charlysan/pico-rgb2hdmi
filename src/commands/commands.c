#include <stdlib.h>
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>
#include "commands.h"
#include "videoAdjust.h"
#include "rgbScan.h"
#include "wm8213Afe.h"
#include "overlay.h"
#include "security.h"
#include "settings.h"
#include "version.h"
#include "hardware/watchdog.h"

int command_info_afe_error;
int command_info_scanner_error;
int command_info_missed_lines; // TODO: implement

///////////   GLOBALS   ///////////
bool command_license_is_valid;
const void *security_key_in_flash;

#define COMMAND_GET_PRINTF_WORD() command_get_current_bppx() == rgb_16_565 ? "%04X%s" : "%02X%s"
#define COMMAND_GET_WORD_FROM_BUFFER(width, height) command_get_current_bppx() == rgb_16_565 ? \
    GET_RGB16_BUFFER(GET_VIDEO_PROPS().video_buffer)[GET_VIDEO_PROPS().width * height + width] : \
    GET_RGB8_BUFFER(GET_VIDEO_PROPS().video_buffer)[GET_VIDEO_PROPS().width * height + width]

/////////// END GLOBALS ///////////

bool command_is_license_valid() {
    return command_license_is_valid;
}

void command_validate_license(const uint8_t *security_key) {
    #ifdef USE_LICENSE
        int token = -1;
	    command_license_is_valid = security_key_is_valid(security_key, token) <= 0;
    #else
        command_license_is_valid = true;
    #endif
}

void command_reboot() {
	printf("Rebooting\n");
	watchdog_reboot(0, SRAM_END, 10);
}

void command_save_settings() {
    printf("Saving settings\n");
    settings_update();
}

void command_factory_reset() {
    printf("Factory reset\n");
    settings_factory();
}

color_bppx command_get_current_bppx() {
    return settings_get()->flags.symbols_per_word ? rgb_16_565 : rgb_8_332; 
}

void command_enable_usb(bool status) {
    static bool first_time = true;
    if (first_time) {
        if (status) {
            stdio_init_all();
            first_time = false;
        }
    } else {
        // tiny USB implementation does not disables or allows to disable low level IRQs, so reboot
        //stdio_set_driver_enabled(&stdio_usb, status);
        //irq_set_enabled(low_priority_irq_num, status);
        command_reboot();
    }
}

// Parse "r,g,b" or a single value applied to the three channels
static bool command_parse_rgb(const char *data, int *red, int *green, int *blue) {
    if (data == NULL) {
        return false;
    }
    int cnt = sscanf(data, "%d,%d,%d", red, green, blue);
    if (cnt == 1) {
        *green = *red;
        *blue = *red;
    } else if (cnt != 3) {
        printf("Expected <value> or <r,g,b>, got %s\n", data);
        return false;
    }
    return true;
}

static inline uint8_t command_clamp_uint8(int value) {
    return value < 0 ? 0 : (value > 255 ? 255 : value);
}

// Keep the stored display slot in sync with console porch changes so `save`
// persists them (the menu paths do their own syncing)
static void command_sync_h_porch_settings() {
    display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
    display->h_front_porch = GET_VIDEO_PROPS().horizontal_front_porch;
    display->h_back_porch  = GET_VIDEO_PROPS().horizontal_back_porch;
}

static void command_sync_v_porch_settings() {
    display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
    display->v_front_porch = GET_VIDEO_PROPS().vertical_front_porch;
    display->v_back_porch  = GET_VIDEO_PROPS().vertical_back_porch;
}

// Any horizontal porch change alters BOTH what each line's DMA consumes and
// what the gated SM must emit per line: recompute the per-line sample count
// and rebuild the capture so production and consumption stay equal. Without
// this, every line leaks the difference into the FIFO and the image scrambles
static void command_update_line_length(void) {
    rgbScannerEnable(false);
    wm8213_afe_capture_set_line_length(get_video_prop_horizontal_front_porch() + GET_VIDEO_PROPS().width, true);
    rgbScannerEnable(true);
}

// Switch to a display slot (0 based) applying its full config: AFE calibration,
// video timing and capture reconfiguration. Shared by the menu and the console.
void command_select_display(uint display_no) {
    settings_get()->flags.default_display = display_no;
    display_t *display = &(settings_get()->displays[display_no]);

    // Gain and negative offset do not commit changes, offset commits all
    wm8213_afe_update_gain(display->gain.red, display->gain.green, display->gain.blue, false);
    wm8213_afe_update_negative_offset(display->offset.negative, false);
    wm8213_afe_update_offset(display->offset.red, display->offset.green, display->offset.blue, true);
    // Timing and aligment
    // Stored fine_tune is in spinbox units (1 = 1kHz), video_props wants Hz
    set_video_props(display->v_front_porch, display->v_back_porch,
        display->h_front_porch, display->h_back_porch,
        GET_VIDEO_PROPS().width, GET_VIDEO_PROPS().height, display->refresh_rate, 1000 * display->fine_tune, settings_get()->flags.symbols_per_word, GET_VIDEO_PROPS().video_buffer);
    rgbScannerUpdateData(GET_VIDEO_PROPS().vertical_front_porch, 0);
    rgbScannerEnable(false);
    wm8213_afe_capture_set_line_length(get_video_prop_horizontal_front_porch() + GET_VIDEO_PROPS().width, false);
    wm8213_afe_capture_update_sampling_rate(GET_VIDEO_PROPS().sampling_rate);
    rgbScannerEnable(true);
}




int command_on_receive(int option, const void *data, bool convert) {
    int   integer_value = 0;
    bool  bool_value = false;

    // In convert mode data is a string at an arbitrary offset inside the
    // command line: it must never be dereferenced as an int, the M0+
    // hardfaults on unaligned 32 bit reads (and the release hardfault
    // handler silently resets the board)
    if (convert) {
        if (data != NULL) {
            integer_value = atoi((const char *)data);
            bool_value = strcmp((const char *)data, "true") == 0;
        }
    } else if (data != NULL) {
        integer_value = *(const int *) data;
        bool_value = *((const char *)data) > 0;
    }

    switch(option) {
            case 'u':
                printf("Move screen up %d positions\n", integer_value);
				GET_VIDEO_PROPS().vertical_front_porch += integer_value;
				GET_VIDEO_PROPS().vertical_back_porch  -= integer_value;

				rgbScannerUpdateData(GET_VIDEO_PROPS().vertical_front_porch, 0);
                break;
            case 'd': 
                printf("Move screen down %d positions\n", integer_value);
				GET_VIDEO_PROPS().vertical_front_porch -= integer_value;
				GET_VIDEO_PROPS().vertical_back_porch  += integer_value;

				rgbScannerUpdateData(GET_VIDEO_PROPS().vertical_front_porch, 0);
                break;
            case 'l':
                printf("Move screen right %d positions\n", integer_value);
				GET_VIDEO_PROPS().horizontal_front_porch += integer_value;
				GET_VIDEO_PROPS().horizontal_back_porch  -= integer_value;

				command_sync_h_porch_settings();
				command_update_line_length();
                break;
            case 'r':
                printf("Move screen left %d positions\n", integer_value);
				GET_VIDEO_PROPS().horizontal_front_porch -= integer_value;
				GET_VIDEO_PROPS().horizontal_back_porch  += integer_value;

				command_sync_h_porch_settings();
				command_update_line_length();
                break;
			case 'i':
				if (command_license_is_valid) {
					printf("Show on screen info to %s\n", bool_value > 0 ? "on" : "off");
					command_show_info(bool_value);
				} else {
					printf("Invalid license, will not change info screen\n");
				}
                break;
			case 'c':
				printf("Capture screen: %dx%d@%dbppx", GET_VIDEO_PROPS().width, GET_VIDEO_PROPS().height, bppx_to_int(command_get_current_bppx(), color_part_all));
				rgbScannerEnable(false);
				for(int height=0; height < GET_VIDEO_PROPS().height; height++) {
					printf("\n");
					for(int width=0; width < GET_VIDEO_PROPS().width; width++) {
						printf(COMMAND_GET_PRINTF_WORD(), COMMAND_GET_WORD_FROM_BUFFER(width, height), (width < GET_VIDEO_PROPS().width -1) ? ",": "");
					}
				}
				rgbScannerEnable(true);
                break;
			case 'I':
                printf("Device is: %s\n", security_get_uid());
				break;
            case 'S': {
                display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
                printf("Mode: %dx%d@%dbppx %dHz\n", GET_VIDEO_PROPS().width, GET_VIDEO_PROPS().height,
                    bppx_to_int(command_get_current_bppx(), color_part_all), GET_VIDEO_PROPS().refresh_rate);
                printf("Gain: %d,%d,%d\n", wm8213_afe_get_gain(color_part_red), wm8213_afe_get_gain(color_part_green), wm8213_afe_get_gain(color_part_blue));
                printf("Offset: %d,%d,%d negative: %d\n", wm8213_afe_get_offset(color_part_red), wm8213_afe_get_offset(color_part_green), wm8213_afe_get_offset(color_part_blue), wm8213_afe_get_negative_offset());
                printf("Fine tune: %d\n", display->fine_tune);
                printf("H porch front/back: %d/%d V porch front/back: %d/%d\n",
                    GET_VIDEO_PROPS().horizontal_front_porch, GET_VIDEO_PROPS().horizontal_back_porch,
                    GET_VIDEO_PROPS().vertical_front_porch, GET_VIDEO_PROPS().vertical_back_porch);
                printf("Sampling rate: %u Hz\n", (unsigned int)GET_VIDEO_PROPS().sampling_rate);
                printf("Sampling phase: %d/12\n", wm8213_afe_capture_get_phase());
                }
                break;
            case 'q': {
                // Single machine-parseable line containing all data
                display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
                printf("STATUS slot=%d bpp=%d w=%d h=%d refresh=%d finetune=%d phase=%d "
                       "hf=%d hb=%d vf=%d vb=%d rate=%d "
                       "gain=%d,%d,%d offset=%d,%d,%d neg=%d "
                       "usb=%d sync=%d hsyncns=%u vsyncns=%lu lines=%d\n",
                    settings_get()->flags.default_display + 1,
                    bppx_to_int(command_get_current_bppx(), color_part_all),
                    GET_VIDEO_PROPS().width, GET_VIDEO_PROPS().height,
                    GET_VIDEO_PROPS().refresh_rate, display->fine_tune, wm8213_afe_capture_get_phase(),
                    GET_VIDEO_PROPS().horizontal_front_porch, GET_VIDEO_PROPS().horizontal_back_porch,
                    GET_VIDEO_PROPS().vertical_front_porch, GET_VIDEO_PROPS().vertical_back_porch,
                    GET_VIDEO_PROPS().sampling_rate,
                    wm8213_afe_get_gain(color_part_red), wm8213_afe_get_gain(color_part_green), wm8213_afe_get_gain(color_part_blue),
                    wm8213_afe_get_offset(color_part_red), wm8213_afe_get_offset(color_part_green), wm8213_afe_get_offset(color_part_blue),
                    wm8213_afe_get_negative_offset(),
                    0, // TODO: // settings_get()->flags.usb_enabled,
                    (int)rgbScannerGetSyncType(),
                    // The scanner keeps the last measurement forever; report 0
                    // once the sync detector has decayed to "none" so hosts
                    // don't display stale rates after the source is unplugged
                    rgbScannerGetSyncType() != rgbscan_sync_none ? rgbScannerGetHsyncNanoSec() : 0,
                    rgbScannerGetSyncType() != rgbscan_sync_none ? rgbScannerGetVsyncNanoSec() : 0,
                    rgbScannerGetSyncType() != rgbscan_sync_none ? rgbScannerGetHorizontalLines() : 0);
                }
                break;
            // gain
            case 'g': {
                int red = integer_value, green = integer_value, blue = integer_value;
                if (convert && !command_parse_rgb((const char *)data, &red, &green, &blue)) {
                    return 0;
                }
                display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
                wm8213_afe_update_gain(red, green, blue, true);
                // Read back so the stored value carries the clamping
                display->gain.red   = wm8213_afe_get_gain(color_part_red);
                display->gain.green = wm8213_afe_get_gain(color_part_green);
                display->gain.blue  = wm8213_afe_get_gain(color_part_blue);
                printf("Gain set to %d,%d,%d\n", display->gain.red, display->gain.green, display->gain.blue);
                }
                break;
            // offset
            case 'o': {
                int red = integer_value, green = integer_value, blue = integer_value;
                if (convert && !command_parse_rgb((const char *)data, &red, &green, &blue)) {
                    return 0;
                }
                display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
                display->offset.red   = command_clamp_uint8(red);
                display->offset.green = command_clamp_uint8(green);
                display->offset.blue  = command_clamp_uint8(blue);
                wm8213_afe_update_offset(display->offset.red, display->offset.green, display->offset.blue, true);
                printf("Offset set to %d,%d,%d\n", display->offset.red, display->offset.green, display->offset.blue);
                }
                break;
            // negoffset
            case 'n': {
                display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
                wm8213_afe_update_negative_offset(integer_value < 0 ? 0 : integer_value, true);
                display->offset.negative = wm8213_afe_get_negative_offset();
                printf("Negative offset set to %d\n", display->offset.negative);
                }
                break;
            // finetune
            case 'f': {
                // 1 step = 1 kHz of sampling rate
                int max_steps = VIDEO_FINE_TUNE_MAX / 1000;
                if (integer_value >  max_steps) { integer_value =  max_steps; }
                if (integer_value < -max_steps) { integer_value = -max_steps; }
                display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
                display->fine_tune = integer_value;
                GET_VIDEO_PROPS().fine_tune = 1000 * integer_value;
                update_sampling_rate();
                rgbScannerEnable(false);
                wm8213_afe_capture_update_sampling_rate(GET_VIDEO_PROPS().sampling_rate);
                rgbScannerEnable(true);
                printf("Fine tune set to %d (sampling rate %d Hz)\n", integer_value, GET_VIDEO_PROPS().sampling_rate);
                }
                break;
            // sub-pixel sampling phase
            case 'x': {
                if (integer_value < 0)  { integer_value = 0; }
                if (integer_value > 11) { integer_value = 11; }
                rgbScannerEnable(false);
                wm8213_afe_capture_set_phase(integer_value, true);
                rgbScannerEnable(true);
                printf("Sampling phase set to %d/12 of a pixel\n", integer_value);
                }
                break;
            // refresh
            case 'F': {
                if (integer_value < 1 || integer_value > 255) {
                    printf("Refresh rate must be 1 to 255 Hz\n");
                    return 0;
                }
                display_t *display = &(settings_get()->displays[settings_get()->flags.default_display]);
                display->refresh_rate = integer_value;
                GET_VIDEO_PROPS().refresh_rate = integer_value;
                update_sampling_rate();
                rgbScannerEnable(false);
                wm8213_afe_capture_update_sampling_rate(GET_VIDEO_PROPS().sampling_rate);
                rgbScannerEnable(true);
                printf("Refresh set to %d Hz (sampling rate %d Hz)\n", integer_value, GET_VIDEO_PROPS().sampling_rate);
                }
                break; 
            // Pixel Width
            case 'W': {
                int total = 0;
                if (data == NULL || sscanf((const char *)data, "%d", &total) != 1) {
                    printf("Pixel width %d (front %d + back %d)\n",
                        GET_VIDEO_PROPS().horizontal_front_porch + GET_VIDEO_PROPS().horizontal_back_porch,
                        GET_VIDEO_PROPS().horizontal_front_porch, GET_VIDEO_PROPS().horizontal_back_porch);
                    return 0;
                }
                int front = GET_VIDEO_PROPS().horizontal_front_porch;
                int back = total - front;
                if (back < 0) {
                    // Not enough room: pull the window left
                    front = total;
                    back = 0;
                }
                GET_VIDEO_PROPS().horizontal_front_porch = front;
                GET_VIDEO_PROPS().horizontal_back_porch  = back;

                // Changing the total horizontal porch changes the pixel clock,
                // so recompute the sampling rate and reconfigure the AFE.
                // The front porch also changes the gated SM's per-line count
                command_sync_h_porch_settings();
                update_sampling_rate();
                rgbScannerEnable(false);
                wm8213_afe_capture_set_line_length(get_video_prop_horizontal_front_porch() + GET_VIDEO_PROPS().width, false);
                wm8213_afe_capture_update_sampling_rate(GET_VIDEO_PROPS().sampling_rate);
                rgbScannerEnable(true);

                printf("Pixel width %d (front %d + back %d)\n", front + back, front, back);
                }
                break;
            // Porch
            case 'P': {
                int front = 0, back = 0;
                int parsed = data != NULL ? sscanf((const char *)data, "%d,%d", &front, &back) : 0;
                if (parsed < 1) {
                    printf("Expected <front> or <front,back> raw porch values\n");
                    return 0;
                }
                if (parsed == 1) {
                    back = GET_VIDEO_PROPS().horizontal_back_porch;
                }
                if (front < 1)   { front = 1; }
                if (front > 300) { front = 300; }
                if (back  < 0)   { back  = 0; }
                if (back  > 300) { back  = 300; }

                GET_VIDEO_PROPS().horizontal_front_porch = front;
                GET_VIDEO_PROPS().horizontal_back_porch  = back;

                // Changing the total horizontal porch changes the pixel clock,
                // so recompute the sampling rate and reconfigure the AFE.
                // The front porch also changes the gated SM's per-line count
                command_sync_h_porch_settings();
                update_sampling_rate();
                rgbScannerEnable(false);
                wm8213_afe_capture_set_line_length(get_video_prop_horizontal_front_porch() + GET_VIDEO_PROPS().width, false);
                wm8213_afe_capture_update_sampling_rate(GET_VIDEO_PROPS().sampling_rate);
                rgbScannerEnable(true);

                printf("H porch %d/%d (pixel width %d): capture starts %d samples after sync (sampling rate %d Hz)\n",
                    front, back, front + back, get_video_prop_horizontal_front_porch(), GET_VIDEO_PROPS().sampling_rate);

                }
                break;
            // afe_reg
            case 'A': {
                static const char *afe_reg_names[WM8213_REG_SETUP_TOTAL] = {
                    "SETUP1", "SETUP2", "SETUP3", "SETUP4", "SETUP5", "SETUP6",
                    "OFFSET_R", "OFFSET_G", "OFFSET_B",
                    "GAIN_LSB_R", "GAIN_MSB_R", "GAIN_LSB_G", "GAIN_MSB_G", "GAIN_LSB_B", "GAIN_MSB_B" };
                int reg_index = 0;
                unsigned int reg_value = 0;
                if (data != NULL && sscanf((const char *)data, "%d,%x", &reg_index, &reg_value) == 2) {
                    if (reg_index < 0 || reg_index >= WM8213_REG_SETUP_TOTAL) {
                        printf("Register index must be 0 to %d\n", WM8213_REG_SETUP_TOTAL - 1);
                        return 0;
                    }
                    rgbScannerEnable(false);
                    uint res = wm8213_afe_set_setup_byte(reg_index, reg_value & 0xFF, true);
                    rgbScannerEnable(true);
                    printf("%s[%d] = 0x%02X%s\n", afe_reg_names[reg_index], reg_index, reg_value & 0xFF,
                        res > 0 ? " COMMIT FAILED (chip readback mismatch)" : "");
                } else {
                    // Anything else (e.g. `afereg dump`) lists all registers
                    printf("idx name       ram  chip\n");
                    for (int i = 0; i < WM8213_REG_SETUP_TOTAL; i++) {
                        uint8_t chip_value = 0;
                        bool read_ok = wm8213_afe_read_setup_byte(i, &chip_value) == 0;
                        printf("%2d  %-10s 0x%02X ", i, afe_reg_names[i], wm8213_afe_get_setup_byte(i));
                        if (read_ok) {
                            printf("0x%02X\n", chip_value);
                        } else {
                            printf("--\n");
                        }
                    }
                    printf("Write: afereg <idx>,<hex> (volatile, boot restores defaults)\n");
                }
                }
                break;
            // retore setting (slot)
            case 'y':
                if (integer_value < 1 || integer_value > SETTINGS_DISPLAY_MAX) {
                    printf("Display slot must be 1 to %d\n", SETTINGS_DISPLAY_MAX);
                    return 0;
                }
                command_select_display(integer_value - 1);
                printf("Display slot %d selected\n", integer_value);
                break;
            // save settings
            case 's':
                rgbScannerEnable(false);
                command_save_settings();
                command_reboot(); // A software reboot is required after storing the new settings
                break;
#ifdef TEST_MODE
			case 'k': {
				printf("Storing key: %s\n", (const char *)data);
				int len = strlen(data);
				if (len != (SECURITY_SHA_SIZE * 2)) {
					printf("Key error received: %d chars, requires %d\n",len, SECURITY_SHA_SIZE * 2);
					return 0;
				}
				security_str_2_hexa(data, settings_get()->security_key, SECURITY_SHA_SIZE);
				settings_update();
				command_reboot();
				}
			    break;
			case 'K': {
				char key_str[41];
				security_hexa_2_str(security_key_in_flash, key_str, SECURITY_SHA_SIZE);
				printf("Stored key is: %s\n", key_str);
				}
			break;
#endif
			case 'v':
                printf("%s - Integration Test - version %s\n", PROJECT_NAME, PROJECT_VER);
				break;
			case 'm':
                printf("%s %dx%d@%dbppx\n", PROJECT_NAME, GET_VIDEO_PROPS().width, GET_VIDEO_PROPS().height, bppx_to_int(command_get_current_bppx(), color_part_all));
				break;
			case 'R':
				printf("Software reboot requested\n");
				command_reboot();
				break;
            case 'D': {
                bool current_state = dvi_is_started(&dvi0);
                    if (bool_value) {
                        dvi_start(&dvi0);
                    } else {
                        dvi_stop(&dvi0);
                    }
                    printf("DVI request to %s while %s\n", bool_value ? "start": "stop", current_state ? "active" : "inactive");
                }
				break;
            default:  return 1;
    }
    return 0;
}