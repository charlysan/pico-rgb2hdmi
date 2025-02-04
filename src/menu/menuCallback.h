#ifndef EVENT_CALLBACK_H
#define EVENT_CALLBACK_H

#include "gui.h"
// Event Callback handlers
bool on_exit_button_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_fine_tune_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_alignment_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_gain_offset_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_display_selection_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_palette_option_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_palette_color_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_save_reboot_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_factory_reboot_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_usb_enable_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_shutdown_display_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_scanline_display_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);
bool on_display_mode_event(gui_status_t status, gui_base_t *origin, gui_object_t *destination);

// GUI Callbacks handlers
void gui_draw_palette_choice(gui_base_t *base);
void menu_diagnostic_print(print_delegate_t printer);
void menu_scan_print(print_delegate_t printer);
void menu_palette_opt_print(print_delegate_t printer);
void menu_about_print(print_delegate_t printer);

//Utility
void menu_setup_selected_color();
const char *menu_get_usb_button_txt(bool status);
const char *menu_get_shutdown_opt_txt(bool status);
const char *menu_get_scanline_opt_txt(bool status);
const char *menu_get_display_mode_opt_txt(bool mode, bool confirmation);
#endif