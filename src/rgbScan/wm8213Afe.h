/*
*/

#ifndef _WM8213_AFE_H
#define _WM8213_AFE_H

#include "hardware/spi.h"
#include "hardware/clocks.h"

/* Wolfson AFE WM8213 Registers */
// Setup registers
#define WM8213_REG_SETUP1            0x01
#define WM8213_REG_SETUP2            0x02
#define WM8213_REG_SETUP3            0x03
#define WM8213_REG_SETUP4            0x06
#define WM8213_REG_SETUP5            0x07
#define WM8213_REG_SETUP6            0x08

// Reset registers
#define WM8213_REG_SWRESET           0x04
#define WM8213_REG_AUTO_RESET        0x05

// Dac registers
#define WM8213_REG_DAC_RED           0x20
#define WM8213_REG_DAC_GRN           0x21
#define WM8213_REG_DAC_BLU           0x22
#define WM8213_REG_DAC_RGB           0x23

// Gain register per color
#define WM8213_REG_PGA_GAIN_LSB_RED  0x24
#define WM8213_REG_PGA_GAIN_LSB_GRN  0x25
#define WM8213_REG_PGA_GAIN_LSB_BLU  0x26
#define WM8213_REG_PGA_GAIN_LSB_RGB  0x27
#define WM8213_REG_PGA_GAIN_MSB_RED  0x28
#define WM8213_REG_PGA_GAIN_MSB_GRN  0x29
#define WM8213_REG_PGA_GAIN_MSB_BLU  0x2a
#define WM8213_REG_PGA_GAIN_MSB_RGB  0x2b

// Read Mode Mask
#define WM8213_ADDR_READ(x)           0x10|x
#define WM8213_ADDR_WRITE(x)          0xEF&x

typedef struct wm8213_afe_setup_1 {
    io_rw_8  enable:1;
    io_rw_8  cds:1; //Correlated Double Sampler
    io_rw_8  mono:1;
    io_rw_8  two_chan:1;
    io_rw_8  pgafs:2;
    io_rw_8  mode_4_legacy:1;
    io_rw_8  legacy:1;
} wm8213_afe_setup_1_t;

typedef struct wm8213_afe_setup_2 {
    io_rw_8  opp_form:2; //x0-8bit multiplexed, 01-8 bit parallel, 11-4bits multiplex 
    io_rw_8  invop:1; //Inver polarity of output data
    io_rw_8  opd:1; //output disable 
    io_rw_8  low_refs:1; //Reduce the adc ref range
    io_rw_8  rlc_dac_rng:1; //Output range of the rlcdac (0 = AVdd, 1=VRT)
    io_rw_8  del:2; // data latency
} wm8213_afe_setup_2_t;

typedef struct wm8213_afe_setup_3 {
    io_rw_8  rlc_dac:4;
    io_rw_8  cds_ref:2;
    io_rw_8  chan:2;
} wm8213_afe_setup_3_t;

typedef struct wm8213_afe_setup_4 {
    io_rw_8  line_by_line:1;
    io_rw_8  acyc:1;
    io_rw_8  intm:2;
    io_rw_8  reserved:4; 
} wm8213_afe_setup_4_t;

typedef struct wm8213_afe_setup_5 {
    io_rw_8  red_pd:1;
    io_rw_8  green_pd:1;
    io_rw_8  blue_pd:1;
    io_rw_8  adc_pd:1;
    io_rw_8  vrlc_dac_pd:1;
    io_rw_8  adc_ref_pd:1;
    io_rw_8  vrx_pd:1;
    io_rw_8  reserved:1;  
} wm8213_afe_setup_5_t;

typedef struct wm8213_afe_setup_6 {
    io_rw_8  vsm_pdet:1;
    io_rw_8  vdel:3;
    io_rw_8  posn_neg:1;
    io_rw_8  rlc_en:1;
    io_rw_8  clamp_ctrl:1;
    io_rw_8  reserved:1;  
} wm8213_afe_setup_6_t;

typedef struct wm8213_afe_setups {
    wm8213_afe_setup_1_t setup1;
    wm8213_afe_setup_2_t setup2;
    wm8213_afe_setup_3_t setup3;
    wm8213_afe_setup_4_t setup4;
    wm8213_afe_setup_5_t setup5;
    wm8213_afe_setup_6_t setup6;
} wm8213_afe_setups_t;

typedef struct wm8213_afe_pins { 
        uint sck, sdi, sdo, cs;
    } wm8213_afe_pins_t;

typedef struct wm8213_afe_config {
    spi_inst_t         *spi;
    uint                baudrate;
    wm8213_afe_pins_t   pins;
    wm8213_afe_setups_t setups;
    char                verify_retries;
} wm8213_afe_config_t;

int wm8213_afe_setup(const wm8213_afe_config_t* config);

#endif