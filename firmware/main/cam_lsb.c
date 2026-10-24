/* Copyright 2026 CEMAXECUTER LLC
 * SPDX-License-Identifier: Apache-2.0
 *
 * LCD_CAM camera input used as a free-running 8- or 16-bit parallel
 * capture, for the I/Q low bits next to PARLIO. PCLK comes from a pad:
 * either an external sample clock, or (clk_div != 0) this unit's own clock
 * output, PLL_F160M / clk_div, which PARLIO then takes as its external
 * clock from the same pad. DE and VSYNC are tied high (VSYNC low = no
 * capture), EOF by byte count, AXI GDMA into a ring of equal nodes.
 * Clock/reset sequence as the IDF DVP driver.
 */
#include <stdio.h>
#include "driver/gpio.h"
#include "esp_private/gpio.h"
#include "soc/io_mux_reg.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "esp_cpu.h"
#include "esp_private/periph_ctrl.h"
#include "esp_private/esp_clk_tree_common.h"
#include "esp_private/gdma.h"
#include "esp_private/gdma_link.h"
#include "hal/cam_ll.h"
#include "soc/gpio_pins.h"
#include "cam_lsb.h"
#include "dma_pos.h"

#define SIG_CAM_CLK    150
#define SIG_CAM_PCLK   158
#define SIG_CAM_DE     159
#define SIG_CAM_HSYNC  160
#define SIG_CAM_VSYNC  161
#define SIG_CAM_DATA0  162

static lcd_cam_dev_t *hw;
static gdma_channel_handle_t ch;
static gdma_link_list_handle_t link;
static struct dma_pos cpos;             /* ring write position (dma_pos.h) */

/* The position comes from the hardware descriptor, not from counting EOF
 * interrupts, which merge or go missing during interrupt stalls. */
static bool IRAM_ATTR on_eof(gdma_channel_handle_t c, gdma_event_data_t *e, void *arg)
{
    dma_pos_update(&cpos);
    return false;
}

esp_err_t cam_lsb_init(int pclk_pad, const int data_pads[4], const int data_lanes[4], int width,
                       uint8_t *ring, size_t node, int nodes, int clk_div, uint32_t bytes_per_us)
{
    dma_pos_init(&cpos, ring, node * nodes, node, bytes_per_us);
    ESP_ERROR_CHECK(esp_clk_tree_acquire_src((soc_module_clk_t)CAM_CORE_CLK_SRC_DEFAULT));
    PERIPH_RCC_ACQUIRE_ATOMIC(PERIPH_LCD_CAM_MODULE, ref_count) {
        if (ref_count == 0) {
            cam_ll_enable_bus_clock(0, true);
            cam_ll_reset_register(0);
            cam_ll_select_core_clk_src(0, CAM_CORE_CLK_SRC_DEFAULT);
            cam_ll_set_core_clock_divider(0, 2, 0, 0);
        }
    }
    ESP_ERROR_CHECK(esp_clk_tree_acquire_src((soc_module_clk_t)CAM_CLK_SRC_DEFAULT));
    PERIPH_RCC_ATOMIC() {
        cam_ll_select_clk_src(0, CAM_CLK_SRC_DEFAULT);
        if (clk_div) cam_ll_set_group_clock_coeff(0, clk_div, 0, 0);
        cam_ll_enable_clk(0, true);
    }
    hw = CAM_LL_GET_HW(0);
    cam_ll_enable_stop_signal(hw, 0);
    cam_ll_swap_dma_data_byte_order(hw, 0);
    cam_ll_reverse_dma_data_bit_order(hw, 0);
    cam_ll_enable_vsync_generate_eof(hw, 0);
    cam_ll_set_recv_data_bytelen(hw, node - 1);
    cam_ll_set_line_int_num(hw, 0);
    cam_ll_enable_vsync_filter(hw, 0);
    cam_ll_set_input_data_width(hw, width);
    cam_ll_enable_invert_de(hw, 0);
    cam_ll_enable_invert_vsync(hw, 0);
    cam_ll_enable_invert_hsync(hw, 0);
    cam_ll_set_vh_de_mode(hw, 0);
    cam_ll_enable_rgb_yuv_convert(hw, 0);

    if (clk_div) {                          /* drive the sample clock onto the pad */
        gpio_func_sel(pclk_pad, PIN_FUNC_GPIO);
        esp_rom_gpio_connect_out_signal(pclk_pad, SIG_CAM_CLK, false, false);
        gpio_set_drive_capability(pclk_pad, GPIO_DRIVE_CAP_3);
    }
    gpio_input_enable(pclk_pad);
    esp_rom_gpio_connect_in_signal(pclk_pad, SIG_CAM_PCLK, false);
    for (int k = 0; k < width; k++)
        esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ZERO_INPUT, SIG_CAM_DATA0 + k, false);
    for (int k = 0; k < 4; k++) {
        gpio_input_enable(data_pads[k]);
        esp_rom_gpio_connect_in_signal(data_pads[k], SIG_CAM_DATA0 + data_lanes[k], false);
    }
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, SIG_CAM_DE, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, SIG_CAM_HSYNC, false);
    esp_rom_gpio_connect_in_signal(GPIO_MATRIX_CONST_ONE_INPUT, SIG_CAM_VSYNC, false);

    gdma_channel_alloc_config_t acfg = {0};
    esp_err_t err = gdma_new_axi_channel(&acfg, NULL, &ch);
    if (err != ESP_OK) return err;
    ESP_ERROR_CHECK(gdma_connect(ch, GDMA_MAKE_TRIGGER(GDMA_TRIG_PERIPH_CAM, 0)));
    gdma_strategy_config_t st = { .owner_check = false, .auto_update_desc = false };
    ESP_ERROR_CHECK(gdma_apply_strategy(ch, &st));
    gdma_transfer_config_t tc = { .max_data_burst_size = 16, .access_ext_mem = false };
    ESP_ERROR_CHECK(gdma_config_transfer(ch, &tc));
    gdma_link_list_config_t lcfg = { .num_items = nodes, .item_alignment = 8 };
    ESP_ERROR_CHECK(gdma_new_link_list(&lcfg, &link));
    gdma_buffer_mount_config_t mc[nodes];
    for (int i = 0; i < nodes; i++) {
        mc[i] = (gdma_buffer_mount_config_t){0};
        mc[i].buffer = ring + i * node;
        mc[i].buffer_alignment = 4;
        mc[i].length = node;
        mc[i].flags.mark_final = (i == nodes - 1) ? GDMA_FINAL_LINK_TO_HEAD : GDMA_FINAL_LINK_TO_DEFAULT;
    }
    ESP_ERROR_CHECK(gdma_link_mount_buffers(link, 0, mc, nodes, NULL));
    gdma_rx_event_callbacks_t cbs = { .on_recv_eof = on_eof };
    ESP_ERROR_CHECK(gdma_register_rx_event_callbacks(ch, &cbs, NULL));
    return ESP_OK;
}

void cam_lsb_start(bool invert_pclk)
{
    cam_ll_enable_invert_pclk(hw, invert_pclk);
    dma_pos_reset(&cpos);
    gdma_start(ch, gdma_link_get_head_addr(link));
    cam_ll_reset(hw);
    cam_ll_fifo_reset(hw);
    cam_ll_start(hw);
}

void cam_lsb_rephase(void)
{
    PERIPH_RCC_ATOMIC() { cam_ll_enable_clk(0, false); }
    esp_rom_delay_us(1 + (esp_cpu_get_cycle_count() & 7));
    PERIPH_RCC_ATOMIC() { cam_ll_enable_clk(0, true); }
}

void cam_lsb_stop(void)
{
    cam_ll_stop(hw);
    gdma_stop(ch);
    gdma_reset(ch);
}

uint64_t cam_lsb_written(void)
{
    return dma_pos_written(&cpos);
}

uint32_t cam_lsb_laps(void) { return cpos.laps; }
