/*
 * This file is part of AtomGL.
 *
 * Copyright 2024 Paul Guyot <pguyot@kallisys.net>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "dsi_display.h"

#include <string.h>

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_log.h>
#include <esp_ldo_regulator.h>

#include <globalcontext.h>
#include <interop.h>
#include <term.h>
#include <utils.h>

#include "display_common.h"

static const char *TAG = "dsi_display";

bool dsi_display_write_command(struct DSIDisplay *dsi_disp, uint8_t command, const uint8_t *data, size_t data_len)
{
    if (dsi_disp->io_handle == NULL) {
        ESP_LOGE(TAG, "IO handle is NULL");
        return false;
    }

    esp_err_t ret = esp_lcd_panel_io_tx_param(dsi_disp->io_handle, command, data, data_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command 0x%02x: %s", command, esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool dsi_display_draw_bitmap(struct DSIDisplay *dsi_disp, int x_start, int y_start, int x_end, int y_end, const void *color_data)
{
    if (dsi_disp->panel_handle == NULL) {
        ESP_LOGE(TAG, "Panel handle is NULL");
        return false;
    }

    // Use DPI panel's draw_bitmap which handles frame buffer DMA
    esp_err_t ret = esp_lcd_panel_draw_bitmap(dsi_disp->panel_handle, x_start, y_start, x_end, y_end, color_data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to draw bitmap: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool dsi_display_parse_config(struct DSIDisplayConfig *dsi_config, term opts, GlobalContext *global)
{
    // Parse num_lanes (default 2)
    term num_lanes_atom = globalcontext_make_atom(global, ATOM_STR("\x9", "num_lanes"));
    term num_lanes_term = interop_proplist_get_value_default(opts, num_lanes_atom, term_from_int(2));
    if (term_is_integer(num_lanes_term)) {
        dsi_config->num_lanes = term_to_int(num_lanes_term);
    } else {
        dsi_config->num_lanes = 2;
    }

    // Parse lane_bit_rate_mbps (default 900)
    term lane_rate_atom = globalcontext_make_atom(global, ATOM_STR("\x11", "lane_bit_rate_mbps"));
    term lane_rate_term = interop_proplist_get_value_default(opts, lane_rate_atom, term_from_int(900));
    if (term_is_integer(lane_rate_term)) {
        dsi_config->lane_bit_rate_mbps = term_to_int(lane_rate_term);
    } else {
        dsi_config->lane_bit_rate_mbps = 900;
    }

    return true;
}

bool dsi_display_init(struct DSIDisplay *dsi_disp, struct DSIDisplayConfig *dsi_config)
{
    memset(dsi_disp, 0, sizeof(struct DSIDisplay));

    // Power on MIPI DSI PHY via LDO
    ESP_LOGI(TAG, "Powering on MIPI DSI PHY");
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = 3,
        .voltage_mv = 2500,
    };
    esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &dsi_disp->ldo_mipi_phy);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire LDO channel for MIPI PHY: %s", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "MIPI DSI PHY powered on");

    ESP_LOGI(TAG, "Installing MIPI-DSI bus with %d lanes at %d Mbps",
             dsi_config->num_lanes, dsi_config->lane_bit_rate_mbps);

    // Configure MIPI-DSI bus
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = dsi_config->num_lanes,
        .phy_clk_src = 0,  // Use explicit 0 instead of MIPI_DSI_PHY_CLK_SRC_DEFAULT
        .lane_bit_rate_mbps = dsi_config->lane_bit_rate_mbps,
    };

    ESP_LOGI(TAG, "DSI bus config: bus_id=%d, lanes=%d, phy_clk_src=%d, rate=%d Mbps",
             bus_config.bus_id, bus_config.num_data_lanes, bus_config.phy_clk_src, bus_config.lane_bit_rate_mbps);

    ESP_LOGI(TAG, "About to call esp_lcd_new_dsi_bus...");
    ESP_LOGI(TAG, "%s:%d>%s", __FILE__, __LINE__, __FUNCTION__);

    ret = esp_lcd_new_dsi_bus(&bus_config, &dsi_disp->dsi_bus);

    ESP_LOGI(TAG, "esp_lcd_new_dsi_bus returned with ret=%d", ret);
    ESP_LOGI(TAG, "%s:%d>%s", __FILE__, __LINE__, __FUNCTION__);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DSI bus: %s", esp_err_to_name(ret));
        return false;
    }

    ESP_LOGI(TAG, "Installing MIPI-DSI panel IO (DBI for commands)");

    // Configure panel IO (DBI interface for sending initialization commands)
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };

    ret = esp_lcd_new_panel_io_dbi(dsi_disp->dsi_bus, &dbi_config, &dsi_disp->io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        esp_lcd_del_dsi_bus(dsi_disp->dsi_bus);
        return false;
    }

    ESP_LOGI(TAG, "Installing MIPI-DPI panel (video mode for pixel data)");

    // Configure DPI panel for pixel data transfer via frame buffer
    esp_lcd_dpi_panel_config_t dpi_config = {
        .virtual_channel = 0,
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = 80,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .video_timing = {
            .h_size = dsi_config->panel_width,
            .v_size = dsi_config->panel_height,
            .hsync_back_porch = dsi_config->hporch_back,
            .hsync_pulse_width = dsi_config->hsync,
            .hsync_front_porch = dsi_config->hporch_front,
            .vsync_back_porch = dsi_config->vporch_back,
            .vsync_pulse_width = dsi_config->vsync,
            .vsync_front_porch = dsi_config->vporch_front,
        },
        .flags = {
            .use_dma2d = false,
        },
    };

    ret = esp_lcd_new_panel_dpi(dsi_disp->dsi_bus, &dpi_config, &dsi_disp->panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create DPI panel: %s", esp_err_to_name(ret));
        esp_lcd_panel_io_del(dsi_disp->io_handle);
        esp_lcd_del_dsi_bus(dsi_disp->dsi_bus);
        return false;
    }

    ESP_LOGI(TAG, "DSI display infrastructure initialized successfully");
    return true;
}

void dsi_display_init_config(struct DSIDisplayConfig *dsi_config)
{
    memset(dsi_config, 0, sizeof(struct DSIDisplayConfig));

    // Default configuration for ESP32-P4 Function EV Board with EK79007
    dsi_config->num_lanes = 2;
    dsi_config->lane_bit_rate_mbps = 900;  // Match EK79007_PANEL_BUS_DSI_2CH_CONFIG
    dsi_config->panel_width = 1024;
    dsi_config->panel_height = 600;

    // Default timing parameters for 1024x600@60Hz
    dsi_config->hporch_front = 160;
    dsi_config->hporch_back = 160;
    dsi_config->hsync = 10;
    dsi_config->vporch_front = 23;
    dsi_config->vporch_back = 23;
    dsi_config->vsync = 10;
}
