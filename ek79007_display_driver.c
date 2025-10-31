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

#include "display_driver.h"

#include <math.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_log.h>

#include <atom.h>
#include <bif.h>
#include <context.h>
#include <debug.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <module.h>
#include <port.h>
#include <sys.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#include <trace.h>

#include "backlight_gpio.h"
#include "display_common.h"
#include "display_items.h"
#include "dsi_display.h"
#include "image_helpers.h"

#define CHAR_WIDTH 8

// EK79007 Commands
#define EK79007_LANE_CONFIG 0xB2
#define EK79007_SLPOUT 0x11
#define EK79007_DISPON 0x29
#define EK79007_MADCTL 0x36
#define EK79007_COLMOD 0x3A

// Memory Data Access Control bits
#define EK79007_MADCTL_MY 0x80
#define EK79007_MADCTL_MX 0x40
#define EK79007_MADCTL_MV 0x20
#define EK79007_MADCTL_ML 0x10
#define EK79007_MADCTL_RGB 0x00
#define EK79007_MADCTL_BGR 0x08

#include "font.c"

static const char *TAG = "ek79007_display_driver";

static void send_message(term pid, term message, GlobalContext *global);

static inline void delay(int ms)
{
    vTaskDelay(ms / portTICK_PERIOD_MS);
}

struct DSI
{
    struct DSIDisplay dsi_disp;
    int reset_gpio;

    avm_int_t rotation;

    Context *ctx;
};

struct Screen
{
    int w;
    int h;
    uint16_t *pixels;
    uint16_t *pixels_out;
};

static struct Screen *screen;

// Alpha blending function for RGB565
static inline uint16_t alpha_blend_rgb565(uint32_t fg, uint32_t bg, uint8_t alpha)
{
    alpha = (alpha + 4) >> 3;
    bg = (bg | (bg << 16)) & 0b00000111111000001111100000011111;
    fg = (fg | (fg << 16)) & 0b00000111111000001111100000011111;
    uint32_t result = ((((fg - bg) * alpha) >> 5) + bg) & 0b00000111111000001111100000011111;
    return (uint16_t)((result >> 16) | result);
}

static inline uint8_t rgba8888_get_alpha(uint32_t color)
{
    return color & 0xFF;
}

static inline uint16_t rgba8888_color_to_rgb565(struct Screen *s, uint32_t color)
{
    uint8_t r = color >> 24;
    uint8_t g = (color >> 16) & 0xFF;
    uint8_t b = (color >> 8) & 0xFF;

    return (((uint16_t)(r >> 3)) << 11) | (((uint16_t)(g >> 2)) << 5) | ((uint16_t) b >> 3);
}

static inline uint16_t rgb565_color_to_surface(struct Screen *s, uint16_t color16)
{
    // For DSI, we may need different byte order than SPI
    return color16;
}

static inline uint16_t uint32_color_to_surface(struct Screen *s, uint32_t color)
{
    uint16_t color16 = rgba8888_color_to_rgb565(s, color);
    return rgb565_color_to_surface(s, color16);
}

struct PendingReply
{
    uint64_t pending_call_ref_ticks;
    term pending_call_pid;
};

static QueueHandle_t display_messages_queue;

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx);
static void display_init(Context *ctx, term opts);
static void display_init_ek79007(struct DSI *dsi);

static inline void writecommand(struct DSI *dsi, uint8_t command, const uint8_t *data, size_t data_len)
{
    dsi_display_write_command(&dsi->dsi_disp, command, data, data_len);
}

static inline void set_rotation(struct DSI *dsi, avm_int_t rotation)
{
    uint8_t madctl = EK79007_MADCTL_RGB;

    switch (rotation) {
        case 0:
            // Portrait
            break;
        case 1:
            // Landscape
            madctl |= EK79007_MADCTL_MV | EK79007_MADCTL_MY;
            break;
        case 2:
            // Portrait inverted
            madctl |= EK79007_MADCTL_MY | EK79007_MADCTL_MX;
            break;
        case 3:
            // Landscape inverted
            madctl |= EK79007_MADCTL_MV | EK79007_MADCTL_MX;
            break;
    }

    writecommand(dsi, EK79007_MADCTL, &madctl, 1);
}

// Drawing functions - adapted from ST7789
static void draw_pixel(int x, int y, uint16_t *px, uint32_t color)
{
    uint8_t alpha = rgba8888_get_alpha(color);
    if (alpha == 0) {
        return;
    }

    uint16_t color16 = uint32_color_to_surface(screen, color);

    // Calculate offset in framebuffer: y * width + x
    int offset = y * screen->w + x;

    if (alpha == 255) {
        px[offset] = color16;
    } else {
        px[offset] = alpha_blend_rgb565(color16, px[offset], alpha);
    }
}

static int draw_rect_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int fill_pixels = item->width;

    if (max_line_len < fill_pixels) {
        fill_pixels = max_line_len;
    }

    int start_offset = xpos - item->x;
    int write_start = start_offset;
    int write_end = start_offset + fill_pixels;

    for (int i = write_start; i < write_end; i++) {
        // xpos is the starting screen x coordinate for this call
        // i is the offset within the item being drawn
        // So screen x = xpos + (i - start_offset)
        int screen_x = xpos + (i - start_offset);
        draw_pixel(screen_x, ypos, screen->pixels, item->brcolor);
    }

    return fill_pixels;
}

static int draw_text_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int fill_pixels = item->width;

    if (max_line_len < fill_pixels) {
        fill_pixels = max_line_len;
    }

    int start_offset = xpos - item->x;

    int y_offset = (ypos - item->y);

    const char *text = item->data.text_data.text;

    int end_offset = start_offset + fill_pixels;
    for (int i = start_offset; i < end_offset; i++) {
        int char_index = i / CHAR_WIDTH;

        // Get the character and calculate glyph pointer
        unsigned char c = (unsigned char)text[char_index];
        // Font is 8x16, so 16 bytes per character
        const unsigned char *glyph = fontdata + c * 16;
        unsigned char char_line = glyph[y_offset];

        int pos = i % CHAR_WIDTH;
        uint32_t pixel_value = (char_line & (0x80 >> pos)) ? item->data.text_data.fgcolor : item->brcolor;

        int screen_x = xpos + (i - start_offset);
        draw_pixel(screen_x, ypos, screen->pixels, pixel_value);
    }

    return fill_pixels;
}

static int draw_image_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int fill_pixels = item->width;

    if (max_line_len < fill_pixels) {
        fill_pixels = max_line_len;
    }

    int start_offset = xpos - item->x;
    int y_offset = (ypos - item->y);

    const uint32_t *image_data = (const uint32_t *) item->data.image_data.pix;
    for (int i = start_offset; i < start_offset + fill_pixels; i++) {
        int img_offset = y_offset * item->width + i;
        uint32_t img_pix = image_data[img_offset];
        int screen_x = xpos + (i - start_offset);
        draw_pixel(screen_x, ypos, screen->pixels, img_pix);
    }

    return fill_pixels;
}

static int draw_circle_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    // Circle center relative to bounding box
    int cx = item->data.circle_data.radius;
    int cy = item->data.circle_data.radius;
    int radius = item->data.circle_data.radius;
    bool filled = item->data.circle_data.filled;

    // Current pixel position relative to bounding box
    int x_offset = xpos - item->x;
    int y_offset = ypos - item->y;

    // Distance from center
    int dx = x_offset - cx;
    int dy = y_offset - cy;
    int dist2 = dx * dx + dy * dy;
    int r2 = radius * radius;

    bool should_draw = false;
    if (filled) {
        // Filled circle: draw if inside radius
        should_draw = (dist2 <= r2);
    } else {
        // Unfilled circle: draw only on the edge (with 1-pixel thickness)
        int inner_r2 = (radius - 1) * (radius - 1);
        should_draw = (dist2 >= inner_r2 && dist2 <= r2);
    }

    if (should_draw) {
        draw_pixel(xpos, ypos, screen->pixels, item->brcolor);
        return 1;
    }

    // Not part of circle - return 0 to allow underlying items to show
    return 0;
}

static int draw_arc_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    // Arc center relative to bounding box (based on outer radius)
    int cx = item->data.arc_data.outer_radius;
    int cy = item->data.arc_data.outer_radius;
    int inner_radius = item->data.arc_data.inner_radius;
    int outer_radius = item->data.arc_data.outer_radius;
    bool filled = item->data.arc_data.filled;
    int start_angle = item->data.arc_data.start_angle;
    int end_angle = item->data.arc_data.end_angle;

    // Current pixel position relative to bounding box
    int x_offset = xpos - item->x;
    int y_offset = ypos - item->y;

    // Distance from center
    int dx = x_offset - cx;
    int dy = y_offset - cy;
    int dist2 = dx * dx + dy * dy;
    int inner_r2 = inner_radius * inner_radius;
    int outer_r2 = outer_radius * outer_radius;

    // Check if pixel is between inner and outer radius
    bool in_radial_range = (dist2 >= inner_r2 && dist2 <= outer_r2);

    if (in_radial_range) {
        // Calculate angle in degrees (0° = right, 90° = down, counter-clockwise from right)
        // atan2 returns radians from -π to π
        double angle_rad = atan2(dy, dx);
        int angle_deg = (int)(angle_rad * 180.0 / M_PI);

        // Normalize to 0-359
        if (angle_deg < 0) {
            angle_deg += 360;
        }

        // Check if angle is within arc range
        bool in_angular_range = false;
        if (start_angle <= end_angle) {
            // Normal case: arc doesn't cross 0°
            in_angular_range = (angle_deg >= start_angle && angle_deg <= end_angle);
        } else {
            // Arc crosses 0° (e.g., 270° to 90°)
            in_angular_range = (angle_deg >= start_angle || angle_deg <= end_angle);
        }

        if (in_angular_range) {
            if (filled) {
                // Filled: draw all pixels in the angular and radial range
                draw_pixel(xpos, ypos, screen->pixels, item->brcolor);
                return 1;
            } else {
                // Unfilled: only draw the outline
                // Check if on inner edge, outer edge, or on angular boundaries
                int dist = (int)sqrt(dist2);
                bool on_inner_edge = (dist >= inner_radius - 1 && dist <= inner_radius + 1);
                bool on_outer_edge = (dist >= outer_radius - 1 && dist <= outer_radius + 1);

                // Check if near angular boundaries (within ~2 degrees tolerance)
                int angle_tolerance = 2;
                bool on_start_edge = false;
                bool on_end_edge = false;

                if (start_angle <= end_angle) {
                    on_start_edge = (angle_deg >= start_angle && angle_deg <= start_angle + angle_tolerance);
                    on_end_edge = (angle_deg >= end_angle - angle_tolerance && angle_deg <= end_angle);
                } else {
                    // Arc crosses 0°
                    on_start_edge = (angle_deg >= start_angle && angle_deg <= start_angle + angle_tolerance) ||
                                   (angle_deg >= 360 - angle_tolerance);
                    on_end_edge = (angle_deg <= end_angle) && (angle_deg <= angle_tolerance);
                }

                if (on_inner_edge || on_outer_edge || on_start_edge || on_end_edge) {
                    draw_pixel(xpos, ypos, screen->pixels, item->brcolor);
                    return 1;
                }
            }
        }
    }

    // Not part of arc - return 0 to allow underlying items to show
    return 0;
}

static int draw_scaled_cropped_img_x(int xpos, int ypos, int max_line_len, BaseDisplayItem *item)
{
    int fill_pixels = item->width;

    if (max_line_len < fill_pixels) {
        fill_pixels = max_line_len;
    }

    int start_offset = xpos - item->x;
    int y_offset = (ypos - item->y);

    int crop_x = item->source_x;
    int crop_y = item->source_y;

    int source_width = item->data.image_data_with_size.width;

    int scale_x = item->x_scale;
    int scale_y = item->y_scale;

    const uint32_t *image_data = (const uint32_t *) item->data.image_data_with_size.pix;
    for (int i = start_offset; i < start_offset + fill_pixels; i++) {
        int source_x = crop_x + (i * 256 / scale_x);
        int source_y = crop_y + (y_offset * 256 / scale_y);

        int img_offset = source_y * source_width + source_x;
        uint32_t img_pix = image_data[img_offset];
        int screen_x = xpos + (i - start_offset);
        draw_pixel(screen_x, ypos, screen->pixels, img_pix);
    }

    return fill_pixels;
}

static int find_max_line_len(BaseDisplayItem *items, int count, int xpos, int ypos)
{
    int line_len = screen->w - xpos;

    for (int i = 0; i < count; i++) {
        BaseDisplayItem *item = &items[i];

        if ((xpos < item->x) && (ypos >= item->y) && (ypos < item->y + item->height)) {
            int len_to_item = item->x - xpos;
            line_len = (line_len > len_to_item) ? len_to_item : line_len;
        }
    }

    return line_len;
}

static int draw_x(int xpos, int ypos, BaseDisplayItem *items, int items_count)
{
    bool below = false;

    for (int i = 0; i < items_count; i++) {
        BaseDisplayItem *item = &items[i];
        if ((xpos < item->x) || (xpos >= item->x + item->width) || (ypos < item->y) || (ypos >= item->y + item->height)) {
            continue;
        }

        int max_line_len = below ? 1 : find_max_line_len(items, i, xpos, ypos);

        int drawn_pixels = 0;
        switch (items[i].primitive) {
            case Image:
                drawn_pixels = draw_image_x(xpos, ypos, max_line_len, item);
                break;

            case Rect:
                drawn_pixels = draw_rect_x(xpos, ypos, max_line_len, item);
                break;

            case Circle:
                drawn_pixels = draw_circle_x(xpos, ypos, max_line_len, item);
                break;

            case Arc:
                drawn_pixels = draw_arc_x(xpos, ypos, max_line_len, item);
                break;

            case ScaledCroppedImage:
                drawn_pixels = draw_scaled_cropped_img_x(xpos, ypos, max_line_len, item);
                break;

            case Text:
                drawn_pixels = draw_text_x(xpos, ypos, max_line_len, item);
                break;
            default: {
                fprintf(stderr, "unexpected display list command.\n");
            }
        }

        if (drawn_pixels != 0) {
            return drawn_pixels;
        }

        below = true;
    }

    return 1;
}

static void do_update(Context *ctx, term display_list)
{
    int proper;
    int len = term_list_length(display_list, &proper);

    BaseDisplayItem *items = malloc(sizeof(BaseDisplayItem) * len);

    term t = display_list;
    for (int i = 0; i < len; i++) {
        init_item(&items[i], term_get_list_head(t), ctx);
        t = term_get_list_tail(t);
    }

    int screen_width = screen->w;
    int screen_height = screen->h;
    struct DSI *dsi = ctx->platform_data;

    // Clear the entire framebuffer
    memset(screen->pixels, 0, screen_width * screen_height * sizeof(uint16_t));

    // Render all scanlines to the framebuffer
    for (int ypos = 0; ypos < screen_height; ypos++) {
        int xpos = 0;
        while (xpos < screen_width) {
            int drawn_pixels = draw_x(xpos, ypos, items, len);
            xpos += drawn_pixels;
        }
    }

    // Transfer the entire framebuffer to the display in one call
    dsi_display_draw_bitmap(&dsi->dsi_disp, 0, 0, screen_width, screen_height, screen->pixels);

    destroy_items(items, len);
}

static void process_message(Message *message, Context *ctx)
{
    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        fprintf(stderr, "Received invalid message.");
        AVM_ABORT();
    }

    term req = gen_message.req;
    if (UNLIKELY(!term_is_tuple(req) || term_get_tuple_arity(req) < 1)) {
        AVM_ABORT();
    }

    term cmd = term_get_tuple_element(req, 0);

    if (cmd == context_make_atom(ctx, ATOM_STR("\x6", "update"))) {
        term display_list = term_get_tuple_element(req, 1);
        do_update(ctx, display_list);
    } else {
        ESP_LOGE(TAG, "Unknown command");
    }

    // Send reply back to caller
    BEGIN_WITH_STACK_HEAP(TUPLE_SIZE(2) + REF_SIZE, heap);
    term return_tuple = term_alloc_tuple(2, &heap);
    term_put_tuple_element(return_tuple, 0, gen_message.ref);
    term_put_tuple_element(return_tuple, 1, OK_ATOM);

    send_message(gen_message.pid, return_tuple, ctx->global);
    END_WITH_STACK_HEAP(heap, ctx->global);
}

static void process_messages(void *dsi_ptr)
{
    struct DSI *dsi = (struct DSI *) dsi_ptr;

    while (true) {
        Message *message;
        xQueueReceive(display_messages_queue, &message, portMAX_DELAY);
        process_message(message, dsi->ctx);

        BEGIN_WITH_STACK_HEAP(1, temp_heap);
        mailbox_message_dispose(&message->base, &temp_heap);
        END_WITH_STACK_HEAP(temp_heap, dsi->ctx->global);
    }
}

static void display_init_ek79007(struct DSI *dsi)
{
    ESP_LOGI(TAG, "Initializing EK79007 LCD controller");

    // EK79007 initialization sequence based on Espressif's implementation
    uint8_t lane_config = 0x10; // 2-lane mode
    writecommand(dsi, EK79007_LANE_CONFIG, &lane_config, 1);
    delay(10);

    // Vendor-specific initialization
    uint8_t data;

    data = 0x8B;
    writecommand(dsi, 0x80, &data, 1);

    data = 0x78;
    writecommand(dsi, 0x81, &data, 1);

    data = 0x84;
    writecommand(dsi, 0x82, &data, 1);

    data = 0x88;
    writecommand(dsi, 0x83, &data, 1);

    data = 0xA8;
    writecommand(dsi, 0x84, &data, 1);

    data = 0xE3;
    writecommand(dsi, 0x85, &data, 1);

    data = 0x88;
    writecommand(dsi, 0x86, &data, 1);

    // Exit sleep mode
    writecommand(dsi, EK79007_SLPOUT, NULL, 0);
    delay(120);

    // Set color mode to RGB565
    data = 0x55; // 16-bit color
    writecommand(dsi, EK79007_COLMOD, &data, 1);

    ESP_LOGI(TAG, "EK79007 initialization complete");
}

static void display_init(Context *ctx, term opts)
{
    ESP_LOGI(TAG, "EK79007 display_init called");

    struct DSI *dsi = calloc(1, sizeof(struct DSI));
    dsi->ctx = ctx;
    ctx->native_handler = display_driver_consume_mailbox;
    ctx->platform_data = dsi;

    int screen_w = 1024;
    int screen_h = 600;

    // Parse width and height
    term width_term = interop_kv_get_value_default(opts, ATOM_STR("\x5", "width"), term_from_int(1024), ctx->global);
    term height_term = interop_kv_get_value_default(opts, ATOM_STR("\x6", "height"), term_from_int(600), ctx->global);

    bool ok = true;
    if (term_is_integer(width_term)) {
        screen_w = term_to_int(width_term);
    } else {
        ok = false;
    }

    if (term_is_integer(height_term)) {
        screen_h = term_to_int(height_term);
    } else {
        ok = false;
    }

    // Parse rotation
    term rotation_term = interop_kv_get_value_default(opts, ATOM_STR("\x8", "rotation"), term_from_int(0), ctx->global);
    if (term_is_integer(rotation_term)) {
        dsi->rotation = term_to_int(rotation_term);
    } else {
        ok = false;
    }

    // Parse reset GPIO
    bool reset_configured = display_common_gpio_from_opts(opts, ATOM_STR("\x5", "reset"), &dsi->reset_gpio, ctx->global);

    if (UNLIKELY(!ok)) {
        ESP_LOGE(TAG, "Failed init: invalid display parameters.");
        return;
    }

    // Initialize DSI infrastructure
    struct DSIDisplayConfig dsi_config;
    dsi_display_init_config(&dsi_config);
    dsi_display_parse_config(&dsi_config, opts, ctx->global);

    dsi_config.panel_width = screen_w;
    dsi_config.panel_height = screen_h;

    if (!dsi_display_init(&dsi->dsi_disp, &dsi_config)) {
        ESP_LOGE(TAG, "Failed to initialize DSI display");
        return;
    }

    ESP_LOGI(TAG, "EK79007 will use DBI command mode (no DPI panel needed)");

    // Hardware reset
    if (reset_configured) {
        gpio_set_direction(dsi->reset_gpio, GPIO_MODE_OUTPUT);
        gpio_set_level(dsi->reset_gpio, 1);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        gpio_set_level(dsi->reset_gpio, 0);
        vTaskDelay(20 / portTICK_PERIOD_MS);
        gpio_set_level(dsi->reset_gpio, 1);
        vTaskDelay(120 / portTICK_PERIOD_MS);
    }

    // Initialize EK79007
    display_init_ek79007(dsi);

    // Initialize the DPI panel (must be done after sending init commands but before display on)
    if (dsi->dsi_disp.panel_handle) {
        ESP_LOGI(TAG, "Initializing DPI panel");
        esp_err_t ret = esp_lcd_panel_init(dsi->dsi_disp.panel_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init DPI panel: %s", esp_err_to_name(ret));
        }
    }

    // Set rotation
    set_rotation(dsi, dsi->rotation);

    // Turn on display
    writecommand(dsi, EK79007_DISPON, NULL, 0);
    delay(120);

    // Initialize backlight
    struct BacklightGPIOConfig backlight_config;
    backlight_gpio_init_config(&backlight_config);
    backlight_gpio_parse_config(&backlight_config, opts, ctx->global);
    backlight_gpio_init(&backlight_config);

    // Allocate screen buffers
    screen = calloc(1, sizeof(struct Screen));
    screen->w = screen_w;
    screen->h = screen_h;

    // Allocate full framebuffer in PSRAM (1024x600x2 bytes = ~1.2MB)
    // Use MALLOC_CAP_SPIRAM for large buffers
    size_t framebuffer_size = screen_w * screen_h * sizeof(uint16_t);
    screen->pixels = heap_caps_malloc(framebuffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // Allocate scanline buffer in DMA-capable memory for transfers
    screen->pixels_out = heap_caps_malloc(screen_w * sizeof(uint16_t), MALLOC_CAP_DMA);

    if (!screen->pixels || !screen->pixels_out) {
        ESP_LOGE(TAG, "Failed to allocate pixel buffers");
        return;
    }

    ESP_LOGI(TAG, "Allocated framebuffer: %zu bytes in PSRAM", framebuffer_size);

    ESP_LOGI(TAG, "EK79007 display driver initialized successfully");

    xTaskCreate(process_messages, "display", 10000, dsi, 1, NULL);
}

static NativeHandlerResult display_driver_consume_mailbox(Context *ctx)
{
    MailboxMessage *mbox_msg = mailbox_take_message(&ctx->mailbox);
    Message *msg = CONTAINER_OF(mbox_msg, Message, base);

    xQueueSend(display_messages_queue, &msg, 1);

    return NativeContinue;
}

Context *ek79007_display_create_port(GlobalContext *global, term opts)
{
    Context *ctx = context_new(global);

    display_messages_queue = xQueueCreate(10, sizeof(term));

    display_init(ctx, opts);

    return ctx;
}

static void send_message(term pid, term message, GlobalContext *global)
{
    int local_process_id = term_to_local_process_id(pid);
    globalcontext_send_message(global, local_process_id, message);
}
