#include "dshot_rmt_encoder.h"

#include <esp_check.h>

static const char *TAG = "dshot_encoder";

/**
 * @brief Type of Dshot ESC frame
 */
typedef union
{
    struct
    {
        uint16_t crc : 4;       /*!< CRC checksum */
        uint16_t telemetry : 1; /*!< Telemetry request */
        uint16_t throttle : 11; /*!< Throttle value */
    };
    uint16_t val;
} dshot_frame_t;

#ifndef __cplusplus
_Static_assert(sizeof(dshot_frame_t) == 0x02, "Invalid size of dshot_frame_t structure");
#endif

typedef struct
{
    rmt_encoder_t base;                   // the base "class" declares the standard encoder interface
    rmt_encoder_t *bytes_encoder;         // bytes_encoder to encode the dshot data
    rmt_encoder_t *copy_encoder;          // copy_encoder to encode the delay symbol
    rmt_symbol_word_t dshot_delay_symbol; // Delay between frames in RMT representation
    int state;                            // the current encoding state, i.e., we are in which encoding phase
    bool bidirectional;
} dshot_rmt_encoder_t;

static void make_dshot_frame(dshot_frame_t *frame, uint16_t throttle, bool telemetry, bool bidirectional)
{
    frame->throttle = throttle;
    frame->telemetry = telemetry;
    uint16_t val = frame->val;

    // CRC example for throttle = 1046, telemetry = false
    // value  = 100000101100
    // (>>4)  = 000010000010 # right shift value by 4
    // (^)    = 100010101110 # XOR with value
    // (>>8)  = 000000001000 # right shift value by 8
    // (^)    = 100010100110 # XOR with previous XOR
    // # if bidirectional
    // (~)    = 011101011001 # Invert
    // (0x0F) = 000000001111 # Mask 0x0F
    // (&)    = 000000001001 # CRC
    // # else
    // (0x0F) = 000000001111 # Mask 0x0F
    // (&)    = 000000000110 # CRC

    uint8_t crc = val ^ (val >> 4) ^ (val >> 8);
    if (bidirectional)
        crc = ~crc; // Invert
    crc = crc & 0xF0;

    frame->crc = crc >> 4; // right shift by 4 to convert to big endian

    // change the endian (esp32 little-endian to dshot big endian)
    val = frame->val;
    frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
}

static size_t rmt_encode_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                   const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
    dshot_rmt_encoder_t *dshot_encoder = __containerof(encoder, dshot_rmt_encoder_t, base);
    rmt_encoder_handle_t bytes_encoder = dshot_encoder->bytes_encoder;
    rmt_encoder_handle_t copy_encoder = dshot_encoder->copy_encoder;
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;

    // convert user data into dshot frame
    dshot_rmt_throttle_t *throttle = (dshot_rmt_throttle_t *)primary_data;
    dshot_frame_t frame = {};
    make_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req, dshot_encoder->bidirectional);

    switch (dshot_encoder->state)
    {
    case 0: // send the dshot frame
        encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &frame, sizeof(frame), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            dshot_encoder->state = 1; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    // fall-through
    case 1: // send dshot post delay
        encoded_symbols += copy_encoder->encode(copy_encoder, channel, &dshot_encoder->dshot_delay_symbol,
                                                sizeof(rmt_symbol_word_t), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE)
        {
            state |= RMT_ENCODING_COMPLETE;
            dshot_encoder->state = RMT_ENCODING_RESET; // switch to next state when current encoding session finished
        }
        if (session_state & RMT_ENCODING_MEM_FULL)
        {
            state |= RMT_ENCODING_MEM_FULL;
            goto out; // yield if there's no free space for encoding artifacts
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_dshot_encoder(rmt_encoder_t *encoder)
{
    dshot_rmt_encoder_t *dshot_encoder = __containerof(encoder, dshot_rmt_encoder_t, base);
    rmt_del_encoder(dshot_encoder->bytes_encoder);
    rmt_del_encoder(dshot_encoder->copy_encoder);
    free(dshot_encoder);
    return ESP_OK;
}

static esp_err_t rmt_dshot_encoder_reset(rmt_encoder_t *encoder)
{
    dshot_rmt_encoder_t *dshot_encoder = __containerof(encoder, dshot_rmt_encoder_t, base);
    rmt_encoder_reset(dshot_encoder->bytes_encoder);
    rmt_encoder_reset(dshot_encoder->copy_encoder);
    dshot_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

esp_err_t rmt_new_dshot_esc_encoder(const dshot_rmt_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    dshot_rmt_encoder_t *dshot_encoder = NULL;
    ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
    dshot_encoder = rmt_alloc_encoder_mem(sizeof(dshot_rmt_encoder_t));
    ESP_GOTO_ON_FALSE(dshot_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for musical score encoder");
    dshot_encoder->base.encode = rmt_encode_dshot_esc;
    dshot_encoder->base.del = rmt_del_dshot_encoder;
    dshot_encoder->base.reset = rmt_dshot_encoder_reset;
    uint32_t delay_ticks = config->resolution / 1e6 * config->post_delay_us;
    rmt_symbol_word_t dshot_delay_symbol = {
        .level0 = 0,
        .duration0 = delay_ticks / 2,
        .level1 = 0,
        .duration1 = delay_ticks / 2,
    };
    dshot_encoder->dshot_delay_symbol = dshot_delay_symbol;
    dshot_encoder->bidirectional = config->bidirectional;

    // different dshot protocol have its own timing requirements,
    float period_ticks = (float)config->resolution / config->baud_rate;
    // 1 and 0 is represented by a 74.850% and 37.425% duty cycle respectively
    unsigned int t1h_ticks = (unsigned int)(period_ticks * 0.7485);
    unsigned int t1l_ticks = (unsigned int)(period_ticks - t1h_ticks);
    unsigned int t0h_ticks = (unsigned int)(period_ticks * 0.37425);
    unsigned int t0l_ticks = (unsigned int)(period_ticks - t0h_ticks);
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = t0h_ticks,
            .level1 = 0,
            .duration1 = t0l_ticks,
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = t1h_ticks,
            .level1 = 0,
            .duration1 = t1l_ticks,
        },
        .flags.msb_first = 1,
    };
    ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &dshot_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &dshot_encoder->copy_encoder), err, TAG, "create copy encoder failed");
    *ret_encoder = &dshot_encoder->base;
    return ESP_OK;
err:
    if (dshot_encoder)
    {
        if (dshot_encoder->bytes_encoder)
        {
            rmt_del_encoder(dshot_encoder->bytes_encoder);
        }
        if (dshot_encoder->copy_encoder)
        {
            rmt_del_encoder(dshot_encoder->copy_encoder);
        }
        free(dshot_encoder);
    }
    return ret;
}
