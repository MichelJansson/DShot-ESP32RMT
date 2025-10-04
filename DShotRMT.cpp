#include "DShotRMT.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>

#include "dshot_rmt_encoder.h"

static const char *TAG = "DShotRMT";

// Constructor that takes gpio and rmtChannel as arguments
DShotRMT::DShotRMT(gpio_num_t gpio, dshot_mode_t dshot_mode)
{
    gpio_num = gpio;

    uint32_t baudrate = 0;
    uint32_t post_delay_us = 0;
    switch (dshot_mode)
    {
    case DSHOT150:
        baudrate = 150000;
        post_delay_us = 50;
        break;
    case DSHOT300:
    case DSHOT300_BIDIRECTIONAL:
        baudrate = 300000;
        post_delay_us = 3;
        break;
    case DSHOT600:
    case DSHOT600_BIDIRECTIONAL:
        baudrate = 600000;
        post_delay_us = 3;
        break;
    case DSHOT1200:
    case DSHOT1200_BIDIRECTIONAL:
        baudrate = 1200000;
        post_delay_us = 3;
        break;
    default:
        break;
    }

    if (dshot_mode >= DSHOT300_BIDIRECTIONAL && dshot_mode <= DSHOT1200_BIDIRECTIONAL)
    {
        is_bidirectional = true;
        uint32_t telem_baudrate = baudrate * 5 / 4;
        telemetry_bit_len_ticks = (unsigned int)((float)DSHOT_RMT_RESOLUTION_HZ / telem_baudrate);

        uint32_t throttle_frame_length = ((float)1000000 / baudrate * 16);
        uint32_t telemetry_frame_length = ((float)1000000 / telem_baudrate * 21);
        telemetry_timeout_us = throttle_frame_length + 30 + telemetry_frame_length + 40; // throttle frame + receive delay + telemetry frame + RMT delay;
    }

    ESP_LOGI(TAG, "Install Dshot RMT encoder");
    encoder_config = {
        .resolution = DSHOT_RMT_RESOLUTION_HZ,
        .baud_rate = baudrate,
        .bidirectional = is_bidirectional,
        .post_delay_us = post_delay_us, // extra delay between each frame
    };

    ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &dshot_encoder));

    if (is_bidirectional)
    {
        ESP_LOGI(TAG, "Create RMT RX channel");
        const rmt_rx_channel_config_t rmt_rx_channel_config = {
            .gpio_num = gpio,
            .clk_src = RMT_CLK_SRC_DEFAULT,
            .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
            .mem_block_symbols = MAX_BLOCKS,
            .flags = {
                .with_dma = true,
                .io_loop_back = true,
            },
            .intr_priority = 1,
        };
        ESP_ERROR_CHECK(rmt_new_rx_channel(&rmt_rx_channel_config, &rmt_rx_channel));

        ESP_LOGI(TAG, "Register RX callback");
        rmt_rx_event_callbacks_t rx_callbacks = {
            .on_recv_done = rxDoneCallback_ISR,
        };
        ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rmt_rx_channel, &rx_callbacks, this));

        rx_config = {
            .signal_range_min_ns = 150,  //(uint32_t)(313 * 0.8),  // dshot 1200 shortest pulse is 0.313us
            .signal_range_max_ns = 25000 //(uint32_t)(5000 * 1.2), // dshot 150 longest pulse is 5.0us
        };
    }

    ESP_LOGI(TAG, "Create RMT TX channel");
    const rmt_tx_channel_config_t rmt_tx_channel_config = {
        .gpio_num = gpio,
        .clk_src = RMT_CLK_SRC_DEFAULT, // a clock that can provide needed resolution
        .resolution_hz = DSHOT_RMT_RESOLUTION_HZ,
        .mem_block_symbols = MAX_BLOCKS,
        .trans_queue_depth = 1, // set the number of transactions that can be pending in the background
        .intr_priority = 1,
        .flags = {
            .invert_out = is_bidirectional,
            .with_dma = true,
            .io_loop_back = true,
        }};
    ESP_ERROR_CHECK(rmt_new_tx_channel(&rmt_tx_channel_config, &rmt_tx_channel));

    if (is_bidirectional)
    {
        ESP_LOGI(TAG, "Register TX callback");
        rmt_tx_event_callbacks_t tx_callbacks = {
            .on_trans_done = txDoneCallback_ISR,
        };
        ESP_ERROR_CHECK(rmt_tx_register_event_callbacks(rmt_tx_channel, &tx_callbacks, this));
    }
    tx_config = {
        .loop_count = 0,
        .flags = {
            .queue_nonblocking = true,
        },
    };

    // Initialize structs
    throttle = {
        .throttle = 0,
        .telemetry_req = false, // separate telemetry wire is not supported in this lib
    };
}

DShotRMT::~DShotRMT()
{
    // Uninstall the RMT driver
    if (dshot_encoder)
    {
        rmt_del_encoder(dshot_encoder);
    }

    if (rmt_rx_channel)
    {
        if (enabled)
        {
            ESP_ERROR_CHECK(rmt_disable(rmt_rx_channel));
        }
        ESP_ERROR_CHECK(rmt_del_channel(rmt_rx_channel));
    }

    if (rmt_tx_channel)
    {
        if (enabled)
        {
            ESP_ERROR_CHECK(rmt_disable(rmt_tx_channel));
        }
        ESP_ERROR_CHECK(rmt_del_channel(rmt_tx_channel));
    }

    enabled = false;
}

void DShotRMT::begin()
{
    ESP_LOGI(TAG, "Enable RMT TX channel");
    ESP_ERROR_CHECK(rmt_enable(rmt_tx_channel));
    mode = 1;
    enabled = true;

    ESP_LOGI(TAG, "Resetting and Arming ESC...");
    sendTicks(0, pdMS_TO_TICKS(DSHOT_ARM_DELAY));

    ESP_LOGI(TAG, "Done!");
}

void DShotRMT::sendThrottle(uint16_t throttle_value)
{
    if (throttle_value > DSHOT_THROTTLE_MAX)
    {
        throttle_value = DSHOT_THROTTLE_MAX;
    }
    else if (throttle_value < DSHOT_THROTTLE_MIN)
    {
        throttle_value = DSHOT_THROTTLE_MIN;
    }

    send(throttle_value);
}

uint32_t DShotRMT::getErpm()
{
    if (!is_bidirectional)
        return INVALID_TELEMETRY_VALUE;

    uint32_t erpm;
    auto erpmData = convertGcrToErpmData(telemetry_gcr);
    // ESP_LOGI(TAG, "eRPM Data : %lu", erpmData);
    erpm = convertErpmDataToErpmPeriod(erpmData);
    // ESP_LOGI(TAG, "eRPM : %lu", erpm);
    return erpm;
}

static unsigned long IRAM_ATTR get_micros()
{
    return (unsigned long)(esp_timer_get_time());
}

static esp_err_t IRAM_ATTR waitForFlag(const bool *flag, const uint32_t timout_us)
{
    if (!*flag && timout_us)
    {
        uint32_t m = get_micros();
        uint32_t e = (m + timout_us);
        if (m > e)
        { // overflow
            while (!*flag && get_micros() > e)
            {
                asm("nop");
            }
        }
        while (!*flag && get_micros() < e)
        {
            asm("nop");
        }
    }

    return *flag ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t DShotRMT::waitForErpm(uint32_t &erpm)
{
    erpm = INVALID_TELEMETRY_VALUE;

    if (!enabled || !is_bidirectional)
        return ESP_ERR_INVALID_STATE;

    if (waitForFlag(&telemetry_received, telemetry_timeout_us) == ESP_OK)
    {
        erpm = getErpm();
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

void DShotRMT::send(uint16_t value)
{
    if (!enabled)
        return;

    throttle.throttle = value;
    telemetry_received = false;

    modeTx();

    ESP_ERROR_CHECK(rmt_transmit(rmt_tx_channel, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
}

void DShotRMT::sendTicks(uint16_t value, TickType_t ticks)
{
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(rmt_tx_channel, ticks));

    auto xLastWakeTime = xTaskGetTickCount();
    TickType_t repeatStop = xLastWakeTime + ticks;
    while (xTaskGetTickCount() < repeatStop)
    {
        send(value);
        xTaskDelayUntil(&xLastWakeTime, 1);
    }
}

void IRAM_ATTR DShotRMT::modeTx()
{
    if (mode != 1)
    {
        disableRx();
        mode = 1;
    }
}

void IRAM_ATTR DShotRMT::modeRx()
{
    if (mode != 0)
    {
        enableRx();
        mode = 0;

        ESP_ERROR_CHECK(rmt_receive(rmt_rx_channel, rx_buf, sizeof(rx_buf), &rx_config));
    }
}

void IRAM_ATTR DShotRMT::enableRx()
{
    // NOTE: time critical function, execution must not exceed 5-6us

    // Change pin to open drain to allow ESC to drive the wire
    gpio_ll_od_enable(&GPIO, gpio_num);

    ESP_ERROR_CHECK(rmt_enable(rmt_rx_channel));
}

void IRAM_ATTR DShotRMT::disableRx()
{
    ESP_ERROR_CHECK(rmt_disable(rmt_rx_channel));

    // Disable pin open drain to create a clean signal on the wire
    gpio_ll_od_disable(&GPIO, gpio_num);
}

bool IRAM_ATTR DShotRMT::rxDoneCallback_ISR(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    DShotRMT *instance = (DShotRMT *)user_data;

    // parse the received RMT symbols
    instance->telemetry_gcr = extractTelemetryGcr(edata->received_symbols, edata->num_symbols, instance->telemetry_bit_len_ticks);
    // raise flag that telemetry has been received
    instance->telemetry_received = true;

    return high_task_wakeup == pdTRUE;
}

bool IRAM_ATTR DShotRMT::txDoneCallback_ISR(rmt_channel_handle_t channel, const rmt_tx_done_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_wakeup = pdFALSE;
    DShotRMT *instance = (DShotRMT *)user_data;

    instance->modeRx();

    return high_task_wakeup == pdTRUE;
}

static uint32_t IRAM_ATTR durationToBitLen(uint32_t duration, uint32_t len)
{
    return (duration + (len >> 1)) / len;
}

static uint32_t IRAM_ATTR pushBits(uint32_t value, uint32_t bitVal, size_t bitLen)
{
    while (bitLen--)
    {
        value <<= 1;
        value |= bitVal;
    }
    return value;
}

/**
 * @param rmt_symbols Pointer to the RMT symbols
 * @param len Number of symbols
 * @return uint32_t raw gcr value
 */
uint32_t IRAM_ATTR DShotRMT::extractTelemetryGcr(rmt_symbol_word_t *rmt_symbols, size_t symbol_num, uint32_t bit_len_ticks)
{
    rmt_symbol_word_t *cur = rmt_symbols;

    // First bit should be 0 starting bit
    if (cur->level0 != 0)
    {
        return 0;
    }

    int bitCount = 0;
    uint32_t value = 0;
    for (size_t i = 0; i < symbol_num; i++)
    {
        // printf("{%d:%d},{%d:%d}\r\n", cur->level0, cur->duration0, cur->level1, cur->duration1);

        if (!cur->duration0)
            break;
        uint32_t bitLen0 = durationToBitLen(cur->duration0, bit_len_ticks);
        if (bitLen0)
        {
            value = pushBits(value, cur->level0, bitLen0);
            bitCount += bitLen0;
        }

        if (!cur->duration1)
            break;
        uint32_t bitLen1 = durationToBitLen(cur->duration1, bit_len_ticks);
        if (bitLen1)
        {
            value = pushBits(value, cur->level1, bitLen1);
            bitCount += bitLen1;
        }
        cur++;
    }

    // fill missing bits with 1
    if (bitCount < 21)
    {
        value = pushBits(value, 0x1, 21 - bitCount);
    }

    // First bit is start bit so discard it.
    value &= 0xfffff;

    value = value ^ (value >> 1); // extract gcr

    return value;
}

/**
 * Converts the GCR value into eRPM Data, and validates it's CRC
 * @param value 20-bit GCR value
 * @return uint32_t 12-bit eRPM Data
 */
uint32_t DShotRMT::convertGcrToErpmData(uint32_t value)
{
    if (!value)
    {
        return INVALID_TELEMETRY_VALUE;
    }

    // ...shifting 5 bits -> 4 bits (0xff => invalid)
    static const unsigned char GCR_decode[32] =
        {
            0xFF, 0xFF, 0xFF, 0xFF, // 0 - 3
            0xFF, 0xFF, 0xFF, 0xFF, // 4 - 7
            0xFF, 9, 10, 11,        // 8 - 11
            0xFF, 13, 14, 15,       // 12 - 15
            0xFF, 0xFF, 2, 3,       // 16 - 19
            0xFF, 5, 6, 7,          // 20 - 23
            0xFF, 0, 8, 1,          // 24 - 27
            0xFF, 4, 12, 0xFF,      // 28 - 31
        };

    uint32_t decodedValue = GCR_decode[value & 0x1f];
    decodedValue |= GCR_decode[(value >> 5) & 0x1f] << 4;
    decodedValue |= GCR_decode[(value >> 10) & 0x1f] << 8;
    decodedValue |= GCR_decode[(value >> 15) & 0x1f] << 12;

    uint32_t csum = decodedValue;
    csum = csum ^ (csum >> 8); // xor bytes
    csum = csum ^ (csum >> 4); // xor nibbles

    if ((csum & 0xf) != 0xf || decodedValue > 0xffff)
    {
        return INVALID_TELEMETRY_VALUE;
    }
    value = decodedValue >> 4;

    return value;
}

/**
 * Converts eRPM Data into an eRPM period
 * @param value 12-bit eRPM Data
 * @return uint32_t
 */
uint32_t DShotRMT::convertErpmDataToErpmPeriod(uint32_t value)
{
    if (!value || value == INVALID_TELEMETRY_VALUE)
    {
        return INVALID_TELEMETRY_VALUE;
    }

    // eRPM range
    if (value == 0x0fff)
    {
        return 0;
    }

    // Convert value to 16 bit from the GCR telemetry format (eeem mmmm mmmm)
    value = (value & 0x01ff) << ((value & 0xfe00) >> 9);

    if (!value || value == INVALID_TELEMETRY_VALUE)
    {
        return INVALID_TELEMETRY_VALUE;
    }

    // Convert period to erpm * 100
    return (1000000 * 60 / 100 + value / 2) / value;
}
