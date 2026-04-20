#include "signal_scope.h"

#include "ad7616.h"
#include "arm_math.h"
#include "lcd.h"
#include "main.h"
#include <stdbool.h>
#include <string.h>

#define APP_SAMPLE_RATE_HZ        10000U
#define APP_SAMPLE_PERIOD_US      (1000000U / APP_SAMPLE_RATE_HZ)
#define APP_FFT_SIZE              256U
#define APP_CAPTURE_BUFFER_COUNT  2U
#define APP_INVALID_FRAME         0xFFU

#define APP_RANGE_HIGH_THRESHOLD  29491U
#define APP_RANGE_LOW_THRESHOLD   12288U
#define APP_OVERLOAD_THRESHOLD    32000U
#define APP_KEY_DEBOUNCE_MS       180U
#define APP_LED_TOGGLE_MS         250U

#define APP_TEXT_HEIGHT           36U
#define APP_WAVE_X                4U
#define APP_WAVE_Y                42U
#define APP_WAVE_WIDTH            120U
#define APP_WAVE_HEIGHT           48U
#define APP_SPEC_X                4U
#define APP_SPEC_Y                100U
#define APP_SPEC_WIDTH            120U
#define APP_SPEC_HEIGHT           52U
#define APP_RENDER_COLUMNS_PER_STEP 4U

#define APP_LED_RED_ON()  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define APP_LED_RED_OFF() HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)

typedef enum
{
    APP_ACQ_IDLE = 0,
    APP_ACQ_WAIT_DISCARD,
    APP_ACQ_WAIT_SAMPLE
} AppAcqState;

typedef enum
{
    APP_VIEW_IDLE = 0,
    APP_VIEW_TEXT,
    APP_VIEW_WAVE_CLEAR,
    APP_VIEW_WAVE_DRAW,
    APP_VIEW_SPEC_CLEAR,
    APP_VIEW_SPEC_DRAW
} AppViewPhase;

typedef struct
{
    int16_t sample_a[APP_FFT_SIZE];
    int16_t sample_b[APP_FFT_SIZE];
    uint16_t peak_code;
    int16_t latest_a;
    int16_t latest_b;
    uint8_t pair_index;
    uint8_t range_code;
    uint8_t overload;
} AppCaptureFrame;

typedef struct
{
    uint8_t pair_index;
    uint8_t range_code;
    uint8_t overload;
    int32_t latest_a_mv;
    int32_t latest_b_mv;
    uint32_t dominant_freq_hz;
    uint16_t peak_code;
    int16_t wave_codes[APP_WAVE_WIDTH];
    uint8_t spectrum_heights[APP_SPEC_WIDTH];
} AppDisplaySnapshot;

static AppCaptureFrame g_capture_frames[APP_CAPTURE_BUFFER_COUNT];
static AppDisplaySnapshot g_display_front;
static AppDisplaySnapshot g_display_back;

static arm_rfft_fast_instance_f32 g_rfft;
static float32_t g_fft_input[APP_FFT_SIZE];
static float32_t g_fft_output[APP_FFT_SIZE];
static float32_t g_fft_magnitude[APP_FFT_SIZE / 2U];

static volatile uint8_t g_key_irq_pending = 0U;

static uint8_t g_fill_buffer = 0U;
static uint8_t g_ready_buffer = APP_INVALID_FRAME;
static uint8_t g_display_back_ready = 0U;
static uint8_t g_fill_index = 0U;
static uint8_t g_active_pair = 0U;
static uint8_t g_applied_pair = 0U;
static uint8_t g_requested_pair = 0U;
static uint8_t g_requested_range = Range_5_V;
static uint8_t g_applied_range = Range_5_V;
static uint8_t g_pair_change_pending = 1U;
static uint8_t g_range_change_pending = 0U;
static uint8_t g_discard_next_sample = 1U;
static uint8_t g_fft_ready = 0U;

static AppAcqState g_acq_state = APP_ACQ_IDLE;
static AppViewPhase g_view_phase = APP_VIEW_IDLE;

static uint16_t g_fill_peak = 0U;
static uint32_t g_cycles_per_us = 168U;
static uint32_t g_last_trigger_us = 0U;
static uint32_t g_last_key_event_ms = 0U;
static uint32_t g_last_led_toggle_ms = 0U;
static uint16_t g_render_column = 0U;
static uint16_t g_prev_wave_y = 0U;

static uint16_t App_AbsCode(int16_t sample)
{
    if (sample >= 0)
    {
        return (uint16_t)sample;
    }

    if (sample == INT16_MIN)
    {
        return 32768U;
    }

    return (uint16_t)(-sample);
}

static uint32_t App_NowUs(void)
{
    return DWT->CYCCNT / g_cycles_per_us;
}

static uint16_t App_RangeMilliVolt(uint8_t range_code)
{
    return (range_code == Range_10_V) ? 10000U : 5000U;
}

static int32_t App_CodeToMilliVolt(int16_t sample, uint8_t range_code)
{
    int32_t scaled = (int32_t)sample * (int32_t)App_RangeMilliVolt(range_code);

    if (scaled >= 0)
    {
        scaled += 16384L;
    }
    else
    {
        scaled -= 16384L;
    }

    return scaled / 32768L;
}

static const char *App_RangeText(uint8_t range_code)
{
    return (range_code == Range_10_V) ? "10V" : "5V";
}

static char *App_AppendString(char *dst, const char *src)
{
    while (*src != '\0')
    {
        *dst++ = *src++;
    }

    return dst;
}

static char *App_AppendUnsigned(char *dst, uint32_t value)
{
    char digits[10];
    uint32_t count = 0U;

    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count > 0U)
    {
        *dst++ = digits[--count];
    }

    return dst;
}

static void App_FormatMilliVolt(int32_t milli_volt, char *buffer)
{
    uint32_t abs_mv;
    uint32_t volts;
    uint32_t frac_mv;

    if (milli_volt < 0)
    {
        *buffer++ = '-';
        abs_mv = (uint32_t)(-milli_volt);
    }
    else
    {
        *buffer++ = '+';
        abs_mv = (uint32_t)milli_volt;
    }

    volts = abs_mv / 1000U;
    frac_mv = abs_mv % 1000U;

    if (volts >= 10U)
    {
        *buffer++ = (char)('0' + (volts / 10U));
        *buffer++ = (char)('0' + (volts % 10U));
    }
    else
    {
        *buffer++ = (char)('0' + volts);
    }

    *buffer++ = '.';
    *buffer++ = (char)('0' + (frac_mv / 100U));
    *buffer++ = (char)('0' + ((frac_mv / 10U) % 10U));
    *buffer++ = (char)('0' + (frac_mv % 10U));
    *buffer = '\0';
}

static void App_ResetFillFrame(void)
{
    AppCaptureFrame *frame = &g_capture_frames[g_fill_buffer];

    frame->pair_index = g_active_pair;
    frame->range_code = g_applied_range;
    frame->peak_code = 0U;
    frame->latest_a = 0;
    frame->latest_b = 0;
    frame->overload = 0U;
    g_fill_index = 0U;
    g_fill_peak = 0U;
}

static void App_RequestPairSwitch(uint8_t pair_index)
{
    g_requested_pair = (uint8_t)(pair_index & 0x01U);
    g_pair_change_pending = 1U;
}

static void App_RequestRangeSwitch(uint8_t range_code)
{
    g_requested_range = range_code;
    g_range_change_pending = 1U;
}

static void App_ApplyPendingConfig(void)
{
    if (g_pair_change_pending != 0U)
    {
        g_active_pair = g_requested_pair;
        g_applied_pair = g_active_pair;
        g_pair_change_pending = 0U;
        g_discard_next_sample = 1U;
        App_ResetFillFrame();
    }

    if (g_range_change_pending != 0U)
    {
        AD7616_Parallel_Set_voltage(g_requested_range);
        g_applied_range = g_requested_range;
        g_range_change_pending = 0U;
        g_discard_next_sample = 1U;
        App_ResetFillFrame();
    }
}

static void App_FinalizeFrame(void)
{
    AppCaptureFrame *frame = &g_capture_frames[g_fill_buffer];

    frame->pair_index = g_applied_pair;
    frame->range_code = g_applied_range;
    frame->peak_code = g_fill_peak;
    frame->overload = (uint8_t)((g_applied_range == Range_10_V) && (g_fill_peak >= APP_OVERLOAD_THRESHOLD));

    if (g_ready_buffer == APP_INVALID_FRAME)
    {
        g_ready_buffer = g_fill_buffer;
        g_fill_buffer ^= 0x01U;
        App_ResetFillFrame();
        g_fft_ready = 1U;
    }
    else
    {
        App_ResetFillFrame();
    }
}

static void App_StoreSample(int16_t sample_a, int16_t sample_b)
{
    AppCaptureFrame *frame = &g_capture_frames[g_fill_buffer];
    uint16_t abs_a = App_AbsCode(sample_a);
    uint16_t abs_b = App_AbsCode(sample_b);

    if (g_fill_index >= APP_FFT_SIZE)
    {
        App_ResetFillFrame();
    }

    frame->sample_a[g_fill_index] = sample_a;
    frame->sample_b[g_fill_index] = sample_b;
    frame->latest_a = sample_a;
    frame->latest_b = sample_b;

    if (abs_a > g_fill_peak)
    {
        g_fill_peak = abs_a;
    }

    if (abs_b > g_fill_peak)
    {
        g_fill_peak = abs_b;
    }

    ++g_fill_index;

    if (g_fill_index >= APP_FFT_SIZE)
    {
        App_FinalizeFrame();
    }
}

static void App_ServiceKey(void)
{
    uint32_t now_ms;

    if (g_key_irq_pending == 0U)
    {
        return;
    }

    now_ms = HAL_GetTick();
    if ((now_ms - g_last_key_event_ms) < APP_KEY_DEBOUNCE_MS)
    {
        return;
    }

    g_key_irq_pending = 0U;
    g_last_key_event_ms = now_ms;
    App_RequestPairSwitch((uint8_t)(g_active_pair ^ 0x01U));
}

static void App_ServiceAcquisition(void)
{
    int16_t sample_a = 0;
    int16_t sample_b = 0;
    HAL_StatusTypeDef status;
    uint32_t now_us = App_NowUs();

    if (g_acq_state == APP_ACQ_IDLE)
    {
        App_ApplyPendingConfig();

        if ((now_us - g_last_trigger_us) < APP_SAMPLE_PERIOD_US)
        {
            return;
        }

        AD7616_Parallel_Channel_Select(g_applied_pair);
        AD7616_Conversion();
        g_last_trigger_us = now_us;
        g_acq_state = (g_discard_next_sample != 0U) ? APP_ACQ_WAIT_DISCARD : APP_ACQ_WAIT_SAMPLE;
        return;
    }

    status = AD7616_Read_Data_NonBlocking(&sample_a, &sample_b);
    if (status == HAL_BUSY)
    {
        return;
    }

    if (status != HAL_OK)
    {
        Error_Handler();
    }

    if (g_acq_state == APP_ACQ_WAIT_DISCARD)
    {
        g_discard_next_sample = 0U;
        g_acq_state = APP_ACQ_IDLE;
        return;
    }

    App_StoreSample(sample_a, sample_b);
    g_acq_state = APP_ACQ_IDLE;
}

static uint32_t App_FindDominantFrequency(void)
{
    uint32_t peak_bin = 0U;
    float32_t peak_value = 0.0f;

    g_fft_magnitude[0] = 0.0f;
    for (uint32_t index = 1U; index < (APP_FFT_SIZE / 2U); ++index)
    {
        if (g_fft_magnitude[index] > peak_value)
        {
            peak_value = g_fft_magnitude[index];
            peak_bin = index;
        }
    }

    if (peak_value < 100.0f)
    {
        return 0U;
    }

    return (peak_bin * APP_SAMPLE_RATE_HZ + (APP_FFT_SIZE / 2U)) / APP_FFT_SIZE;
}

static void App_BuildWaveColumns(AppDisplaySnapshot *snapshot, const AppCaptureFrame *frame)
{
    for (uint32_t column = 0U; column < APP_WAVE_WIDTH; ++column)
    {
        uint32_t sample_index = (column * APP_FFT_SIZE) / APP_WAVE_WIDTH;
        snapshot->wave_codes[column] = frame->sample_b[sample_index];
    }
}

static void App_BuildSpectrumColumns(AppDisplaySnapshot *snapshot)
{
    float32_t spectrum_max = 0.0f;
    uint32_t total_bins = (APP_FFT_SIZE / 2U) - 1U;

    for (uint32_t bin = 1U; bin < (APP_FFT_SIZE / 2U); ++bin)
    {
        if (g_fft_magnitude[bin] > spectrum_max)
        {
            spectrum_max = g_fft_magnitude[bin];
        }
    }

    if (spectrum_max <= 0.0f)
    {
        memset(snapshot->spectrum_heights, 0, sizeof(snapshot->spectrum_heights));
        return;
    }

    for (uint32_t column = 0U; column < APP_SPEC_WIDTH; ++column)
    {
        uint32_t start_bin = 1U + ((column * total_bins) / APP_SPEC_WIDTH);
        uint32_t end_bin = 1U + (((column + 1U) * total_bins) / APP_SPEC_WIDTH);
        float32_t column_peak = 0.0f;

        if (end_bin <= start_bin)
        {
            end_bin = start_bin + 1U;
        }

        if (end_bin > (APP_FFT_SIZE / 2U))
        {
            end_bin = APP_FFT_SIZE / 2U;
        }

        for (uint32_t bin = start_bin; bin < end_bin; ++bin)
        {
            if (g_fft_magnitude[bin] > column_peak)
            {
                column_peak = g_fft_magnitude[bin];
            }
        }

        snapshot->spectrum_heights[column] = (uint8_t)(((float32_t)(APP_SPEC_HEIGHT - 2U) * column_peak) / spectrum_max);
    }
}

static void App_BuildDisplaySnapshot(const AppCaptureFrame *frame)
{
    AppDisplaySnapshot snapshot;
    float32_t mean_value = 0.0f;

    memset(&snapshot, 0, sizeof(snapshot));

    for (uint32_t index = 0U; index < APP_FFT_SIZE; ++index)
    {
        g_fft_input[index] = (float32_t)frame->sample_b[index];
    }

    arm_mean_f32(g_fft_input, APP_FFT_SIZE, &mean_value);
    for (uint32_t index = 0U; index < APP_FFT_SIZE; ++index)
    {
        g_fft_input[index] -= mean_value;
    }

    arm_rfft_fast_f32(&g_rfft, g_fft_input, g_fft_output, 0U);
    arm_cmplx_mag_f32(g_fft_output, g_fft_magnitude, APP_FFT_SIZE / 2U);

    snapshot.pair_index = frame->pair_index;
    snapshot.range_code = frame->range_code;
    snapshot.overload = frame->overload;
    snapshot.latest_a_mv = App_CodeToMilliVolt(frame->latest_a, frame->range_code);
    snapshot.latest_b_mv = App_CodeToMilliVolt(frame->latest_b, frame->range_code);
    snapshot.peak_code = frame->peak_code;
    snapshot.dominant_freq_hz = App_FindDominantFrequency();

    App_BuildWaveColumns(&snapshot, frame);
    App_BuildSpectrumColumns(&snapshot);

    memcpy(&g_display_back, &snapshot, sizeof(g_display_back));
    g_display_back_ready = 1U;
}

static void App_ServiceFft(void)
{
    AppCaptureFrame *frame;

    if ((g_fft_ready == 0U) || (g_ready_buffer == APP_INVALID_FRAME))
    {
        return;
    }

    frame = &g_capture_frames[g_ready_buffer];
    App_BuildDisplaySnapshot(frame);

    if ((frame->peak_code > APP_RANGE_HIGH_THRESHOLD) && (frame->range_code == Range_5_V))
    {
        App_RequestRangeSwitch(Range_10_V);
    }
    else if ((frame->peak_code < APP_RANGE_LOW_THRESHOLD) && (frame->range_code == Range_10_V))
    {
        App_RequestRangeSwitch(Range_5_V);
    }

    g_ready_buffer = APP_INVALID_FRAME;
    g_fft_ready = 0U;
}

static uint16_t App_MapWaveY(int16_t code)
{
    int32_t scaled = ((int32_t)code + 32768L) * (int32_t)(APP_WAVE_HEIGHT - 1U);
    int32_t offset = scaled / 65535L;

    return (uint16_t)(APP_WAVE_Y + (APP_WAVE_HEIGHT - 1U) - (uint16_t)offset);
}

static void App_DrawTextLine(uint16_t y, const char *text, uint16_t color)
{
    LCD_Fill(0, y, LCD_W, (uint16_t)(y + 12U), BLACK);
    LCD_ShowString(0, y, (const u8 *)text, color, BLACK, 12, 0);
}

static void App_DrawTextPanel(void)
{
    char line[32];
    char a_text[10];
    char b_text[10];
    char *cursor;

    cursor = line;
    cursor = App_AppendString(cursor, "PAIR A");
    *cursor++ = (char)('0' + g_display_front.pair_index);
    cursor = App_AppendString(cursor, "/B");
    *cursor++ = (char)('0' + g_display_front.pair_index);
    cursor = App_AppendString(cursor, " RNG ");
    cursor = App_AppendString(cursor, App_RangeText(g_display_front.range_code));
    *cursor = '\0';
    App_DrawTextLine(0U, line, CYAN);

    App_FormatMilliVolt(g_display_front.latest_a_mv, a_text);
    App_FormatMilliVolt(g_display_front.latest_b_mv, b_text);
    cursor = line;
    cursor = App_AppendString(cursor, "A ");
    cursor = App_AppendString(cursor, a_text);
    cursor = App_AppendString(cursor, "  B ");
    cursor = App_AppendString(cursor, b_text);
    *cursor = '\0';
    App_DrawTextLine(12U, line, WHITE);

    cursor = line;
    cursor = App_AppendString(cursor, "FFT ");
    cursor = App_AppendUnsigned(cursor, g_display_front.dominant_freq_hz);
    cursor = App_AppendString(cursor, "Hz ");
    cursor = App_AppendString(cursor, (g_display_front.overload != 0U) ? "OVR" : "TRACK");
    *cursor = '\0';
    App_DrawTextLine(24U, line, YELLOW);
}

static void App_DrawWaveDecorations(void)
{
    uint16_t mid_y = (uint16_t)(APP_WAVE_Y + (APP_WAVE_HEIGHT / 2U));

    LCD_DrawRectangle((uint16_t)(APP_WAVE_X - 1U), (uint16_t)(APP_WAVE_Y - 1U),
                      (uint16_t)(APP_WAVE_X + APP_WAVE_WIDTH), (uint16_t)(APP_WAVE_Y + APP_WAVE_HEIGHT),
                      GRAYBLUE);
    LCD_DrawLine(APP_WAVE_X, mid_y, (uint16_t)(APP_WAVE_X + APP_WAVE_WIDTH - 1U), mid_y, DARKBLUE);
    LCD_ShowString(APP_WAVE_X, (uint16_t)(APP_WAVE_Y - 12U), (const u8 *)"TIME", CYAN, BLACK, 12, 0);
}

static void App_DrawSpectrumDecorations(void)
{
    uint16_t base_y = (uint16_t)(APP_SPEC_Y + APP_SPEC_HEIGHT - 1U);

    LCD_DrawRectangle((uint16_t)(APP_SPEC_X - 1U), (uint16_t)(APP_SPEC_Y - 1U),
                      (uint16_t)(APP_SPEC_X + APP_SPEC_WIDTH), (uint16_t)(APP_SPEC_Y + APP_SPEC_HEIGHT),
                      GRAYBLUE);
    LCD_DrawLine(APP_SPEC_X, base_y, (uint16_t)(APP_SPEC_X + APP_SPEC_WIDTH - 1U), base_y, DARKBLUE);
    LCD_ShowString(APP_SPEC_X, (uint16_t)(APP_SPEC_Y - 12U), (const u8 *)"FFT", CYAN, BLACK, 12, 0);
}

static void App_ServiceView(void)
{
    uint16_t step;

    if ((g_view_phase == APP_VIEW_IDLE) && (g_display_back_ready != 0U))
    {
        memcpy(&g_display_front, &g_display_back, sizeof(g_display_front));
        g_display_back_ready = 0U;
        g_view_phase = APP_VIEW_TEXT;
    }

    switch (g_view_phase)
    {
        case APP_VIEW_IDLE:
            break;

        case APP_VIEW_TEXT:
            LCD_Fill(0, 0, LCD_W, APP_TEXT_HEIGHT, BLACK);
            App_DrawTextPanel();
            g_render_column = 0U;
            g_view_phase = APP_VIEW_WAVE_CLEAR;
            break;

        case APP_VIEW_WAVE_CLEAR:
            for (step = 0U; (step < APP_RENDER_COLUMNS_PER_STEP) && (g_render_column < APP_WAVE_WIDTH); ++step, ++g_render_column)
            {
                LCD_Fill((uint16_t)(APP_WAVE_X + g_render_column), APP_WAVE_Y,
                         (uint16_t)(APP_WAVE_X + g_render_column + 1U), (uint16_t)(APP_WAVE_Y + APP_WAVE_HEIGHT),
                         BLACK);
            }

            if (g_render_column >= APP_WAVE_WIDTH)
            {
                App_DrawWaveDecorations();
                g_render_column = 0U;
                g_prev_wave_y = App_MapWaveY(g_display_front.wave_codes[0]);
                g_view_phase = APP_VIEW_WAVE_DRAW;
            }
            break;

        case APP_VIEW_WAVE_DRAW:
            for (step = 0U; (step < APP_RENDER_COLUMNS_PER_STEP) && (g_render_column < APP_WAVE_WIDTH); ++step, ++g_render_column)
            {
                uint16_t x = (uint16_t)(APP_WAVE_X + g_render_column);
                uint16_t y = App_MapWaveY(g_display_front.wave_codes[g_render_column]);

                if (g_render_column == 0U)
                {
                    LCD_DrawPoint(x, y, YELLOW);
                }
                else
                {
                    LCD_DrawLine((uint16_t)(x - 1U), g_prev_wave_y, x, y, YELLOW);
                }

                g_prev_wave_y = y;
            }

            if (g_render_column >= APP_WAVE_WIDTH)
            {
                g_render_column = 0U;
                g_view_phase = APP_VIEW_SPEC_CLEAR;
            }
            break;

        case APP_VIEW_SPEC_CLEAR:
            for (step = 0U; (step < APP_RENDER_COLUMNS_PER_STEP) && (g_render_column < APP_SPEC_WIDTH); ++step, ++g_render_column)
            {
                LCD_Fill((uint16_t)(APP_SPEC_X + g_render_column), APP_SPEC_Y,
                         (uint16_t)(APP_SPEC_X + g_render_column + 1U), (uint16_t)(APP_SPEC_Y + APP_SPEC_HEIGHT),
                         BLACK);
            }

            if (g_render_column >= APP_SPEC_WIDTH)
            {
                App_DrawSpectrumDecorations();
                g_render_column = 0U;
                g_view_phase = APP_VIEW_SPEC_DRAW;
            }
            break;

        case APP_VIEW_SPEC_DRAW:
            for (step = 0U; (step < APP_RENDER_COLUMNS_PER_STEP) && (g_render_column < APP_SPEC_WIDTH); ++step, ++g_render_column)
            {
                uint16_t x = (uint16_t)(APP_SPEC_X + g_render_column);
                uint16_t bar_height = g_display_front.spectrum_heights[g_render_column];
                uint16_t base_y = (uint16_t)(APP_SPEC_Y + APP_SPEC_HEIGHT - 1U);

                if (bar_height != 0U)
                {
                    LCD_DrawLine(x, base_y, x, (uint16_t)(base_y - bar_height), GREEN);
                }
            }

            if (g_render_column >= APP_SPEC_WIDTH)
            {
                g_view_phase = APP_VIEW_IDLE;
            }
            break;

        default:
            g_view_phase = APP_VIEW_IDLE;
            break;
    }
}

static void App_ServiceLed(void)
{
    uint32_t now_ms = HAL_GetTick();

    if (g_display_front.overload != 0U)
    {
        APP_LED_RED_ON();
    }
    else
    {
        APP_LED_RED_OFF();
    }

    if ((now_ms - g_last_led_toggle_ms) >= APP_LED_TOGGLE_MS)
    {
        HAL_GPIO_TogglePin(LED_BULE_GPIO_Port, LED_BULE_Pin);
        g_last_led_toggle_ms = now_ms;
    }
}

void SignalScope_Init(void)
{
    arm_status status;

    memset(g_capture_frames, 0, sizeof(g_capture_frames));
    memset(&g_display_front, 0, sizeof(g_display_front));
    memset(&g_display_back, 0, sizeof(g_display_back));

    g_cycles_per_us = HAL_RCC_GetSysClockFreq() / 1000000U;
    g_last_trigger_us = App_NowUs();

    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(0, 0, (const u8 *)"AD7616 FFT SCOPE", CYAN, BLACK, 12, 0);

    AD7616_Init(HARDWARE_MODE);
    AD7616_Reset();
    AD7616_Parallel_Channel_Select(0U);
    AD7616_Parallel_Set_voltage(Range_5_V);

    g_requested_pair = 0U;
    g_requested_range = Range_5_V;
    g_applied_pair = 0U;
    g_active_pair = 0U;
    g_applied_range = Range_5_V;
    App_ResetFillFrame();

    status = arm_rfft_fast_init_f32(&g_rfft, APP_FFT_SIZE);
    if (status != ARM_MATH_SUCCESS)
    {
        Error_Handler();
    }

    App_BuildDisplaySnapshot(&g_capture_frames[0]);
}

void SignalScope_Task(void)
{
    App_ServiceKey();
    App_ServiceAcquisition();
    App_ServiceFft();
    App_ServiceView();
    App_ServiceLed();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
        g_key_irq_pending = 1U;
    }
}