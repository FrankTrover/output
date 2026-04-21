#include "signal_scope.h"

#include "ad7616.h"
#include "arm_math.h"
#include "lcd.h"
#include "main.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define APP_SAMPLE_RATE_HZ           10000U
#define APP_TIMER_TICK_HZ            1000000U
#define APP_ADC_TIMEOUT_US           20U
#define APP_FFT_SIZE                 512U
#define APP_CAPTURE_BUFFER_COUNT     4U
#define APP_READY_QUEUE_LENGTH       (APP_CAPTURE_BUFFER_COUNT - 1U)
#define APP_INVALID_FRAME            0xFFU

#define APP_RANGE_HIGH_THRESHOLD     29491U
#define APP_RANGE_LOW_THRESHOLD      12288U
#define APP_OVERLOAD_THRESHOLD       32000U
#define APP_KEY_DEBOUNCE_MS          180U
#define APP_LED_TOGGLE_MS            250U

#define APP_TEXT_HEIGHT              36U
#define APP_WAVE_X                   4U
#define APP_WAVE_Y                   42U
#define APP_WAVE_WIDTH               120U
#define APP_WAVE_HEIGHT              48U
#define APP_SPEC_X                   4U
#define APP_SPEC_Y                   100U
#define APP_SPEC_WIDTH               120U
#define APP_SPEC_HEIGHT              52U
#define APP_RENDER_COLUMNS_PER_STEP  4U

#define APP_FRAME_EMPTY              0U
#define APP_FRAME_FILLING            1U
#define APP_FRAME_READY              2U

#define APP_LED_RED_ON()  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define APP_LED_RED_OFF() HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)

typedef enum
{
    APP_PAGE_MAIN = 0,
    APP_PAGE_DEBUG
} AppPage;

typedef enum
{
    APP_VIEW_IDLE = 0,
    APP_VIEW_TEXT,
    APP_VIEW_WAVE_CLEAR,
    APP_VIEW_WAVE_DRAW,
    APP_VIEW_SPEC_CLEAR,
    APP_VIEW_SPEC_DRAW,
    APP_VIEW_DEBUG
} AppViewPhase;

typedef struct
{
    int16_t sample_b[APP_FFT_SIZE];
    uint16_t peak_code;
    int16_t latest_a;
    int16_t latest_b;
    uint32_t sequence;
    uint8_t range_code;
    uint8_t overload;
} AppCaptureFrame;

typedef struct
{
    uint8_t range_code;
    uint8_t overload;
    int32_t latest_a_mv;
    int32_t latest_b_mv;
    uint32_t dominant_freq_hz;
    uint32_t dropped_frames;
    uint32_t adc_timeout_count;
    uint32_t sample_rate_hz;
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
static float32_t g_fft_window[APP_FFT_SIZE];

static volatile uint8_t g_key_irq_pending = 0U;
static volatile uint8_t g_ready_queue[APP_READY_QUEUE_LENGTH];
static volatile uint8_t g_ready_head = 0U;
static volatile uint8_t g_ready_tail = 0U;
static volatile uint8_t g_ready_count = 0U;
static volatile uint8_t g_fill_buffer = 0U;
static volatile uint16_t g_fill_index = 0U;
static volatile uint16_t g_fill_peak = 0U;
static volatile uint32_t g_frame_drop_count = 0U;
static volatile uint32_t g_adc_timeout_count = 0U;
static volatile uint32_t g_frame_sequence = 0U;
static volatile uint8_t g_frame_state[APP_CAPTURE_BUFFER_COUNT];
static volatile uint8_t g_requested_range = Range_5_V;
static volatile uint8_t g_applied_range = Range_5_V;
static volatile uint8_t g_range_change_pending = 0U;
static volatile uint8_t g_discard_next_sample = 1U;

static uint8_t g_display_back_ready = 0U;
static uint8_t g_force_full_redraw = 1U;
static AppPage g_current_page = APP_PAGE_MAIN;
static AppViewPhase g_view_phase = APP_VIEW_IDLE;
static uint32_t g_last_key_event_ms = 0U;
static uint32_t g_last_led_toggle_ms = 0U;
static uint32_t g_timer_sample_rate_hz = APP_SAMPLE_RATE_HZ;
static uint16_t g_render_column = 0U;
static uint16_t g_prev_wave_y = 0U;

static uint32_t App_Lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void App_Unlock(uint32_t primask)
{
    if (primask == 0U)
    {
        __enable_irq();
    }
}

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

static void App_InitWindow(void)
{
    for (uint32_t index = 0U; index < APP_FFT_SIZE; ++index)
    {
        g_fft_window[index] = 0.5f - 0.5f * cosf((2.0f * (float32_t)M_PI * (float32_t)index) / (float32_t)(APP_FFT_SIZE - 1U));
    }
}

static uint8_t App_FindEmptyBuffer(void)
{
    uint8_t index;

    for (index = 0U; index < APP_CAPTURE_BUFFER_COUNT; ++index)
    {
        if (g_frame_state[index] == APP_FRAME_EMPTY)
        {
            return index;
        }
    }

    return APP_INVALID_FRAME;
}

static void App_ResetFillFrame(uint8_t buffer_index)
{
    AppCaptureFrame *frame = &g_capture_frames[buffer_index];

    frame->range_code = g_applied_range;
    frame->peak_code = 0U;
    frame->latest_a = 0;
    frame->latest_b = 0;
    frame->overload = 0U;
    g_fill_index = 0U;
    g_fill_peak = 0U;
}

static void App_RequestRangeSwitch(uint8_t range_code)
{
    uint32_t primask = App_Lock();

    g_requested_range = range_code;
    g_range_change_pending = 1U;

    App_Unlock(primask);
}

static void App_ApplyPendingConfigISR(void)
{
    if (g_range_change_pending != 0U)
    {
        AD7616_Parallel_Set_voltage(g_requested_range);
        g_applied_range = g_requested_range;
        g_range_change_pending = 0U;
        g_discard_next_sample = 1U;
        App_ResetFillFrame(g_fill_buffer);
    }
}

static uint8_t App_EnqueueReadyBufferISR(uint8_t buffer_index)
{
    if (g_ready_count >= APP_READY_QUEUE_LENGTH)
    {
        return 0U;
    }

    g_ready_queue[g_ready_tail] = buffer_index;
    g_ready_tail = (uint8_t)((g_ready_tail + 1U) % APP_READY_QUEUE_LENGTH);
    ++g_ready_count;
    return 1U;
}

static uint8_t App_DequeueReadyBuffer(void)
{
    uint8_t buffer_index = APP_INVALID_FRAME;
    uint32_t primask = App_Lock();

    if (g_ready_count != 0U)
    {
        buffer_index = g_ready_queue[g_ready_head];
        g_ready_head = (uint8_t)((g_ready_head + 1U) % APP_READY_QUEUE_LENGTH);
        --g_ready_count;
    }

    App_Unlock(primask);
    return buffer_index;
}

static void App_ReleaseFrame(uint8_t buffer_index)
{
    uint32_t primask;

    if (buffer_index >= APP_CAPTURE_BUFFER_COUNT)
    {
        return;
    }

    primask = App_Lock();
    g_frame_state[buffer_index] = APP_FRAME_EMPTY;
    App_Unlock(primask);
}

static void App_FinalizeFrameISR(void)
{
    AppCaptureFrame *frame = &g_capture_frames[g_fill_buffer];
    uint8_t next_fill;

    frame->range_code = g_applied_range;
    frame->peak_code = g_fill_peak;
    frame->overload = (uint8_t)((g_applied_range == Range_10_V) && (g_fill_peak >= APP_OVERLOAD_THRESHOLD));
    frame->sequence = ++g_frame_sequence;

    next_fill = App_FindEmptyBuffer();
    if ((next_fill == APP_INVALID_FRAME) || (App_EnqueueReadyBufferISR(g_fill_buffer) == 0U))
    {
        ++g_frame_drop_count;
        App_ResetFillFrame(g_fill_buffer);
        return;
    }

    g_frame_state[g_fill_buffer] = APP_FRAME_READY;
    g_fill_buffer = next_fill;
    g_frame_state[g_fill_buffer] = APP_FRAME_FILLING;
    App_ResetFillFrame(g_fill_buffer);
}

static void App_StoreSampleISR(int16_t sample_a, int16_t sample_b)
{
    AppCaptureFrame *frame = &g_capture_frames[g_fill_buffer];
    uint16_t abs_b = App_AbsCode(sample_b);

    if (g_fill_index >= APP_FFT_SIZE)
    {
        App_ResetFillFrame(g_fill_buffer);
    }

    frame->sample_b[g_fill_index] = sample_b;
    frame->latest_a = sample_a;
    frame->latest_b = sample_b;

    if (abs_b > g_fill_peak)
    {
        g_fill_peak = abs_b;
    }

    ++g_fill_index;

    if (g_fill_index >= APP_FFT_SIZE)
    {
        App_FinalizeFrameISR();
    }
}

static uint32_t App_GetTimerClockHz(void)
{
    uint32_t timer_clock = HAL_RCC_GetPCLK1Freq();

    if ((RCC->CFGR & RCC_CFGR_PPRE1) != 0U)
    {
        timer_clock *= 2U;
    }

    return timer_clock;
}

static void App_InitSampleTimer(void)
{
    uint32_t timer_clock = App_GetTimerClockHz();
    uint32_t prescaler = (timer_clock / APP_TIMER_TICK_HZ) - 1U;
    uint32_t period = (APP_TIMER_TICK_HZ / APP_SAMPLE_RATE_HZ) - 1U;

    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->CR1 = 0U;
    TIM2->CR2 = 0U;
    TIM2->SMCR = 0U;
    TIM2->DIER = 0U;
    TIM2->SR = 0U;
    TIM2->PSC = prescaler;
    TIM2->ARR = period;
    TIM2->CNT = 0U;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_UIE;
    TIM2->CR1 = TIM_CR1_ARPE;

    g_timer_sample_rate_hz = timer_clock / ((prescaler + 1U) * (period + 1U));

    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

static void App_StartSampleTimer(void)
{
    TIM2->CNT = 0U;
    TIM2->SR = 0U;
    TIM2->CR1 |= TIM_CR1_CEN;
}

static void App_AcquireSampleTickISR(void)
{
    HAL_StatusTypeDef status;
    int16_t sample_a = 0;
    int16_t sample_b = 0;

    App_ApplyPendingConfigISR();

    AD7616_Parallel_Channel_Select(0U);
    AD7616_Conversion();
    status = AD7616_Read_Data_Timeout(&sample_a, &sample_b, APP_ADC_TIMEOUT_US);
    if (status != HAL_OK)
    {
        ++g_adc_timeout_count;
        g_discard_next_sample = 1U;
        return;
    }

    if (g_discard_next_sample != 0U)
    {
        g_discard_next_sample = 0U;
        return;
    }

    App_StoreSampleISR(sample_a, sample_b);
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
    g_current_page = (g_current_page == APP_PAGE_MAIN) ? APP_PAGE_DEBUG : APP_PAGE_MAIN;
    g_force_full_redraw = 1U;
    g_view_phase = (g_current_page == APP_PAGE_MAIN) ? APP_VIEW_TEXT : APP_VIEW_DEBUG;
}

static uint32_t App_FindDominantFrequency(void)
{
    uint32_t peak_bin = 0U;
    float32_t peak_value = 0.0f;
    float32_t estimated_bin;

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

    estimated_bin = (float32_t)peak_bin;
    if ((peak_bin > 1U) && (peak_bin < ((APP_FFT_SIZE / 2U) - 1U)))
    {
        float32_t alpha = g_fft_magnitude[peak_bin - 1U];
        float32_t beta = g_fft_magnitude[peak_bin];
        float32_t gamma = g_fft_magnitude[peak_bin + 1U];
        float32_t denominator = alpha - (2.0f * beta) + gamma;

        if (fabsf(denominator) > 1.0e-12f)
        {
            estimated_bin += 0.5f * (alpha - gamma) / denominator;
        }
    }

    if (estimated_bin < 0.0f)
    {
        estimated_bin = 0.0f;
    }

    return (uint32_t)((estimated_bin * (float32_t)g_timer_sample_rate_hz) / (float32_t)APP_FFT_SIZE + 0.5f);
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
    uint32_t primask;

    memset(&snapshot, 0, sizeof(snapshot));

    for (uint32_t index = 0U; index < APP_FFT_SIZE; ++index)
    {
        g_fft_input[index] = (float32_t)frame->sample_b[index];
    }

    arm_mean_f32(g_fft_input, APP_FFT_SIZE, &mean_value);
    for (uint32_t index = 0U; index < APP_FFT_SIZE; ++index)
    {
        g_fft_input[index] = (g_fft_input[index] - mean_value) * g_fft_window[index];
    }

    arm_rfft_fast_f32(&g_rfft, g_fft_input, g_fft_output, 0U);
    arm_cmplx_mag_f32(g_fft_output, g_fft_magnitude, APP_FFT_SIZE / 2U);

    snapshot.range_code = frame->range_code;
    snapshot.overload = frame->overload;
    snapshot.latest_a_mv = App_CodeToMilliVolt(frame->latest_a, frame->range_code);
    snapshot.latest_b_mv = App_CodeToMilliVolt(frame->latest_b, frame->range_code);
    snapshot.peak_code = frame->peak_code;
    snapshot.dominant_freq_hz = App_FindDominantFrequency();
    snapshot.sample_rate_hz = g_timer_sample_rate_hz;

    primask = App_Lock();
    snapshot.dropped_frames = g_frame_drop_count;
    snapshot.adc_timeout_count = g_adc_timeout_count;
    App_Unlock(primask);

    App_BuildWaveColumns(&snapshot, frame);
    App_BuildSpectrumColumns(&snapshot);

    memcpy(&g_display_back, &snapshot, sizeof(g_display_back));
    g_display_back_ready = 1U;
}

static void App_ServiceFft(void)
{
    uint8_t ready_buffer = App_DequeueReadyBuffer();
    AppCaptureFrame *frame;

    if (ready_buffer == APP_INVALID_FRAME)
    {
        return;
    }

    frame = &g_capture_frames[ready_buffer];
    App_BuildDisplaySnapshot(frame);

    if ((frame->peak_code > APP_RANGE_HIGH_THRESHOLD) && (frame->range_code == Range_5_V))
    {
        App_RequestRangeSwitch(Range_10_V);
    }
    else if ((frame->peak_code < APP_RANGE_LOW_THRESHOLD) && (frame->range_code == Range_10_V))
    {
        App_RequestRangeSwitch(Range_5_V);
    }

    App_ReleaseFrame(ready_buffer);
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

static void App_DrawMainTextPanel(void)
{
    char line[32];
    char a_text[10];
    char b_text[10];
    char *cursor;

    cursor = line;
    cursor = App_AppendString(cursor, "B0 FFT RNG ");
    cursor = App_AppendString(cursor, App_RangeText(g_display_front.range_code));
    *cursor = '\0';
    App_DrawTextLine(0U, line, CYAN);

    App_FormatMilliVolt(g_display_front.latest_a_mv, a_text);
    App_FormatMilliVolt(g_display_front.latest_b_mv, b_text);
    cursor = line;
    cursor = App_AppendString(cursor, "A0 ");
    cursor = App_AppendString(cursor, a_text);
    cursor = App_AppendString(cursor, " B0 ");
    cursor = App_AppendString(cursor, b_text);
    *cursor = '\0';
    App_DrawTextLine(12U, line, WHITE);

    cursor = line;
    cursor = App_AppendString(cursor, "FFT ");
    cursor = App_AppendUnsigned(cursor, g_display_front.dominant_freq_hz);
    cursor = App_AppendString(cursor, "Hz ");
    cursor = App_AppendString(cursor, (g_display_front.overload != 0U) ? "OVR D" : "OK  D");
    cursor = App_AppendUnsigned(cursor, g_display_front.dropped_frames);
    *cursor = '\0';
    App_DrawTextLine(24U, line, YELLOW);
}

static void App_DrawDebugPage(void)
{
    char line[32];
    char *cursor;

    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    App_DrawTextLine(0U, "DEBUG PAGE", CYAN);

    cursor = line;
    cursor = App_AppendString(cursor, "SR ");
    cursor = App_AppendUnsigned(cursor, g_display_front.sample_rate_hz);
    cursor = App_AppendString(cursor, "Hz");
    *cursor = '\0';
    App_DrawTextLine(16U, line, WHITE);

    cursor = line;
    cursor = App_AppendString(cursor, "FFT ");
    cursor = App_AppendUnsigned(cursor, g_display_front.dominant_freq_hz);
    cursor = App_AppendString(cursor, "Hz");
    *cursor = '\0';
    App_DrawTextLine(32U, line, WHITE);

    cursor = line;
    cursor = App_AppendString(cursor, "DROP ");
    cursor = App_AppendUnsigned(cursor, g_display_front.dropped_frames);
    *cursor = '\0';
    App_DrawTextLine(48U, line, YELLOW);

    cursor = line;
    cursor = App_AppendString(cursor, "TO ");
    cursor = App_AppendUnsigned(cursor, g_display_front.adc_timeout_count);
    *cursor = '\0';
    App_DrawTextLine(64U, line, YELLOW);

    cursor = line;
    cursor = App_AppendString(cursor, "PEAK ");
    cursor = App_AppendUnsigned(cursor, g_display_front.peak_code);
    *cursor = '\0';
    App_DrawTextLine(80U, line, GREEN);

    cursor = line;
    cursor = App_AppendString(cursor, "RNG ");
    cursor = App_AppendString(cursor, App_RangeText(g_display_front.range_code));
    *cursor = '\0';
    App_DrawTextLine(96U, line, GREEN);

    cursor = line;
    cursor = App_AppendString(cursor, "OVR ");
    cursor = App_AppendString(cursor, (g_display_front.overload != 0U) ? "YES" : "NO");
    *cursor = '\0';
    App_DrawTextLine(112U, line, MAGENTA);

    App_DrawTextLine(144U, "KEY: PAGE TOGGLE", GRAYBLUE);
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

static void App_ServiceMainView(void)
{
    uint16_t step;

    switch (g_view_phase)
    {
        case APP_VIEW_IDLE:
            break;

        case APP_VIEW_TEXT:
            LCD_Fill(0, 0, LCD_W, APP_TEXT_HEIGHT, BLACK);
            App_DrawMainTextPanel();
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

static void App_ServiceView(void)
{
    if (g_display_back_ready != 0U)
    {
        memcpy(&g_display_front, &g_display_back, sizeof(g_display_front));
        g_display_back_ready = 0U;
        g_view_phase = (g_current_page == APP_PAGE_MAIN) ? APP_VIEW_TEXT : APP_VIEW_DEBUG;
    }

    if (g_force_full_redraw != 0U)
    {
        g_force_full_redraw = 0U;
        if (g_current_page == APP_PAGE_DEBUG)
        {
            App_DrawDebugPage();
            g_view_phase = APP_VIEW_IDLE;
            return;
        }

        g_view_phase = APP_VIEW_TEXT;
    }

    if (g_current_page == APP_PAGE_DEBUG)
    {
        if (g_view_phase == APP_VIEW_DEBUG)
        {
            App_DrawDebugPage();
            g_view_phase = APP_VIEW_IDLE;
        }
        return;
    }

    App_ServiceMainView();
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
    memset((void *)g_frame_state, 0, sizeof(g_frame_state));

    LCD_Init();
    LCD_Fill(0, 0, LCD_W, LCD_H, BLACK);
    LCD_ShowString(0, 0, (const u8 *)"B0 FFT SCOPE", CYAN, BLACK, 12, 0);

    AD7616_Init(HARDWARE_MODE);
    AD7616_Reset();
    AD7616_Parallel_Channel_Select(0U);
    AD7616_Parallel_Set_voltage(Range_5_V);

    g_requested_range = Range_5_V;
    g_applied_range = Range_5_V;
    g_fill_buffer = 0U;
    g_frame_state[g_fill_buffer] = APP_FRAME_FILLING;
    App_ResetFillFrame(g_fill_buffer);

    status = arm_rfft_fast_init_f32(&g_rfft, APP_FFT_SIZE);
    if (status != ARM_MATH_SUCCESS)
    {
        Error_Handler();
    }

    App_InitWindow();
    App_InitSampleTimer();
    App_StartSampleTimer();
}

void SignalScope_Task(void)
{
    App_ServiceKey();
    App_ServiceFft();
    App_ServiceView();
    App_ServiceLed();
}

void SignalScope_Timer_IRQHandler(void)
{
    if ((TIM2->SR & TIM_SR_UIF) != 0U)
    {
        TIM2->SR &= ~TIM_SR_UIF;
        App_AcquireSampleTickISR();
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_Pin)
    {
        g_key_irq_pending = 1U;
    }
}