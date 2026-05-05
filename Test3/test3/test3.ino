#include <SPI.h>

#define SPI_FREQ_FAST           4000000UL
#define SPI_FREQ_SLOW           500000UL
#define HSPI_MOSI_PIN           26
#define HSPI_SCK_PIN            27
#define VSPI_MOSI_PIN           11
#define VSPI_SCK_PIN            13

#define MUX_EN             1
#define MUX_DIS            0
#define NUM_ELECTRODES     8
#define NUM_MEAS           NUM_ELECTRODES*NUM_ELECTRODES

#define AD5930_CLK_FREQ    50000000
#define TEST_FREQ          10000
#define NUM_PERIODS        12
#define ADC_AVG            8

// AD5270 commands
#define CMD_WR_RDAC        0x01
#define CMD_RD_RDAC        0x02
#define CMD_ST_RDAC        0x03
#define CMD_RST            0x04
#define CMD_RD_MEM         0x05
#define CMD_RD_ADDR        0x06
#define CMD_WR_CTRL        0x07
#define CMD_RD_CTRL        0x08
#define CMD_SHTDN          0x09

// AD5930 register addresses
#define CTRL_REG           0x00
#define NUM_INCR_REG       0x01
#define DFREQ_LOW_REG      0x02
#define DFREQ_HIGH_REG     0x03
#define TIME_INCR_REG      0x04
#define TIME_BURST_REG     0x08
#define SFREQ_LOW_REG      0x0C
#define SFREQ_HIGH_REG     0x0D

#define CHIP_SEL_AD5930    3
#define CHIP_SEL_DRIVE     0
#define CHIP_SEL_MEAS      1

#define CHIP_SEL_MUX_SRC   29
#define CHIP_SEL_MUX_SINK  25
#define CHIP_SEL_MUX_VP    33
#define CHIP_SEL_MUX_VN    31

#define AD5930_MSBOUT_PIN  6
#define AD5930_INT_PIN     5
#define AD5930_CTRL_PIN    4
#define AD5930_STANDBY_PIN 2

#define ADS_PWR            9
#define ADS_OE             10

int16_t sine_table[1024] = {
    0, 3, 6, 9, 12, 15, 18, 21, 25, 28, 31, 34, 37, 40, 43, 47,
    50, 53, 56, 59, 62, 65, 68, 72, 75, 78, 81, 84, 87, 90, 93, 96,
    99, 102, 106, 109, 112, 115, 118, 121, 124, 127, 130, 133, 136, 139, 142, 145,
    148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181, 184, 187, 190, 193,
    195, 198, 201, 204, 207, 210, 213, 216, 218, 221, 224, 227, 230, 233, 235, 238,
    241, 244, 246, 249, 252, 255, 257, 260, 263, 265, 268, 271, 273, 276, 279, 281,
    284, 287, 289, 292, 294, 297, 299, 302, 304, 307, 310, 312, 314, 317, 319, 322,
    324, 327, 329, 332, 334, 336, 339, 341, 343, 346, 348, 350, 353, 355, 357, 359,
    362, 364, 366, 368, 370, 372, 375, 377, 379, 381, 383, 385, 387, 389, 391, 393,
    395, 397, 399, 401, 403, 405, 407, 409, 411, 413, 414, 416, 418, 420, 422, 423,
    425, 427, 429, 430, 432, 434, 435, 437, 439, 440, 442, 443, 445, 447, 448, 450,
    451, 453, 454, 455, 457, 458, 460, 461, 462, 464, 465, 466, 468, 469, 470, 471,
    473, 474, 475, 476, 477, 478, 479, 481, 482, 483, 484, 485, 486, 487, 488, 489,
    489, 490, 491, 492, 493, 494, 495, 495, 496, 497, 498, 498, 499, 500, 500, 501,
    502, 502, 503, 503, 504, 504, 505, 505, 506, 506, 507, 507, 508, 508, 508, 509,
    509, 509, 510, 510, 510, 510, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511,
    512, 511, 511, 511, 511, 511, 511, 511, 511, 511, 511, 510, 510, 510, 510, 509,
    509, 509, 508, 508, 508, 507, 507, 506, 506, 505, 505, 504, 504, 503, 503, 502,
    502, 501, 500, 500, 499, 498, 498, 497, 496, 495, 495, 494, 493, 492, 491, 490,
    489, 489, 488, 487, 486, 485, 484, 483, 482, 481, 479, 478, 477, 476, 475, 474,
    473, 471, 470, 469, 468, 466, 465, 464, 462, 461, 460, 458, 457, 455, 454, 453,
    451, 450, 448, 447, 445, 443, 442, 440, 439, 437, 435, 434, 432, 430, 429, 427,
    425, 423, 422, 420, 418, 416, 414, 413, 411, 409, 407, 405, 403, 401, 399, 397,
    395, 393, 391, 389, 387, 385, 383, 381, 379, 377, 375, 372, 370, 368, 366, 364,
    362, 359, 357, 355, 353, 350, 348, 346, 343, 341, 339, 336, 334, 332, 329, 327,
    324, 322, 319, 317, 314, 312, 310, 307, 304, 302, 299, 297, 294, 292, 289, 287,
    284, 281, 279, 276, 273, 271, 268, 265, 263, 260, 257, 255, 252, 249, 246, 244,
    241, 238, 235, 233, 230, 227, 224, 221, 218, 216, 213, 210, 207, 204, 201, 198,
    195, 193, 190, 187, 184, 181, 178, 175, 172, 169, 166, 163, 160, 157, 154, 151,
    148, 145, 142, 139, 136, 133, 130, 127, 124, 121, 118, 115, 112, 109, 106, 102,
    99, 96, 93, 90, 87, 84, 81, 78, 75, 72, 68, 65, 62, 59, 56, 53,
    50, 47, 43, 40, 37, 34, 31, 28, 25, 21, 18, 15, 12, 9, 6, 3,
    0, -3, -6, -9, -12, -15, -18, -21, -25, -28, -31, -34, -37, -40, -43, -47,
    -50, -53, -56, -59, -62, -65, -68, -72, -75, -78, -81, -84, -87, -90, -93, -96,
    -99, -102, -106, -109, -112, -115, -118, -121, -124, -127, -130, -133, -136, -139, -142, -145,
    -148, -151, -154, -157, -160, -163, -166, -169, -172, -175, -178, -181, -184, -187, -190, -193,
    -195, -198, -201, -204, -207, -210, -213, -216, -218, -221, -224, -227, -230, -233, -235, -238,
    -241, -244, -246, -249, -252, -255, -257, -260, -263, -265, -268, -271, -273, -276, -279, -281,
    -284, -287, -289, -292, -294, -297, -299, -302, -304, -307, -310, -312, -314, -317, -319, -322,
    -324, -327, -329, -332, -334, -336, -339, -341, -343, -346, -348, -350, -353, -355, -357, -359,
    -362, -364, -366, -368, -370, -372, -375, -377, -379, -381, -383, -385, -387, -389, -391, -393,
    -395, -397, -399, -401, -403, -405, -407, -409, -411, -413, -414, -416, -418, -420, -422, -423,
    -425, -427, -429, -430, -432, -434, -435, -437, -439, -440, -442, -443, -445, -447, -448, -450,
    -451, -453, -454, -455, -457, -458, -460, -461, -462, -464, -465, -466, -468, -469, -470, -471,
    -473, -474, -475, -476, -477, -478, -479, -481, -482, -483, -484, -485, -486, -487, -488, -489,
    -489, -490, -491, -492, -493, -494, -495, -495, -496, -497, -498, -498, -499, -500, -500, -501,
    -502, -502, -503, -503, -504, -504, -505, -505, -506, -506, -507, -507, -508, -508, -508, -509,
    -509, -509, -510, -510, -510, -510, -511, -511, -511, -511, -511, -511, -511, -511, -511, -511,
    -512, -511, -511, -511, -511, -511, -511, -511, -511, -511, -511, -510, -510, -510, -510, -509,
    -509, -509, -508, -508, -508, -507, -507, -506, -506, -505, -505, -504, -504, -503, -503, -502,
    -502, -501, -500, -500, -499, -498, -498, -497, -496, -495, -495, -494, -493, -492, -491, -490,
    -489, -489, -488, -487, -486, -485, -484, -483, -482, -481, -479, -478, -477, -476, -475, -474,
    -473, -471, -470, -469, -468, -466, -465, -464, -462, -461, -460, -458, -457, -455, -454, -453,
    -451, -450, -448, -447, -445, -443, -442, -440, -439, -437, -435, -434, -432, -430, -429, -427,
    -425, -423, -422, -420, -418, -416, -414, -413, -411, -409, -407, -405, -403, -401, -399, -397,
    -395, -393, -391, -389, -387, -385, -383, -381, -379, -377, -375, -372, -370, -368, -366, -364,
    -362, -359, -357, -355, -353, -350, -348, -346, -343, -341, -339, -336, -334, -332, -329, -327,
    -324, -322, -319, -317, -314, -312, -310, -307, -304, -302, -299, -297, -294, -292, -289, -287,
    -284, -281, -279, -276, -273, -271, -268, -265, -263, -260, -257, -255, -252, -249, -246, -244,
    -241, -238, -235, -233, -230, -227, -224, -221, -218, -216, -213, -210, -207, -204, -201, -198,
    -195, -193, -190, -187, -184, -181, -178, -175, -172, -169, -166, -163, -160, -157, -154, -151,
    -148, -145, -142, -139, -136, -133, -130, -127, -124, -121, -118, -115, -112, -109, -106, -102,
    -99, -96, -93, -90, -87, -84, -81, -78, -75, -72, -68, -65, -62, -59, -56, -53,
    -50, -47, -43, -40, -37, -34, -31, -28, -25, -21, -18, -15, -12, -9, -6, -3
};

typedef enum { AD, OP, MONO } meas_t;

extern volatile uint32_t F_CPU_ACTUAL;
extern const uint8_t pin_to_channel[42];

const uint8_t elec_to_mux[32] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };

uint16_t current_gain, voltage_gain;
float sample_rate;
uint16_t samples_per_period;
uint16_t num_samples;
double ref_signal_mag;
double phase_offset;

double signal_rms[NUM_MEAS];
double signal_mag[NUM_MEAS];
double signal_phase[NUM_MEAS];

uint32_t frame_delay = 0;
uint32_t frame_delay_prev = 0;
uint8_t pin_num = 0;
uint16_t rheo_val = 1023;

// Static buffers to avoid VLA stack allocation in read_signal
#define MAX_SAMPLES_PER_PERIOD 500
#define MAX_SAMPLES (MAX_SAMPLES_PER_PERIOD * NUM_PERIODS)
static uint16_t _gpio_buf[MAX_SAMPLES][ADC_AVG];
static uint16_t _adc_buf[MAX_SAMPLES];
static uint8_t  _ref_buf[MAX_SAMPLES];
static uint16_t _adc_peaks[MAX_SAMPLES];
static uint16_t _adc_troughs[MAX_SAMPLES];
static uint16_t _phase_cycles[MAX_SAMPLES];

void spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val)
{
    uint32_t period = (freq >= 500000) ? 1 : (500000 / freq);
    uint8_t cpol = (mode == SPI_MODE2 || mode == SPI_MODE3);
    uint8_t cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    uint8_t sck = cpol ? HIGH : LOW;
    uint8_t i;
    uint32_t start_time;

    digitalWrite(clock_pin, sck);
    delayMicroseconds(period * 4);

    for (i = 0; i < bits; i++) {
        start_time = micros();
        if (bit_order == LSBFIRST)
            digitalWrite(data_pin, !!(val & (1 << i)));
        else
            digitalWrite(data_pin, !!(val & (1 << ((bits-1) - i))));

        sck = !sck;
        if (cpha) { digitalWrite(clock_pin, sck); while(micros() - start_time < period); }
        else       { while(micros() - start_time < period); digitalWrite(clock_pin, sck); }

        start_time = micros();
        sck = !sck;
        if (cpha) { digitalWrite(clock_pin, sck); while(micros() - start_time < period); }
        else       { while(micros() - start_time < period); digitalWrite(clock_pin, sck); }
    }
}

void AD5270_Write(const int chip_sel, uint8_t cmd, uint16_t data)
{
    uint16_t data_word = ((cmd & 0x0F) << 10) | (data & 0x03FF);
    digitalWrite(chip_sel, LOW);
    delayMicroseconds(500);
    spi_write(VSPI_MOSI_PIN, VSPI_SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    delayMicroseconds(500);
    digitalWrite(chip_sel, HIGH);
}

void AD5270_Lock(const int chip_sel, uint8_t lock)
{
    AD5270_Write(chip_sel, CMD_WR_CTRL, lock ? 0 : 0x002);
}

void AD5270_Shutdown(const int chip_sel, uint8_t shutdown)
{
    AD5270_Write(chip_sel, CMD_SHTDN, shutdown ? 1 : 0);
}

void AD5270_Set(const int chip_sel, uint16_t val)
{
    AD5270_Write(chip_sel, CMD_WR_RDAC, val);
}

void AD5930_Write(uint8_t reg, uint16_t data)
{
    uint16_t data_word = ((reg & 0x0F) << 12) | (data & 0x0FFF);
    digitalWrite(CHIP_SEL_AD5930, LOW);
    spi_write(VSPI_MOSI_PIN, VSPI_SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    digitalWrite(CHIP_SEL_AD5930, HIGH);
}

void AD5930_Set_Start_Freq(uint32_t freq)
{
    uint32_t scaled_freq = (freq * 1.0 / AD5930_CLK_FREQ) * 0x00FFFFFF;
    uint16_t freq_low  = scaled_freq & 0x0FFF;
    uint16_t freq_high = (scaled_freq >> 12) & 0x0FFF;
    AD5930_Write(SFREQ_LOW_REG,  freq_low);
    AD5930_Write(SFREQ_HIGH_REG, freq_high);
}

void mux_write(const int chip_sel, uint8_t pin_sel, uint8_t enable)
{
    digitalWrite(chip_sel, LOW);
    if (enable)
        spi_write(HSPI_MOSI_PIN, HSPI_SCK_PIN, SPI_FREQ_SLOW, MSBFIRST, SPI_MODE1, 8, pin_sel & 0x1F);
    else
        spi_write(HSPI_MOSI_PIN, HSPI_SCK_PIN, SPI_FREQ_SLOW, MSBFIRST, SPI_MODE1, 8, 0xC0 | (pin_sel & 0x1F));
    digitalWrite(chip_sel, HIGH);
}

uint16_t gpio_read()
{
    return (*(&GPIO6_DR + 2) >> 16);
}

uint16_t gpio_convert(uint16_t gpio_reg)
{
    uint16_t val = ((gpio_reg & 0x0200) >> 9) |
                   ((gpio_reg & 0x0100) >> 7) |
                   ((gpio_reg & 0x0800) >> 9) |
                   ((gpio_reg & 0x0400) >> 7) |
                   ((gpio_reg & 0x0003) << 4) |
                    (gpio_reg & 0x00C0)        |
                   ((gpio_reg & 0x0008) << 5)  |
                   ((gpio_reg & 0x0004) << 7);
    return val;
}

uint16_t sine_compare(uint16_t *signal, uint16_t pk_pk, uint16_t points_per_period, uint8_t num_periods)
{
    if (points_per_period == 0) return 0;
    uint16_t num_points = points_per_period * num_periods;
    uint16_t i;
    uint32_t error_sum = 0;
    for (i = 0; i < num_points; i++) {
        uint32_t ref_index = ((i * 1024) / points_per_period) % 1024;
        int32_t  ref_point = (sine_table[ref_index] * pk_pk) / 1024;
        int32_t  signal_val = (int16_t)signal[i] - 512;
        error_sum += abs(signal_val - ref_point);
    }
    return error_sum / num_points;
}

uint32_t read_signal(double *rms, double *mag, double *phase, uint16_t *error_rate, uint8_t debug)
{
    uint16_t i, j;
    uint16_t phase_count = 0;
    uint16_t adc_min = 1023, adc_max = 0;
    uint8_t  adc_peak_count = 0, adc_trough_count = 0;
    uint8_t  ref_period_count = 0, adc_period_count = 0;
    uint8_t  phase_readings = 0;
    uint16_t phase_start_index = 0;
    uint32_t time1, time2;
    uint32_t count, num_cycles;
    uint32_t sample_sum, total_sum = 0;

    time1 = micros();
    for (i = 0; i < num_samples; i++) {
        num_cycles = 20;
        count = 0;
        for (j = 0; j < ADC_AVG; j++) {
            while (ARM_DWT_CYCCNT - count < num_cycles);
            count = ARM_DWT_CYCCNT;
            _gpio_buf[i][j] = gpio_read();
        }
        _ref_buf[i] = digitalRead(AD5930_MSBOUT_PIN);
    }
    time2 = micros();

    for (i = 0; i < num_samples; i++) {
        for (j = 0, sample_sum = 0; j < ADC_AVG; j++)
            sample_sum += gpio_convert(_gpio_buf[i][j]);
        _adc_buf[i] = sample_sum / ADC_AVG;

        int16_t adc_val = (int16_t)_adc_buf[i] - 512;
        total_sum += adc_val * adc_val;
        if (_adc_buf[i] > adc_max) adc_max = _adc_buf[i];
        if (_adc_buf[i] < adc_min) adc_min = _adc_buf[i];

        if (i > 0) {
            if (_adc_buf[i] > 512 && _adc_buf[i-1] <= 512) {
                if (adc_period_count > 0) {
                    _adc_troughs[adc_trough_count++] = adc_min;
                    adc_min = 1023;
                    if (phase_count <= samples_per_period)
                        _phase_cycles[phase_readings++] = phase_count;
                }
                adc_period_count++;
                if (phase_start_index == 0) phase_start_index = i;
            } else if (_adc_buf[i] < 512 && _adc_buf[i-1] >= 512) {
                if (adc_period_count > 0) {
                    _adc_peaks[adc_peak_count++] = adc_max;
                    adc_max = 0;
                    if (phase_count <= samples_per_period)
                        _phase_cycles[phase_readings++] = phase_count;
                }
                adc_period_count++;
            }
            phase_count++;
            if (((_ref_buf[i] && !_ref_buf[i-1]) || (!_ref_buf[i] && _ref_buf[i-1]))) {
                ref_period_count++;
                phase_count = 0;
            }
        }
    }

    for (i = 0, sample_sum = 0; i < adc_peak_count;   i++) sample_sum += _adc_peaks[i];
    adc_max = adc_peak_count   > 0 ? sample_sum / adc_peak_count   : 512;
    for (i = 0, sample_sum = 0; i < adc_trough_count; i++) sample_sum += _adc_troughs[i];
    adc_min = adc_trough_count > 0 ? sample_sum / adc_trough_count : 512;

    int16_t phase_offset_cycles;
    for (i = 0, sample_sum = 0; i < phase_readings; i++) sample_sum += _phase_cycles[i];
    phase_offset_cycles = phase_readings > 0 ? sample_sum / phase_readings : 0;

    uint16_t mag_10bit = adc_max - adc_min;
    uint16_t rms_10bit = sqrt(total_sum / num_samples);

    if (rms)   *rms   = (double)rms_10bit * 2.2 / 1024;
    if (mag)   *mag   = (double)mag_10bit * 2.2 / 1024;
    if (phase) *phase = (sample_rate * phase_offset_cycles / 1000000.0) * TEST_FREQ * 2 * PI;

    if (error_rate) {
        uint16_t compare_periods = 2;
        if ((num_samples - phase_start_index) >= (samples_per_period * compare_periods))
            *error_rate = sine_compare(_adc_buf + phase_start_index, mag_10bit, samples_per_period, compare_periods);
    }

    return (time2 - time1);
}

void calibrate_samples()
{
    num_samples = 10000;
    if (num_samples > MAX_SAMPLES) num_samples = MAX_SAMPLES;
    uint32_t sample_time = read_signal(NULL, NULL, NULL, NULL, 0);
    sample_rate = (float)sample_time / (float)num_samples;
    samples_per_period = (1000000 / sample_rate) / TEST_FREQ;
    if (samples_per_period > MAX_SAMPLES_PER_PERIOD) samples_per_period = MAX_SAMPLES_PER_PERIOD;
    num_samples = samples_per_period * NUM_PERIODS;
}

void calibrate_gain(meas_t drive_type, meas_t meas_type)
{
    uint16_t i, j, k;
    uint16_t _gain;
    uint32_t error_sum;
    double mag_sum, rms_sum;
    uint16_t min_current_gain = 0;
    uint16_t min_voltage_gain = 0;

    AD5270_Shutdown(CHIP_SEL_MEAS, 1);

    for (i = 0; i < NUM_ELECTRODES; i++) {
        Serial.print(".");
        mux_write(CHIP_SEL_MUX_SRC,  elec_to_mux[i],                    MUX_EN);
        mux_write(CHIP_SEL_MUX_VP,   elec_to_mux[i],                    MUX_EN);
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[(i+1)%NUM_ELECTRODES], MUX_EN);
        mux_write(CHIP_SEL_MUX_VN,   elec_to_mux[(i+1)%NUM_ELECTRODES], MUX_EN);
        delay(2);

        for (j = 0; j < 1024; j++) {
            _gain = j;
            AD5270_Set(CHIP_SEL_DRIVE, _gain);
            delayMicroseconds(100);
            mag_sum = 0; rms_sum = 0; error_sum = 0;
            for (k = 0; k < 10; k++) {
                double mag, rms; uint16_t error;
                read_signal(&rms, &mag, NULL, &error, 0);
                mag_sum += mag; rms_sum += rms; error_sum += error;
            }
            mag_sum /= 10; rms_sum /= 10; error_sum /= 10;
            if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 35) {
                Serial.print(_gain); break;
            }
        }
        if (_gain > min_current_gain) min_current_gain = _gain;
    }
    AD5270_Shutdown(CHIP_SEL_DRIVE, 0);
    current_gain = min_current_gain;
    AD5270_Set(CHIP_SEL_DRIVE, current_gain);
    Serial.println();

    AD5270_Shutdown(CHIP_SEL_MEAS, 0);

    for (i = 0; i < NUM_ELECTRODES; i++) {
        Serial.print(".");
        mux_write(CHIP_SEL_MUX_SRC,  elec_to_mux[i],                      MUX_EN);
        mux_write(CHIP_SEL_MUX_VP,   elec_to_mux[(i+2)%NUM_ELECTRODES],   MUX_EN);
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[(i+1)%NUM_ELECTRODES],   MUX_EN);
        mux_write(CHIP_SEL_MUX_VN,   elec_to_mux[(i+3)%NUM_ELECTRODES],   MUX_EN);
        delay(2);

        for (j = 0; j < 1024; j++) {
            _gain = j;
            AD5270_Set(CHIP_SEL_MEAS, _gain);
            delayMicroseconds(100);
            mag_sum = 0; rms_sum = 0; error_sum = 0;
            for (k = 0; k < 10; k++) {
                double mag, rms; uint16_t error;
                read_signal(&rms, &mag, NULL, &error, 0);
                mag_sum += mag; rms_sum += rms; error_sum += error;
            }
            mag_sum /= 10; rms_sum /= 10; error_sum /= 10;
            if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 35) {
                Serial.print(_gain); break;
            }
        }
        if (_gain > min_voltage_gain) min_voltage_gain = _gain;
    }
    AD5270_Shutdown(CHIP_SEL_MEAS, 0);
    voltage_gain = min_voltage_gain;
    AD5270_Set(CHIP_SEL_MEAS, voltage_gain);
    Serial.println();

    mux_write(CHIP_SEL_MUX_SRC,  0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_SINK, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VP,   0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VN,   0, MUX_DIS);
}

void calibrate_signal(meas_t drive_type, meas_t meas_type)
{
    mux_write(CHIP_SEL_MUX_SRC,  elec_to_mux[0], MUX_EN);
    if (drive_type == AD)
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[1], MUX_EN);
    else if (drive_type == OP)
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[NUM_ELECTRODES/2], MUX_EN);

    if (meas_type == AD) {
        mux_write(CHIP_SEL_MUX_VP, elec_to_mux[2], MUX_EN);
        mux_write(CHIP_SEL_MUX_VN, elec_to_mux[3], MUX_EN);
    } else if (meas_type == OP) {
        mux_write(CHIP_SEL_MUX_VP, elec_to_mux[1], MUX_EN);
        mux_write(CHIP_SEL_MUX_VN, elec_to_mux[2], MUX_EN);
    }
    delay(5);

    ref_signal_mag = 1.0;
    phase_offset   = 0;
    read_signal(NULL, &ref_signal_mag, &phase_offset, NULL, 0);

    mux_write(CHIP_SEL_MUX_SRC,  0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_SINK, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VP,   0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VN,   0, MUX_DIS);
}

/*
 * Full-matrix measurement (NUM_ELECTRODES x NUM_ELECTRODES).
 *
 * Drive type AD: src=tx, sink=(tx+1)%N  →  adjacent pairs
 * Meas type  AD: vp=rx,  vn=(rx+1)%N   →  adjacent pairs
 *
 * Invalid (2-wire) pairs — where meas electrodes overlap drive electrodes —
 * are skipped: mag/phase set to 0, rms_array entry left at 0 (global init).
 *
 * Index: rms_array[tx * NUM_ELECTRODES + rx]
 */
void read_frame(meas_t drive_type, meas_t meas_type, double *rms_array, double *mag_array, double *phase_array, uint8_t num_elec)
{
    int8_t tx_pair, rx_pair;
    uint8_t src_pin, sink_pin, vp_pin, vn_pin;
    uint16_t num_meas = 0;

    for (tx_pair = 0; tx_pair < num_elec; tx_pair++) {
        switch (drive_type) {
            case AD:
                src_pin  = tx_pair;
                sink_pin = (tx_pair + 1) % num_elec;
                break;
            case OP:
                src_pin  = tx_pair;
                sink_pin = (tx_pair + num_elec/2) % num_elec;
                break;
            case MONO:
                src_pin  = tx_pair;
                sink_pin = 0;
                break;
        }

        mux_write(CHIP_SEL_MUX_SRC,  elec_to_mux[src_pin],  MUX_EN);
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[sink_pin], MUX_EN);
        delayMicroseconds(150);

        for (rx_pair = 0; rx_pair < num_elec; rx_pair++, num_meas++) {
            switch (meas_type) {
                case AD:
                    vp_pin = rx_pair;
                    vn_pin = (rx_pair + 1) % num_elec;
                    break;
                case OP:
                    vp_pin = rx_pair;
                    vn_pin = (rx_pair + num_elec/2) % num_elec;
                    break;
                case MONO:
                    vp_pin = rx_pair;
                    vn_pin = sink_pin;
                    break;
            }

            if (meas_type == MONO) {
                if ((vp_pin == src_pin) || (vp_pin == vn_pin) || (src_pin == sink_pin)) {
                    mag_array[num_meas]   = 0;
                    phase_array[num_meas] = 0;
                } else {
                    mux_write(CHIP_SEL_MUX_VP, elec_to_mux[vp_pin], MUX_EN);
                    mux_write(CHIP_SEL_MUX_VN, elec_to_mux[vn_pin], MUX_EN);
                    delayMicroseconds(100);
                    read_signal(rms_array + num_meas, mag_array + num_meas, phase_array + num_meas, NULL, 0);
                    mux_write(CHIP_SEL_MUX_VP, 0, MUX_DIS);
                    mux_write(CHIP_SEL_MUX_VN, 0, MUX_DIS);
                }
            } else {
                if ((vp_pin == src_pin) || (vp_pin == sink_pin) || (vn_pin == src_pin) || (vn_pin == sink_pin)) {
                    rms_array[num_meas]   = 0;
                    mag_array[num_meas]   = 0;
                    phase_array[num_meas] = 0;
                } else {
                    mux_write(CHIP_SEL_MUX_VP, elec_to_mux[vp_pin], MUX_EN);
                    mux_write(CHIP_SEL_MUX_VN, elec_to_mux[vn_pin], MUX_EN);
                    delayMicroseconds(100);
                    read_signal(rms_array + num_meas, mag_array + num_meas, phase_array + num_meas, NULL, 0);
                    mux_write(CHIP_SEL_MUX_VP, 0, MUX_DIS);
                    mux_write(CHIP_SEL_MUX_VN, 0, MUX_DIS);
                }
            }
        }

        mux_write(CHIP_SEL_MUX_SRC,  0, MUX_DIS);
        mux_write(CHIP_SEL_MUX_SINK, 0, MUX_DIS);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

    pinMode(HSPI_MOSI_PIN,     OUTPUT);
    pinMode(HSPI_SCK_PIN,      OUTPUT);
    pinMode(VSPI_MOSI_PIN,     OUTPUT);
    pinMode(VSPI_SCK_PIN,      OUTPUT);
    pinMode(CHIP_SEL_DRIVE,    OUTPUT);
    pinMode(CHIP_SEL_MEAS,     OUTPUT);
    pinMode(CHIP_SEL_MUX_SRC,  OUTPUT);
    pinMode(CHIP_SEL_MUX_SINK, OUTPUT);
    pinMode(CHIP_SEL_MUX_VP,   OUTPUT);
    pinMode(CHIP_SEL_MUX_VN,   OUTPUT);
    pinMode(CHIP_SEL_AD5930,   OUTPUT);
    pinMode(AD5930_INT_PIN,     OUTPUT);
    pinMode(AD5930_CTRL_PIN,    OUTPUT);
    pinMode(AD5930_STANDBY_PIN, OUTPUT);
    pinMode(AD5930_MSBOUT_PIN,  INPUT);
    pinMode(ADS_PWR,            OUTPUT);
    pinMode(ADS_OE,             OUTPUT);

    pinMode(14, INPUT); pinMode(15, INPUT); pinMode(16, INPUT); pinMode(17, INPUT);
    pinMode(18, INPUT); pinMode(19, INPUT); pinMode(20, INPUT); pinMode(21, INPUT);
    pinMode(22, INPUT); pinMode(23, INPUT);

    digitalWrite(CHIP_SEL_DRIVE,    HIGH);
    digitalWrite(CHIP_SEL_MEAS,     HIGH);
    digitalWrite(CHIP_SEL_MUX_SRC,  HIGH);
    digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
    digitalWrite(CHIP_SEL_MUX_VP,   HIGH);
    digitalWrite(CHIP_SEL_MUX_VN,   HIGH);
    digitalWrite(CHIP_SEL_AD5930,   HIGH);
    digitalWrite(AD5930_INT_PIN,     LOW);
    digitalWrite(AD5930_CTRL_PIN,    LOW);
    digitalWrite(AD5930_STANDBY_PIN, LOW);
    digitalWrite(ADS_PWR,            LOW);
    digitalWrite(ADS_OE,             LOW);

    AD5930_Write(CTRL_REG, 0b011111110011);
    AD5930_Set_Start_Freq(TEST_FREQ);

    AD5270_Lock(CHIP_SEL_DRIVE, 0);
    AD5270_Lock(CHIP_SEL_MEAS,  0);

    digitalWrite(AD5930_CTRL_PIN, HIGH);
    delay(100);

    calibrate_samples();
    // calibrate_gain(AD, AD);
    AD5270_Set(CHIP_SEL_DRIVE, 1000);
    AD5270_Set(CHIP_SEL_MEAS,  10);
    calibrate_signal(AD, AD);

    mux_write(CHIP_SEL_MUX_SRC,  elec_to_mux[0], MUX_EN);
    mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[1], MUX_EN);
    mux_write(CHIP_SEL_MUX_VP,   elec_to_mux[0], MUX_EN);
    mux_write(CHIP_SEL_MUX_VN,   elec_to_mux[1], MUX_EN);
}

void loop()
{
    uint16_t i;

    read_frame(AD, AD, signal_rms, signal_mag, signal_phase, NUM_ELECTRODES);

    if (millis() - frame_delay > 5) {
        for (i = 0; i < NUM_MEAS; i++) {
            Serial.print(signal_rms[i], 4);
            if (i < NUM_MEAS - 1) Serial.print(",");
        }
        Serial.print("\n");
        frame_delay_prev = frame_delay;
        frame_delay = millis();
    }
}
