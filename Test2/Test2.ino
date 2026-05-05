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
// Opposite drive: 2 adjacent pairs on each side of drive electrodes
#define NUM_MEAS           ((NUM_ELECTRODES / 2) * (NUM_ELECTRODES - 4))  // 4 * 4 = 16

#define AD5930_CLK_FREQ    50000000
#define TEST_FREQ          50000 // CHANGE FREQUENCY
#define NUM_PERIODS        10        // Number of signal periods to measure
#define ADC_AVG            10        // Number of ADC samples to average for each analog reading

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

// AD55930 register addresses
#define CTRL_REG           0x00
#define NUM_INCR_REG       0x01
#define DFREQ_LOW_REG      0x02
#define DFREQ_HIGH_REG     0x03
#define TIME_INCR_REG      0x04
#define TIME_BURST_REG     0x08
#define SFREQ_LOW_REG      0x0C
#define SFREQ_HIGH_REG     0x0D

#define CHIP_SEL_AD5930    3  // Chip select pin for AD5930
#define CHIP_SEL_DRIVE     0  // Chip select pin for driving digital rheostat
#define CHIP_SEL_MEAS      1 // Chip select pin for measuring digital rheostat
#define CHIP_SEL_MUX_SRC   29 // Chip select pin for source electrodes MUX - mux1
#define CHIP_SEL_MUX_SINK  25 // Chip select pin for sink electrodes MUX - mux1
#define CHIP_SEL_MUX_VP    33 // Chip select for voltage measurement positive electrodes MUX - mux1
#define CHIP_SEL_MUX_VN    31 // Chip select for voltage measurement negative electrodes MUX - mux1

#define AD5930_MSBOUT_PIN  6
#define AD5930_INT_PIN     5  // Pulse high to reset internal state machine
#define AD5930_CTRL_PIN    4  // Pull high to start frequency sweep. Pull low to end the burst. Pull high again to increment frequency
#define AD5930_STANDBY_PIN 2  // Pull high to power down

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

// GPIO Pin to analog channel mapping from Arduino\hardware\teensy\avr\cores\teensy4\analog.c
extern const uint8_t pin_to_channel[42];

// Mapping of electrode number (input) to MUX channel (output)
const uint8_t elec_to_mux[32] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31 };

// Global calibration parameters
uint16_t current_gain, voltage_gain;
float sample_rate;
uint16_t samples_per_period;
uint16_t num_samples;
double ref_signal_mag;
double phase_offset;

// At 1kHz, ~3000 samples/period * 10 periods = 30000; cap samples_per_period at 500
#define MAX_SAMPLES_PER_PERIOD 500
#define MAX_SAMPLES (MAX_SAMPLES_PER_PERIOD * NUM_PERIODS)  // 5000
static uint16_t gpio_buf[MAX_SAMPLES][ADC_AVG];
static uint16_t adc_buf[MAX_SAMPLES];
static uint8_t  ref_buf[MAX_SAMPLES];
static uint16_t adc_peaks[MAX_SAMPLES];
static uint16_t adc_troughs[MAX_SAMPLES];
static uint16_t phase_cycles[MAX_SAMPLES];

double signal_rms[NUM_MEAS];    // Store signal RMS data
double signal_mag[NUM_MEAS];    // Store signal magnitude data
double signal_phase[NUM_MEAS];  // Store signal phase data

double cur_frame[NUM_MEAS] = {0};
uint32_t frame_delay = 0;

uint8_t pin_num = 0;
uint16_t rheo_val = 1023;

/* Shift a byte out serially with the given frequency in Hz (<= 500kHz) */
void spi_write(uint8_t data_pin, uint8_t clock_pin, uint32_t freq, uint8_t bit_order, uint8_t mode, uint8_t bits, uint32_t val)
{
    uint32_t period = (freq >= 500000) ? 1 : (500000 / freq);   // Half clock period in uS
    uint8_t cpol = (mode == SPI_MODE2 || mode == SPI_MODE3);
    uint8_t cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    uint8_t sck = cpol ? HIGH : LOW;

    uint8_t i;
    uint32_t start_time;

    // Set clock idle for 2 periods
    digitalWrite(clock_pin, sck);
    delayMicroseconds(period*4);

    for (i = 0; i < bits; i++)  {
        start_time = micros();

        // Shift bit out
        if (bit_order == LSBFIRST)
            digitalWrite(data_pin, !!(val & (1 << i)));
        else
            digitalWrite(data_pin, !!(val & (1 << ((bits-1) - i))));

        // Toggle clock leading edge
        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while(micros() - start_time < period);
        } else {
            while(micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }

        // Toggle clock trailing edge
        start_time = micros();
        sck = !sck;
        if (cpha) {
            digitalWrite(clock_pin, sck);
            while(micros() - start_time < period);
        } else {
            while(micros() - start_time < period);
            digitalWrite(clock_pin, sck);
        }
    }
}

/* Write a 4-bit command and a 10-bit data word */
void AD5270_Write(const int chip_sel, uint8_t cmd, uint16_t data)
{
    uint16_t data_word = ((cmd & 0x0F) << 10) | (data & 0x03FF);

    digitalWrite(chip_sel, LOW);
    delayMicroseconds(500);
    spi_write(VSPI_MOSI_PIN, VSPI_SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    delayMicroseconds(500);
    digitalWrite(chip_sel, HIGH);
}

/* Enable/disable rheostat value changes */
void AD5270_Lock(const int chip_sel, uint8_t lock)
{
    AD5270_Write(chip_sel, CMD_WR_CTRL, lock ? 0 : 0x002);
}

/* Enable/disable hardware shutdown */
void AD5270_Shutdown(const int chip_sel, uint8_t shutdown)
{
    AD5270_Write(chip_sel, CMD_SHTDN, shutdown ? 1 : 0);
}

/* Set the value of the digital rheostat - range is 0-0x3FF (0-100kOhm) */
void AD5270_Set(const int chip_sel, uint16_t val)
{
    AD5270_Write(chip_sel, CMD_WR_RDAC, val);
}

/* Write a 12-bit data word into a register. Register addresses are 4 bits */
void AD5930_Write(uint8_t reg, uint16_t data)
{
    uint16_t data_word = ((reg & 0x0F) << 12) | (data & 0x0FFF);

    digitalWrite(CHIP_SEL_AD5930, LOW);
    spi_write(VSPI_MOSI_PIN, VSPI_SCK_PIN, SPI_FREQ_FAST, MSBFIRST, SPI_MODE1, 16, data_word);
    digitalWrite(CHIP_SEL_AD5930, HIGH);
}

/* Program the given frequency (in Hz) */
void AD5930_Set_Start_Freq(uint32_t freq)
{
    uint32_t scaled_freq = (freq * 1.0 / AD5930_CLK_FREQ) * 0x00FFFFFF;
    uint16_t freq_low = scaled_freq & 0x0FFF;
    uint16_t freq_high = (scaled_freq >> 12) & 0x0FFF;

    AD5930_Write(SFREQ_LOW_REG, freq_low);
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

/* Return unsigned integer (0-1023) from 10 continuous GPIO pins (14-23, MSb on 14) (takes ~50.1ns) */
uint16_t analog_read()
{
    uint16_t gpio_reg = *(&GPIO6_DR + 2) >> 16;
    uint16_t val = ((gpio_reg & 0x0200) >> 9) |
                   ((gpio_reg & 0x0100) >> 7) |
                   ((gpio_reg & 0x0800) >> 9) |
                   ((gpio_reg & 0x0400) >> 7) |
                   ((gpio_reg & 0x0003) << 4) |
                    (gpio_reg & 0x00C0) |
                   ((gpio_reg & 0x0008) << 5) |
                   ((gpio_reg & 0x0004) << 7);
    return val;
}

/* Read 10 continuous GPIO pins (14-23) (takes ~16.8ns) */
uint16_t gpio_read()
{
    return (*(&GPIO6_DR + 2) >> 16);
}

/* Convert GPIO reading to 10-bit unsigned integer (takes ~33.3ns) */
uint16_t gpio_convert(uint16_t gpio_reg)
{
    uint16_t val = ((gpio_reg & 0x0200) >> 9) |
                   ((gpio_reg & 0x0100) >> 7) |
                   ((gpio_reg & 0x0800) >> 9) |
                   ((gpio_reg & 0x0400) >> 7) |
                   ((gpio_reg & 0x0003) << 4) |
                    (gpio_reg & 0x00C0) |
                   ((gpio_reg & 0x0008) << 5) |
                   ((gpio_reg & 0x0004) << 7);
    return val;
}

/* Return the magnitude and phase offset of a sinusoidal input signal */
uint32_t read_signal(double * rms, double * mag, double * phase, uint16_t * error_rate, uint8_t debug)
{
    uint16_t i, j;
    uint16_t phase_count = 0;
    uint16_t adc_min = 1023;
    uint16_t adc_max = 0;
    uint8_t adc_peak_count = 0;
    uint8_t adc_trough_count = 0;
    uint8_t ref_period_count = 0;
    uint8_t adc_period_count = 0;
    uint8_t phase_readings = 0;
    uint16_t phase_start_index = 0;

    uint32_t time1, time2;
    uint32_t count, num_cycles;
    uint32_t sample_sum, total_sum = 0;

    time1 = micros();

    /* Collect samples */
    for(i = 0; i < num_samples; i++)
    {
        num_cycles = 20;
        count = 0;

        for (j = 0; j < ADC_AVG; j++)
        {
            while (ARM_DWT_CYCCNT - count < num_cycles);
            count = ARM_DWT_CYCCNT;
            gpio_buf[i][j] = gpio_read();
        }
        ref_buf[i] = digitalRead(AD5930_MSBOUT_PIN);
    }

    time2 = micros();

    /* Process samples */
    for(i = 0; i < num_samples; i++)
    {
        for (j = 0, sample_sum = 0; j < ADC_AVG; j++)
            sample_sum += gpio_convert(gpio_buf[i][j]);
        adc_buf[i] = sample_sum / ADC_AVG;

        int16_t adc_val = (int16_t)adc_buf[i] - 512;
        total_sum += adc_val * adc_val;

        if (adc_buf[i] > adc_max)
            adc_max = adc_buf[i];
        if (adc_buf[i] < adc_min)
            adc_min = adc_buf[i];

        if (i > 0)
        {
            if (adc_buf[i] > 512 && adc_buf[i-1] <= 512)
            {
                if (adc_period_count > 0)
                {
                    adc_troughs[adc_trough_count] = adc_min;
                    adc_trough_count++;
                    adc_min = 1023;

                    if (phase_count <= samples_per_period)
                    {
                        phase_cycles[phase_readings] = phase_count;
                        phase_readings++;
                    }
                }
                adc_period_count++;

                if (phase_start_index == 0)
                    phase_start_index = i;
            }
            else if (adc_buf[i] < 512 && adc_buf[i-1] >= 512)
            {
                if (adc_period_count > 0)
                {
                    adc_peaks[adc_peak_count] = adc_max;
                    adc_peak_count++;
                    adc_max = 0;

                    if (phase_count <= samples_per_period)
                    {
                        phase_cycles[phase_readings] = phase_count;
                        phase_readings++;
                    }
                }
                adc_period_count++;
            }

            phase_count++;

            if ((ref_buf[i] && !ref_buf[i-1]) || (!ref_buf[i] && ref_buf[i-1]))
            {
                ref_period_count++;
                phase_count = 0;
            }
        }
    }

    for (i = 0, sample_sum =  0; i < adc_peak_count; i++)
        sample_sum += adc_peaks[i];
    adc_max = adc_peak_count > 0 ? sample_sum / adc_peak_count : 512;
    for (i = 0, sample_sum = 0; i < adc_trough_count; i++)
        sample_sum += adc_troughs[i];
    adc_min = adc_trough_count > 0 ? sample_sum / adc_trough_count : 512;

    int16_t phase_offset_cycles;
    for (i = 0, sample_sum = 0; i < phase_readings; i++)
        sample_sum += phase_cycles[i];
    phase_offset_cycles = phase_readings > 0 ? sample_sum / phase_readings : 0;

    uint16_t mag_10bit = adc_max - adc_min;
    uint16_t rms_10bit = sqrt(total_sum / num_samples);

    if (rms)
        *rms = (double)rms_10bit * 2.2 / 1024;
    if (mag)
        *mag = (double)mag_10bit * 2.2 / 1024;
    if (phase)
        *phase = (sample_rate * phase_offset_cycles / 1000000) * TEST_FREQ * 2*PI;

    if (error_rate)
    {
        uint16_t compare_periods = 2;
        if ((num_samples - phase_start_index) >= (samples_per_period * compare_periods))
            *error_rate = sine_compare(adc_buf+phase_start_index, mag_10bit, samples_per_period, compare_periods);
    }

    return (time2 - time1);
}

/* Find the optimal number of samples to read the desired number of periods of the input signal */
void calibrate_samples() {

    num_samples = MAX_SAMPLES;
    uint32_t sample_time = read_signal(NULL, NULL, NULL, NULL, 0);

    sample_rate = (float)sample_time / (float)MAX_SAMPLES;
    samples_per_period = (1000000 / sample_rate) / TEST_FREQ;
    if (samples_per_period > MAX_SAMPLES_PER_PERIOD)
        samples_per_period = MAX_SAMPLES_PER_PERIOD;
    num_samples = samples_per_period * NUM_PERIODS;
}

/* Find the gains that produce the highest sinusoidal current and voltage measurements */
void calibrate_gain(meas_t drive_type, meas_t meas_type) {
    uint16_t i, j, k;
    uint16_t _gain;
    uint32_t error_sum;
    double mag_sum;
    double rms_sum;
    uint16_t min_current_gain = 0;
    uint16_t min_voltage_gain = 0;

    AD5270_Shutdown(CHIP_SEL_MEAS, 1);

    for (i = 0; i < NUM_ELECTRODES; i++) {
        Serial.print(".");
        mux_write(CHIP_SEL_MUX_SRC, elec_to_mux[i], MUX_EN);
        mux_write(CHIP_SEL_MUX_VP, elec_to_mux[i], MUX_EN);
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[(i+1)%NUM_ELECTRODES], MUX_EN);
        mux_write(CHIP_SEL_MUX_VN, elec_to_mux[(i+1)%NUM_ELECTRODES], MUX_EN);
        delay(2);

        for (j = 0; j < 1024; j++) {
            _gain = j;
            AD5270_Set(CHIP_SEL_DRIVE, _gain);
            delayMicroseconds(100);

            double mag;
            double rms;
            uint16_t error;
            mag_sum = 0;
            rms_sum = 0;
            error_sum = 0;
            for (k = 0; k < 10; k++) {
                read_signal(&rms, &mag, NULL, &error, 0);
                mag_sum += mag;
                rms_sum += rms;
                error_sum += error;
            }
            mag_sum = mag_sum / 10;
            rms_sum = rms_sum / 10;
            error_sum = error_sum / 10;

            if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 35) {
                Serial.print(_gain);
                break;
            }
        }
        if (_gain > min_current_gain)
            min_current_gain = _gain;
    }
    AD5270_Shutdown(CHIP_SEL_DRIVE, 0);
    current_gain = min_current_gain;
    AD5270_Set(CHIP_SEL_DRIVE, current_gain);
    Serial.println();

    AD5270_Shutdown(CHIP_SEL_MEAS, 0);

    for (i = 0; i < NUM_ELECTRODES; i++) {
        Serial.print(".");
        mux_write(CHIP_SEL_MUX_SRC, elec_to_mux[i], MUX_EN);
        mux_write(CHIP_SEL_MUX_VP, elec_to_mux[((i+2)%NUM_ELECTRODES)], MUX_EN);
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[((i+1)%NUM_ELECTRODES)], MUX_EN);
        mux_write(CHIP_SEL_MUX_VN, elec_to_mux[((i+3)%NUM_ELECTRODES)], MUX_EN);
        delay(2);

        for (j = 0; j < 1024; j++) {
            _gain = j;
            AD5270_Set(CHIP_SEL_MEAS, _gain);
            delayMicroseconds(100);

            double mag;
            double rms;
            uint16_t error;
            mag_sum = 0;
            rms_sum = 0;
            error_sum = 0;
            for (k = 0; k < 10; k++) {
                read_signal(&rms, &mag, NULL, &error, 0);
                mag_sum += mag;
                rms_sum += rms;
                error_sum += error;
            }
            mag_sum = mag_sum / 10;
            rms_sum = rms_sum / 10;
            error_sum = error_sum / 10;

            if (mag_sum > 0.5 && mag_sum < 2.1 && error_sum < 35) {
                Serial.print(_gain);
                break;
            }
        }
        if (_gain > min_voltage_gain)
            min_voltage_gain = _gain;
    }
    AD5270_Shutdown(CHIP_SEL_MEAS, 0);
    voltage_gain = min_voltage_gain;
    AD5270_Set(CHIP_SEL_MEAS, voltage_gain);
    Serial.println();

    mux_write(CHIP_SEL_MUX_SRC, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_SINK, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VP, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VN, 0, MUX_DIS);
}

/* Find the magnitude and phase offset of the highest voltage differential point */
void calibrate_signal(meas_t drive_type, meas_t meas_type) {

    mux_write(CHIP_SEL_MUX_SRC, elec_to_mux[0], MUX_EN);
    if (drive_type == AD)
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[1], MUX_EN);
    else if (drive_type == OP)
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[16], MUX_EN);

    if (meas_type == AD) {
        mux_write(CHIP_SEL_MUX_VP, elec_to_mux[2], MUX_EN);
        mux_write(CHIP_SEL_MUX_VN, elec_to_mux[3], MUX_EN);
    } else if (meas_type == OP) {
        mux_write(CHIP_SEL_MUX_VP, elec_to_mux[1], MUX_EN);
        mux_write(CHIP_SEL_MUX_VN, elec_to_mux[3], MUX_EN);
    }

    delay(5);

    ref_signal_mag = 1.0;
    phase_offset = 0;
    read_signal(NULL, &ref_signal_mag, &phase_offset, NULL, 0);

    mux_write(CHIP_SEL_MUX_SRC, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_SINK, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VP, 0, MUX_DIS);
    mux_write(CHIP_SEL_MUX_VN, 0, MUX_DIS);
}

uint16_t sine_compare(uint16_t * signal, uint16_t pk_pk, uint16_t points_per_period, uint8_t num_periods) {

    if (points_per_period == 0)
        return 0;

    uint16_t num_points = points_per_period * num_periods;

    uint16_t i;
    uint16_t point_error;
    uint32_t error_sum = 0;

    for (i = 0; i < num_points; i++) {
        uint32_t ref_index = ((i * 1024) / points_per_period) % 1024;
        int32_t ref_point = (sine_table[ref_index] * pk_pk) / 1024;

        int32_t signal_val = (int16_t)signal[i] - 512;

        point_error = abs(signal_val - ref_point);
        error_sum += point_error;
    }
    error_sum = error_sum / num_points;
    return error_sum;
}

/*
 * Opposite-drive, both-sides-only frame acquisition for pyEIT.
 *
 * Drive pattern : opposite  (src -> src + N/2)
 * Measurement   : 2 adjacent pairs on the src side  +
 *                 2 adjacent pairs on the sink side
 *                 (the bridging pair across the sink is NOT measured)
 *
 * For NUM_ELECTRODES=8:
 *   4 drive pairs x 4 meas pairs = 16 values total
 *
 * Matches pyeit protocol.create(8, dist_exc=4, step_meas=1, parser_meas="std")
 *
 * Index layout (d = drive index 0-3):
 *   d*4+0 : vp=(d+1)%8  vn=(d+2)%8   source side, near
 *   d*4+1 : vp=(d+2)%8  vn=(d+3)%8   source side, far
 *   d*4+2 : vp=(d+5)%8  vn=(d+6)%8   sink side, near
 *   d*4+3 : vp=(d+6)%8  vn=(d+7)%8   sink side, far
 */
void read_frame_opposite(double * rms_array, uint8_t num_elec)
{
    uint8_t d, k;
    uint16_t meas_idx = 0;
    uint8_t half = num_elec / 2;           // 4
    uint8_t side_meas = half - 2;          // 2 pairs per side

    for (d = 0; d < half; d++)
    {
        uint8_t src  = d;
        uint8_t sink = (d + half) % num_elec;

        mux_write(CHIP_SEL_MUX_SRC,  elec_to_mux[src],  MUX_EN);
        mux_write(CHIP_SEL_MUX_SINK, elec_to_mux[sink], MUX_EN);
        delayMicroseconds(150);

        /* Source side: (src+1,src+2), (src+2,src+3) */
        for (k = 0; k < side_meas; k++, meas_idx++)
        {
            uint8_t vp = (src + 1 + k) % num_elec;
            uint8_t vn = (src + 2 + k) % num_elec;

            mux_write(CHIP_SEL_MUX_VP, elec_to_mux[vp], MUX_EN);
            mux_write(CHIP_SEL_MUX_VN, elec_to_mux[vn], MUX_EN);
            delayMicroseconds(100);
            read_signal(rms_array + meas_idx, NULL, NULL, NULL, 0);
            mux_write(CHIP_SEL_MUX_VP, 0, MUX_DIS);
            mux_write(CHIP_SEL_MUX_VN, 0, MUX_DIS);
        }

        /* Sink side: (sink+1,sink+2), (sink+2,sink+3) */
        for (k = 0; k < side_meas; k++, meas_idx++)
        {
            uint8_t vp = (sink + 1 + k) % num_elec;
            uint8_t vn = (sink + 2 + k) % num_elec;

            mux_write(CHIP_SEL_MUX_VP, elec_to_mux[vp], MUX_EN);
            mux_write(CHIP_SEL_MUX_VN, elec_to_mux[vn], MUX_EN);
            delayMicroseconds(100);
            read_signal(rms_array + meas_idx, NULL, NULL, NULL, 0);
            mux_write(CHIP_SEL_MUX_VP, 0, MUX_DIS);
            mux_write(CHIP_SEL_MUX_VN, 0, MUX_DIS);
        }

        mux_write(CHIP_SEL_MUX_SRC,  0, MUX_DIS);
        mux_write(CHIP_SEL_MUX_SINK, 0, MUX_DIS);
    }
}

void setup()
{
    Serial.begin(115200);

    while(!Serial);
    Serial.println("[1] Serial OK");

    pinMode(HSPI_MOSI_PIN, OUTPUT);
    pinMode(HSPI_SCK_PIN, OUTPUT);
    pinMode(VSPI_MOSI_PIN, OUTPUT);
    pinMode(VSPI_SCK_PIN, OUTPUT);

    pinMode(CHIP_SEL_DRIVE, OUTPUT);
    pinMode(CHIP_SEL_MEAS, OUTPUT);
    pinMode(CHIP_SEL_MUX_SRC, OUTPUT);
    pinMode(CHIP_SEL_MUX_SINK, OUTPUT);
    pinMode(CHIP_SEL_MUX_VP, OUTPUT);
    pinMode(CHIP_SEL_MUX_VN, OUTPUT);
    pinMode(CHIP_SEL_AD5930, OUTPUT);

    pinMode(AD5930_INT_PIN, OUTPUT);
    pinMode(AD5930_CTRL_PIN, OUTPUT);
    pinMode(AD5930_STANDBY_PIN, OUTPUT);
    pinMode(AD5930_MSBOUT_PIN, INPUT);

    pinMode(14, INPUT);
    pinMode(15, INPUT);
    pinMode(16, INPUT);
    pinMode(17, INPUT);
    pinMode(18, INPUT);
    pinMode(19, INPUT);
    pinMode(20, INPUT);
    pinMode(21, INPUT);
    pinMode(22, INPUT);
    pinMode(23, INPUT);

    digitalWrite(CHIP_SEL_DRIVE, HIGH);
    digitalWrite(CHIP_SEL_MEAS, HIGH);
    digitalWrite(CHIP_SEL_MUX_SRC, HIGH);
    digitalWrite(CHIP_SEL_MUX_SINK, HIGH);
    digitalWrite(CHIP_SEL_MUX_VP, HIGH);
    digitalWrite(CHIP_SEL_MUX_VN, HIGH);
    digitalWrite(CHIP_SEL_AD5930, HIGH);
    digitalWrite(AD5930_INT_PIN, LOW);
    digitalWrite(AD5930_CTRL_PIN, LOW);
    digitalWrite(AD5930_STANDBY_PIN, LOW);

    digitalWrite(ADS_PWR, LOW);
    digitalWrite(ADS_OE, LOW);

    Serial.println("[2] Configuring AD5930...");
    AD5930_Write(CTRL_REG, 0b011111110011);
    AD5930_Set_Start_Freq(TEST_FREQ);
    Serial.print("[2] AD5930 freq set to "); Serial.print(TEST_FREQ); Serial.println(" Hz");

    AD5270_Lock(CHIP_SEL_DRIVE, 0);
    AD5270_Lock(CHIP_SEL_MEAS, 0);

    digitalWrite(AD5930_CTRL_PIN, HIGH);
    delay(100);

    Serial.println("[3] Calibrating samples...");
    calibrate_samples();
    Serial.print("[3] sample_rate="); Serial.print(sample_rate, 3);
    Serial.print(" us  samples_per_period="); Serial.print(samples_per_period);
    Serial.print("  num_samples="); Serial.println(num_samples);

    AD5270_Set(CHIP_SEL_DRIVE, 10); // BETWEEN 1 AND 1023, CURRENT GAIN
    AD5270_Set(CHIP_SEL_MEAS,  80); // BETWEEN 1 AND 1023, MEASUREMENT GAIN

    // Serial.println("[3.5] Calibrating gain...");
    // calibrate_gain(AD, AD);
    // Serial.println("[3.5] Gain calibration done");

    Serial.println("[4] Calibrating signal...");
    calibrate_signal(AD, AD);
    Serial.print("[4] ref_mag="); Serial.print(ref_signal_mag, 4);
    Serial.print("  phase_offset="); Serial.println(phase_offset, 4);

    Serial.print("[5] NUM_MEAS="); Serial.print(NUM_MEAS);
    Serial.println(" (4 opposite drive pairs x 4 meas pairs)");
    Serial.println("[5] Setup complete, entering loop");
}

void loop()
{
    static uint32_t loop_count = 0;
    loop_count++;
    if (loop_count == 1)
        Serial.println("[L] loop() started");

    read_frame_opposite(signal_rms, NUM_ELECTRODES);

    if (loop_count == 1)
        Serial.println("[L] read_frame_opposite() returned");

    if (millis() - frame_delay > 350) {
        // Output all NUM_MEAS (40) RMS values as comma-separated line.
        // pyEIT index: tx_pair * 5 + k  (tx_pair=0..7, k=0..4)
        uint16_t i;
        for (i = 0; i < NUM_MEAS; i++)
        {
            Serial.print(signal_rms[i], 4);
            if (i < NUM_MEAS - 1) Serial.print(",");
        }
        Serial.print("\n");
        frame_delay = millis();
        delay(10);
    }
}
