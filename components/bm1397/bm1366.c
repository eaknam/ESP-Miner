#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "serial.h"
#include "bm1366.h"
#include "utils.h"
#include "crc.h"
#include "global_state.h"

#define BM1366_RST_PIN  GPIO_NUM_1


#define TYPE_JOB 0x20
#define TYPE_CMD 0x40

#define GROUP_SINGLE 0x00
#define GROUP_ALL 0x10

#define CMD_JOB 0x01

#define CMD_SETADDRESS 0x00
#define CMD_WRITE 0x01
#define CMD_READ 0x02
#define CMD_INACTIVE 0x03

#define RESPONSE_CMD 0x00
#define RESPONSE_JOB 0x80



#define SLEEP_TIME 20
#define FREQ_MULT 25.0

#define CLOCK_ORDER_CONTROL_0 0x80
#define CLOCK_ORDER_CONTROL_1 0x84
#define ORDERED_CLOCK_ENABLE 0x20
#define CORE_REGISTER_CONTROL 0x3C
#define PLL3_PARAMETER 0x68
#define FAST_UART_CONFIGURATION 0x28
#define TICKET_MASK 0x14
#define MISC_CONTROL 0x18

static const char *TAG = "bm1366Module";

static uint8_t asic_response_buffer[CHUNK_SIZE];
static asic_result _asic_result;

/// @brief
/// @param ftdi
/// @param header
/// @param data
/// @param len
static void _send_BM1366(uint8_t header, uint8_t * data, uint8_t data_len, bool debug) {
    packet_type_t packet_type = (header & TYPE_JOB) ? JOB_PACKET : CMD_PACKET;
    uint8_t total_length = (packet_type == JOB_PACKET) ? (data_len+6) : (data_len+5);

    //allocate memory for buffer
    unsigned char *buf = malloc(total_length);

    //add the preamble
    buf[0] = 0x55;
    buf[1] = 0xAA;

    //add the header field
    buf[2] = header;

    //add the length field
    buf[3] = (packet_type == JOB_PACKET) ? (data_len+4) : (data_len+3);

    //add the data
    memcpy(buf+4, data, data_len);

    //add the correct crc type
    if (packet_type == JOB_PACKET) {
        uint16_t crc16_total = crc16_false(buf+2, data_len+2);
        buf[4+data_len] = (crc16_total >> 8) & 0xFF;
        buf[5+data_len] = crc16_total & 0xFF;
    } else {
        buf[4+data_len] = crc5(buf+2, data_len+2);
    }

    //send serial data
    SERIAL_send(buf, total_length, true);

    free(buf);
}

static void _send_simple(uint8_t * data, uint8_t total_length){
    unsigned char *buf = malloc(total_length);
   memcpy(buf, data, total_length);
    SERIAL_send(buf, total_length, true);

    free(buf);
}

static void _send_chain_inactive(void) {

    unsigned char read_address[2] = {0x00, 0x00};
    //send serial data
    _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_INACTIVE), read_address, 2, false);
}

static void _set_chip_address(uint8_t chipAddr) {

    unsigned char read_address[2] = {chipAddr, 0x00};
    //send serial data
    _send_BM1366((TYPE_CMD | GROUP_SINGLE | CMD_SETADDRESS), read_address, 2, false);
}

static unsigned char _reverse_bits(unsigned char num) {
    unsigned char reversed = 0;
    int i;

    for (i = 0; i < 8; i++) {
        reversed <<= 1;     // Left shift the reversed variable by 1
        reversed |= num & 1; // Use bitwise OR to set the rightmost bit of reversed to the current bit of num
        num >>= 1;          // Right shift num by 1 to get the next bit
    }

    return reversed;
}

static int _largest_power_of_two(int num) {
    int power = 0;

    while (num > 1) {
        num = num >> 1;
        power++;
    }

    return 1 << power;
}

// borrowed from cgminer driver-gekko.c calc_gsf_freq()
void BM1366_send_hash_frequency(float frequency) {

    unsigned char prefreq1[9] = {0x00, 0x70, 0x0F, 0x0F, 0x0F, 0x00}; //prefreq - pll0_divider

	// default 200Mhz if it fails
    unsigned char freqbuf[9] = {0x00, 0x08, 0x40, 0xA0, 0x02, 0x25}; //freqbuf - pll0_parameter

	float deffreq = 200.0;

	float fa, fb, fc1, fc2, newf;
	float f1, basef, famax = 0xf0, famin = 0x10;
	int i;

    //bound the frequency setting
    // You can go as low as 13 but it doesn't really scale or
    // produce any nonces
    if (frequency < 50) {
        f1 = 50;
    } else if (frequency > 500) {
        f1 = 500;
    } else {
        f1 = frequency;
    }


	fb = 2; fc1 = 1; fc2 = 5; // initial multiplier of 10
	if (f1 >= 500) {
		// halve down to '250-400'
		fb = 1;
	} else if (f1 <= 150) {
		// triple up to '300-450'
		fc1 = 3;
	} else if (f1 <= 250) {
		// double up to '300-500'
		fc1 = 2;
	}
	// else f1 is 250-500

	// f1 * fb * fc1 * fc2 is between 2500 and 5000
	// - so round up to the next 25 (freq_mult)
	basef = FREQ_MULT * ceil(f1 * fb * fc1 * fc2 / FREQ_MULT);

	// fa should be between 100 (0x64) and 200 (0xC8)
	fa = basef / FREQ_MULT;

	// code failure ... basef isn't 400 to 6000
	if (fa < famin || fa > famax) {
		newf = deffreq;
	} else {
		freqbuf[3] = (int)fa;
		freqbuf[4] = (int)fb;
		// fc1, fc2 'should' already be 1..15
		freqbuf[5] = (((int)fc1 & 0xf) << 4) + ((int)fc2 & 0xf);

		newf = basef / ((float)fb * (float)fc1 * (float)fc2);
	}

	for (i = 0; i < 2; i++) {
        vTaskDelay(10 / portTICK_RATE_MS);
        _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), prefreq1, 6, false);
	}
	for (i = 0; i < 2; i++) {
        vTaskDelay(10 / portTICK_RATE_MS);
        _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), freqbuf, 6, false);
	}

    vTaskDelay(10 / portTICK_RATE_MS);

    ESP_LOGI(TAG, "Setting Frequency to %.2fMHz (%.2f)", frequency, newf);

}

static void _send_init(u_int64_t frequency) {

    //send serial data
    vTaskDelay(SLEEP_TIME / portTICK_RATE_MS);
    // _send_chain_inactive();

    // _set_chip_address(0x00);

    // unsigned char init[6] = {0x00, CLOCK_ORDER_CONTROL_0, 0x00, 0x00, 0x00, 0x00}; //init1 - clock_order_control0
    // _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), init, 6, false);

    // unsigned char init2[6] = {0x00, CLOCK_ORDER_CONTROL_1, 0x00, 0x00, 0x00, 0x00}; //init2 - clock_order_control1
    // _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), init2, 6, false);

    // unsigned char init3[9] = {0x00, ORDERED_CLOCK_ENABLE, 0x00, 0x00, 0x00, 0x01}; //init3 - ordered_clock_enable
    // _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), init3, 6, false);

    // unsigned char init4[9] = {0x00, CORE_REGISTER_CONTROL, 0x80, 0x00, 0x80, 0x74}; //init4 - init_4_?
    // _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), init4, 6, false);

    // BM1366_set_job_difficulty_mask(256);

    // unsigned char init5[9] = {0x00, PLL3_PARAMETER, 0xC0, 0x70, 0x01, 0x11}; //init5 - pll3_parameter
    // _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), init5, 6, false);

    // unsigned char init6[9] = {0x00, FAST_UART_CONFIGURATION, 0x06, 0x00, 0x00, 0x0F}; //init6 - fast_uart_configuration
    // _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), init6, 6, false);

    // BM1366_set_default_baud();

    // BM1366_send_hash_frequency(frequency);

    //     Opened USB device 0403:6001
    // ->55 AA 51 09 00 A4 90 00 FF FF 1C
    // ->55 AA 51 09 00 A4 90 00 FF FF 1C
    // ->55 AA 51 09 00 A4 90 00 FF FF 1C
    // ->55 AA 52 05 00 00 0A
    // <-AA 55 13 66 00 00 00 00 00 00 05
    // cmd packet
    // ->55 AA 51 09 00 A8 00 07 00 00 03
    // ->55 AA 51 09 00 18 FF 0F C1 00 00
    // ->55 AA 53 05 00 00 03
    // ->55 AA 40 05 00 00 1C
    // ->55 AA 51 09 00 3C 80 00 85 40 0C
    // ->55 AA 21 56 28 01 00 00 00 00 F8 95 24 19 E0 16 AF 64 F3 C0 09 10 CB DA F2 3F 0F FF DF 3F 7B 42 AC C9 8D 70 ED 9B BF BA 1B 1D 44 74 ED E0 32 28 85 C7 00 00 00 00 0C 00 00 00 65 90 49 10 58 57 DD A2 47 8A 5E 7C CC FE 03 6F E8 BE 8E F8 C2 57 21 D4 00 00 00 20 3C 3F
    // <-AA 55 08 39 28 A9 03 2B 00 03 96
    // response packet
    // <-AA 55 1A 51 4E BE 01 28 00 00 8A
    // response packet
    // <-AA 55 30 70 1A 0A 00 28 00 00 96
    // response packet
    // <-AA 55 42 72 D2 DF 04 29 00 01 99
    // response packet
    // <-AA 55 56 B3 5F 2B 03 29 00 01 92
    // response packet
    // <-AA 55 43 91 70 1E 00 2B 00 03 9F
    // response packet

    unsigned char init[11] =  {0x55, 0xAA, 0x51, 0x09, 0x00, 0xA4, 0x90, 0x00, 0xFF, 0xFF, 0x1C};
    _send_simple(init, 11);
     _send_simple(init, 11);
     _send_simple(init, 11);

    unsigned char readRegister[7] = { 0x55, 0xAA, 0x52, 0x05, 0x00, 0x00, 0x0A};
     _send_simple(readRegister, 7);

     int received = SERIAL_rx(asic_response_buffer, 11, 10000);


       printf("<-");
        prettyHex((unsigned char*)asic_response_buffer, received);
        printf("\n");

     unsigned char init2[11] =  {0x55, 0xAA, 0x51, 0x09, 0x00, 0xA8, 0x00, 0x07, 0x00, 0x00, 0x03};
     _send_simple(init2, 11);

     unsigned char init3[11] =  {0x55, 0xAA, 0x51, 0x09, 0x00, 0x18, 0xFF, 0x0F, 0xC1, 0x00, 0x00 };
     _send_simple(init3, 11);

     unsigned char init4[7] =  { 0x55, 0xAA, 0x53, 0x05, 0x00, 0x00, 0x03 };
     _send_simple(init4, 7);

     unsigned char init5[7] =  { 0x55, 0xAA, 0x40, 0x05, 0x00, 0x00, 0x1C };
     _send_simple(init5, 7);

      unsigned char init6[11] =  { 0x55, 0xAA, 0x51, 0x09, 0x00, 0x3C, 0x80, 0x00, 0x85, 0x40, 0x0C };
     _send_simple(init6, 11);

}



//reset the BM1366 via the RTS line
static void _reset(void) {
    gpio_set_level(BM1366_RST_PIN, 0);

    //delay for 100ms
    vTaskDelay(100 / portTICK_RATE_MS);

    //set the gpio pin high
    gpio_set_level(BM1366_RST_PIN, 1);

    //delay for 100ms
    vTaskDelay(100 / portTICK_RATE_MS);

}


static void _send_read_address(void) {

    unsigned char read_address[2] = {0x00, 0x00};
    //send serial data
    _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_READ), read_address, 2, false);
}


void BM1366_init(u_int64_t frequency) {
    ESP_LOGI(TAG, "Initializing BM1366");

    memset(asic_response_buffer, 0, 1024);

    gpio_pad_select_gpio(BM1366_RST_PIN);
    gpio_set_direction(BM1366_RST_PIN, GPIO_MODE_OUTPUT);

    //reset the bm1366
    _reset();

    //send the init command
    //_send_read_address();

    _send_init(frequency);


}





// Baud formula = 25M/((denominator+1)*8)
// The denominator is 5 bits found in the misc_control (bits 9-13)
int BM1366_set_default_baud(void){
    //default divider of 26 (11010) for 115,749
    unsigned char baudrate[9] = {0x00, MISC_CONTROL, 0x00, 0x00, 0b01111010, 0b00110001}; //baudrate - misc_control
    _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6, false);
    return 115749;
}

int BM1366_set_max_baud(void){

    return 115749;

    // divider of 0 for 3,125,000
    ESP_LOGI(TAG, "Setting max baud of 3125000");
    unsigned char baudrate[9] = { 0x00, MISC_CONTROL, 0x00, 0x00, 0b01100000, 0b00110001 };; //baudrate - misc_control
    _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), baudrate, 6, false);
    return 3125000;
}

void BM1366_set_job_difficulty_mask(int difficulty){

    return;

    // Default mask of 256 diff
    unsigned char job_difficulty_mask[9] = {0x00, TICKET_MASK, 0b00000000, 0b00000000, 0b00000000, 0b11111111};

    // The mask must be a power of 2 so there are no holes
    // Correct:  {0b00000000, 0b00000000, 0b11111111, 0b11111111}
    // Incorrect: {0b00000000, 0b00000000, 0b11100111, 0b11111111}
    difficulty = _largest_power_of_two(difficulty) -1; // (difficulty - 1) if it is a pow 2 then step down to second largest for more hashrate sampling

    // convert difficulty into char array
    // Ex: 256 = {0b00000000, 0b00000000, 0b00000000, 0b11111111}, {0x00, 0x00, 0x00, 0xff}
    // Ex: 512 = {0b00000000, 0b00000000, 0b00000001, 0b11111111}, {0x00, 0x00, 0x01, 0xff}
     for (int i = 0; i < 4; i++) {
        char value = (difficulty >> (8 * i)) & 0xFF;
        //The char is read in backwards to the register so we need to reverse them
        //So a mask of 512 looks like 0b00000000 00000000 00000001 1111111
        //and not 0b00000000 00000000 10000000 1111111

        job_difficulty_mask[5 - i] = _reverse_bits(value);
    }

    ESP_LOGI(TAG, "Setting job ASIC mask to %d", difficulty);

    _send_BM1366((TYPE_CMD | GROUP_ALL | CMD_WRITE), job_difficulty_mask, 6, false);
}



static uint8_t id = 0;

void BM1366_send_work(void *pvParameters, bm_job *next_bm_job) {
    // unsigned char faux_job[] = {0x28, 0x01, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x95, 0x24, 0x19, 0xE0, 0x16, 0xAF, 0x64, 0xF3, 0xC0, 0x09, 0x10, 0xCB, 0xDA, 0xF2, 0x3F, 0x0F, 0xFF, 0xDF, 0x3F, 0x7B, 0x42, 0xAC, 0xC9, 0x8D, 0x70, 0xED, 0x9B, 0xBF, 0xBA, 0x1B, 0x1D, 0x44, 0x74, 0xED, 0xE0, 0x32, 0x28, 0x85, 0xC7, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x65, 0x90, 0x49, 0x10, 0x58, 0x57, 0xDD, 0xA2, 0x47, 0x8A, 0x5E, 0x7C, 0xCC, 0xFE, 0x03, 0x6F, 0xE8, 0xBE, 0x8E, 0xF8, 0xC2, 0x57, 0x21, 0xD4, 0x00, 0x00, 0x00, 0x20}; //init5

    // _send_BM1366( (TYPE_JOB | GROUP_SINGLE | CMD_WRITE), faux_job, 82, false);

    GlobalState *GLOBAL_STATE = (GlobalState*) pvParameters;

    BM1366_job job;
    id = (id + 4) % 128;
    job.job_id = id;
    job.num_midstates = 0x01;
    memcpy(&job.starting_nonce, &next_bm_job->starting_nonce, 4);
    memcpy(&job.nbits, &next_bm_job->target, 4);
    memcpy(&job.ntime, &next_bm_job->ntime, 4);
    memcpy(job.merkle_root, next_bm_job->midstate1, 32);
    memcpy(job.prev_block_hash, next_bm_job->prev_block_hash, 32);
    memcpy(&job.version, &next_bm_job->version, 4);



    GLOBAL_STATE->ASIC_TASK_MODULE.active_jobs[job.job_id] = next_bm_job;

    pthread_mutex_lock(&GLOBAL_STATE->valid_jobs_lock);
    GLOBAL_STATE-> valid_jobs[job.job_id] = 1;
    ESP_LOGI(TAG, "Added Job: %i", job.job_id);
    pthread_mutex_unlock(&GLOBAL_STATE->valid_jobs_lock);

    _send_BM1366((TYPE_JOB | GROUP_SINGLE | CMD_WRITE), &job, sizeof(BM1366_job), false);
}




asic_result * BM1366_receive_work(void){
        ESP_LOGI(TAG, "Receiving work");

        //wait for a response, wait time is pretty arbitrary
        int received = SERIAL_rx(asic_response_buffer, 11, 60000);


       printf("<-");
        prettyHex((unsigned char*)asic_response_buffer, received);
        printf("\n");

        if (received < 0) {
            ESP_LOGI(TAG, "Error in serial RX");
            return NULL;
        } else if(received == 0){
            // Didn't find a solution, restart and try again
            return NULL;
        }

        if(received != 11 || asic_response_buffer[0] != 0xAA || asic_response_buffer[1] != 0x55){
            ESP_LOGI(TAG, "Serial RX invalid %i", received);
            ESP_LOG_BUFFER_HEX(TAG, asic_response_buffer, received);
            return NULL;
        }


        memcpy((void *) &_asic_result, asic_response_buffer, sizeof(asic_result));

        return &_asic_result;


}


