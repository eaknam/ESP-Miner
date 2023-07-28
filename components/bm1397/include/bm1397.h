#ifndef BM1397_H_
#define BM1397_H_

#include "driver/gpio.h"
#include "mining.h"

#define CRC5_MASK 0x1F


static const u_int64_t BM1397_FREQUENCY = CONFIG_BM1397_FREQUENCY;
static const u_int64_t BM1397_CORE_COUNT = 672;
static const u_int64_t BM1397_HASHRATE_S = BM1397_FREQUENCY * BM1397_CORE_COUNT * 1000000;
//2^32
static const u_int64_t NONCE_SPACE = 4294967296;
static const double  BM1397_FULLSCAN_MS = ((double)NONCE_SPACE / (double)BM1397_HASHRATE_S) * 1000;

typedef struct {
  float frequency;
} bm1397Module;

typedef enum {
  JOB_PACKET = 0,
  CMD_PACKET = 1,
} packet_type_t;

typedef enum {
  JOB_RESP = 0,
  CMD_RESP = 1,
} response_type_t;

typedef struct __attribute__((__packed__))  {
  uint8_t job_id;
  uint8_t num_midstates;
  uint8_t starting_nonce[4];
  uint8_t nbits[4];
  uint8_t ntime[4];
  uint8_t merkle4[4];
  uint8_t midstate[32];
  uint8_t midstate1[32];
  uint8_t midstate2[32];
  uint8_t midstate3[32];
} job_packet;

typedef struct __attribute__((__packed__)) {
    uint8_t preamble[2];
    uint32_t nonce;
    uint8_t midstate_num;
    uint8_t job_id;
    uint8_t crc;
} asic_result;

void BM1397_init(u_int64_t frequency);

void BM1397_send_work(void *GLOBAL_STATE, bm_job *next_bm_job);
void BM1397_set_job_difficulty_mask(int);
int BM1397_set_max_baud(void);
int BM1397_set_default_baud(void);
void BM1397_send_hash_frequency(float frequency);
asic_result * BM1397_receive_work(void);

#endif /* BM1397_H_ */