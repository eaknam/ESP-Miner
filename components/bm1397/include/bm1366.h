#ifndef BM1366_H_
#define BM1366_H_

#include "driver/gpio.h"
#include "bm1397.h"


#define CRC5_MASK 0x1F

static const u_int64_t BM1366_FREQUENCY = CONFIG_BM1397_FREQUENCY;
static const u_int64_t BM1366_CORE_COUNT = 672;
static const u_int64_t BM1366_HASHRATE_S = BM1366_FREQUENCY * BM1366_CORE_COUNT * 1000000;
//2^32
//static const u_int64_t NONCE_SPACE = 4294967296;
static const double  BM1366_FULLSCAN_MS = ((double)NONCE_SPACE / (double)BM1366_HASHRATE_S) * 1000;

typedef struct {
  float frequency;
} bm1366Module;

typedef struct __attribute__((__packed__))  {
  uint8_t job_id;
  uint8_t num_midstates;
  uint8_t starting_nonce[4];
  uint8_t nbits[4];
  uint8_t ntime[4];
  uint8_t merkle_root[32];
  uint8_t prev_block_hash[32];
  uint8_t version[4];
} BM1366_job;


void BM1366_init(u_int64_t frequency);

void BM1366_send_init(void);
void BM1366_send_work(void *GLOBAL_STATE, bm_job *next_bm_job);
void BM1366_set_job_difficulty_mask(int);
int BM1366_set_max_baud(void);
int BM1366_set_default_baud(void);
void BM1366_send_hash_frequency(float frequency);
asic_result * BM1366_receive_work(void);

#endif /* BM1366_H_ */