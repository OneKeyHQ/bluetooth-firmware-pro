#ifndef __NORDIC_52832_NFC_
#define __NORDIC_52832_NFC_

enum
{
    READSTATE_IDLE,
    READSTATE_READ_INFO,
    READSTATE_READ_DATA,
};

enum
{
    NFCSTATE_IDLE,
    NFCSTATE_READ_INFO,
    NFCSTATE_READ_DATA,
};

typedef struct
{
    uint8_t* source;
    uint32_t br;
    uint32_t bw;
    uint32_t btoRead;
    uint32_t length;
} ringbuffer_t;

extern bool data_recived_flag;
extern uint8_t data_recived_buf[APDU_BUFF_SIZE];
extern uint16_t data_recived_len;

bool i2c_master_write(uint8_t* buf, uint32_t len);
bool i2c_master_read(void);

int nfc_init(void);
void nfc_disable(void);
void read_st_resp_data(void);
int twi_master_init(void);
void nfc_poll(void* p_event_data, uint16_t event_size);
#endif
