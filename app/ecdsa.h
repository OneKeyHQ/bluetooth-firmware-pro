#ifndef __NORDIC_52832_ECDSA_
#define __NORDIC_52832_ECDSA_

#include "sdk_errors.h"

ret_code_t generate_ecdsa_keypair(uint8_t* pri_key, uint8_t* pubkey);
ret_code_t sign_ecdsa(uint8_t* pri_key, uint8_t* hash, uint8_t* signature);
ret_code_t sign_ecdsa_msg(uint8_t* pri_key, uint8_t* msg, uint32_t msg_len, uint8_t* signature);

#endif
