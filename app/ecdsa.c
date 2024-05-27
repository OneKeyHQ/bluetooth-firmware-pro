#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_crypto.h"
#include "nrf_crypto_ecc.h"
#include "nrf_crypto_ecdsa.h"
#include "nrf_crypto_shared.h"
#include "nrf_crypto_error.h"

ret_code_t generate_ecdsa_keypair(uint8_t* pri_key, uint8_t* pubkey)
{
    ret_code_t err_code = NRF_SUCCESS;
    nrf_crypto_ecc_key_pair_generate_context_t context;

    size_t private_key_size = 32;
    size_t public_key_size = 64;

    uint8_t sk[32], pk[64];

    nrf_crypto_ecc_private_key_t private_key;
    nrf_crypto_ecc_public_key_t public_key;

    err_code =
        nrf_crypto_ecc_key_pair_generate(&context, &g_nrf_crypto_ecc_secp256k1_curve_info, &private_key, &public_key);
    if ( err_code != NRF_SUCCESS )
    {
        return err_code;
    }

    err_code = nrf_crypto_ecc_private_key_to_raw(&private_key, sk, &private_key_size);
    if ( err_code != NRF_SUCCESS )
    {
        return err_code;
    }

    err_code = nrf_crypto_ecc_public_key_to_raw(&public_key, pk, &public_key_size);
    if ( err_code != NRF_SUCCESS )
    {
        return err_code;
    }
    nrf_crypto_internal_swap_endian(pri_key, sk, 32);
    nrf_crypto_internal_double_swap_endian(pubkey, pk, 32);
    return NRF_SUCCESS;
}

ret_code_t sign_ecdsa(uint8_t* pri_key, uint8_t* hash, uint8_t* signature)
{
    ret_code_t err_code = NRF_SUCCESS;
    nrf_crypto_ecdsa_sign_context_t context;
    size_t signature_size = 64;
    nrf_crypto_ecc_private_key_t private_key;

    uint8_t sk[32], sign[64], hash1[32];

    nrf_crypto_internal_swap_endian(sk, pri_key, 32);
    nrf_crypto_internal_swap_endian(hash1, hash, 32);

    err_code = nrf_crypto_ecc_private_key_from_raw(&g_nrf_crypto_ecc_secp256k1_curve_info, &private_key, sk, 32);
    if ( err_code != NRF_SUCCESS )
    {
        return err_code;
    }

    err_code = nrf_crypto_ecdsa_sign(&context, &private_key, hash1, 32, sign, &signature_size);
    if ( err_code != NRF_SUCCESS )
    {
        return err_code;
    }
    nrf_crypto_internal_double_swap_endian(signature, sign, 32);
    return NRF_SUCCESS;
}

ret_code_t sign_ecdsa_msg(uint8_t* pri_key, uint8_t* msg, uint32_t msg_len, uint8_t* signature)
{
    ret_code_t err_code = NRF_SUCCESS;
    nrf_crypto_hash_context_t hash_context = {0};
    uint8_t hash[32];
    size_t hash_len = 32;

    err_code = nrf_crypto_hash_calculate(&hash_context, &g_nrf_crypto_hash_sha256_info, msg, msg_len, hash, &hash_len);
    if ( err_code != NRF_SUCCESS )
    {
        return err_code;
    }

    err_code = sign_ecdsa(pri_key, hash, signature);
    if ( err_code != NRF_SUCCESS )
    {
        return err_code;
    }
    return NRF_SUCCESS;
}
