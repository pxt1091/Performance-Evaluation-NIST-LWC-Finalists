/*
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "api.h"
#include "grain128aead.h"

#include "platform.h"
#include "xil_printf.h"

#include "xparameters.h"
#include "xtime_l.h"
#define TIMER_LOAD_VALUE 0XFFFFFFFF

XTime time;
unsigned int cycles1, cycles2;

unsigned char grain_round;

unsigned char swapsb(unsigned char n);

void init_grain(grain_state* grain, const unsigned char* key, const unsigned char* iv)
{
	unsigned char key_bits[128];
	unsigned char iv_bits[96];

	// expand the packed bytes and place one bit per array cell (like a flip flop in HW)
	for (int i = 0; i < 16; i++) {
		for (int j = 0; j < 8; j++) {
			key_bits[8 * i + j] = (key[i] & (1 << (7 - j))) >> (7 - j);
		}
	}

	for (int i = 0; i < 12; i++) {
		for (int j = 0; j < 8; j++) {
			iv_bits[8 * i + j] = (iv[i] & (1 << (7 - j))) >> (7 - j);
		}
	}

	/* set up LFSR */
	for (int i = 0; i < 96; i++) {
		grain->lfsr[i] = iv_bits[i];
	}

	for (int i = 96; i < 127; i++) {
		grain->lfsr[i] = 1;
	}

	grain->lfsr[127] = 0;

	/* set up NFSR */
	for (int i = 0; i < 128; i++) {
		grain->nfsr[i] = key_bits[i];
	}

	for (int i = 0; i < 64; i++) {
		grain->auth_acc[i] = 0;
		grain->auth_sr[i] = 0;
	}

	/* initialize grain and skip output */
	grain_round = INIT;
	for (int i = 0; i < 320; i++) {
		next_z(grain, 0, 0);
	}

	grain_round = ADDKEY;

	/* re-introduce the key into LFSR and NFSR in parallel during the next 64 clocks */
	for (int i = 0; i < 64; i++) {
		unsigned char addkey_0 = key_bits[i];
		unsigned char addkey_64 = key_bits[64 + i];
		next_z(grain, addkey_0, addkey_64);
	}

	grain_round = NORMAL;

	/* inititalize the accumulator and shift register */
	for (int i = 0; i < 64; i++) {
		grain->auth_acc[i] = next_z(grain, 0, 0);
	}

	for (int i = 0; i < 64; i++) {
		grain->auth_sr[i] = next_z(grain, 0, 0);
	}
}

void init_data(grain_data* data, const unsigned char* msg, unsigned long long msg_len)
{
	// allocate enough space for message, including the padding bit 1 (byte 0x80)
	data->message = (unsigned char*)calloc(8 * msg_len + 1, 1);
	data->msg_len = 8 * msg_len + 1;
	for (unsigned long long i = 0; i < msg_len; i++) {
		for (int j = 0; j < 8; j++) {
			data->message[8 * i + j] = (msg[i] & (1 << (7 - j))) >> (7 - j);
		}
	}

	// always pad data with the bit 1 (byte 0x80)
	data->message[data->msg_len - 1] = 1;
}

unsigned char next_lfsr_fb(grain_state* grain)
{
	/* f(x) = 1 + x^32 + x^47 + x^58 + x^90 + x^121 + x^128 */
	return grain->lfsr[96] ^ grain->lfsr[81] ^ grain->lfsr[70] ^ grain->lfsr[38] ^ grain->lfsr[7] ^ grain->lfsr[0];
}

unsigned char next_nfsr_fb(grain_state* grain)
{
	return grain->nfsr[96] ^ grain->nfsr[91] ^ grain->nfsr[56] ^ grain->nfsr[26] ^ grain->nfsr[0] ^ (grain->nfsr[84] & grain->nfsr[68]) ^
		(grain->nfsr[67] & grain->nfsr[3]) ^ (grain->nfsr[65] & grain->nfsr[61]) ^ (grain->nfsr[59] & grain->nfsr[27]) ^
		(grain->nfsr[48] & grain->nfsr[40]) ^ (grain->nfsr[18] & grain->nfsr[17]) ^ (grain->nfsr[13] & grain->nfsr[11]) ^
		(grain->nfsr[82] & grain->nfsr[78] & grain->nfsr[70]) ^ (grain->nfsr[25] & grain->nfsr[24] & grain->nfsr[22]) ^
		(grain->nfsr[95] & grain->nfsr[93] & grain->nfsr[92] & grain->nfsr[88]);
}

unsigned char next_h(grain_state* grain)
{
	// h(x) = x0x1 + x2x3 + x4x5 + x6x7 + x0x4x8
#define x0 grain->nfsr[12]	// bi+12
#define x1 grain->lfsr[8]		// si+8
#define x2 grain->lfsr[13]	// si+13
#define x3 grain->lfsr[20]	// si+20
#define x4 grain->nfsr[95]	// bi+95
#define x5 grain->lfsr[42]	// si+42
#define x6 grain->lfsr[60]	// si+60
#define x7 grain->lfsr[79]	// si+79
#define x8 grain->lfsr[94]	// si+94

	unsigned char h_out = (x0 & x1) ^ (x2 & x3) ^ (x4 & x5) ^ (x6 & x7) ^ (x0 & x4 & x8);
	return h_out;
}

unsigned char shift(unsigned char fsr[128], unsigned char fb)
{
	unsigned char out = fsr[0];
	for (int i = 0; i < 127; i++) {
		fsr[i] = fsr[i + 1];
	}
	fsr[127] = fb;

	return out;
}

void auth_shift(unsigned char sr[64], unsigned char fb)
{
	for (int i = 0; i < 63; i++) {
		sr[i] = sr[i + 1];
	}
	sr[63] = fb;
}

void accumulate(grain_state* grain)
{
	for (int i = 0; i < 64; i++) {
		grain->auth_acc[i] ^= grain->auth_sr[i];
	}
}

unsigned char next_z(grain_state* grain, unsigned char keybit, unsigned char keybit_64)
{
	unsigned char lfsr_fb = next_lfsr_fb(grain);
	unsigned char nfsr_fb = next_nfsr_fb(grain);
	unsigned char h_out = next_h(grain);

	/* y = h + s_{i+93} + sum(b_{i+j}), j \in A */
	unsigned char A[] = { 2, 15, 36, 45, 64, 73, 89 };

	unsigned char nfsr_tmp = 0;
	for (int i = 0; i < 7; i++) {
		nfsr_tmp ^= grain->nfsr[A[i]];
	}

	unsigned char y = h_out ^ grain->lfsr[93] ^ nfsr_tmp;

	unsigned char lfsr_out;

	/* feedback y if we are in the initialization instance */
	if (grain_round == INIT) {
		lfsr_out = shift(grain->lfsr, lfsr_fb ^ y);
		shift(grain->nfsr, nfsr_fb ^ lfsr_out ^ y);
	}
	else if (grain_round == ADDKEY) {
		lfsr_out = shift(grain->lfsr, lfsr_fb ^ y ^ keybit_64);
		shift(grain->nfsr, nfsr_fb ^ lfsr_out ^ y ^ keybit);
	}
	else if (grain_round == NORMAL) {
		lfsr_out = shift(grain->lfsr, lfsr_fb);
		shift(grain->nfsr, nfsr_fb ^ lfsr_out);
	}

	return y;
}

int encode_der(unsigned long long len, unsigned char** der)
{
	unsigned long long len_tmp;
	int der_len = 0;

	if (len < 128) {
		*der = (unsigned char*)malloc(1);
		(*der)[0] = swapsb((unsigned char)len);
		return 1;
	}

	len_tmp = len;
	do {
		len_tmp >>= 8;
		der_len++;
	} while (len_tmp != 0);

	// one extra byte to describe the number of bytes used
	*der = (unsigned char*)malloc(der_len + 1);
	(*der)[0] = swapsb(0x80 | der_len);

	len_tmp = len;
	for (int i = der_len; i > 0; i--) {
		(*der)[i] = swapsb(len_tmp & 0xff);
		len_tmp >>= 8;
	}

	return der_len + 1;
}

unsigned char swapsb(unsigned char n)
{
	// swaps significant bit
	unsigned char val = 0;
	for (int i = 0; i < 8; i++) {
		val |= ((n >> i) & 1) << (7 - i);
	}
	return val;
}


int crypto_aead_encrypt(unsigned char* c, unsigned long long* clen,
	const unsigned char* mp, unsigned long long mlen,
	const unsigned char* adp, unsigned long long adlen,
	const unsigned char* nsec,
	const unsigned char* npubp,
	const unsigned char* kp
)
{
	/* This implementation assumes that the most significant bit is processed first,
	 * in a byte. The optimized version however, processes the lsb first.
	 * In order to give the same test vectors, we here change the interpretation of the bits.
	 */
	unsigned char* m = (unsigned char*)malloc(mlen);
	unsigned char* ad = (unsigned char*)malloc(adlen);
	unsigned char npub[12];
	unsigned char k[16];

	for (unsigned long long i = 0; i < mlen; i++) {
		m[i] = swapsb(mp[i]);
	}
	for (unsigned long long i = 0; i < adlen; i++) {
		ad[i] = swapsb(adp[i]);
	}
	for (unsigned long long i = 0; i < 12; i++) {
		npub[i] = swapsb(npubp[i]);
	}
	for (unsigned long long i = 0; i < 16; i++) {
		k[i] = swapsb(kp[i]);
	}

	grain_state grain;
	grain_data data;

	init_grain(&grain, k, npub);
	init_data(&data, m, mlen);

	//*clen = 0;

	// authenticate adlen by prepeding to ad, using DER encoding
	unsigned char* ader;
	int aderlen = encode_der(adlen, &ader);
	// append ad to buffer
	ader = (unsigned char*)realloc(ader, aderlen + adlen);
	memcpy(ader + aderlen, ad, adlen);

	unsigned long long ad_cnt = 0;
	unsigned char adval = 0;

	/* accumulate tag for associated data only */
	for (unsigned long long i = 0; i < aderlen + adlen; i++) {
		/* every second bit is used for keystream, the others for MAC */
		for (int j = 0; j < 16; j++) {
			unsigned char z_next = next_z(&grain, 0, 0);
			if (j % 2 == 0) {
				// do not encrypt
			}
			else {
				adval = ader[ad_cnt / 8] & (1 << (7 - (ad_cnt % 8)));
				if (adval) {
					accumulate(&grain);
				}
				auth_shift(grain.auth_sr, z_next);
				ad_cnt++;
			}
		}
	}

	free(ader);

	unsigned long long ac_cnt = 0;
	unsigned long long m_cnt = 0;
	unsigned long long c_cnt = 0;
	unsigned char cc = 0;

	// generate keystream for message
	for (unsigned long long i = 0; i < mlen; i++) {
		// every second bit is used for keystream, the others for MAC
		cc = 0;
		for (int j = 0; j < 16; j++) {
			unsigned char z_next = next_z(&grain, 0, 0);
			if (j % 2 == 0) {
				// transform it back to 8 bits per byte
				cc |= (data.message[m_cnt++] ^ z_next) << (7 - (c_cnt % 8));
				c_cnt++;
			}
			else {
				if (data.message[ac_cnt++] == 1) {
					accumulate(&grain);
				}
				auth_shift(grain.auth_sr, z_next);
			}
		}
		c[i] = swapsb(cc);
		//*clen += 1;
	}

	// generate unused keystream bit
	next_z(&grain, 0, 0);
	// the 1 in the padding means accumulation
	accumulate(&grain);

	/* append MAC to ciphertext */
	unsigned long long acc_idx = 0;
	for (unsigned long long i = mlen; i < mlen + 8; i++) {
		unsigned char acc = 0;
		// transform back to 8 bits per byte
		for (int j = 0; j < 8; j++) {
			acc |= grain.auth_acc[8 * acc_idx + j] << (7 - j);
		}
		c[i] = swapsb(acc);
		acc_idx++;
		//*clen += 1;
	}

	free(data.message);
	free(m);
	free(ad);

	return 0;
}

int main()
{
    init_platform();

    // Default parameters (32 bytes/256 bits)
    unsigned char ciphertext[81] = "";
    unsigned long long ciphertex_len = 40;
    const unsigned char message[65] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    unsigned long long message_len = 32;

    /*
    // Short message size (16 bytes/128 bits)
    unsigned char ciphertext[49] = "";
    uint64_t ciphertex_len = 24;
    const unsigned char message[33] = "000102030405060708090A0B0C0D0E0F";
    uint64_t message_len = 16;
	*/

    /*
    // Medium message size (64 bytes/512 bits)
    unsigned char ciphertext[145] = "";
    uint64_t ciphertex_len = 72;
    const unsigned char message[129] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    uint64_t message_len = 64;
	*/

    /*
    // Long message size (1536 bytes/12288 bits)
    unsigned char ciphertext[3089] = "";
    uint64_t ciphertex_len = 1544;
    const unsigned char message[3073] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    uint64_t message_len = 1536;
	*/

    const unsigned char associated_data[65] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    unsigned long long adata_len = 32;
    const unsigned char nsec1[1] = "";
    const unsigned char nonce_public[25] = "000102030405060708090A0B";
    const unsigned char key[33] = "000102030405060708090A0B0C0D0E0F";


    Xil_DCacheDisable();
    XTime_SetTime(0);

    crypto_aead_encrypt(ciphertext, (unsigned long long*)ciphertex_len, message, message_len, associated_data, adata_len, nsec1, nonce_public, key);

    XTime_GetTime(&time);
    cycles1 = time;

    //printf("CT: %s\n", ciphertext);
    xil_printf("\nClock cycles for encryption 1 time (cache disabled) = %d", cycles1);


    /*
    int loop;
    Xil_DCacheEnable();
    XTime_SetTime(0);

    for(loop=0; loop<100; loop++)
    	crypto_aead_encrypt(ciphertext, (unsigned long long*)ciphertex_len, message, message_len, associated_data, adata_len, nsec1, nonce_public, key);

    XTime_GetTime(&time);
    cycles2 = time;

    xil_printf("\nClock cycles for encryption 100 times (cache enabled) = %d", cycles2);
	*/

    cleanup_platform();
    return 0;
}
