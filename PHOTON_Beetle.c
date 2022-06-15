/*
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "platform.h"
#include "xil_printf.h"

#include "api.h"
#include "beetle.h"
#include "photon.h"

#include "xparameters.h"
#include "xtime_l.h"
#define TIMER_LOAD_VALUE 0XFFFFFFFF

XTime time;
unsigned int cycles1, cycles2;

/* Declaration of basic internal functions */
static uint8_t selectConst(
	const bool condition1,
	const bool condition2,
	const uint8_t option1,
	const uint8_t option2,
	const uint8_t option3,
	const uint8_t option4);

static void concatenate(
	uint8_t* out,
	const uint8_t* in_left, const size_t leftlen_inbytes,
	const uint8_t* in_right, const size_t rightlen_inbytes);

static void XOR(
	uint8_t* out,
	const uint8_t* in_left,
	const uint8_t* in_right,
	const size_t iolen_inbytes);

static void XOR_const(
	uint8_t* State_inout,
	const uint8_t  Constant);

static void ROTR1(
	uint8_t* out,
	const uint8_t* in,
	const size_t iolen_inbytes);

static void ShuffleXOR(
	uint8_t* DataBlock_out,
	const uint8_t* OuterState_in,
	const uint8_t* DataBlock_in,
	const size_t DBlen_inbytes);

static void rhoohr(
	uint8_t* OuterState_inout,
	uint8_t* DataBlock_out,
	const uint8_t* DataBlock_in,
	const size_t DBlen_inbytes,
	const uint32_t EncDecInd);

static void HASH(
	uint8_t* State_inout,
	const uint8_t* Data_in,
	const uint64_t Dlen_inbytes,
	const uint8_t  Constant);

static void ENCorDEC(
	uint8_t* State_inout,
	uint8_t* Data_out,
	const uint8_t* Data_in,
	const uint64_t Dlen_inbytes,
	const uint8_t Constant,
	const uint32_t EncDecInd);

static void TAG(
	uint8_t* Tag_out,
	uint8_t* State);

/* Definition of basic internal functions */
static uint8_t selectConst(
	const bool condition1,
	const bool condition2,
	const uint8_t option1,
	const uint8_t option2,
	const uint8_t option3,
	const uint8_t option4)
{
	if (condition1 && condition2) return option1;
	if (condition1) return option2;
	if (condition2) return option3;
	return option4;
}

static void concatenate(
	uint8_t* out,
	const uint8_t* in_left, const size_t leftlen_inbytes,
	const uint8_t* in_right, const size_t rightlen_inbytes)
{
	memcpy(out, in_left, leftlen_inbytes);
	memcpy(out + leftlen_inbytes, in_right, rightlen_inbytes);
}

static void XOR(
	uint8_t* out,
	const uint8_t* in_left,
	const uint8_t* in_right,
	const size_t iolen_inbytes)
{
	size_t i;
	for (i = 0; i < iolen_inbytes; i++) out[i] = in_left[i] ^ in_right[i];
}

static void XOR_const(
	uint8_t* State_inout,
	const uint8_t  Constant)
{
	State_inout[STATE_INBYTES - 1] ^= (Constant << LAST_THREE_BITS_OFFSET);
}

static void ROTR1(
	uint8_t* out,
	const uint8_t* in,
	const size_t iolen_inbytes)
{
	uint8_t tmp = in[0];
	size_t i;
	for (i = 0; i < iolen_inbytes - 1; i++)
	{
		out[i] = (in[i] >> 1) | ((in[(i + 1)] & 1) << 7);
	}
	out[iolen_inbytes - 1] = (in[i] >> 1) | ((tmp & 1) << 7);
}

static void ShuffleXOR(
	uint8_t* DataBlock_out,
	const uint8_t* OuterState_in,
	const uint8_t* DataBlock_in,
	const size_t DBlen_inbytes)
{
	const uint8_t* OuterState_part1 = OuterState_in;
	const uint8_t* OuterState_part2 = OuterState_in + RATE_INBYTES / 2;

	uint8_t OuterState_part1_ROTR1[RATE_INBYTES / 2] = { 0 };
	size_t i;

	ROTR1(OuterState_part1_ROTR1, OuterState_part1, RATE_INBYTES / 2);

	i = 0;
	while ((i < DBlen_inbytes) && (i < RATE_INBYTES / 2))
	{
		DataBlock_out[i] = OuterState_part2[i] ^ DataBlock_in[i];
		i++;
	}
	while (i < DBlen_inbytes)
	{
		DataBlock_out[i] = OuterState_part1_ROTR1[i - RATE_INBYTES / 2] ^ DataBlock_in[i];
		i++;
	}
}

static void rhoohr(
	uint8_t* OuterState_inout,
	uint8_t* DataBlock_out,
	const uint8_t* DataBlock_in,
	const size_t DBlen_inbytes,
	const uint32_t EncDecInd)
{
	ShuffleXOR(DataBlock_out, OuterState_inout, DataBlock_in, DBlen_inbytes);

	if (EncDecInd == ENC)
	{
		XOR(OuterState_inout, OuterState_inout, DataBlock_in, DBlen_inbytes);
	}
	else
	{
		XOR(OuterState_inout, OuterState_inout, DataBlock_out, DBlen_inbytes);
	}
}

static void HASH(
	uint8_t* State_inout,
	const uint8_t* Data_in,
	const uint64_t Dlen_inbytes,
	const uint8_t  Constant)
{
	uint8_t* State = State_inout;
	size_t Dlen_inblocks = (Dlen_inbytes + RATE_INBYTES - 1) / RATE_INBYTES;
	size_t LastDBlocklen;
	size_t i;

	for (i = 0; i < Dlen_inblocks - 1; i++)
	{
		PHOTON_Permutation(State);
		XOR(State, State, Data_in + i * RATE_INBYTES, RATE_INBYTES);
	}
	PHOTON_Permutation(State);
	LastDBlocklen = Dlen_inbytes - i * RATE_INBYTES;
	XOR(State, State, Data_in + i * RATE_INBYTES, LastDBlocklen);
	if (LastDBlocklen < RATE_INBYTES) State[LastDBlocklen] ^= 0x01; // ozs

	XOR_const(State, Constant);
}

static void ENCorDEC(
	uint8_t* State_inout,
	uint8_t* Data_out,
	const uint8_t* Data_in,
	const uint64_t Dlen_inbytes,
	const uint8_t Constant,
	const uint32_t EncDecInd)
{
	uint8_t* State = State_inout;
	size_t Dlen_inblocks = (Dlen_inbytes + RATE_INBYTES - 1) / RATE_INBYTES;
	size_t LastDBlocklen;
	size_t i;

	for (i = 0; i < Dlen_inblocks - 1; i++)
	{
		PHOTON_Permutation(State);
		rhoohr(State, Data_out + i * RATE_INBYTES, Data_in + i * RATE_INBYTES, RATE_INBYTES, EncDecInd);
	}
	PHOTON_Permutation(State);
	LastDBlocklen = Dlen_inbytes - i * RATE_INBYTES;
	rhoohr(State, Data_out + i * RATE_INBYTES, Data_in + i * RATE_INBYTES, LastDBlocklen, EncDecInd);
	if (LastDBlocklen < RATE_INBYTES) State[LastDBlocklen] ^= 0x01; // ozs

	XOR_const(State, Constant);
}

static void TAG(
	uint8_t* Tag_out,
	uint8_t* State)
{
	size_t i;

	i = TAG_INBYTES;
	while (i > SQUEEZE_RATE_INBYTES)
	{
		PHOTON_Permutation(State);
		memcpy(Tag_out, State, SQUEEZE_RATE_INBYTES);
		Tag_out += SQUEEZE_RATE_INBYTES;
		i -= SQUEEZE_RATE_INBYTES;
	}
	PHOTON_Permutation(State);
	memcpy(Tag_out, State, i);
}

int crypto_aead_encrypt(
	unsigned char* c, unsigned long long* clen,
	const unsigned char* m, unsigned long long mlen,
	const unsigned char* ad, unsigned long long adlen,
	const unsigned char* nsec,
	const unsigned char* npub,
	const unsigned char* k
)
{
	/*
	...
	... the code for the cipher implementation goes here,
	... generating a ciphertext c[0],c[1],...,c[*clen-1]
	... from a plaintext m[0],m[1],...,m[mlen-1]
	... and associated data ad[0],ad[1],...,ad[adlen-1]
	... and nonce npub[0],npub[1],..
	... and secret key k[0],k[1],...
	... the implementation shall not use nsec
	...
	... return 0;
	*/
	uint8_t* C = c;
	uint8_t* T = c + mlen;
	const uint8_t* M = m;
	const uint8_t* A = ad;
	const uint8_t* N = npub;
	const uint8_t* K = k;

	uint8_t State[STATE_INBYTES] = { 0 };
	uint8_t c0;
	uint8_t c1;

	(void)nsec;

	concatenate(State, N, NOUNCE_INBYTES, K, KEY_INBYTES);

	if ((adlen == 0) && (mlen == 0))
	{
		XOR_const(State, 1);
		TAG(T, State);
		*clen = TAG_INBYTES;
		return 0;
	}

	c0 = selectConst((mlen != 0), ((adlen % RATE_INBYTES) == 0), 1, 2, 3, 4);
	c1 = selectConst((adlen != 0), ((mlen % RATE_INBYTES) == 0), 1, 2, 5, 6);

	if (adlen != 0) HASH(State, A, adlen, c0);
	if (mlen != 0) ENCorDEC(State, C, M, mlen, c1, ENC);

	TAG(T, State);
	//*clen = mlen + TAG_INBYTES;
	return 0;
}

int main()
{
    init_platform();


    // Default parameters (32 bytes/256 bits)
    unsigned char ciphertext[97] = "";
    unsigned long long ciphertex_len = 48;
    const unsigned char message[65] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    unsigned long long message_len = 32;


    /*
    // Short message size (16 bytes/128 bits)
    unsigned char ciphertext[65] = "";
    uint64_t ciphertex_len = 32;
    const unsigned char message[33] = "000102030405060708090A0B0C0D0E0F";
    uint64_t message_len = 16;
	*/

    /*
    // Medium message size (64 bytes/512 bits)
    unsigned char ciphertext[161] = "";
    uint64_t ciphertex_len = 80;
    const unsigned char message[129] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    uint64_t message_len = 64;
	*/

    /*
    // Long message size (1536 bytes/12288 bits)
    unsigned char ciphertext[3105] = "";
    uint64_t ciphertex_len = 1552;
    const unsigned char message[3073] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    uint64_t message_len = 1536;
	*/

    const unsigned char associated_data[65] = "000102030405060708090A0B0C0D0E0F101112131415161718191A1B1C1D1E1F";
    unsigned long long adata_len = 32;
    const unsigned char nsec1[1] = "";
    const unsigned char nonce_public[33] = "000102030405060708090A0B0C0D0E0F";
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
