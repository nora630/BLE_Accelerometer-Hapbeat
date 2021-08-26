#include "adpcm.h"

/* Table of index changes */
int8_t IndexTable[16] = {
 -1, -1, -1, -1, 2, 4, 6, 8,
 -1, -1, -1, -1, 2, 4, 6, 8,
};

/* Quantizer step size lookup table */
int32_t StepSizeTable[89] = {
 7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
 50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
 130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
 337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
 2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
 5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};




uint8_t ADPCMEncoder(int16_t sample, struct ADPCMstate *state)
{
	int32_t code;							/* ADPCM output value */
	int32_t diff;							/* Difference between sample and the predicted sample */
	int32_t step;							/* Quantizer step size */
	int32_t predsample;						/* Output of ADPCM predictor */
	int32_t diffq;							/* Dequantized predicted difference */
	int32_t index;							/* Index into step size table */

	/* Restore previous values of predicted sample and quantizer step
	 size index
	*/
	predsample = (int32_t)(state->prevsample);
	index = state->previndex;
	step = StepSizeTable[index];

	/* Compute the difference between the acutal sample (sample) and the
	 the predicted sample (predsample)
	*/
	diff = sample - predsample;
	if (diff >= 0)
		code = 0;
	else
	{
		code = 8;
		diff = -diff;
	}

	/* Quantize the difference into the 4-bit ADPCM code using the
	 the quantizer step size
	*/
	/* Inverse quantize the ADPCM code into a predicted difference
	 using the quantizer step size
	*/
	diffq = step >> 3;
	if (diff >= step)
	{
		code |= 4;
		diff -= step;
		diffq += step;
	}
	step >>= 1;
	if (diff >= step)
	{
		code |= 2;
		diff -= step;
		diffq += step;
	}
	step >>= 1;
	if (diff >= step)
	{
		code |= 1;
		diffq += step;
	}

	/* Fixed predictor computes new predicted sample by adding the
	 old predicted sample to predicted difference
	*/

	if (code & 8)
		predsample -= diffq;
	else
		predsample += diffq;

	/* Check for overflow of the new predicted sample
	*/
	if (predsample > 32767)
		predsample = 32767;
	else if (predsample < -32767)
		predsample = -32767;

	/* Find new quantizer stepsize index by adding the old index
	 to a table lookup using the ADPCM code
	*/
	index += IndexTable[code];

	/* Check for overflow of the new quantizer step size index
	*/
	if (index < 0)
		index = 0;
	if (index > 88)
		index = 88;

	/* Save the predicted sample and quantizer step size index for
	 next iteration
	*/
	state->prevsample = (int16_t)predsample;
	state->previndex = index;
	/* Return the new ADPCM code
*/
	return (code & 0x0f);
}

int32_t ADPCMDecoder(uint8_t code, struct ADPCMstate *state)
{
	int32_t step;							/* Quantizer step size */
	int32_t predsample;						/* Output of ADPCM predictor */
	int32_t diffq;							/* Dequantized predicted difference */
	int32_t index;							/* Index into step size table */

	/* Restore previous values of predicted sample and quantizer step
	 size index
	*/
	predsample = (int32_t)(state->prevsample);
	index = state->previndex;

	/* Find quantizer step size from lookup table using index
	*/
	step = StepSizeTable[index];

	/* Inverse quantize the ADPCM code into a difference using the
	 quantizer step size
	*/
	diffq = step >> 3;
	if (code & 4)
		diffq += step;
	if (code & 2)
		diffq += step >> 1;
	if (code & 1)
		diffq += step >> 2;

	/* Add the difference to the predicted sample
	*/
	if (code & 8)
		predsample -= diffq;
	else
		predsample += diffq;

	/* Check for overflow of the new predicted sample
	*/
	if (predsample > 32767)
		predsample = 32767;
	else if (predsample < -32767)
		predsample = -32767;

	/* Find new quantizer step size by adding the old index and a
	 table lookup using the ADPCM code
	*/
	index += IndexTable[code];

	/* Check for overflow of the new quantizer step size index
	*/
	if (index < 0)
		index = 0;
	if (index > 88)
		index = 88;

	/* Save predicted sample and quantizer step size index for next
	 iteration
	*/
	state->prevsample = (int16_t)predsample;
	state->previndex = index;

	/* Return the new speech sample */
	return(predsample);
}