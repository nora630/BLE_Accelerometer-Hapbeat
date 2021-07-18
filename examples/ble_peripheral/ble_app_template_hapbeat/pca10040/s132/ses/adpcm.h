#ifndef ADPCM_H__
#define ADPCM_H__

#include <stdint.h>

struct ADPCMstate {
    int16_t prevsample;   /* Predicted sample */
    int32_t previndex;    /* Index into step size table */
};

/* Function prototype for the ADPCM Encoder routine */
uint8_t ADPCMEncoder(int16_t, struct ADPCMstate *);

/* Function prototype for the ADPCM Decoder routine */
int32_t ADPCMDecoder(uint8_t, struct ADPCMstate *);

#endif