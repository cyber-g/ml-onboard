#ifndef MODELS_PAF_H
#define MODELS_PAF_H

#include <stdint.h>

#define MODEL_NB_OUTPUTS 6
extern const uint8_t model[];
extern const char *model_labels[MODEL_NB_OUTPUTS];

#define MODEL_INPUT_SCALE 0.15846575796604156
#define MODEL_INPUT_ZERO 14
#define MODEL_OUTPUT_SCALE 0.00390625
#define MODEL_OUTPUT_ZERO -128

#endif // MODELS_PAF_H
