#ifndef NN_H
#define NN_H

#include "paf.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

void load_model(void);
void run_inference(const float data[3 * 80], int offset,
                   float result[MODEL_NB_OUTPUTS]);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif // NN_H