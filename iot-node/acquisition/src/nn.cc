#include <math.h>
#include <stdint.h>

#include "nn.h"

#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"

static tflite::MicroInterpreter *nn_interpreter;

void load_model(void) {
  static const tflite::Model *tflite_model = ::tflite::GetModel(model);
  static tflite::ErrorReporter *reporter = tflite::GetMicroErrorReporter();
  static tflite::AllOpsResolver resolver;
  resolver.AddExpandDims();
#define TENSOR_ARENA_SIZE (20 * 1024)
  static uint8_t tensor_arena[TENSOR_ARENA_SIZE];
  static tflite::MicroInterpreter interpreter(
      tflite_model, resolver, tensor_arena, TENSOR_ARENA_SIZE, reporter);
  interpreter.AllocateTensors();
  nn_interpreter = &interpreter;
  TfLiteTensor *input = nn_interpreter->input(0);
  printf("Input dims: %d\n", input->dims->size);
  printf("dim[0]: %d\n", input->dims->data[0]);
  printf("dim[1]: %d\n", input->dims->data[1]);
  printf("dim[2]: %d\n", input->dims->data[2]);
  printf("Type is int8? %s\n", input->type == kTfLiteInt8 ? "yes" : "no");
  TfLiteTensor *output = nn_interpreter->output(0);
  printf("Output dims: %d\n", output->dims->size);
  printf("dim[0]: %d\n", output->dims->data[0]);
  printf("dim[1]: %d\n", output->dims->data[1]);
  printf("Type is int8? %s\n", output->type == kTfLiteInt8 ? "yes" : "no");
}

void run_inference(const float data[3 * 80], int offset,
                   float result[MODEL_NB_OUTPUTS]) {
  (void)result;
  TfLiteTensor *input = nn_interpreter->input(0);
  for (int row = 0; row < 80; row++) {
    for (int axis = 0; axis < 3; axis++) {
      float original = data[(offset + row * 3 + axis) % 240];
      int converted = round(original / MODEL_INPUT_SCALE + MODEL_INPUT_ZERO);
      input->data.uint8[row * 3 + axis] = (int8_t) converted;
    }
  }
  nn_interpreter->Invoke();
  TfLiteTensor *output = nn_interpreter->output(0);
  for (int i = 0; i < MODEL_NB_OUTPUTS; i++)
    result[i] = ((int8_t) output->data.uint8[i] - MODEL_OUTPUT_ZERO) *
                MODEL_OUTPUT_SCALE;
}