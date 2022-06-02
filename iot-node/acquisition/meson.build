project('tpstm32', ['c', 'cpp'], default_options: ['b_staticpic=false'])

inc = include_directories('include')
c_args = ['-DSTM32L475xx', '-DUSE_HAL_DRIVER',
          '-Og', '-Wall', '-Wextra', '-Werror',
          '-g', '-gdwarf-2',
          '-ffunction-sections', '-fdata-sections']
linkfiles = files('STM32L475VGTx_FLASH.ld')

link_args = ['-Wl,--gc-sections', '-lc', '-lm', '-lnosys']
foreach linkfile : linkfiles
  link_args += ['-Wl,-T,@0@/@1@'.format(meson.current_source_dir(), linkfile)]
endforeach
link_args += [ '-Wl,-lnosys' ] # Needed to be able to link with g++

#
# FreeRTOS
#

freertos_dir = '../FreeRTOS-Kernel'
freertos_files = [
  'tasks.c', 'queue.c', 'list.c',
  'portable/GCC/ARM_CM4F/port.c',
  'portable/MemMang/heap_1.c']
freertos_inc = include_directories([freertos_dir / 'include',
                                    freertos_dir / 'portable/GCC/ARM_CM4F'])
freertos_sources = []
foreach f: freertos_files
  freertos_sources += freertos_dir / f
endforeach
freertos_lib = static_library('freertos',
  freertos_sources,
  c_args: c_args,
  include_directories: [freertos_inc, inc])
freertos = declare_dependency(link_with: freertos_lib, include_directories: freertos_inc)

#
# CMSIS
#

hal_dir = '../stm32l4xx_hal_driver'
hal_inc = include_directories(hal_dir / 'Inc') # needed for compilation of CMSIS

cmsis_base_dir = '../CMSIS_5'
cmsis_core_dir = cmsis_base_dir / 'CMSIS/Core'
cmsis_dev_dir = '../cmsis_device_l4'
cmsis_inc = include_directories([
  cmsis_core_dir / 'Include',
  cmsis_dev_dir / 'Include'
  ])
cmsis_sources = [
  cmsis_dev_dir / 'Source/Templates/gcc/startup_stm32l475xx.s',
  cmsis_dev_dir / 'Source/Templates/system_stm32l4xx.c'
  ]
cmsis_lib = static_library('cmsis',
  cmsis_sources,
  c_args: c_args,
  include_directories: [cmsis_inc, hal_inc, inc])
cmsis = declare_dependency(link_with: cmsis_lib, include_directories: cmsis_inc)

#
# STM32L4 HAL
#

hal_modules = ['i2c', 'i2c_ex', 'rcc', 'rcc_ex', 'flash', 'flash_ex', 'flash_ramfunc',
               'gpio', 'dma', 'dma_ex', 'pwr', 'pwr_ex', 'cortex', 'exti', 'iwdg',
               'tim', 'tim_ex', 'uart', 'uart_ex', 'qspi']
hal_sources = [hal_dir / 'Src/stm32l4xx_hal.c']
foreach m: hal_modules
  hal_sources += hal_dir / 'Src' / 'stm32l4xx_hal_@0@.c'.format(m)
endforeach
hal_lib = static_library('hal',
  hal_sources,
  c_args: c_args + ['-Wno-unused-parameter'],
  include_directories: [hal_inc, cmsis_inc, inc])
hal = declare_dependency(link_with: hal_lib, include_directories: hal_inc)

#
# Flatbuffers
#

flatbuffers_dir = '../flatbuffers'
flatbuffers_inc = flatbuffers_dir / 'include'
flatbuffers_sources = []
foreach s: ['idl_parser.cpp', 'idl_gen_text.cpp', 'reflection.cpp', 'util.cpp']
  flatbuffers_sources += flatbuffers_dir / 'src' / s
endforeach
flatbuffers_lib = static_library('flatbuffers',
  flatbuffers_sources,
  cpp_args: c_args + ['-Wno-psabi'],
  include_directories: flatbuffers_inc)
flatbuffers = declare_dependency(link_with: flatbuffers_lib,
                                 include_directories: flatbuffers_inc)

#
# Ruy
#

ruy_dir = '../ruy'
ruy_sources = []
foreach s: ['apply_multiplier.cc', 'denormal.cc', 'kernel_arm32.cc']
  ruy_sources += ruy_dir / 'ruy' / s
endforeach
ruy_lib = static_library('ruy',
  ruy_sources,
  cpp_args: c_args,
  include_directories: ruy_dir)
ruy = declare_dependency(link_with: ruy_lib, include_directories: ruy_dir)

#
# CMSIS NN
#

cmsis_nn_dir = cmsis_base_dir / 'CMSIS/NN'
cmsis_nn_inc = cmsis_nn_dir / 'Include'
cmsis_dsp_dir = cmsis_base_dir / 'CMSIS/DSP'
cmsis_dsp_inc = cmsis_dsp_dir / 'Include'
cmsis_nn_sources = []
foreach s: [
  'ActivationFunctions/arm_nn_activations_q15.c',
  'ActivationFunctions/arm_nn_activations_q7.c',
  'ActivationFunctions/arm_relu6_s8.c',
  'ActivationFunctions/arm_relu_q15.c',
  'ActivationFunctions/arm_relu_q7.c',
  'BasicMathFunctions/arm_elementwise_add_s8.c',
  'BasicMathFunctions/arm_elementwise_mul_s8.c',
  'ConcatenationFunctions/arm_concatenation_s8_w.c',
  'ConcatenationFunctions/arm_concatenation_s8_x.c',
  'ConcatenationFunctions/arm_concatenation_s8_y.c',
  'ConcatenationFunctions/arm_concatenation_s8_z.c',
  'ConvolutionFunctions/arm_convolve_1x1_HWC_q7_fast_nonsquare.c',
  'ConvolutionFunctions/arm_convolve_1x1_s8_fast.c',
  'ConvolutionFunctions/arm_convolve_1_x_n_s8.c',
  'ConvolutionFunctions/arm_convolve_HWC_q15_basic.c',
  'ConvolutionFunctions/arm_convolve_HWC_q15_fast.c',
  'ConvolutionFunctions/arm_convolve_HWC_q15_fast_nonsquare.c',
  'ConvolutionFunctions/arm_convolve_HWC_q7_basic.c',
  'ConvolutionFunctions/arm_convolve_HWC_q7_basic_nonsquare.c',
  'ConvolutionFunctions/arm_convolve_HWC_q7_fast.c',
  'ConvolutionFunctions/arm_convolve_HWC_q7_fast_nonsquare.c',
  'ConvolutionFunctions/arm_convolve_HWC_q7_RGB.c',
  'ConvolutionFunctions/arm_convolve_s8.c',
  'ConvolutionFunctions/arm_convolve_wrapper_s8.c',
  'ConvolutionFunctions/arm_depthwise_conv_3x3_s8.c',
  'ConvolutionFunctions/arm_depthwise_conv_s8.c',
  'ConvolutionFunctions/arm_depthwise_conv_s8_opt.c',
  'ConvolutionFunctions/arm_depthwise_conv_u8_basic_ver1.c',
  'ConvolutionFunctions/arm_depthwise_conv_wrapper_s8.c',
  'ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7.c',
  'ConvolutionFunctions/arm_depthwise_separable_conv_HWC_q7_nonsquare.c',
  'ConvolutionFunctions/arm_nn_depthwise_conv_s8_core.c',
  'ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15.c',
  'ConvolutionFunctions/arm_nn_mat_mult_kernel_q7_q15_reordered.c',
  'ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16.c',
  'ConvolutionFunctions/arm_nn_mat_mult_kernel_s8_s16_reordered.c',
  'ConvolutionFunctions/arm_nn_mat_mult_s8.c',
  'FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15.c',
  'FullyConnectedFunctions/arm_fully_connected_mat_q7_vec_q15_opt.c',
  'FullyConnectedFunctions/arm_fully_connected_q15.c',
  'FullyConnectedFunctions/arm_fully_connected_q15_opt.c',
  'FullyConnectedFunctions/arm_fully_connected_q7.c',
  'FullyConnectedFunctions/arm_fully_connected_q7_opt.c',
  'FullyConnectedFunctions/arm_fully_connected_s8.c',
  'NNSupportFunctions/arm_nn_accumulate_q7_to_q15.c',
  'NNSupportFunctions/arm_nn_add_q7.c',
  'NNSupportFunctions/arm_nn_depthwise_conv_nt_t_padded_s8.c',
  'NNSupportFunctions/arm_nn_depthwise_conv_nt_t_s8.c',
  'NNSupportFunctions/arm_nn_mat_mul_core_1x_s8.c',
  'NNSupportFunctions/arm_nn_mat_mul_core_4x_s8.c',
  'NNSupportFunctions/arm_nn_mat_mult_nt_t_s8.c',
  'NNSupportFunctions/arm_nn_mult_q15.c',
  'NNSupportFunctions/arm_nn_mult_q7.c',
  'NNSupportFunctions/arm_nntables.c',
  'NNSupportFunctions/arm_nn_vec_mat_mult_t_s8.c',
  'NNSupportFunctions/arm_nn_vec_mat_mult_t_svdf_s8.c',
  'NNSupportFunctions/arm_q7_to_q15_no_shift.c',
  'NNSupportFunctions/arm_q7_to_q15_reordered_no_shift.c',
  'NNSupportFunctions/arm_q7_to_q15_reordered_with_offset.c',
  'NNSupportFunctions/arm_q7_to_q15_with_offset.c',
  'PoolingFunctions/arm_avgpool_s8.c',
  'PoolingFunctions/arm_max_pool_s8.c',
  'PoolingFunctions/arm_pool_q7_HWC.c',
  'ReshapeFunctions/arm_reshape_s8.c',
  'SoftmaxFunctions/arm_softmax_q15.c',
  'SoftmaxFunctions/arm_softmax_q7.c',
  'SoftmaxFunctions/arm_softmax_s8.c',
  'SoftmaxFunctions/arm_softmax_u8.c',
  'SoftmaxFunctions/arm_softmax_with_batch_q7.c',
  'SVDFunctions/arm_svdf_s8.c',
]
    cmsis_nn_sources += cmsis_nn_dir / 'Source' / s
endforeach
cmsis_nn_lib = static_library('cmsis_nn',
  cmsis_nn_sources,
  c_args: c_args,
  include_directories: [cmsis_nn_inc, cmsis_dsp_inc, cmsis_inc])
cmsis_nn = declare_dependency(link_with: cmsis_nn_lib, include_directories: [cmsis_nn_inc, cmsis_dsp_inc, cmsis_inc])

#
# Tensorflow
#

tf_dir = '../tflite-micro'
tf_micro_sources = [
  'all_ops_resolver.cc',
  'debug_log.cc',
  'kernels/activations.cc',
  'kernels/add_n.cc',
  'kernels/arg_min_max.cc',
  'kernels/batch_to_space_nd.cc',
  'kernels/ceil.cc',
  'kernels/cmsis_nn/add.cc',
  'kernels/cmsis_nn/conv.cc',
  'kernels/cmsis_nn/depthwise_conv.cc',
  'kernels/cmsis_nn/fully_connected.cc',
  'kernels/cmsis_nn/mul.cc',
  'kernels/cmsis_nn/pooling.cc',
  'kernels/cmsis_nn/softmax.cc',
  'kernels/cmsis_nn/svdf.cc',
  'kernels/comparisons.cc',
  'kernels/concatenation.cc',
  'kernels/conv_common.cc',
  'kernels/cumsum.cc',
  'kernels/depth_to_space.cc',
  'kernels/dequantize.cc',
  'kernels/depthwise_conv_common.cc',
  'kernels/detection_postprocess.cc',
  'kernels/elementwise.cc',
  'kernels/elu.cc',
  'kernels/expand_dims.cc',
  'kernels/ethosu.cc',
  'kernels/floor.cc',
  'kernels/floor_div.cc',
  'kernels/floor_mod.cc',
  'kernels/fully_connected_common.cc',
  'kernels/hard_swish.cc',
  'kernels/kernel_util.cc',
  'kernels/l2norm.cc',
  'kernels/l2_pool_2d.cc',
  'kernels/leaky_relu.cc',
  'kernels/logical.cc',
  'kernels/logistic.cc',
  'kernels/maximum_minimum.cc',
  'kernels/neg.cc',
  'kernels/pack.cc',
  'kernels/pad.cc',
  'kernels/pooling_common.cc',
  'kernels/prelu.cc',
  'kernels/quantize.cc',
  'kernels/quantize_common.cc',
  'kernels/reduce.cc',
  'kernels/reshape.cc',
  'kernels/resize_bilinear.cc',
  'kernels/resize_nearest_neighbor.cc',
  'kernels/round.cc',
  'kernels/shape.cc',
  'kernels/softmax_common.cc',
  'kernels/space_to_batch_nd.cc',
  'kernels/space_to_depth.cc',
  'kernels/split.cc',
  'kernels/split_v.cc',
  'kernels/squeeze.cc',
  'kernels/strided_slice.cc',
  'kernels/sub.cc',
  'kernels/tanh.cc',
  'kernels/transpose.cc',
  'kernels/transpose_conv.cc',
  'kernels/unpack.cc',
  'memory_helpers.cc',
  'memory_planner/greedy_memory_planner.cc',
  'micro_allocator.cc',
  'micro_error_reporter.cc',
  'micro_graph.cc',
  'micro_interpreter.cc',
  'micro_string.cc',
  'micro_time.cc',
  'micro_utils.cc',
  'simple_memory_allocator.cc',
]
tf_lite_sources = [
  'c/common.c',
  'core/api/error_reporter.cc',
  'core/api/flatbuffer_conversions.cc',
  'core/api/op_resolver.cc',
  'core/api/tensor_utils.cc',
  'kernels/internal/quantization_util.cc',
  'kernels/kernel_util.cc',
  'schema/schema_utils.cc',
]
tf_sources = []
foreach s: tf_micro_sources
  tf_sources += tf_dir / 'tensorflow/lite/micro' / s
endforeach
foreach s: tf_lite_sources
  tf_sources += tf_dir / 'tensorflow/lite' / s
endforeach
cmsis_dsp_dir = cmsis_base_dir / 'CMSIS/DSP'
cmsis_dsp_inc = cmsis_dsp_dir / 'Include'
cmsis_core_inc = cmsis_core_dir / 'Include'
fixedpoint_inc = '../gemmlowp'
tf_lib = static_library('tf',
  tf_sources,
  cpp_args: c_args + ['-Wno-unused-parameter', '-Wno-maybe-uninitialized',
                      '-Wno-deprecated-declarations', '-DCMSIS_NN', '-D__FPU_PRESENT=1',
                      #'-DTF_LITE_STATIC_MEMORY',
                      '-DTF_LITE_MCU_DEBUG_LOG', '-DCPU_CM4=1'],
  include_directories: [tf_dir, cmsis_base_dir, cmsis_dsp_inc, cmsis_core_inc,
                        fixedpoint_inc, ruy_dir],
  dependencies: [flatbuffers, ruy, cmsis_nn])
tf = declare_dependency(
  link_with: tf_lib,
  include_directories: tf_dir,
  compile_args: '-DCMSIS_NN'
)

#
# Executable
#

models_dir = '../../models'
main = executable('main',
         [
           'src/main.c',
           'src/i2cll.c',
           'src/lsm6dsl.c',
           'src/activity.c',
           'src/button.c',
           'src/matrix.c',
           'src/font.c',
           'src/morse.c',
           'src/nn.cc',
           models_dir / 'paf.c',
           'src/stm32l4xx_it.c',
           'src/stm32l4xx_hal_msp.c',
           'src/stm32l4xx_hal_timebase_tim.c'
         ],
         name_suffix: 'elf',
         c_args: c_args,
         cpp_args: c_args + ['-Wno-unused-parameter'],
         link_args: link_args,
         include_directories: [inc, models_dir],
         dependencies: [freertos, cmsis, hal, tf, flatbuffers])
