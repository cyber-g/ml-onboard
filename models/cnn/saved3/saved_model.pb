??
??
B
AssignVariableOp
resource
value"dtype"
dtypetype?
~
BiasAdd

value"T	
bias"T
output"T" 
Ttype:
2	"-
data_formatstringNHWC:
NHWCNCHW
8
Const
output"dtype"
valuetensor"
dtypetype
?
Conv2D

input"T
filter"T
output"T"
Ttype:	
2"
strides	list(int)"
use_cudnn_on_gpubool(",
paddingstring:
SAMEVALIDEXPLICIT""
explicit_paddings	list(int)
 "-
data_formatstringNHWC:
NHWCNCHW" 
	dilations	list(int)

W

ExpandDims

input"T
dim"Tdim
output"T"	
Ttype"
Tdimtype0:
2	
.
Identity

input"T
output"T"	
Ttype
q
MatMul
a"T
b"T
product"T"
transpose_abool( "
transpose_bbool( "
Ttype:

2	
?
MaxPool

input"T
output"T"
Ttype0:
2	"
ksize	list(int)(0"
strides	list(int)(0",
paddingstring:
SAMEVALIDEXPLICIT""
explicit_paddings	list(int)
 ":
data_formatstringNHWC:
NHWCNCHWNCHW_VECT_C
?
Mean

input"T
reduction_indices"Tidx
output"T"
	keep_dimsbool( " 
Ttype:
2	"
Tidxtype0:
2	
e
MergeV2Checkpoints
checkpoint_prefixes
destination_prefix"
delete_old_dirsbool(?

NoOp
M
Pack
values"T*N
output"T"
Nint(0"	
Ttype"
axisint 
C
Placeholder
output"dtype"
dtypetype"
shapeshape:
@
ReadVariableOp
resource
value"dtype"
dtypetype?
E
Relu
features"T
activations"T"
Ttype:
2	
o
	RestoreV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0?
l
SaveV2

prefix
tensor_names
shape_and_slices
tensors2dtypes"
dtypes
list(type)(0?
?
Select
	condition

t"T
e"T
output"T"	
Ttype
H
ShardedFilename
basename	
shard

num_shards
filename
9
Softmax
logits"T
softmax"T"
Ttype:
2
N
Squeeze

input"T
output"T"	
Ttype"
squeeze_dims	list(int)
 (
?
StatefulPartitionedCall
args2Tin
output2Tout"
Tin
list(type)("
Tout
list(type)("	
ffunc"
configstring "
config_protostring "
executor_typestring ?
@
StaticRegexFullMatch	
input

output
"
patternstring
N

StringJoin
inputs*N

output"
Nint(0"
	separatorstring 
?
VarHandleOp
resource"
	containerstring "
shared_namestring "
dtypetype"
shapeshape"#
allowed_deviceslist(string)
 ?"serve*2.5.02v2.5.0-rc3-213-ga4dfb8d1a718??
?
conv1d_20/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:

*!
shared_nameconv1d_20/kernel
y
$conv1d_20/kernel/Read/ReadVariableOpReadVariableOpconv1d_20/kernel*"
_output_shapes
:

*
dtype0
t
conv1d_20/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:
*
shared_nameconv1d_20/bias
m
"conv1d_20/bias/Read/ReadVariableOpReadVariableOpconv1d_20/bias*
_output_shapes
:
*
dtype0
?
conv1d_21/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape:

*!
shared_nameconv1d_21/kernel
y
$conv1d_21/kernel/Read/ReadVariableOpReadVariableOpconv1d_21/kernel*"
_output_shapes
:

*
dtype0
t
conv1d_21/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_nameconv1d_21/bias
m
"conv1d_21/bias/Read/ReadVariableOpReadVariableOpconv1d_21/bias*
_output_shapes
:*
dtype0
z
dense_13/kernelVarHandleOp*
_output_shapes
: *
dtype0*
shape
:* 
shared_namedense_13/kernel
s
#dense_13/kernel/Read/ReadVariableOpReadVariableOpdense_13/kernel*
_output_shapes

:*
dtype0
r
dense_13/biasVarHandleOp*
_output_shapes
: *
dtype0*
shape:*
shared_namedense_13/bias
k
!dense_13/bias/Read/ReadVariableOpReadVariableOpdense_13/bias*
_output_shapes
:*
dtype0
f
	Adam/iterVarHandleOp*
_output_shapes
: *
dtype0	*
shape: *
shared_name	Adam/iter
_
Adam/iter/Read/ReadVariableOpReadVariableOp	Adam/iter*
_output_shapes
: *
dtype0	
j
Adam/beta_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdam/beta_1
c
Adam/beta_1/Read/ReadVariableOpReadVariableOpAdam/beta_1*
_output_shapes
: *
dtype0
j
Adam/beta_2VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nameAdam/beta_2
c
Adam/beta_2/Read/ReadVariableOpReadVariableOpAdam/beta_2*
_output_shapes
: *
dtype0
h

Adam/decayVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name
Adam/decay
a
Adam/decay/Read/ReadVariableOpReadVariableOp
Adam/decay*
_output_shapes
: *
dtype0
x
Adam/learning_rateVarHandleOp*
_output_shapes
: *
dtype0*
shape: *#
shared_nameAdam/learning_rate
q
&Adam/learning_rate/Read/ReadVariableOpReadVariableOpAdam/learning_rate*
_output_shapes
: *
dtype0
^
totalVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_nametotal
W
total/Read/ReadVariableOpReadVariableOptotal*
_output_shapes
: *
dtype0
^
countVarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_namecount
W
count/Read/ReadVariableOpReadVariableOpcount*
_output_shapes
: *
dtype0
b
total_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	total_1
[
total_1/Read/ReadVariableOpReadVariableOptotal_1*
_output_shapes
: *
dtype0
b
count_1VarHandleOp*
_output_shapes
: *
dtype0*
shape: *
shared_name	count_1
[
count_1/Read/ReadVariableOpReadVariableOpcount_1*
_output_shapes
: *
dtype0
?
Adam/conv1d_20/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:

*(
shared_nameAdam/conv1d_20/kernel/m
?
+Adam/conv1d_20/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv1d_20/kernel/m*"
_output_shapes
:

*
dtype0
?
Adam/conv1d_20/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:
*&
shared_nameAdam/conv1d_20/bias/m
{
)Adam/conv1d_20/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv1d_20/bias/m*
_output_shapes
:
*
dtype0
?
Adam/conv1d_21/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:

*(
shared_nameAdam/conv1d_21/kernel/m
?
+Adam/conv1d_21/kernel/m/Read/ReadVariableOpReadVariableOpAdam/conv1d_21/kernel/m*"
_output_shapes
:

*
dtype0
?
Adam/conv1d_21/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_nameAdam/conv1d_21/bias/m
{
)Adam/conv1d_21/bias/m/Read/ReadVariableOpReadVariableOpAdam/conv1d_21/bias/m*
_output_shapes
:*
dtype0
?
Adam/dense_13/kernel/mVarHandleOp*
_output_shapes
: *
dtype0*
shape
:*'
shared_nameAdam/dense_13/kernel/m
?
*Adam/dense_13/kernel/m/Read/ReadVariableOpReadVariableOpAdam/dense_13/kernel/m*
_output_shapes

:*
dtype0
?
Adam/dense_13/bias/mVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_13/bias/m
y
(Adam/dense_13/bias/m/Read/ReadVariableOpReadVariableOpAdam/dense_13/bias/m*
_output_shapes
:*
dtype0
?
Adam/conv1d_20/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:

*(
shared_nameAdam/conv1d_20/kernel/v
?
+Adam/conv1d_20/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv1d_20/kernel/v*"
_output_shapes
:

*
dtype0
?
Adam/conv1d_20/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:
*&
shared_nameAdam/conv1d_20/bias/v
{
)Adam/conv1d_20/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv1d_20/bias/v*
_output_shapes
:
*
dtype0
?
Adam/conv1d_21/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:

*(
shared_nameAdam/conv1d_21/kernel/v
?
+Adam/conv1d_21/kernel/v/Read/ReadVariableOpReadVariableOpAdam/conv1d_21/kernel/v*"
_output_shapes
:

*
dtype0
?
Adam/conv1d_21/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*&
shared_nameAdam/conv1d_21/bias/v
{
)Adam/conv1d_21/bias/v/Read/ReadVariableOpReadVariableOpAdam/conv1d_21/bias/v*
_output_shapes
:*
dtype0
?
Adam/dense_13/kernel/vVarHandleOp*
_output_shapes
: *
dtype0*
shape
:*'
shared_nameAdam/dense_13/kernel/v
?
*Adam/dense_13/kernel/v/Read/ReadVariableOpReadVariableOpAdam/dense_13/kernel/v*
_output_shapes

:*
dtype0
?
Adam/dense_13/bias/vVarHandleOp*
_output_shapes
: *
dtype0*
shape:*%
shared_nameAdam/dense_13/bias/v
y
(Adam/dense_13/bias/v/Read/ReadVariableOpReadVariableOpAdam/dense_13/bias/v*
_output_shapes
:*
dtype0

NoOpNoOp
?,
ConstConst"/device:CPU:0*
_output_shapes
: *
dtype0*?+
value?+B?+ B?+
?
layer_with_weights-0
layer-0
layer-1
layer_with_weights-1
layer-2
layer-3
layer-4
layer_with_weights-2
layer-5
	optimizer
regularization_losses
		variables

trainable_variables
	keras_api

signatures
h

kernel
bias
regularization_losses
	variables
trainable_variables
	keras_api
R
regularization_losses
	variables
trainable_variables
	keras_api
h

kernel
bias
regularization_losses
	variables
trainable_variables
	keras_api
R
regularization_losses
	variables
trainable_variables
 	keras_api
R
!regularization_losses
"	variables
#trainable_variables
$	keras_api
h

%kernel
&bias
'regularization_losses
(	variables
)trainable_variables
*	keras_api
?
+iter

,beta_1

-beta_2
	.decay
/learning_ratem^m_m`ma%mb&mcvdvevfvg%vh&vi
 
*
0
1
2
3
%4
&5
*
0
1
2
3
%4
&5
?
regularization_losses
0metrics
		variables

trainable_variables
1non_trainable_variables
2layer_metrics

3layers
4layer_regularization_losses
 
\Z
VARIABLE_VALUEconv1d_20/kernel6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv1d_20/bias4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUE
 

0
1

0
1
?
regularization_losses
5metrics
	variables
trainable_variables
6non_trainable_variables
7layer_metrics

8layers
9layer_regularization_losses
 
 
 
?
regularization_losses
:metrics
	variables
trainable_variables
;non_trainable_variables
<layer_metrics

=layers
>layer_regularization_losses
\Z
VARIABLE_VALUEconv1d_21/kernel6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUE
XV
VARIABLE_VALUEconv1d_21/bias4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUE
 

0
1

0
1
?
regularization_losses
?metrics
	variables
trainable_variables
@non_trainable_variables
Alayer_metrics

Blayers
Clayer_regularization_losses
 
 
 
?
regularization_losses
Dmetrics
	variables
trainable_variables
Enon_trainable_variables
Flayer_metrics

Glayers
Hlayer_regularization_losses
 
 
 
?
!regularization_losses
Imetrics
"	variables
#trainable_variables
Jnon_trainable_variables
Klayer_metrics

Llayers
Mlayer_regularization_losses
[Y
VARIABLE_VALUEdense_13/kernel6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUE
WU
VARIABLE_VALUEdense_13/bias4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUE
 

%0
&1

%0
&1
?
'regularization_losses
Nmetrics
(	variables
)trainable_variables
Onon_trainable_variables
Player_metrics

Qlayers
Rlayer_regularization_losses
HF
VARIABLE_VALUE	Adam/iter)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUE
LJ
VARIABLE_VALUEAdam/beta_1+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUE
LJ
VARIABLE_VALUEAdam/beta_2+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUE
JH
VARIABLE_VALUE
Adam/decay*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUE
ZX
VARIABLE_VALUEAdam/learning_rate2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUE

S0
T1
 
 
*
0
1
2
3
4
5
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
4
	Utotal
	Vcount
W	variables
X	keras_api
D
	Ytotal
	Zcount
[
_fn_kwargs
\	variables
]	keras_api
OM
VARIABLE_VALUEtotal4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUE
OM
VARIABLE_VALUEcount4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUE

U0
V1

W	variables
QO
VARIABLE_VALUEtotal_14keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUE
QO
VARIABLE_VALUEcount_14keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUE
 

Y0
Z1

\	variables
}
VARIABLE_VALUEAdam/conv1d_20/kernel/mRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv1d_20/bias/mPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv1d_21/kernel/mRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv1d_21/bias/mPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_13/kernel/mRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_13/bias/mPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv1d_20/kernel/vRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv1d_20/bias/vPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
}
VARIABLE_VALUEAdam/conv1d_21/kernel/vRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
{y
VARIABLE_VALUEAdam/conv1d_21/bias/vPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
~|
VARIABLE_VALUEAdam/dense_13/kernel/vRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
zx
VARIABLE_VALUEAdam/dense_13/bias/vPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUE
?
serving_default_conv1d_20_inputPlaceholder*+
_output_shapes
:?????????P*
dtype0* 
shape:?????????P
?
StatefulPartitionedCallStatefulPartitionedCallserving_default_conv1d_20_inputconv1d_20/kernelconv1d_20/biasconv1d_21/kernelconv1d_21/biasdense_13/kerneldense_13/bias*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *-
f(R&
$__inference_signature_wrapper_131023
O
saver_filenamePlaceholder*
_output_shapes
: *
dtype0*
shape: 
?

StatefulPartitionedCall_1StatefulPartitionedCallsaver_filename$conv1d_20/kernel/Read/ReadVariableOp"conv1d_20/bias/Read/ReadVariableOp$conv1d_21/kernel/Read/ReadVariableOp"conv1d_21/bias/Read/ReadVariableOp#dense_13/kernel/Read/ReadVariableOp!dense_13/bias/Read/ReadVariableOpAdam/iter/Read/ReadVariableOpAdam/beta_1/Read/ReadVariableOpAdam/beta_2/Read/ReadVariableOpAdam/decay/Read/ReadVariableOp&Adam/learning_rate/Read/ReadVariableOptotal/Read/ReadVariableOpcount/Read/ReadVariableOptotal_1/Read/ReadVariableOpcount_1/Read/ReadVariableOp+Adam/conv1d_20/kernel/m/Read/ReadVariableOp)Adam/conv1d_20/bias/m/Read/ReadVariableOp+Adam/conv1d_21/kernel/m/Read/ReadVariableOp)Adam/conv1d_21/bias/m/Read/ReadVariableOp*Adam/dense_13/kernel/m/Read/ReadVariableOp(Adam/dense_13/bias/m/Read/ReadVariableOp+Adam/conv1d_20/kernel/v/Read/ReadVariableOp)Adam/conv1d_20/bias/v/Read/ReadVariableOp+Adam/conv1d_21/kernel/v/Read/ReadVariableOp)Adam/conv1d_21/bias/v/Read/ReadVariableOp*Adam/dense_13/kernel/v/Read/ReadVariableOp(Adam/dense_13/bias/v/Read/ReadVariableOpConst*(
Tin!
2	*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *(
f#R!
__inference__traced_save_131371
?
StatefulPartitionedCall_2StatefulPartitionedCallsaver_filenameconv1d_20/kernelconv1d_20/biasconv1d_21/kernelconv1d_21/biasdense_13/kerneldense_13/bias	Adam/iterAdam/beta_1Adam/beta_2
Adam/decayAdam/learning_ratetotalcounttotal_1count_1Adam/conv1d_20/kernel/mAdam/conv1d_20/bias/mAdam/conv1d_21/kernel/mAdam/conv1d_21/bias/mAdam/dense_13/kernel/mAdam/dense_13/bias/mAdam/conv1d_20/kernel/vAdam/conv1d_20/bias/vAdam/conv1d_21/kernel/vAdam/conv1d_21/bias/vAdam/dense_13/kernel/vAdam/dense_13/bias/v*'
Tin 
2*
Tout
2*
_collective_manager_ids
 *
_output_shapes
: * 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *+
f&R$
"__inference__traced_restore_131462??
?
?
E__inference_conv1d_21_layer_call_and_return_conditional_losses_130770

inputsA
+conv1d_expanddims_1_readvariableop_resource:

-
biasadd_readvariableop_resource:
identity??BiasAdd/ReadVariableOp?"conv1d/ExpandDims_1/ReadVariableOpy
conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2
conv1d/ExpandDims/dim?
conv1d/ExpandDims
ExpandDimsinputsconv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????
2
conv1d/ExpandDims?
"conv1d/ExpandDims_1/ReadVariableOpReadVariableOp+conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02$
"conv1d/ExpandDims_1/ReadVariableOpt
conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2
conv1d/ExpandDims_1/dim?
conv1d/ExpandDims_1
ExpandDims*conv1d/ExpandDims_1/ReadVariableOp:value:0 conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d/ExpandDims_1?
conv1dConv2Dconv1d/ExpandDims:output:0conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv1d?
conv1d/SqueezeSqueezeconv1d:output:0*
T0*+
_output_shapes
:?????????*
squeeze_dims

?????????2
conv1d/Squeeze?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddconv1d/Squeeze:output:0BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????2	
BiasAdd\
ReluReluBiasAdd:output:0*
T0*+
_output_shapes
:?????????2
Relu?
IdentityIdentityRelu:activations:0^BiasAdd/ReadVariableOp#^conv1d/ExpandDims_1/ReadVariableOp*
T0*+
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:?????????
: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2H
"conv1d/ExpandDims_1/ReadVariableOp"conv1d/ExpandDims_1/ReadVariableOp:S O
+
_output_shapes
:?????????

 
_user_specified_nameinputs
?

?
D__inference_dense_13_layer_call_and_return_conditional_losses_131258

inputs0
matmul_readvariableop_resource:-
biasadd_readvariableop_resource:
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2	
BiasAdda
SoftmaxSoftmaxBiasAdd:output:0*
T0*'
_output_shapes
:?????????2	
Softmax?
IdentityIdentitySoftmax:softmax:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:?????????: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
M
1__inference_max_pooling1d_10_layer_call_fn_130700

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *=
_output_shapes+
):'???????????????????????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *U
fPRN
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_1306942
PartitionedCall?
IdentityIdentityPartitionedCall:output:0*
T0*=
_output_shapes+
):'???????????????????????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*<
_input_shapes+
):'???????????????????????????:e a
=
_output_shapes+
):'???????????????????????????
 
_user_specified_nameinputs
?
?
I__inference_sequential_10_layer_call_and_return_conditional_losses_130922

inputs&
conv1d_20_130903:


conv1d_20_130905:
&
conv1d_21_130909:


conv1d_21_130911:!
dense_13_130916:
dense_13_130918:
identity??!conv1d_20/StatefulPartitionedCall?!conv1d_21/StatefulPartitionedCall? dense_13/StatefulPartitionedCall?"dropout_10/StatefulPartitionedCall?
!conv1d_20/StatefulPartitionedCallStatefulPartitionedCallinputsconv1d_20_130903conv1d_20_130905*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????G
*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_20_layer_call_and_return_conditional_losses_1307472#
!conv1d_20/StatefulPartitionedCall?
 max_pooling1d_10/PartitionedCallPartitionedCall*conv1d_20/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????
* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *U
fPRN
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_1306942"
 max_pooling1d_10/PartitionedCall?
!conv1d_21/StatefulPartitionedCallStatefulPartitionedCall)max_pooling1d_10/PartitionedCall:output:0conv1d_21_130909conv1d_21_130911*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_21_layer_call_and_return_conditional_losses_1307702#
!conv1d_21/StatefulPartitionedCall?
+global_average_pooling1d_10/PartitionedCallPartitionedCall*conv1d_21/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *`
f[RY
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_1307812-
+global_average_pooling1d_10/PartitionedCall?
"dropout_10/StatefulPartitionedCallStatefulPartitionedCall4global_average_pooling1d_10/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_dropout_10_layer_call_and_return_conditional_losses_1308532$
"dropout_10/StatefulPartitionedCall?
 dense_13/StatefulPartitionedCallStatefulPartitionedCall+dropout_10/StatefulPartitionedCall:output:0dense_13_130916dense_13_130918*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *M
fHRF
D__inference_dense_13_layer_call_and_return_conditional_losses_1308012"
 dense_13/StatefulPartitionedCall?
IdentityIdentity)dense_13/StatefulPartitionedCall:output:0"^conv1d_20/StatefulPartitionedCall"^conv1d_21/StatefulPartitionedCall!^dense_13/StatefulPartitionedCall#^dropout_10/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 2F
!conv1d_20/StatefulPartitionedCall!conv1d_20/StatefulPartitionedCall2F
!conv1d_21/StatefulPartitionedCall!conv1d_21/StatefulPartitionedCall2D
 dense_13/StatefulPartitionedCall dense_13/StatefulPartitionedCall2H
"dropout_10/StatefulPartitionedCall"dropout_10/StatefulPartitionedCall:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
?
.__inference_sequential_10_layer_call_fn_130954
conv1d_20_input
unknown:


	unknown_0:

	unknown_1:


	unknown_2:
	unknown_3:
	unknown_4:
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv1d_20_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *R
fMRK
I__inference_sequential_10_layer_call_and_return_conditional_losses_1309222
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:\ X
+
_output_shapes
:?????????P
)
_user_specified_nameconv1d_20_input
?=
?
__inference__traced_save_131371
file_prefix/
+savev2_conv1d_20_kernel_read_readvariableop-
)savev2_conv1d_20_bias_read_readvariableop/
+savev2_conv1d_21_kernel_read_readvariableop-
)savev2_conv1d_21_bias_read_readvariableop.
*savev2_dense_13_kernel_read_readvariableop,
(savev2_dense_13_bias_read_readvariableop(
$savev2_adam_iter_read_readvariableop	*
&savev2_adam_beta_1_read_readvariableop*
&savev2_adam_beta_2_read_readvariableop)
%savev2_adam_decay_read_readvariableop1
-savev2_adam_learning_rate_read_readvariableop$
 savev2_total_read_readvariableop$
 savev2_count_read_readvariableop&
"savev2_total_1_read_readvariableop&
"savev2_count_1_read_readvariableop6
2savev2_adam_conv1d_20_kernel_m_read_readvariableop4
0savev2_adam_conv1d_20_bias_m_read_readvariableop6
2savev2_adam_conv1d_21_kernel_m_read_readvariableop4
0savev2_adam_conv1d_21_bias_m_read_readvariableop5
1savev2_adam_dense_13_kernel_m_read_readvariableop3
/savev2_adam_dense_13_bias_m_read_readvariableop6
2savev2_adam_conv1d_20_kernel_v_read_readvariableop4
0savev2_adam_conv1d_20_bias_v_read_readvariableop6
2savev2_adam_conv1d_21_kernel_v_read_readvariableop4
0savev2_adam_conv1d_21_bias_v_read_readvariableop5
1savev2_adam_dense_13_kernel_v_read_readvariableop3
/savev2_adam_dense_13_bias_v_read_readvariableop
savev2_const

identity_1??MergeV2Checkpoints?
StaticRegexFullMatchStaticRegexFullMatchfile_prefix"/device:CPU:**
_output_shapes
: *
pattern
^s3://.*2
StaticRegexFullMatchc
ConstConst"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B.part2
Constl
Const_1Const"/device:CPU:**
_output_shapes
: *
dtype0*
valueB B
_temp/part2	
Const_1?
SelectSelectStaticRegexFullMatch:output:0Const:output:0Const_1:output:0"/device:CPU:**
T0*
_output_shapes
: 2
Selectt

StringJoin
StringJoinfile_prefixSelect:output:0"/device:CPU:**
N*
_output_shapes
: 2

StringJoinZ

num_shardsConst*
_output_shapes
: *
dtype0*
value	B :2

num_shards
ShardedFilename/shardConst"/device:CPU:0*
_output_shapes
: *
dtype0*
value	B : 2
ShardedFilename/shard?
ShardedFilenameShardedFilenameStringJoin:output:0ShardedFilename/shard:output:0num_shards:output:0"/device:CPU:0*
_output_shapes
: 2
ShardedFilename?
SaveV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*?
value?B?B6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
SaveV2/tensor_names?
SaveV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*K
valueBB@B B B B B B B B B B B B B B B B B B B B B B B B B B B B 2
SaveV2/shape_and_slices?
SaveV2SaveV2ShardedFilename:filename:0SaveV2/tensor_names:output:0 SaveV2/shape_and_slices:output:0+savev2_conv1d_20_kernel_read_readvariableop)savev2_conv1d_20_bias_read_readvariableop+savev2_conv1d_21_kernel_read_readvariableop)savev2_conv1d_21_bias_read_readvariableop*savev2_dense_13_kernel_read_readvariableop(savev2_dense_13_bias_read_readvariableop$savev2_adam_iter_read_readvariableop&savev2_adam_beta_1_read_readvariableop&savev2_adam_beta_2_read_readvariableop%savev2_adam_decay_read_readvariableop-savev2_adam_learning_rate_read_readvariableop savev2_total_read_readvariableop savev2_count_read_readvariableop"savev2_total_1_read_readvariableop"savev2_count_1_read_readvariableop2savev2_adam_conv1d_20_kernel_m_read_readvariableop0savev2_adam_conv1d_20_bias_m_read_readvariableop2savev2_adam_conv1d_21_kernel_m_read_readvariableop0savev2_adam_conv1d_21_bias_m_read_readvariableop1savev2_adam_dense_13_kernel_m_read_readvariableop/savev2_adam_dense_13_bias_m_read_readvariableop2savev2_adam_conv1d_20_kernel_v_read_readvariableop0savev2_adam_conv1d_20_bias_v_read_readvariableop2savev2_adam_conv1d_21_kernel_v_read_readvariableop0savev2_adam_conv1d_21_bias_v_read_readvariableop1savev2_adam_dense_13_kernel_v_read_readvariableop/savev2_adam_dense_13_bias_v_read_readvariableopsavev2_const"/device:CPU:0*
_output_shapes
 **
dtypes 
2	2
SaveV2?
&MergeV2Checkpoints/checkpoint_prefixesPackShardedFilename:filename:0^SaveV2"/device:CPU:0*
N*
T0*
_output_shapes
:2(
&MergeV2Checkpoints/checkpoint_prefixes?
MergeV2CheckpointsMergeV2Checkpoints/MergeV2Checkpoints/checkpoint_prefixes:output:0file_prefix"/device:CPU:0*
_output_shapes
 2
MergeV2Checkpointsr
IdentityIdentityfile_prefix^MergeV2Checkpoints"/device:CPU:0*
T0*
_output_shapes
: 2

Identitym

Identity_1IdentityIdentity:output:0^MergeV2Checkpoints*
T0*
_output_shapes
: 2

Identity_1"!

identity_1Identity_1:output:0*?
_input_shapes?
?: :

:
:

:::: : : : : : : : : :

:
:

::::

:
:

:::: 2(
MergeV2CheckpointsMergeV2Checkpoints:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix:($
"
_output_shapes
:

: 

_output_shapes
:
:($
"
_output_shapes
:

: 

_output_shapes
::$ 

_output_shapes

:: 

_output_shapes
::

_output_shapes
: :

_output_shapes
: :	

_output_shapes
: :


_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :

_output_shapes
: :($
"
_output_shapes
:

: 

_output_shapes
:
:($
"
_output_shapes
:

: 

_output_shapes
::$ 

_output_shapes

:: 

_output_shapes
::($
"
_output_shapes
:

: 

_output_shapes
:
:($
"
_output_shapes
:

: 

_output_shapes
::$ 

_output_shapes

:: 

_output_shapes
::

_output_shapes
: 
?
h
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_130694

inputs
identityb
ExpandDims/dimConst*
_output_shapes
: *
dtype0*
value	B :2
ExpandDims/dim?

ExpandDims
ExpandDimsinputsExpandDims/dim:output:0*
T0*A
_output_shapes/
-:+???????????????????????????2

ExpandDims?
MaxPoolMaxPoolExpandDims:output:0*A
_output_shapes/
-:+???????????????????????????*
ksize
*
paddingVALID*
strides
2	
MaxPool?
SqueezeSqueezeMaxPool:output:0*
T0*=
_output_shapes+
):'???????????????????????????*
squeeze_dims
2	
Squeezez
IdentityIdentitySqueeze:output:0*
T0*=
_output_shapes+
):'???????????????????????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*<
_input_shapes+
):'???????????????????????????:e a
=
_output_shapes+
):'???????????????????????????
 
_user_specified_nameinputs
?
?
I__inference_sequential_10_layer_call_and_return_conditional_losses_130976
conv1d_20_input&
conv1d_20_130957:


conv1d_20_130959:
&
conv1d_21_130963:


conv1d_21_130965:!
dense_13_130970:
dense_13_130972:
identity??!conv1d_20/StatefulPartitionedCall?!conv1d_21/StatefulPartitionedCall? dense_13/StatefulPartitionedCall?
!conv1d_20/StatefulPartitionedCallStatefulPartitionedCallconv1d_20_inputconv1d_20_130957conv1d_20_130959*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????G
*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_20_layer_call_and_return_conditional_losses_1307472#
!conv1d_20/StatefulPartitionedCall?
 max_pooling1d_10/PartitionedCallPartitionedCall*conv1d_20/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????
* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *U
fPRN
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_1306942"
 max_pooling1d_10/PartitionedCall?
!conv1d_21/StatefulPartitionedCallStatefulPartitionedCall)max_pooling1d_10/PartitionedCall:output:0conv1d_21_130963conv1d_21_130965*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_21_layer_call_and_return_conditional_losses_1307702#
!conv1d_21/StatefulPartitionedCall?
+global_average_pooling1d_10/PartitionedCallPartitionedCall*conv1d_21/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *`
f[RY
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_1307812-
+global_average_pooling1d_10/PartitionedCall?
dropout_10/PartitionedCallPartitionedCall4global_average_pooling1d_10/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_dropout_10_layer_call_and_return_conditional_losses_1307882
dropout_10/PartitionedCall?
 dense_13/StatefulPartitionedCallStatefulPartitionedCall#dropout_10/PartitionedCall:output:0dense_13_130970dense_13_130972*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *M
fHRF
D__inference_dense_13_layer_call_and_return_conditional_losses_1308012"
 dense_13/StatefulPartitionedCall?
IdentityIdentity)dense_13/StatefulPartitionedCall:output:0"^conv1d_20/StatefulPartitionedCall"^conv1d_21/StatefulPartitionedCall!^dense_13/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 2F
!conv1d_20/StatefulPartitionedCall!conv1d_20/StatefulPartitionedCall2F
!conv1d_21/StatefulPartitionedCall!conv1d_21/StatefulPartitionedCall2D
 dense_13/StatefulPartitionedCall dense_13/StatefulPartitionedCall:\ X
+
_output_shapes
:?????????P
)
_user_specified_nameconv1d_20_input
?
G
+__inference_dropout_10_layer_call_fn_131242

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_dropout_10_layer_call_and_return_conditional_losses_1307882
PartitionedCalll
IdentityIdentityPartitionedCall:output:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
d
F__inference_dropout_10_layer_call_and_return_conditional_losses_131225

inputs

identity_1Z
IdentityIdentityinputs*
T0*'
_output_shapes
:?????????2

Identityi

Identity_1IdentityIdentity:output:0*
T0*'
_output_shapes
:?????????2

Identity_1"!

identity_1Identity_1:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
E__inference_conv1d_21_layer_call_and_return_conditional_losses_131189

inputsA
+conv1d_expanddims_1_readvariableop_resource:

-
biasadd_readvariableop_resource:
identity??BiasAdd/ReadVariableOp?"conv1d/ExpandDims_1/ReadVariableOpy
conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2
conv1d/ExpandDims/dim?
conv1d/ExpandDims
ExpandDimsinputsconv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????
2
conv1d/ExpandDims?
"conv1d/ExpandDims_1/ReadVariableOpReadVariableOp+conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02$
"conv1d/ExpandDims_1/ReadVariableOpt
conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2
conv1d/ExpandDims_1/dim?
conv1d/ExpandDims_1
ExpandDims*conv1d/ExpandDims_1/ReadVariableOp:value:0 conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d/ExpandDims_1?
conv1dConv2Dconv1d/ExpandDims:output:0conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv1d?
conv1d/SqueezeSqueezeconv1d:output:0*
T0*+
_output_shapes
:?????????*
squeeze_dims

?????????2
conv1d/Squeeze?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddconv1d/Squeeze:output:0BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????2	
BiasAdd\
ReluReluBiasAdd:output:0*
T0*+
_output_shapes
:?????????2
Relu?
IdentityIdentityRelu:activations:0^BiasAdd/ReadVariableOp#^conv1d/ExpandDims_1/ReadVariableOp*
T0*+
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:?????????
: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2H
"conv1d/ExpandDims_1/ReadVariableOp"conv1d/ExpandDims_1/ReadVariableOp:S O
+
_output_shapes
:?????????

 
_user_specified_nameinputs
?H
?
!__inference__wrapped_model_130685
conv1d_20_inputY
Csequential_10_conv1d_20_conv1d_expanddims_1_readvariableop_resource:

E
7sequential_10_conv1d_20_biasadd_readvariableop_resource:
Y
Csequential_10_conv1d_21_conv1d_expanddims_1_readvariableop_resource:

E
7sequential_10_conv1d_21_biasadd_readvariableop_resource:G
5sequential_10_dense_13_matmul_readvariableop_resource:D
6sequential_10_dense_13_biasadd_readvariableop_resource:
identity??.sequential_10/conv1d_20/BiasAdd/ReadVariableOp?:sequential_10/conv1d_20/conv1d/ExpandDims_1/ReadVariableOp?.sequential_10/conv1d_21/BiasAdd/ReadVariableOp?:sequential_10/conv1d_21/conv1d/ExpandDims_1/ReadVariableOp?-sequential_10/dense_13/BiasAdd/ReadVariableOp?,sequential_10/dense_13/MatMul/ReadVariableOp?
-sequential_10/conv1d_20/conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2/
-sequential_10/conv1d_20/conv1d/ExpandDims/dim?
)sequential_10/conv1d_20/conv1d/ExpandDims
ExpandDimsconv1d_20_input6sequential_10/conv1d_20/conv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????P2+
)sequential_10/conv1d_20/conv1d/ExpandDims?
:sequential_10/conv1d_20/conv1d/ExpandDims_1/ReadVariableOpReadVariableOpCsequential_10_conv1d_20_conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02<
:sequential_10/conv1d_20/conv1d/ExpandDims_1/ReadVariableOp?
/sequential_10/conv1d_20/conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 21
/sequential_10/conv1d_20/conv1d/ExpandDims_1/dim?
+sequential_10/conv1d_20/conv1d/ExpandDims_1
ExpandDimsBsequential_10/conv1d_20/conv1d/ExpandDims_1/ReadVariableOp:value:08sequential_10/conv1d_20/conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2-
+sequential_10/conv1d_20/conv1d/ExpandDims_1?
sequential_10/conv1d_20/conv1dConv2D2sequential_10/conv1d_20/conv1d/ExpandDims:output:04sequential_10/conv1d_20/conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????G
*
paddingVALID*
strides
2 
sequential_10/conv1d_20/conv1d?
&sequential_10/conv1d_20/conv1d/SqueezeSqueeze'sequential_10/conv1d_20/conv1d:output:0*
T0*+
_output_shapes
:?????????G
*
squeeze_dims

?????????2(
&sequential_10/conv1d_20/conv1d/Squeeze?
.sequential_10/conv1d_20/BiasAdd/ReadVariableOpReadVariableOp7sequential_10_conv1d_20_biasadd_readvariableop_resource*
_output_shapes
:
*
dtype020
.sequential_10/conv1d_20/BiasAdd/ReadVariableOp?
sequential_10/conv1d_20/BiasAddBiasAdd/sequential_10/conv1d_20/conv1d/Squeeze:output:06sequential_10/conv1d_20/BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????G
2!
sequential_10/conv1d_20/BiasAdd?
sequential_10/conv1d_20/ReluRelu(sequential_10/conv1d_20/BiasAdd:output:0*
T0*+
_output_shapes
:?????????G
2
sequential_10/conv1d_20/Relu?
-sequential_10/max_pooling1d_10/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
value	B :2/
-sequential_10/max_pooling1d_10/ExpandDims/dim?
)sequential_10/max_pooling1d_10/ExpandDims
ExpandDims*sequential_10/conv1d_20/Relu:activations:06sequential_10/max_pooling1d_10/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????G
2+
)sequential_10/max_pooling1d_10/ExpandDims?
&sequential_10/max_pooling1d_10/MaxPoolMaxPool2sequential_10/max_pooling1d_10/ExpandDims:output:0*/
_output_shapes
:?????????
*
ksize
*
paddingVALID*
strides
2(
&sequential_10/max_pooling1d_10/MaxPool?
&sequential_10/max_pooling1d_10/SqueezeSqueeze/sequential_10/max_pooling1d_10/MaxPool:output:0*
T0*+
_output_shapes
:?????????
*
squeeze_dims
2(
&sequential_10/max_pooling1d_10/Squeeze?
-sequential_10/conv1d_21/conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2/
-sequential_10/conv1d_21/conv1d/ExpandDims/dim?
)sequential_10/conv1d_21/conv1d/ExpandDims
ExpandDims/sequential_10/max_pooling1d_10/Squeeze:output:06sequential_10/conv1d_21/conv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????
2+
)sequential_10/conv1d_21/conv1d/ExpandDims?
:sequential_10/conv1d_21/conv1d/ExpandDims_1/ReadVariableOpReadVariableOpCsequential_10_conv1d_21_conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02<
:sequential_10/conv1d_21/conv1d/ExpandDims_1/ReadVariableOp?
/sequential_10/conv1d_21/conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 21
/sequential_10/conv1d_21/conv1d/ExpandDims_1/dim?
+sequential_10/conv1d_21/conv1d/ExpandDims_1
ExpandDimsBsequential_10/conv1d_21/conv1d/ExpandDims_1/ReadVariableOp:value:08sequential_10/conv1d_21/conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2-
+sequential_10/conv1d_21/conv1d/ExpandDims_1?
sequential_10/conv1d_21/conv1dConv2D2sequential_10/conv1d_21/conv1d/ExpandDims:output:04sequential_10/conv1d_21/conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2 
sequential_10/conv1d_21/conv1d?
&sequential_10/conv1d_21/conv1d/SqueezeSqueeze'sequential_10/conv1d_21/conv1d:output:0*
T0*+
_output_shapes
:?????????*
squeeze_dims

?????????2(
&sequential_10/conv1d_21/conv1d/Squeeze?
.sequential_10/conv1d_21/BiasAdd/ReadVariableOpReadVariableOp7sequential_10_conv1d_21_biasadd_readvariableop_resource*
_output_shapes
:*
dtype020
.sequential_10/conv1d_21/BiasAdd/ReadVariableOp?
sequential_10/conv1d_21/BiasAddBiasAdd/sequential_10/conv1d_21/conv1d/Squeeze:output:06sequential_10/conv1d_21/BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????2!
sequential_10/conv1d_21/BiasAdd?
sequential_10/conv1d_21/ReluRelu(sequential_10/conv1d_21/BiasAdd:output:0*
T0*+
_output_shapes
:?????????2
sequential_10/conv1d_21/Relu?
@sequential_10/global_average_pooling1d_10/Mean/reduction_indicesConst*
_output_shapes
: *
dtype0*
value	B :2B
@sequential_10/global_average_pooling1d_10/Mean/reduction_indices?
.sequential_10/global_average_pooling1d_10/MeanMean*sequential_10/conv1d_21/Relu:activations:0Isequential_10/global_average_pooling1d_10/Mean/reduction_indices:output:0*
T0*'
_output_shapes
:?????????20
.sequential_10/global_average_pooling1d_10/Mean?
!sequential_10/dropout_10/IdentityIdentity7sequential_10/global_average_pooling1d_10/Mean:output:0*
T0*'
_output_shapes
:?????????2#
!sequential_10/dropout_10/Identity?
,sequential_10/dense_13/MatMul/ReadVariableOpReadVariableOp5sequential_10_dense_13_matmul_readvariableop_resource*
_output_shapes

:*
dtype02.
,sequential_10/dense_13/MatMul/ReadVariableOp?
sequential_10/dense_13/MatMulMatMul*sequential_10/dropout_10/Identity:output:04sequential_10/dense_13/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
sequential_10/dense_13/MatMul?
-sequential_10/dense_13/BiasAdd/ReadVariableOpReadVariableOp6sequential_10_dense_13_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02/
-sequential_10/dense_13/BiasAdd/ReadVariableOp?
sequential_10/dense_13/BiasAddBiasAdd'sequential_10/dense_13/MatMul:product:05sequential_10/dense_13/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2 
sequential_10/dense_13/BiasAdd?
sequential_10/dense_13/SoftmaxSoftmax'sequential_10/dense_13/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2 
sequential_10/dense_13/Softmax?
IdentityIdentity(sequential_10/dense_13/Softmax:softmax:0/^sequential_10/conv1d_20/BiasAdd/ReadVariableOp;^sequential_10/conv1d_20/conv1d/ExpandDims_1/ReadVariableOp/^sequential_10/conv1d_21/BiasAdd/ReadVariableOp;^sequential_10/conv1d_21/conv1d/ExpandDims_1/ReadVariableOp.^sequential_10/dense_13/BiasAdd/ReadVariableOp-^sequential_10/dense_13/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 2`
.sequential_10/conv1d_20/BiasAdd/ReadVariableOp.sequential_10/conv1d_20/BiasAdd/ReadVariableOp2x
:sequential_10/conv1d_20/conv1d/ExpandDims_1/ReadVariableOp:sequential_10/conv1d_20/conv1d/ExpandDims_1/ReadVariableOp2`
.sequential_10/conv1d_21/BiasAdd/ReadVariableOp.sequential_10/conv1d_21/BiasAdd/ReadVariableOp2x
:sequential_10/conv1d_21/conv1d/ExpandDims_1/ReadVariableOp:sequential_10/conv1d_21/conv1d/ExpandDims_1/ReadVariableOp2^
-sequential_10/dense_13/BiasAdd/ReadVariableOp-sequential_10/dense_13/BiasAdd/ReadVariableOp2\
,sequential_10/dense_13/MatMul/ReadVariableOp,sequential_10/dense_13/MatMul/ReadVariableOp:\ X
+
_output_shapes
:?????????P
)
_user_specified_nameconv1d_20_input
?

?
D__inference_dense_13_layer_call_and_return_conditional_losses_130801

inputs0
matmul_readvariableop_resource:-
biasadd_readvariableop_resource:
identity??BiasAdd/ReadVariableOp?MatMul/ReadVariableOp?
MatMul/ReadVariableOpReadVariableOpmatmul_readvariableop_resource*
_output_shapes

:*
dtype02
MatMul/ReadVariableOps
MatMulMatMulinputsMatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
MatMul?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddMatMul:product:0BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2	
BiasAdda
SoftmaxSoftmaxBiasAdd:output:0*
T0*'
_output_shapes
:?????????2	
Softmax?
IdentityIdentitySoftmax:softmax:0^BiasAdd/ReadVariableOp^MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:?????????: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2.
MatMul/ReadVariableOpMatMul/ReadVariableOp:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
s
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_131204

inputs
identityr
Mean/reduction_indicesConst*
_output_shapes
: *
dtype0*
value	B :2
Mean/reduction_indicesx
MeanMeaninputsMean/reduction_indices:output:0*
T0*0
_output_shapes
:??????????????????2
Meanj
IdentityIdentityMean:output:0*
T0*0
_output_shapes
:??????????????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*<
_input_shapes+
):'???????????????????????????:e a
=
_output_shapes+
):'???????????????????????????
 
_user_specified_nameinputs
?
?
*__inference_conv1d_21_layer_call_fn_131198

inputs
unknown:


	unknown_0:
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_21_layer_call_and_return_conditional_losses_1307702
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*+
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:?????????
: : 22
StatefulPartitionedCallStatefulPartitionedCall:S O
+
_output_shapes
:?????????

 
_user_specified_nameinputs
?
?
E__inference_conv1d_20_layer_call_and_return_conditional_losses_131164

inputsA
+conv1d_expanddims_1_readvariableop_resource:

-
biasadd_readvariableop_resource:

identity??BiasAdd/ReadVariableOp?"conv1d/ExpandDims_1/ReadVariableOpy
conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2
conv1d/ExpandDims/dim?
conv1d/ExpandDims
ExpandDimsinputsconv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????P2
conv1d/ExpandDims?
"conv1d/ExpandDims_1/ReadVariableOpReadVariableOp+conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02$
"conv1d/ExpandDims_1/ReadVariableOpt
conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2
conv1d/ExpandDims_1/dim?
conv1d/ExpandDims_1
ExpandDims*conv1d/ExpandDims_1/ReadVariableOp:value:0 conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d/ExpandDims_1?
conv1dConv2Dconv1d/ExpandDims:output:0conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????G
*
paddingVALID*
strides
2
conv1d?
conv1d/SqueezeSqueezeconv1d:output:0*
T0*+
_output_shapes
:?????????G
*
squeeze_dims

?????????2
conv1d/Squeeze?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:
*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddconv1d/Squeeze:output:0BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????G
2	
BiasAdd\
ReluReluBiasAdd:output:0*
T0*+
_output_shapes
:?????????G
2
Relu?
IdentityIdentityRelu:activations:0^BiasAdd/ReadVariableOp#^conv1d/ExpandDims_1/ReadVariableOp*
T0*+
_output_shapes
:?????????G
2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:?????????P: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2H
"conv1d/ExpandDims_1/ReadVariableOp"conv1d/ExpandDims_1/ReadVariableOp:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
?
I__inference_sequential_10_layer_call_and_return_conditional_losses_130998
conv1d_20_input&
conv1d_20_130979:


conv1d_20_130981:
&
conv1d_21_130985:


conv1d_21_130987:!
dense_13_130992:
dense_13_130994:
identity??!conv1d_20/StatefulPartitionedCall?!conv1d_21/StatefulPartitionedCall? dense_13/StatefulPartitionedCall?"dropout_10/StatefulPartitionedCall?
!conv1d_20/StatefulPartitionedCallStatefulPartitionedCallconv1d_20_inputconv1d_20_130979conv1d_20_130981*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????G
*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_20_layer_call_and_return_conditional_losses_1307472#
!conv1d_20/StatefulPartitionedCall?
 max_pooling1d_10/PartitionedCallPartitionedCall*conv1d_20/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????
* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *U
fPRN
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_1306942"
 max_pooling1d_10/PartitionedCall?
!conv1d_21/StatefulPartitionedCallStatefulPartitionedCall)max_pooling1d_10/PartitionedCall:output:0conv1d_21_130985conv1d_21_130987*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_21_layer_call_and_return_conditional_losses_1307702#
!conv1d_21/StatefulPartitionedCall?
+global_average_pooling1d_10/PartitionedCallPartitionedCall*conv1d_21/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *`
f[RY
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_1307812-
+global_average_pooling1d_10/PartitionedCall?
"dropout_10/StatefulPartitionedCallStatefulPartitionedCall4global_average_pooling1d_10/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_dropout_10_layer_call_and_return_conditional_losses_1308532$
"dropout_10/StatefulPartitionedCall?
 dense_13/StatefulPartitionedCallStatefulPartitionedCall+dropout_10/StatefulPartitionedCall:output:0dense_13_130992dense_13_130994*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *M
fHRF
D__inference_dense_13_layer_call_and_return_conditional_losses_1308012"
 dense_13/StatefulPartitionedCall?
IdentityIdentity)dense_13/StatefulPartitionedCall:output:0"^conv1d_20/StatefulPartitionedCall"^conv1d_21/StatefulPartitionedCall!^dense_13/StatefulPartitionedCall#^dropout_10/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 2F
!conv1d_20/StatefulPartitionedCall!conv1d_20/StatefulPartitionedCall2F
!conv1d_21/StatefulPartitionedCall!conv1d_21/StatefulPartitionedCall2D
 dense_13/StatefulPartitionedCall dense_13/StatefulPartitionedCall2H
"dropout_10/StatefulPartitionedCall"dropout_10/StatefulPartitionedCall:\ X
+
_output_shapes
:?????????P
)
_user_specified_nameconv1d_20_input
?
d
F__inference_dropout_10_layer_call_and_return_conditional_losses_130788

inputs

identity_1Z
IdentityIdentityinputs*
T0*'
_output_shapes
:?????????2

Identityi

Identity_1IdentityIdentity:output:0*
T0*'
_output_shapes
:?????????2

Identity_1"!

identity_1Identity_1:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?:
?
I__inference_sequential_10_layer_call_and_return_conditional_losses_131099

inputsK
5conv1d_20_conv1d_expanddims_1_readvariableop_resource:

7
)conv1d_20_biasadd_readvariableop_resource:
K
5conv1d_21_conv1d_expanddims_1_readvariableop_resource:

7
)conv1d_21_biasadd_readvariableop_resource:9
'dense_13_matmul_readvariableop_resource:6
(dense_13_biasadd_readvariableop_resource:
identity?? conv1d_20/BiasAdd/ReadVariableOp?,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp? conv1d_21/BiasAdd/ReadVariableOp?,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp?dense_13/BiasAdd/ReadVariableOp?dense_13/MatMul/ReadVariableOp?
conv1d_20/conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2!
conv1d_20/conv1d/ExpandDims/dim?
conv1d_20/conv1d/ExpandDims
ExpandDimsinputs(conv1d_20/conv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????P2
conv1d_20/conv1d/ExpandDims?
,conv1d_20/conv1d/ExpandDims_1/ReadVariableOpReadVariableOp5conv1d_20_conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02.
,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp?
!conv1d_20/conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2#
!conv1d_20/conv1d/ExpandDims_1/dim?
conv1d_20/conv1d/ExpandDims_1
ExpandDims4conv1d_20/conv1d/ExpandDims_1/ReadVariableOp:value:0*conv1d_20/conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d_20/conv1d/ExpandDims_1?
conv1d_20/conv1dConv2D$conv1d_20/conv1d/ExpandDims:output:0&conv1d_20/conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????G
*
paddingVALID*
strides
2
conv1d_20/conv1d?
conv1d_20/conv1d/SqueezeSqueezeconv1d_20/conv1d:output:0*
T0*+
_output_shapes
:?????????G
*
squeeze_dims

?????????2
conv1d_20/conv1d/Squeeze?
 conv1d_20/BiasAdd/ReadVariableOpReadVariableOp)conv1d_20_biasadd_readvariableop_resource*
_output_shapes
:
*
dtype02"
 conv1d_20/BiasAdd/ReadVariableOp?
conv1d_20/BiasAddBiasAdd!conv1d_20/conv1d/Squeeze:output:0(conv1d_20/BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????G
2
conv1d_20/BiasAddz
conv1d_20/ReluReluconv1d_20/BiasAdd:output:0*
T0*+
_output_shapes
:?????????G
2
conv1d_20/Relu?
max_pooling1d_10/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
value	B :2!
max_pooling1d_10/ExpandDims/dim?
max_pooling1d_10/ExpandDims
ExpandDimsconv1d_20/Relu:activations:0(max_pooling1d_10/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????G
2
max_pooling1d_10/ExpandDims?
max_pooling1d_10/MaxPoolMaxPool$max_pooling1d_10/ExpandDims:output:0*/
_output_shapes
:?????????
*
ksize
*
paddingVALID*
strides
2
max_pooling1d_10/MaxPool?
max_pooling1d_10/SqueezeSqueeze!max_pooling1d_10/MaxPool:output:0*
T0*+
_output_shapes
:?????????
*
squeeze_dims
2
max_pooling1d_10/Squeeze?
conv1d_21/conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2!
conv1d_21/conv1d/ExpandDims/dim?
conv1d_21/conv1d/ExpandDims
ExpandDims!max_pooling1d_10/Squeeze:output:0(conv1d_21/conv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????
2
conv1d_21/conv1d/ExpandDims?
,conv1d_21/conv1d/ExpandDims_1/ReadVariableOpReadVariableOp5conv1d_21_conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02.
,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp?
!conv1d_21/conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2#
!conv1d_21/conv1d/ExpandDims_1/dim?
conv1d_21/conv1d/ExpandDims_1
ExpandDims4conv1d_21/conv1d/ExpandDims_1/ReadVariableOp:value:0*conv1d_21/conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d_21/conv1d/ExpandDims_1?
conv1d_21/conv1dConv2D$conv1d_21/conv1d/ExpandDims:output:0&conv1d_21/conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv1d_21/conv1d?
conv1d_21/conv1d/SqueezeSqueezeconv1d_21/conv1d:output:0*
T0*+
_output_shapes
:?????????*
squeeze_dims

?????????2
conv1d_21/conv1d/Squeeze?
 conv1d_21/BiasAdd/ReadVariableOpReadVariableOp)conv1d_21_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02"
 conv1d_21/BiasAdd/ReadVariableOp?
conv1d_21/BiasAddBiasAdd!conv1d_21/conv1d/Squeeze:output:0(conv1d_21/BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????2
conv1d_21/BiasAddz
conv1d_21/ReluReluconv1d_21/BiasAdd:output:0*
T0*+
_output_shapes
:?????????2
conv1d_21/Relu?
2global_average_pooling1d_10/Mean/reduction_indicesConst*
_output_shapes
: *
dtype0*
value	B :24
2global_average_pooling1d_10/Mean/reduction_indices?
 global_average_pooling1d_10/MeanMeanconv1d_21/Relu:activations:0;global_average_pooling1d_10/Mean/reduction_indices:output:0*
T0*'
_output_shapes
:?????????2"
 global_average_pooling1d_10/Mean?
dropout_10/IdentityIdentity)global_average_pooling1d_10/Mean:output:0*
T0*'
_output_shapes
:?????????2
dropout_10/Identity?
dense_13/MatMul/ReadVariableOpReadVariableOp'dense_13_matmul_readvariableop_resource*
_output_shapes

:*
dtype02 
dense_13/MatMul/ReadVariableOp?
dense_13/MatMulMatMuldropout_10/Identity:output:0&dense_13/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_13/MatMul?
dense_13/BiasAdd/ReadVariableOpReadVariableOp(dense_13_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_13/BiasAdd/ReadVariableOp?
dense_13/BiasAddBiasAdddense_13/MatMul:product:0'dense_13/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_13/BiasAdd|
dense_13/SoftmaxSoftmaxdense_13/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
dense_13/Softmax?
IdentityIdentitydense_13/Softmax:softmax:0!^conv1d_20/BiasAdd/ReadVariableOp-^conv1d_20/conv1d/ExpandDims_1/ReadVariableOp!^conv1d_21/BiasAdd/ReadVariableOp-^conv1d_21/conv1d/ExpandDims_1/ReadVariableOp ^dense_13/BiasAdd/ReadVariableOp^dense_13/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 2D
 conv1d_20/BiasAdd/ReadVariableOp conv1d_20/BiasAdd/ReadVariableOp2\
,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp2D
 conv1d_21/BiasAdd/ReadVariableOp conv1d_21/BiasAdd/ReadVariableOp2\
,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp2B
dense_13/BiasAdd/ReadVariableOpdense_13/BiasAdd/ReadVariableOp2@
dense_13/MatMul/ReadVariableOpdense_13/MatMul/ReadVariableOp:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
X
<__inference_global_average_pooling1d_10_layer_call_fn_131215

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *0
_output_shapes
:??????????????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *`
f[RY
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_1307102
PartitionedCallu
IdentityIdentityPartitionedCall:output:0*
T0*0
_output_shapes
:??????????????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*<
_input_shapes+
):'???????????????????????????:e a
=
_output_shapes+
):'???????????????????????????
 
_user_specified_nameinputs
?t
?
"__inference__traced_restore_131462
file_prefix7
!assignvariableop_conv1d_20_kernel:

/
!assignvariableop_1_conv1d_20_bias:
9
#assignvariableop_2_conv1d_21_kernel:

/
!assignvariableop_3_conv1d_21_bias:4
"assignvariableop_4_dense_13_kernel:.
 assignvariableop_5_dense_13_bias:&
assignvariableop_6_adam_iter:	 (
assignvariableop_7_adam_beta_1: (
assignvariableop_8_adam_beta_2: '
assignvariableop_9_adam_decay: 0
&assignvariableop_10_adam_learning_rate: #
assignvariableop_11_total: #
assignvariableop_12_count: %
assignvariableop_13_total_1: %
assignvariableop_14_count_1: A
+assignvariableop_15_adam_conv1d_20_kernel_m:

7
)assignvariableop_16_adam_conv1d_20_bias_m:
A
+assignvariableop_17_adam_conv1d_21_kernel_m:

7
)assignvariableop_18_adam_conv1d_21_bias_m:<
*assignvariableop_19_adam_dense_13_kernel_m:6
(assignvariableop_20_adam_dense_13_bias_m:A
+assignvariableop_21_adam_conv1d_20_kernel_v:

7
)assignvariableop_22_adam_conv1d_20_bias_v:
A
+assignvariableop_23_adam_conv1d_21_kernel_v:

7
)assignvariableop_24_adam_conv1d_21_bias_v:<
*assignvariableop_25_adam_dense_13_kernel_v:6
(assignvariableop_26_adam_dense_13_bias_v:
identity_28??AssignVariableOp?AssignVariableOp_1?AssignVariableOp_10?AssignVariableOp_11?AssignVariableOp_12?AssignVariableOp_13?AssignVariableOp_14?AssignVariableOp_15?AssignVariableOp_16?AssignVariableOp_17?AssignVariableOp_18?AssignVariableOp_19?AssignVariableOp_2?AssignVariableOp_20?AssignVariableOp_21?AssignVariableOp_22?AssignVariableOp_23?AssignVariableOp_24?AssignVariableOp_25?AssignVariableOp_26?AssignVariableOp_3?AssignVariableOp_4?AssignVariableOp_5?AssignVariableOp_6?AssignVariableOp_7?AssignVariableOp_8?AssignVariableOp_9?
RestoreV2/tensor_namesConst"/device:CPU:0*
_output_shapes
:*
dtype0*?
value?B?B6layer_with_weights-0/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-0/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-1/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-1/bias/.ATTRIBUTES/VARIABLE_VALUEB6layer_with_weights-2/kernel/.ATTRIBUTES/VARIABLE_VALUEB4layer_with_weights-2/bias/.ATTRIBUTES/VARIABLE_VALUEB)optimizer/iter/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_1/.ATTRIBUTES/VARIABLE_VALUEB+optimizer/beta_2/.ATTRIBUTES/VARIABLE_VALUEB*optimizer/decay/.ATTRIBUTES/VARIABLE_VALUEB2optimizer/learning_rate/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/0/count/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/total/.ATTRIBUTES/VARIABLE_VALUEB4keras_api/metrics/1/count/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/m/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-0/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-0/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-1/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-1/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBRlayer_with_weights-2/kernel/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEBPlayer_with_weights-2/bias/.OPTIMIZER_SLOT/optimizer/v/.ATTRIBUTES/VARIABLE_VALUEB_CHECKPOINTABLE_OBJECT_GRAPH2
RestoreV2/tensor_names?
RestoreV2/shape_and_slicesConst"/device:CPU:0*
_output_shapes
:*
dtype0*K
valueBB@B B B B B B B B B B B B B B B B B B B B B B B B B B B B 2
RestoreV2/shape_and_slices?
	RestoreV2	RestoreV2file_prefixRestoreV2/tensor_names:output:0#RestoreV2/shape_and_slices:output:0"/device:CPU:0*?
_output_shapesr
p::::::::::::::::::::::::::::**
dtypes 
2	2
	RestoreV2g
IdentityIdentityRestoreV2:tensors:0"/device:CPU:0*
T0*
_output_shapes
:2

Identity?
AssignVariableOpAssignVariableOp!assignvariableop_conv1d_20_kernelIdentity:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOpk

Identity_1IdentityRestoreV2:tensors:1"/device:CPU:0*
T0*
_output_shapes
:2

Identity_1?
AssignVariableOp_1AssignVariableOp!assignvariableop_1_conv1d_20_biasIdentity_1:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_1k

Identity_2IdentityRestoreV2:tensors:2"/device:CPU:0*
T0*
_output_shapes
:2

Identity_2?
AssignVariableOp_2AssignVariableOp#assignvariableop_2_conv1d_21_kernelIdentity_2:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_2k

Identity_3IdentityRestoreV2:tensors:3"/device:CPU:0*
T0*
_output_shapes
:2

Identity_3?
AssignVariableOp_3AssignVariableOp!assignvariableop_3_conv1d_21_biasIdentity_3:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_3k

Identity_4IdentityRestoreV2:tensors:4"/device:CPU:0*
T0*
_output_shapes
:2

Identity_4?
AssignVariableOp_4AssignVariableOp"assignvariableop_4_dense_13_kernelIdentity_4:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_4k

Identity_5IdentityRestoreV2:tensors:5"/device:CPU:0*
T0*
_output_shapes
:2

Identity_5?
AssignVariableOp_5AssignVariableOp assignvariableop_5_dense_13_biasIdentity_5:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_5k

Identity_6IdentityRestoreV2:tensors:6"/device:CPU:0*
T0	*
_output_shapes
:2

Identity_6?
AssignVariableOp_6AssignVariableOpassignvariableop_6_adam_iterIdentity_6:output:0"/device:CPU:0*
_output_shapes
 *
dtype0	2
AssignVariableOp_6k

Identity_7IdentityRestoreV2:tensors:7"/device:CPU:0*
T0*
_output_shapes
:2

Identity_7?
AssignVariableOp_7AssignVariableOpassignvariableop_7_adam_beta_1Identity_7:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_7k

Identity_8IdentityRestoreV2:tensors:8"/device:CPU:0*
T0*
_output_shapes
:2

Identity_8?
AssignVariableOp_8AssignVariableOpassignvariableop_8_adam_beta_2Identity_8:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_8k

Identity_9IdentityRestoreV2:tensors:9"/device:CPU:0*
T0*
_output_shapes
:2

Identity_9?
AssignVariableOp_9AssignVariableOpassignvariableop_9_adam_decayIdentity_9:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_9n
Identity_10IdentityRestoreV2:tensors:10"/device:CPU:0*
T0*
_output_shapes
:2
Identity_10?
AssignVariableOp_10AssignVariableOp&assignvariableop_10_adam_learning_rateIdentity_10:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_10n
Identity_11IdentityRestoreV2:tensors:11"/device:CPU:0*
T0*
_output_shapes
:2
Identity_11?
AssignVariableOp_11AssignVariableOpassignvariableop_11_totalIdentity_11:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_11n
Identity_12IdentityRestoreV2:tensors:12"/device:CPU:0*
T0*
_output_shapes
:2
Identity_12?
AssignVariableOp_12AssignVariableOpassignvariableop_12_countIdentity_12:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_12n
Identity_13IdentityRestoreV2:tensors:13"/device:CPU:0*
T0*
_output_shapes
:2
Identity_13?
AssignVariableOp_13AssignVariableOpassignvariableop_13_total_1Identity_13:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_13n
Identity_14IdentityRestoreV2:tensors:14"/device:CPU:0*
T0*
_output_shapes
:2
Identity_14?
AssignVariableOp_14AssignVariableOpassignvariableop_14_count_1Identity_14:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_14n
Identity_15IdentityRestoreV2:tensors:15"/device:CPU:0*
T0*
_output_shapes
:2
Identity_15?
AssignVariableOp_15AssignVariableOp+assignvariableop_15_adam_conv1d_20_kernel_mIdentity_15:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_15n
Identity_16IdentityRestoreV2:tensors:16"/device:CPU:0*
T0*
_output_shapes
:2
Identity_16?
AssignVariableOp_16AssignVariableOp)assignvariableop_16_adam_conv1d_20_bias_mIdentity_16:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_16n
Identity_17IdentityRestoreV2:tensors:17"/device:CPU:0*
T0*
_output_shapes
:2
Identity_17?
AssignVariableOp_17AssignVariableOp+assignvariableop_17_adam_conv1d_21_kernel_mIdentity_17:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_17n
Identity_18IdentityRestoreV2:tensors:18"/device:CPU:0*
T0*
_output_shapes
:2
Identity_18?
AssignVariableOp_18AssignVariableOp)assignvariableop_18_adam_conv1d_21_bias_mIdentity_18:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_18n
Identity_19IdentityRestoreV2:tensors:19"/device:CPU:0*
T0*
_output_shapes
:2
Identity_19?
AssignVariableOp_19AssignVariableOp*assignvariableop_19_adam_dense_13_kernel_mIdentity_19:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_19n
Identity_20IdentityRestoreV2:tensors:20"/device:CPU:0*
T0*
_output_shapes
:2
Identity_20?
AssignVariableOp_20AssignVariableOp(assignvariableop_20_adam_dense_13_bias_mIdentity_20:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_20n
Identity_21IdentityRestoreV2:tensors:21"/device:CPU:0*
T0*
_output_shapes
:2
Identity_21?
AssignVariableOp_21AssignVariableOp+assignvariableop_21_adam_conv1d_20_kernel_vIdentity_21:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_21n
Identity_22IdentityRestoreV2:tensors:22"/device:CPU:0*
T0*
_output_shapes
:2
Identity_22?
AssignVariableOp_22AssignVariableOp)assignvariableop_22_adam_conv1d_20_bias_vIdentity_22:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_22n
Identity_23IdentityRestoreV2:tensors:23"/device:CPU:0*
T0*
_output_shapes
:2
Identity_23?
AssignVariableOp_23AssignVariableOp+assignvariableop_23_adam_conv1d_21_kernel_vIdentity_23:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_23n
Identity_24IdentityRestoreV2:tensors:24"/device:CPU:0*
T0*
_output_shapes
:2
Identity_24?
AssignVariableOp_24AssignVariableOp)assignvariableop_24_adam_conv1d_21_bias_vIdentity_24:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_24n
Identity_25IdentityRestoreV2:tensors:25"/device:CPU:0*
T0*
_output_shapes
:2
Identity_25?
AssignVariableOp_25AssignVariableOp*assignvariableop_25_adam_dense_13_kernel_vIdentity_25:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_25n
Identity_26IdentityRestoreV2:tensors:26"/device:CPU:0*
T0*
_output_shapes
:2
Identity_26?
AssignVariableOp_26AssignVariableOp(assignvariableop_26_adam_dense_13_bias_vIdentity_26:output:0"/device:CPU:0*
_output_shapes
 *
dtype02
AssignVariableOp_269
NoOpNoOp"/device:CPU:0*
_output_shapes
 2
NoOp?
Identity_27Identityfile_prefix^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9^NoOp"/device:CPU:0*
T0*
_output_shapes
: 2
Identity_27?
Identity_28IdentityIdentity_27:output:0^AssignVariableOp^AssignVariableOp_1^AssignVariableOp_10^AssignVariableOp_11^AssignVariableOp_12^AssignVariableOp_13^AssignVariableOp_14^AssignVariableOp_15^AssignVariableOp_16^AssignVariableOp_17^AssignVariableOp_18^AssignVariableOp_19^AssignVariableOp_2^AssignVariableOp_20^AssignVariableOp_21^AssignVariableOp_22^AssignVariableOp_23^AssignVariableOp_24^AssignVariableOp_25^AssignVariableOp_26^AssignVariableOp_3^AssignVariableOp_4^AssignVariableOp_5^AssignVariableOp_6^AssignVariableOp_7^AssignVariableOp_8^AssignVariableOp_9*
T0*
_output_shapes
: 2
Identity_28"#
identity_28Identity_28:output:0*K
_input_shapes:
8: : : : : : : : : : : : : : : : : : : : : : : : : : : : 2$
AssignVariableOpAssignVariableOp2(
AssignVariableOp_1AssignVariableOp_12*
AssignVariableOp_10AssignVariableOp_102*
AssignVariableOp_11AssignVariableOp_112*
AssignVariableOp_12AssignVariableOp_122*
AssignVariableOp_13AssignVariableOp_132*
AssignVariableOp_14AssignVariableOp_142*
AssignVariableOp_15AssignVariableOp_152*
AssignVariableOp_16AssignVariableOp_162*
AssignVariableOp_17AssignVariableOp_172*
AssignVariableOp_18AssignVariableOp_182*
AssignVariableOp_19AssignVariableOp_192(
AssignVariableOp_2AssignVariableOp_22*
AssignVariableOp_20AssignVariableOp_202*
AssignVariableOp_21AssignVariableOp_212*
AssignVariableOp_22AssignVariableOp_222*
AssignVariableOp_23AssignVariableOp_232*
AssignVariableOp_24AssignVariableOp_242*
AssignVariableOp_25AssignVariableOp_252*
AssignVariableOp_26AssignVariableOp_262(
AssignVariableOp_3AssignVariableOp_32(
AssignVariableOp_4AssignVariableOp_42(
AssignVariableOp_5AssignVariableOp_52(
AssignVariableOp_6AssignVariableOp_62(
AssignVariableOp_7AssignVariableOp_72(
AssignVariableOp_8AssignVariableOp_82(
AssignVariableOp_9AssignVariableOp_9:C ?

_output_shapes
: 
%
_user_specified_namefile_prefix
?
?
*__inference_conv1d_20_layer_call_fn_131173

inputs
unknown:


	unknown_0:

identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????G
*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_20_layer_call_and_return_conditional_losses_1307472
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*+
_output_shapes
:?????????G
2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:?????????P: : 22
StatefulPartitionedCallStatefulPartitionedCall:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
?
I__inference_sequential_10_layer_call_and_return_conditional_losses_130808

inputs&
conv1d_20_130748:


conv1d_20_130750:
&
conv1d_21_130771:


conv1d_21_130773:!
dense_13_130802:
dense_13_130804:
identity??!conv1d_20/StatefulPartitionedCall?!conv1d_21/StatefulPartitionedCall? dense_13/StatefulPartitionedCall?
!conv1d_20/StatefulPartitionedCallStatefulPartitionedCallinputsconv1d_20_130748conv1d_20_130750*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????G
*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_20_layer_call_and_return_conditional_losses_1307472#
!conv1d_20/StatefulPartitionedCall?
 max_pooling1d_10/PartitionedCallPartitionedCall*conv1d_20/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????
* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *U
fPRN
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_1306942"
 max_pooling1d_10/PartitionedCall?
!conv1d_21/StatefulPartitionedCallStatefulPartitionedCall)max_pooling1d_10/PartitionedCall:output:0conv1d_21_130771conv1d_21_130773*
Tin
2*
Tout
2*
_collective_manager_ids
 *+
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *N
fIRG
E__inference_conv1d_21_layer_call_and_return_conditional_losses_1307702#
!conv1d_21/StatefulPartitionedCall?
+global_average_pooling1d_10/PartitionedCallPartitionedCall*conv1d_21/StatefulPartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *`
f[RY
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_1307812-
+global_average_pooling1d_10/PartitionedCall?
dropout_10/PartitionedCallPartitionedCall4global_average_pooling1d_10/PartitionedCall:output:0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_dropout_10_layer_call_and_return_conditional_losses_1307882
dropout_10/PartitionedCall?
 dense_13/StatefulPartitionedCallStatefulPartitionedCall#dropout_10/PartitionedCall:output:0dense_13_130802dense_13_130804*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *M
fHRF
D__inference_dense_13_layer_call_and_return_conditional_losses_1308012"
 dense_13/StatefulPartitionedCall?
IdentityIdentity)dense_13/StatefulPartitionedCall:output:0"^conv1d_20/StatefulPartitionedCall"^conv1d_21/StatefulPartitionedCall!^dense_13/StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 2F
!conv1d_20/StatefulPartitionedCall!conv1d_20/StatefulPartitionedCall2F
!conv1d_21/StatefulPartitionedCall!conv1d_21/StatefulPartitionedCall2D
 dense_13/StatefulPartitionedCall dense_13/StatefulPartitionedCall:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
?
.__inference_sequential_10_layer_call_fn_131040

inputs
unknown:


	unknown_0:

	unknown_1:


	unknown_2:
	unknown_3:
	unknown_4:
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *R
fMRK
I__inference_sequential_10_layer_call_and_return_conditional_losses_1308082
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
X
<__inference_global_average_pooling1d_10_layer_call_fn_131220

inputs
identity?
PartitionedCallPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *`
f[RY
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_1307812
PartitionedCalll
IdentityIdentityPartitionedCall:output:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:?????????:S O
+
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
.__inference_sequential_10_layer_call_fn_130823
conv1d_20_input
unknown:


	unknown_0:

	unknown_1:


	unknown_2:
	unknown_3:
	unknown_4:
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv1d_20_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *R
fMRK
I__inference_sequential_10_layer_call_and_return_conditional_losses_1308082
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:\ X
+
_output_shapes
:?????????P
)
_user_specified_nameconv1d_20_input
?
?
.__inference_sequential_10_layer_call_fn_131057

inputs
unknown:


	unknown_0:

	unknown_1:


	unknown_2:
	unknown_3:
	unknown_4:
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? *R
fMRK
I__inference_sequential_10_layer_call_and_return_conditional_losses_1309222
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
?
E__inference_conv1d_20_layer_call_and_return_conditional_losses_130747

inputsA
+conv1d_expanddims_1_readvariableop_resource:

-
biasadd_readvariableop_resource:

identity??BiasAdd/ReadVariableOp?"conv1d/ExpandDims_1/ReadVariableOpy
conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2
conv1d/ExpandDims/dim?
conv1d/ExpandDims
ExpandDimsinputsconv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????P2
conv1d/ExpandDims?
"conv1d/ExpandDims_1/ReadVariableOpReadVariableOp+conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02$
"conv1d/ExpandDims_1/ReadVariableOpt
conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2
conv1d/ExpandDims_1/dim?
conv1d/ExpandDims_1
ExpandDims*conv1d/ExpandDims_1/ReadVariableOp:value:0 conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d/ExpandDims_1?
conv1dConv2Dconv1d/ExpandDims:output:0conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????G
*
paddingVALID*
strides
2
conv1d?
conv1d/SqueezeSqueezeconv1d:output:0*
T0*+
_output_shapes
:?????????G
*
squeeze_dims

?????????2
conv1d/Squeeze?
BiasAdd/ReadVariableOpReadVariableOpbiasadd_readvariableop_resource*
_output_shapes
:
*
dtype02
BiasAdd/ReadVariableOp?
BiasAddBiasAddconv1d/Squeeze:output:0BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????G
2	
BiasAdd\
ReluReluBiasAdd:output:0*
T0*+
_output_shapes
:?????????G
2
Relu?
IdentityIdentityRelu:activations:0^BiasAdd/ReadVariableOp#^conv1d/ExpandDims_1/ReadVariableOp*
T0*+
_output_shapes
:?????????G
2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*.
_input_shapes
:?????????P: : 20
BiasAdd/ReadVariableOpBiasAdd/ReadVariableOp2H
"conv1d/ExpandDims_1/ReadVariableOp"conv1d/ExpandDims_1/ReadVariableOp:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
d
+__inference_dropout_10_layer_call_fn_131247

inputs
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputs*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????* 
_read_only_resource_inputs
 *-
config_proto

CPU

GPU 2J 8? *O
fJRH
F__inference_dropout_10_layer_call_and_return_conditional_losses_1308532
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:?????????22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
e
F__inference_dropout_10_layer_call_and_return_conditional_losses_131237

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *   @2
dropout/Consts
dropout/MulMulinputsdropout/Const:output:0*
T0*'
_output_shapes
:?????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*'
_output_shapes
:?????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:?????????2
dropout/GreaterEqual
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:?????????2
dropout/Castz
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*'
_output_shapes
:?????????2
dropout/Mul_1e
IdentityIdentitydropout/Mul_1:z:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
)__inference_dense_13_layer_call_fn_131267

inputs
unknown:
	unknown_0:
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallinputsunknown	unknown_0*
Tin
2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*$
_read_only_resource_inputs
*-
config_proto

CPU

GPU 2J 8? *M
fHRF
D__inference_dense_13_layer_call_and_return_conditional_losses_1308012
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:?????????: : 22
StatefulPartitionedCallStatefulPartitionedCall:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?
?
$__inference_signature_wrapper_131023
conv1d_20_input
unknown:


	unknown_0:

	unknown_1:


	unknown_2:
	unknown_3:
	unknown_4:
identity??StatefulPartitionedCall?
StatefulPartitionedCallStatefulPartitionedCallconv1d_20_inputunknown	unknown_0	unknown_1	unknown_2	unknown_3	unknown_4*
Tin
	2*
Tout
2*
_collective_manager_ids
 *'
_output_shapes
:?????????*(
_read_only_resource_inputs

*-
config_proto

CPU

GPU 2J 8? **
f%R#
!__inference__wrapped_model_1306852
StatefulPartitionedCall?
IdentityIdentity StatefulPartitionedCall:output:0^StatefulPartitionedCall*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 22
StatefulPartitionedCallStatefulPartitionedCall:\ X
+
_output_shapes
:?????????P
)
_user_specified_nameconv1d_20_input
?
e
F__inference_dropout_10_layer_call_and_return_conditional_losses_130853

inputs
identity?c
dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *   @2
dropout/Consts
dropout/MulMulinputsdropout/Const:output:0*
T0*'
_output_shapes
:?????????2
dropout/MulT
dropout/ShapeShapeinputs*
T0*
_output_shapes
:2
dropout/Shape?
$dropout/random_uniform/RandomUniformRandomUniformdropout/Shape:output:0*
T0*'
_output_shapes
:?????????*
dtype02&
$dropout/random_uniform/RandomUniformu
dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2
dropout/GreaterEqual/y?
dropout/GreaterEqualGreaterEqual-dropout/random_uniform/RandomUniform:output:0dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:?????????2
dropout/GreaterEqual
dropout/CastCastdropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:?????????2
dropout/Castz
dropout/Mul_1Muldropout/Mul:z:0dropout/Cast:y:0*
T0*'
_output_shapes
:?????????2
dropout/Mul_1e
IdentityIdentitydropout/Mul_1:z:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*&
_input_shapes
:?????????:O K
'
_output_shapes
:?????????
 
_user_specified_nameinputs
?C
?
I__inference_sequential_10_layer_call_and_return_conditional_losses_131148

inputsK
5conv1d_20_conv1d_expanddims_1_readvariableop_resource:

7
)conv1d_20_biasadd_readvariableop_resource:
K
5conv1d_21_conv1d_expanddims_1_readvariableop_resource:

7
)conv1d_21_biasadd_readvariableop_resource:9
'dense_13_matmul_readvariableop_resource:6
(dense_13_biasadd_readvariableop_resource:
identity?? conv1d_20/BiasAdd/ReadVariableOp?,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp? conv1d_21/BiasAdd/ReadVariableOp?,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp?dense_13/BiasAdd/ReadVariableOp?dense_13/MatMul/ReadVariableOp?
conv1d_20/conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2!
conv1d_20/conv1d/ExpandDims/dim?
conv1d_20/conv1d/ExpandDims
ExpandDimsinputs(conv1d_20/conv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????P2
conv1d_20/conv1d/ExpandDims?
,conv1d_20/conv1d/ExpandDims_1/ReadVariableOpReadVariableOp5conv1d_20_conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02.
,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp?
!conv1d_20/conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2#
!conv1d_20/conv1d/ExpandDims_1/dim?
conv1d_20/conv1d/ExpandDims_1
ExpandDims4conv1d_20/conv1d/ExpandDims_1/ReadVariableOp:value:0*conv1d_20/conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d_20/conv1d/ExpandDims_1?
conv1d_20/conv1dConv2D$conv1d_20/conv1d/ExpandDims:output:0&conv1d_20/conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????G
*
paddingVALID*
strides
2
conv1d_20/conv1d?
conv1d_20/conv1d/SqueezeSqueezeconv1d_20/conv1d:output:0*
T0*+
_output_shapes
:?????????G
*
squeeze_dims

?????????2
conv1d_20/conv1d/Squeeze?
 conv1d_20/BiasAdd/ReadVariableOpReadVariableOp)conv1d_20_biasadd_readvariableop_resource*
_output_shapes
:
*
dtype02"
 conv1d_20/BiasAdd/ReadVariableOp?
conv1d_20/BiasAddBiasAdd!conv1d_20/conv1d/Squeeze:output:0(conv1d_20/BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????G
2
conv1d_20/BiasAddz
conv1d_20/ReluReluconv1d_20/BiasAdd:output:0*
T0*+
_output_shapes
:?????????G
2
conv1d_20/Relu?
max_pooling1d_10/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
value	B :2!
max_pooling1d_10/ExpandDims/dim?
max_pooling1d_10/ExpandDims
ExpandDimsconv1d_20/Relu:activations:0(max_pooling1d_10/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????G
2
max_pooling1d_10/ExpandDims?
max_pooling1d_10/MaxPoolMaxPool$max_pooling1d_10/ExpandDims:output:0*/
_output_shapes
:?????????
*
ksize
*
paddingVALID*
strides
2
max_pooling1d_10/MaxPool?
max_pooling1d_10/SqueezeSqueeze!max_pooling1d_10/MaxPool:output:0*
T0*+
_output_shapes
:?????????
*
squeeze_dims
2
max_pooling1d_10/Squeeze?
conv1d_21/conv1d/ExpandDims/dimConst*
_output_shapes
: *
dtype0*
valueB :
?????????2!
conv1d_21/conv1d/ExpandDims/dim?
conv1d_21/conv1d/ExpandDims
ExpandDims!max_pooling1d_10/Squeeze:output:0(conv1d_21/conv1d/ExpandDims/dim:output:0*
T0*/
_output_shapes
:?????????
2
conv1d_21/conv1d/ExpandDims?
,conv1d_21/conv1d/ExpandDims_1/ReadVariableOpReadVariableOp5conv1d_21_conv1d_expanddims_1_readvariableop_resource*"
_output_shapes
:

*
dtype02.
,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp?
!conv1d_21/conv1d/ExpandDims_1/dimConst*
_output_shapes
: *
dtype0*
value	B : 2#
!conv1d_21/conv1d/ExpandDims_1/dim?
conv1d_21/conv1d/ExpandDims_1
ExpandDims4conv1d_21/conv1d/ExpandDims_1/ReadVariableOp:value:0*conv1d_21/conv1d/ExpandDims_1/dim:output:0*
T0*&
_output_shapes
:

2
conv1d_21/conv1d/ExpandDims_1?
conv1d_21/conv1dConv2D$conv1d_21/conv1d/ExpandDims:output:0&conv1d_21/conv1d/ExpandDims_1:output:0*
T0*/
_output_shapes
:?????????*
paddingVALID*
strides
2
conv1d_21/conv1d?
conv1d_21/conv1d/SqueezeSqueezeconv1d_21/conv1d:output:0*
T0*+
_output_shapes
:?????????*
squeeze_dims

?????????2
conv1d_21/conv1d/Squeeze?
 conv1d_21/BiasAdd/ReadVariableOpReadVariableOp)conv1d_21_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02"
 conv1d_21/BiasAdd/ReadVariableOp?
conv1d_21/BiasAddBiasAdd!conv1d_21/conv1d/Squeeze:output:0(conv1d_21/BiasAdd/ReadVariableOp:value:0*
T0*+
_output_shapes
:?????????2
conv1d_21/BiasAddz
conv1d_21/ReluReluconv1d_21/BiasAdd:output:0*
T0*+
_output_shapes
:?????????2
conv1d_21/Relu?
2global_average_pooling1d_10/Mean/reduction_indicesConst*
_output_shapes
: *
dtype0*
value	B :24
2global_average_pooling1d_10/Mean/reduction_indices?
 global_average_pooling1d_10/MeanMeanconv1d_21/Relu:activations:0;global_average_pooling1d_10/Mean/reduction_indices:output:0*
T0*'
_output_shapes
:?????????2"
 global_average_pooling1d_10/Meany
dropout_10/dropout/ConstConst*
_output_shapes
: *
dtype0*
valueB
 *   @2
dropout_10/dropout/Const?
dropout_10/dropout/MulMul)global_average_pooling1d_10/Mean:output:0!dropout_10/dropout/Const:output:0*
T0*'
_output_shapes
:?????????2
dropout_10/dropout/Mul?
dropout_10/dropout/ShapeShape)global_average_pooling1d_10/Mean:output:0*
T0*
_output_shapes
:2
dropout_10/dropout/Shape?
/dropout_10/dropout/random_uniform/RandomUniformRandomUniform!dropout_10/dropout/Shape:output:0*
T0*'
_output_shapes
:?????????*
dtype021
/dropout_10/dropout/random_uniform/RandomUniform?
!dropout_10/dropout/GreaterEqual/yConst*
_output_shapes
: *
dtype0*
valueB
 *   ?2#
!dropout_10/dropout/GreaterEqual/y?
dropout_10/dropout/GreaterEqualGreaterEqual8dropout_10/dropout/random_uniform/RandomUniform:output:0*dropout_10/dropout/GreaterEqual/y:output:0*
T0*'
_output_shapes
:?????????2!
dropout_10/dropout/GreaterEqual?
dropout_10/dropout/CastCast#dropout_10/dropout/GreaterEqual:z:0*

DstT0*

SrcT0
*'
_output_shapes
:?????????2
dropout_10/dropout/Cast?
dropout_10/dropout/Mul_1Muldropout_10/dropout/Mul:z:0dropout_10/dropout/Cast:y:0*
T0*'
_output_shapes
:?????????2
dropout_10/dropout/Mul_1?
dense_13/MatMul/ReadVariableOpReadVariableOp'dense_13_matmul_readvariableop_resource*
_output_shapes

:*
dtype02 
dense_13/MatMul/ReadVariableOp?
dense_13/MatMulMatMuldropout_10/dropout/Mul_1:z:0&dense_13/MatMul/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_13/MatMul?
dense_13/BiasAdd/ReadVariableOpReadVariableOp(dense_13_biasadd_readvariableop_resource*
_output_shapes
:*
dtype02!
dense_13/BiasAdd/ReadVariableOp?
dense_13/BiasAddBiasAdddense_13/MatMul:product:0'dense_13/BiasAdd/ReadVariableOp:value:0*
T0*'
_output_shapes
:?????????2
dense_13/BiasAdd|
dense_13/SoftmaxSoftmaxdense_13/BiasAdd:output:0*
T0*'
_output_shapes
:?????????2
dense_13/Softmax?
IdentityIdentitydense_13/Softmax:softmax:0!^conv1d_20/BiasAdd/ReadVariableOp-^conv1d_20/conv1d/ExpandDims_1/ReadVariableOp!^conv1d_21/BiasAdd/ReadVariableOp-^conv1d_21/conv1d/ExpandDims_1/ReadVariableOp ^dense_13/BiasAdd/ReadVariableOp^dense_13/MatMul/ReadVariableOp*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*6
_input_shapes%
#:?????????P: : : : : : 2D
 conv1d_20/BiasAdd/ReadVariableOp conv1d_20/BiasAdd/ReadVariableOp2\
,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp,conv1d_20/conv1d/ExpandDims_1/ReadVariableOp2D
 conv1d_21/BiasAdd/ReadVariableOp conv1d_21/BiasAdd/ReadVariableOp2\
,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp,conv1d_21/conv1d/ExpandDims_1/ReadVariableOp2B
dense_13/BiasAdd/ReadVariableOpdense_13/BiasAdd/ReadVariableOp2@
dense_13/MatMul/ReadVariableOpdense_13/MatMul/ReadVariableOp:S O
+
_output_shapes
:?????????P
 
_user_specified_nameinputs
?
s
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_130781

inputs
identityr
Mean/reduction_indicesConst*
_output_shapes
: *
dtype0*
value	B :2
Mean/reduction_indiceso
MeanMeaninputsMean/reduction_indices:output:0*
T0*'
_output_shapes
:?????????2
Meana
IdentityIdentityMean:output:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:?????????:S O
+
_output_shapes
:?????????
 
_user_specified_nameinputs
?
s
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_130710

inputs
identityr
Mean/reduction_indicesConst*
_output_shapes
: *
dtype0*
value	B :2
Mean/reduction_indicesx
MeanMeaninputsMean/reduction_indices:output:0*
T0*0
_output_shapes
:??????????????????2
Meanj
IdentityIdentityMean:output:0*
T0*0
_output_shapes
:??????????????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime*<
_input_shapes+
):'???????????????????????????:e a
=
_output_shapes+
):'???????????????????????????
 
_user_specified_nameinputs
?
s
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_131210

inputs
identityr
Mean/reduction_indicesConst*
_output_shapes
: *
dtype0*
value	B :2
Mean/reduction_indiceso
MeanMeaninputsMean/reduction_indices:output:0*
T0*'
_output_shapes
:?????????2
Meana
IdentityIdentityMean:output:0*
T0*'
_output_shapes
:?????????2

Identity"
identityIdentity:output:0*(
_construction_contextkEagerRuntime**
_input_shapes
:?????????:S O
+
_output_shapes
:?????????
 
_user_specified_nameinputs"?L
saver_filename:0StatefulPartitionedCall_1:0StatefulPartitionedCall_28"
saved_model_main_op

NoOp*>
__saved_model_init_op%#
__saved_model_init_op

NoOp*?
serving_default?
O
conv1d_20_input<
!serving_default_conv1d_20_input:0?????????P<
dense_130
StatefulPartitionedCall:0?????????tensorflow/serving/predict:??
?8
layer_with_weights-0
layer-0
layer-1
layer_with_weights-1
layer-2
layer-3
layer-4
layer_with_weights-2
layer-5
	optimizer
regularization_losses
		variables

trainable_variables
	keras_api

signatures
j_default_save_signature
k__call__
*l&call_and_return_all_conditional_losses"?5
_tf_keras_sequential?5{"name": "sequential_10", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "must_restore_from_config": false, "class_name": "Sequential", "config": {"name": "sequential_10", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 80, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv1d_20_input"}}, {"class_name": "Conv1D", "config": {"name": "conv1d_20", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 80, 3]}, "dtype": "float32", "filters": 10, "kernel_size": {"class_name": "__tuple__", "items": [10]}, "strides": {"class_name": "__tuple__", "items": [1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1]}, "groups": 1, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "MaxPooling1D", "config": {"name": "max_pooling1d_10", "trainable": true, "dtype": "float32", "strides": {"class_name": "__tuple__", "items": [3]}, "pool_size": {"class_name": "__tuple__", "items": [3]}, "padding": "valid", "data_format": "channels_last"}}, {"class_name": "Conv1D", "config": {"name": "conv1d_21", "trainable": true, "dtype": "float32", "filters": 16, "kernel_size": {"class_name": "__tuple__", "items": [10]}, "strides": {"class_name": "__tuple__", "items": [1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1]}, "groups": 1, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}, {"class_name": "GlobalAveragePooling1D", "config": {"name": "global_average_pooling1d_10", "trainable": true, "dtype": "float32", "data_format": "channels_last"}}, {"class_name": "Dropout", "config": {"name": "dropout_10", "trainable": true, "dtype": "float32", "rate": 0.5, "noise_shape": null, "seed": null}}, {"class_name": "Dense", "config": {"name": "dense_13", "trainable": true, "dtype": "float32", "units": 6, "activation": "softmax", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}}, "bias_initializer": {"class_name": "Zeros", "config": {}}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}}]}, "shared_object_id": 13, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 3, "axes": {"-1": 3}}, "shared_object_id": 14}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 80, 3]}, "is_graph_network": true, "save_spec": {"class_name": "TypeSpec", "type_spec": "tf.TensorSpec", "serialized": [{"class_name": "TensorShape", "items": [null, 80, 3]}, "float32", "conv1d_20_input"]}, "keras_version": "2.5.0", "backend": "tensorflow", "model_config": {"class_name": "Sequential", "config": {"name": "sequential_10", "layers": [{"class_name": "InputLayer", "config": {"batch_input_shape": {"class_name": "__tuple__", "items": [null, 80, 3]}, "dtype": "float32", "sparse": false, "ragged": false, "name": "conv1d_20_input"}, "shared_object_id": 0}, {"class_name": "Conv1D", "config": {"name": "conv1d_20", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 80, 3]}, "dtype": "float32", "filters": 10, "kernel_size": {"class_name": "__tuple__", "items": [10]}, "strides": {"class_name": "__tuple__", "items": [1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1]}, "groups": 1, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 1}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 2}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 3}, {"class_name": "MaxPooling1D", "config": {"name": "max_pooling1d_10", "trainable": true, "dtype": "float32", "strides": {"class_name": "__tuple__", "items": [3]}, "pool_size": {"class_name": "__tuple__", "items": [3]}, "padding": "valid", "data_format": "channels_last"}, "shared_object_id": 4}, {"class_name": "Conv1D", "config": {"name": "conv1d_21", "trainable": true, "dtype": "float32", "filters": 16, "kernel_size": {"class_name": "__tuple__", "items": [10]}, "strides": {"class_name": "__tuple__", "items": [1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1]}, "groups": 1, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 5}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 6}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 7}, {"class_name": "GlobalAveragePooling1D", "config": {"name": "global_average_pooling1d_10", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "shared_object_id": 8}, {"class_name": "Dropout", "config": {"name": "dropout_10", "trainable": true, "dtype": "float32", "rate": 0.5, "noise_shape": null, "seed": null}, "shared_object_id": 9}, {"class_name": "Dense", "config": {"name": "dense_13", "trainable": true, "dtype": "float32", "units": 6, "activation": "softmax", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 10}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 11}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 12}]}}, "training_config": {"loss": "categorical_crossentropy", "metrics": [[{"class_name": "MeanMetricWrapper", "config": {"name": "accuracy", "dtype": "float32", "fn": "categorical_accuracy"}, "shared_object_id": 15}]], "weighted_metrics": null, "loss_weights": null, "optimizer_config": {"class_name": "Adam", "config": {"name": "Adam", "learning_rate": 0.0010000000474974513, "decay": 0.0, "beta_1": 0.8999999761581421, "beta_2": 0.9990000128746033, "epsilon": 1e-07, "amsgrad": false}}}}
?

kernel
bias
regularization_losses
	variables
trainable_variables
	keras_api
*m&call_and_return_all_conditional_losses
n__call__"?

_tf_keras_layer?	{"name": "conv1d_20", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": {"class_name": "__tuple__", "items": [null, 80, 3]}, "stateful": false, "must_restore_from_config": false, "class_name": "Conv1D", "config": {"name": "conv1d_20", "trainable": true, "batch_input_shape": {"class_name": "__tuple__", "items": [null, 80, 3]}, "dtype": "float32", "filters": 10, "kernel_size": {"class_name": "__tuple__", "items": [10]}, "strides": {"class_name": "__tuple__", "items": [1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1]}, "groups": 1, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 1}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 2}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 3, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 3, "axes": {"-1": 3}}, "shared_object_id": 14}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 80, 3]}}
?
regularization_losses
	variables
trainable_variables
	keras_api
*o&call_and_return_all_conditional_losses
p__call__"?
_tf_keras_layer?{"name": "max_pooling1d_10", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "class_name": "MaxPooling1D", "config": {"name": "max_pooling1d_10", "trainable": true, "dtype": "float32", "strides": {"class_name": "__tuple__", "items": [3]}, "pool_size": {"class_name": "__tuple__", "items": [3]}, "padding": "valid", "data_format": "channels_last"}, "shared_object_id": 4, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 3, "max_ndim": null, "min_ndim": null, "axes": {}}, "shared_object_id": 16}}
?


kernel
bias
regularization_losses
	variables
trainable_variables
	keras_api
*q&call_and_return_all_conditional_losses
r__call__"?	
_tf_keras_layer?	{"name": "conv1d_21", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "class_name": "Conv1D", "config": {"name": "conv1d_21", "trainable": true, "dtype": "float32", "filters": 16, "kernel_size": {"class_name": "__tuple__", "items": [10]}, "strides": {"class_name": "__tuple__", "items": [1]}, "padding": "valid", "data_format": "channels_last", "dilation_rate": {"class_name": "__tuple__", "items": [1]}, "groups": 1, "activation": "relu", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 5}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 6}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 7, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 3, "axes": {"-1": 10}}, "shared_object_id": 17}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 23, 10]}}
?
regularization_losses
	variables
trainable_variables
 	keras_api
*s&call_and_return_all_conditional_losses
t__call__"?
_tf_keras_layer?{"name": "global_average_pooling1d_10", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "class_name": "GlobalAveragePooling1D", "config": {"name": "global_average_pooling1d_10", "trainable": true, "dtype": "float32", "data_format": "channels_last"}, "shared_object_id": 8, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": 3, "max_ndim": null, "min_ndim": null, "axes": {}}, "shared_object_id": 18}}
?
!regularization_losses
"	variables
#trainable_variables
$	keras_api
*u&call_and_return_all_conditional_losses
v__call__"?
_tf_keras_layer?{"name": "dropout_10", "trainable": true, "expects_training_arg": true, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "class_name": "Dropout", "config": {"name": "dropout_10", "trainable": true, "dtype": "float32", "rate": 0.5, "noise_shape": null, "seed": null}, "shared_object_id": 9}
?

%kernel
&bias
'regularization_losses
(	variables
)trainable_variables
*	keras_api
*w&call_and_return_all_conditional_losses
x__call__"?
_tf_keras_layer?{"name": "dense_13", "trainable": true, "expects_training_arg": false, "dtype": "float32", "batch_input_shape": null, "stateful": false, "must_restore_from_config": false, "class_name": "Dense", "config": {"name": "dense_13", "trainable": true, "dtype": "float32", "units": 6, "activation": "softmax", "use_bias": true, "kernel_initializer": {"class_name": "GlorotUniform", "config": {"seed": null}, "shared_object_id": 10}, "bias_initializer": {"class_name": "Zeros", "config": {}, "shared_object_id": 11}, "kernel_regularizer": null, "bias_regularizer": null, "activity_regularizer": null, "kernel_constraint": null, "bias_constraint": null}, "shared_object_id": 12, "input_spec": {"class_name": "InputSpec", "config": {"dtype": null, "shape": null, "ndim": null, "max_ndim": null, "min_ndim": 2, "axes": {"-1": 16}}, "shared_object_id": 19}, "build_input_shape": {"class_name": "TensorShape", "items": [null, 16]}}
?
+iter

,beta_1

-beta_2
	.decay
/learning_ratem^m_m`ma%mb&mcvdvevfvg%vh&vi"
	optimizer
 "
trackable_list_wrapper
J
0
1
2
3
%4
&5"
trackable_list_wrapper
J
0
1
2
3
%4
&5"
trackable_list_wrapper
?
regularization_losses
0metrics
		variables

trainable_variables
1non_trainable_variables
2layer_metrics

3layers
4layer_regularization_losses
k__call__
j_default_save_signature
*l&call_and_return_all_conditional_losses
&l"call_and_return_conditional_losses"
_generic_user_object
,
yserving_default"
signature_map
&:$

2conv1d_20/kernel
:
2conv1d_20/bias
 "
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
?
regularization_losses
5metrics
	variables
trainable_variables
6non_trainable_variables
7layer_metrics

8layers
9layer_regularization_losses
n__call__
*m&call_and_return_all_conditional_losses
&m"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
regularization_losses
:metrics
	variables
trainable_variables
;non_trainable_variables
<layer_metrics

=layers
>layer_regularization_losses
p__call__
*o&call_and_return_all_conditional_losses
&o"call_and_return_conditional_losses"
_generic_user_object
&:$

2conv1d_21/kernel
:2conv1d_21/bias
 "
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
.
0
1"
trackable_list_wrapper
?
regularization_losses
?metrics
	variables
trainable_variables
@non_trainable_variables
Alayer_metrics

Blayers
Clayer_regularization_losses
r__call__
*q&call_and_return_all_conditional_losses
&q"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
regularization_losses
Dmetrics
	variables
trainable_variables
Enon_trainable_variables
Flayer_metrics

Glayers
Hlayer_regularization_losses
t__call__
*s&call_and_return_all_conditional_losses
&s"call_and_return_conditional_losses"
_generic_user_object
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
!regularization_losses
Imetrics
"	variables
#trainable_variables
Jnon_trainable_variables
Klayer_metrics

Llayers
Mlayer_regularization_losses
v__call__
*u&call_and_return_all_conditional_losses
&u"call_and_return_conditional_losses"
_generic_user_object
!:2dense_13/kernel
:2dense_13/bias
 "
trackable_list_wrapper
.
%0
&1"
trackable_list_wrapper
.
%0
&1"
trackable_list_wrapper
?
'regularization_losses
Nmetrics
(	variables
)trainable_variables
Onon_trainable_variables
Player_metrics

Qlayers
Rlayer_regularization_losses
x__call__
*w&call_and_return_all_conditional_losses
&w"call_and_return_conditional_losses"
_generic_user_object
:	 (2	Adam/iter
: (2Adam/beta_1
: (2Adam/beta_2
: (2
Adam/decay
: (2Adam/learning_rate
.
S0
T1"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
J
0
1
2
3
4
5"
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
 "
trackable_dict_wrapper
 "
trackable_list_wrapper
 "
trackable_list_wrapper
?
	Utotal
	Vcount
W	variables
X	keras_api"?
_tf_keras_metric?{"class_name": "Mean", "name": "loss", "dtype": "float32", "config": {"name": "loss", "dtype": "float32"}, "shared_object_id": 20}
?
	Ytotal
	Zcount
[
_fn_kwargs
\	variables
]	keras_api"?
_tf_keras_metric?{"class_name": "MeanMetricWrapper", "name": "accuracy", "dtype": "float32", "config": {"name": "accuracy", "dtype": "float32", "fn": "categorical_accuracy"}, "shared_object_id": 15}
:  (2total
:  (2count
.
U0
V1"
trackable_list_wrapper
-
W	variables"
_generic_user_object
:  (2total
:  (2count
 "
trackable_dict_wrapper
.
Y0
Z1"
trackable_list_wrapper
-
\	variables"
_generic_user_object
+:)

2Adam/conv1d_20/kernel/m
!:
2Adam/conv1d_20/bias/m
+:)

2Adam/conv1d_21/kernel/m
!:2Adam/conv1d_21/bias/m
&:$2Adam/dense_13/kernel/m
 :2Adam/dense_13/bias/m
+:)

2Adam/conv1d_20/kernel/v
!:
2Adam/conv1d_20/bias/v
+:)

2Adam/conv1d_21/kernel/v
!:2Adam/conv1d_21/bias/v
&:$2Adam/dense_13/kernel/v
 :2Adam/dense_13/bias/v
?2?
!__inference__wrapped_model_130685?
???
FullArgSpec
args? 
varargsjargs
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *2?/
-?*
conv1d_20_input?????????P
?2?
.__inference_sequential_10_layer_call_fn_130823
.__inference_sequential_10_layer_call_fn_131040
.__inference_sequential_10_layer_call_fn_131057
.__inference_sequential_10_layer_call_fn_130954?
???
FullArgSpec1
args)?&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults?
p 

 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
I__inference_sequential_10_layer_call_and_return_conditional_losses_131099
I__inference_sequential_10_layer_call_and_return_conditional_losses_131148
I__inference_sequential_10_layer_call_and_return_conditional_losses_130976
I__inference_sequential_10_layer_call_and_return_conditional_losses_130998?
???
FullArgSpec1
args)?&
jself
jinputs

jtraining
jmask
varargs
 
varkw
 
defaults?
p 

 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
E__inference_conv1d_20_layer_call_and_return_conditional_losses_131164?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
*__inference_conv1d_20_layer_call_fn_131173?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_130694?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *3?0
.?+'???????????????????????????
?2?
1__inference_max_pooling1d_10_layer_call_fn_130700?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *3?0
.?+'???????????????????????????
?2?
E__inference_conv1d_21_layer_call_and_return_conditional_losses_131189?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
*__inference_conv1d_21_layer_call_fn_131198?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_131204
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_131210?
???
FullArgSpec%
args?
jself
jinputs
jmask
varargs
 
varkw
 
defaults?

 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
<__inference_global_average_pooling1d_10_layer_call_fn_131215
<__inference_global_average_pooling1d_10_layer_call_fn_131220?
???
FullArgSpec%
args?
jself
jinputs
jmask
varargs
 
varkw
 
defaults?

 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
F__inference_dropout_10_layer_call_and_return_conditional_losses_131225
F__inference_dropout_10_layer_call_and_return_conditional_losses_131237?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
+__inference_dropout_10_layer_call_fn_131242
+__inference_dropout_10_layer_call_fn_131247?
???
FullArgSpec)
args!?
jself
jinputs

jtraining
varargs
 
varkw
 
defaults?
p 

kwonlyargs? 
kwonlydefaults? 
annotations? *
 
?2?
D__inference_dense_13_layer_call_and_return_conditional_losses_131258?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?2?
)__inference_dense_13_layer_call_fn_131267?
???
FullArgSpec
args?
jself
jinputs
varargs
 
varkw
 
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 
?B?
$__inference_signature_wrapper_131023conv1d_20_input"?
???
FullArgSpec
args? 
varargs
 
varkwjkwargs
defaults
 

kwonlyargs? 
kwonlydefaults
 
annotations? *
 ?
!__inference__wrapped_model_130685{%&<?9
2?/
-?*
conv1d_20_input?????????P
? "3?0
.
dense_13"?
dense_13??????????
E__inference_conv1d_20_layer_call_and_return_conditional_losses_131164d3?0
)?&
$?!
inputs?????????P
? ")?&
?
0?????????G

? ?
*__inference_conv1d_20_layer_call_fn_131173W3?0
)?&
$?!
inputs?????????P
? "??????????G
?
E__inference_conv1d_21_layer_call_and_return_conditional_losses_131189d3?0
)?&
$?!
inputs?????????

? ")?&
?
0?????????
? ?
*__inference_conv1d_21_layer_call_fn_131198W3?0
)?&
$?!
inputs?????????

? "???????????
D__inference_dense_13_layer_call_and_return_conditional_losses_131258\%&/?,
%?"
 ?
inputs?????????
? "%?"
?
0?????????
? |
)__inference_dense_13_layer_call_fn_131267O%&/?,
%?"
 ?
inputs?????????
? "???????????
F__inference_dropout_10_layer_call_and_return_conditional_losses_131225\3?0
)?&
 ?
inputs?????????
p 
? "%?"
?
0?????????
? ?
F__inference_dropout_10_layer_call_and_return_conditional_losses_131237\3?0
)?&
 ?
inputs?????????
p
? "%?"
?
0?????????
? ~
+__inference_dropout_10_layer_call_fn_131242O3?0
)?&
 ?
inputs?????????
p 
? "??????????~
+__inference_dropout_10_layer_call_fn_131247O3?0
)?&
 ?
inputs?????????
p
? "???????????
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_131204{I?F
??<
6?3
inputs'???????????????????????????

 
? ".?+
$?!
0??????????????????
? ?
W__inference_global_average_pooling1d_10_layer_call_and_return_conditional_losses_131210`7?4
-?*
$?!
inputs?????????

 
? "%?"
?
0?????????
? ?
<__inference_global_average_pooling1d_10_layer_call_fn_131215nI?F
??<
6?3
inputs'???????????????????????????

 
? "!????????????????????
<__inference_global_average_pooling1d_10_layer_call_fn_131220S7?4
-?*
$?!
inputs?????????

 
? "???????????
L__inference_max_pooling1d_10_layer_call_and_return_conditional_losses_130694?E?B
;?8
6?3
inputs'???????????????????????????
? ";?8
1?.
0'???????????????????????????
? ?
1__inference_max_pooling1d_10_layer_call_fn_130700wE?B
;?8
6?3
inputs'???????????????????????????
? ".?+'????????????????????????????
I__inference_sequential_10_layer_call_and_return_conditional_losses_130976u%&D?A
:?7
-?*
conv1d_20_input?????????P
p 

 
? "%?"
?
0?????????
? ?
I__inference_sequential_10_layer_call_and_return_conditional_losses_130998u%&D?A
:?7
-?*
conv1d_20_input?????????P
p

 
? "%?"
?
0?????????
? ?
I__inference_sequential_10_layer_call_and_return_conditional_losses_131099l%&;?8
1?.
$?!
inputs?????????P
p 

 
? "%?"
?
0?????????
? ?
I__inference_sequential_10_layer_call_and_return_conditional_losses_131148l%&;?8
1?.
$?!
inputs?????????P
p

 
? "%?"
?
0?????????
? ?
.__inference_sequential_10_layer_call_fn_130823h%&D?A
:?7
-?*
conv1d_20_input?????????P
p 

 
? "???????????
.__inference_sequential_10_layer_call_fn_130954h%&D?A
:?7
-?*
conv1d_20_input?????????P
p

 
? "???????????
.__inference_sequential_10_layer_call_fn_131040_%&;?8
1?.
$?!
inputs?????????P
p 

 
? "???????????
.__inference_sequential_10_layer_call_fn_131057_%&;?8
1?.
$?!
inputs?????????P
p

 
? "???????????
$__inference_signature_wrapper_131023?%&O?L
? 
E?B
@
conv1d_20_input-?*
conv1d_20_input?????????P"3?0
.
dense_13"?
dense_13?????????