from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from scipy import stats
import subprocess
import tensorflow as tf


def read_data(file_path,
              column_names=('timestamp',
                            'x-axis',
                            'y-axis',
                            'z-axis',
                            'activity')):
    """
    Read the accelerometer data from a file.

    Args:
        file_path: URL pointing to the CSV file
        column_names: Column names
    Returns:
        A pandas dataframe
    """

    df = pd.read_csv(file_path,
                     header=None,
                     names=column_names)
    # This is very important otherwise the model will not fit and loss
    # will show up as NAN
    df.dropna(axis=0, how='any', inplace=True)

    # Some rows contain 0 as a timestamp and accelerometer data, remove them
    df.drop(df[df["timestamp"] == 0].index, inplace=True)

    # Some rows are duplicates, remove them
    df.drop_duplicates(inplace=True, ignore_index=True)

    return df


def show_basic_dataframe_info(dataframe,
                              preview_rows=20):
    """
    Show basic information for the given dataframe.

    Args:
        dataframe: A Pandas DataFrame expected to contain data
        preview_rows: An integer value of how many rows to preview
    Returns:
        Nothing
    """

    # Shape and how many rows and columns
    print("Number of columns in the dataframe: %i" % (dataframe.shape[1]))
    print("Number of rows in the dataframe: %i\n" % (dataframe.shape[0]))
    print("First 20 rows of the dataframe:\n")
    # Show first 20 rows
    print(dataframe.head(preview_rows))


def plot_axis(ax, x, y, title):
    ax.plot(x, y)
    ax.set_title(title)
    # ax.xaxis.set_visible(False)
    ax.set_ylim([min(y) - np.std(y), max(y) + np.std(y)])
    ax.set_xlim([min(x), max(x)])
    ax.grid(True)


def plot_activity(activity, data):
    fig, (ax0, ax1, ax2) = plt.subplots(nrows=3,
                                        figsize=(15, 10), sharex=True)
    plot_axis(ax0, data['timestamp'], data['x-axis'], 'x-axis')
    plot_axis(ax1, data['timestamp'], data['y-axis'], 'y-axis')
    plot_axis(ax2, data['timestamp'], data['z-axis'], 'z-axis')
    plt.subplots_adjust(hspace=0.2)
    fig.suptitle(activity)
    plt.subplots_adjust(top=0.90)
    plt.show()


def create_segments_and_labels(df, time_steps, step, label_name):
    """
    This function receives a dataframe and returns the reshaped segments
    of x,y,z acceleration as well as the corresponding labels

    Args:
        df: Dataframe in the expected format
        time_steps: Integer value of the length of a segment that is created
    Returns:
        reshaped_segments
        labels:
    """

    # x, y, z acceleration as features
    columns = ['x-axis', 'y-axis', 'z-axis']
    # Number of steps to advance in each iteration (for me, it should always
    # be equal to the time_steps in order to have no overlap between segments)
    # step = time_steps
    segments = []
    labels = []

    for i in range(0, len(df) - time_steps, step):
        values = df.iloc[i: i + time_steps][columns]
        # Retrieve the most often used label in this segment
        label = stats.mode(df[label_name][i: i + time_steps])[0][0]
        segments.append(values.values)
        labels.append(label)

    # Bring the segments into a better shape
    reshaped_segments = np.asarray(
        segments, dtype=np.float32).reshape(-1, time_steps, len(columns))
    labels = np.asarray(labels)

    return reshaped_segments, labels


def save_converted_model(model, name, labels):
    """Save a model in binary and C form."""

    with open(f"../models/{name}", "wb") as binary:
        binary.write(model)
    interpreter = tf.lite.Interpreter(model_content=model)
    input_quant = interpreter.get_input_details()[0]['quantization']
    output_quant = interpreter.get_output_details()[0]['quantization']
    with open(f"../models/{name}.h", "wt") as header:
        guard = f"MODELS_{name.upper()}_H"
        header.write(
            f"#ifndef {guard}\n#define {guard}\n\n#include <stdint.h>\n\n")
        header.write(f"#define MODEL_NB_OUTPUTS {len(labels)}\n")
        header.write("extern const uint8_t model[];\n")
        header.write("extern const char *model_labels[MODEL_NB_OUTPUTS];\n\n")
        for (n, t, i) in [('INPUT_SCALE', input_quant, 0),
                          ('INPUT_ZERO', input_quant, 1),
                          ('OUTPUT_SCALE', output_quant, 0),
                          ('OUTPUT_ZERO', output_quant, 1)]:
            header.write(f"#define MODEL_{n} {t[i]}\n")
        header.write(f"\n#endif // {guard}\n")
    with open(f"../models/{name}.c", "wt") as body:
        body.write(f"""#include "{name}.h"\n\n""")
        body.write("const uint8_t model[] = {\n")
        body.flush()
        subprocess.run(["xxd", "-i"], input=model,
                       stdout=body, check=True)
        body.flush()
        body.write("};\n\n")
        body.write("const char *model_labels[MODEL_NB_OUTPUTS] = {\n")
        for label in labels:
            body.write(f"""  "{label}",\n""")
        body.write("};\n")


def compare(inp, model_m, interpreter):
    iscale, izero = interpreter.get_input_details()[0]["quantization"]
    oscale, ozero = interpreter.get_output_details()[0]["quantization"]
    model_output = model_m.predict(inp)
    input_data = np.array(inp / iscale + izero, dtype=np.int8)
    interpreter.set_tensor(interpreter.get_input_details()[0]["index"],
                           input_data)
    interpreter.invoke()
    output_data = interpreter.get_tensor(
        interpreter.get_output_details()[0]["index"]).astype(float)
    outp = (output_data - ozero) * oscale
    return model_output.round(3), outp
