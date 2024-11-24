# modules
import pandas as pd
import numpy as np
import tensorflow as tf

def resample_preserving_temporal_profile(df, target_length):
    """
    Resamples a DataFrame to a specified length while preserving the temporal profile of the event.

    Parameters:
        df (pd.DataFrame): DataFrame containing 'time(s)_n' and 'jerk_equivalent' columns.
        target_length (int): Desired number of points after resampling.

    Returns:
        pd.DataFrame: A new DataFrame with resampled 'time(s)' and 'jerk_equivalent'.
    """
    original_time = df['time(s)_n'].values
    original_jerk = df['jerk_equivalent'].values

    new_time = np.linspace(original_time.min(), original_time.max(), target_length)
    interpolated_jerk = np.interp(new_time, original_time, original_jerk)

    resampled_df = pd.DataFrame({'time(s)': new_time, 'jerk_equivalent': interpolated_jerk})
    return resampled_df


def pad_or_trim_to_time_symmetric(df, target_length, max_time=1.8, random_range=None):
    """
    Pads or trims a DataFrame to achieve a specified maximum time and length, preserving temporal symmetry.

    Parameters:
        df (pd.DataFrame): DataFrame containing 'time(s)_n' and 'jerk_equivalent' columns.
        target_length (int): Desired number of points after padding or trimming.
        max_time (float, optional): Maximum time value for interpolation. Defaults to 1.8.
        random_range (tuple, optional): A tuple (min_value, max_value) specifying the range of random values for padding. Defaults to None.

    Returns:
        pd.DataFrame: A new DataFrame with padded or trimmed 'time(s)' and 'jerk_equivalent'.
    """
    original_time = df['time(s)_n'].values
    original_jerk = df['jerk_equivalent'].values
    max_original_time = original_time.max()

    if max_original_time < max_time:
        new_time = np.linspace(0, max_original_time, len(original_time))
        interpolated_jerk = np.interp(new_time, original_time, original_jerk)

        padding_needed = target_length - len(interpolated_jerk)
        left_padding = padding_needed // 2
        right_padding = padding_needed - left_padding

        if random_range is None:
            min_value = df['jerk_equivalent'].min()
            max_value = df['jerk_equivalent'].max()
        else:
            min_value, max_value = random_range

        random_left = np.random.uniform(min_value, max_value, left_padding)
        random_right = np.random.uniform(min_value, max_value, right_padding)

        left_time = np.linspace(0, new_time[0], left_padding)
        right_time = np.linspace(max_original_time, max_time, right_padding)

        new_time = np.concatenate([left_time, new_time, right_time])
        new_jerk = np.concatenate([random_left, interpolated_jerk, random_right])
    else:
        new_time = np.linspace(0, max_time, target_length)
        new_jerk = np.interp(new_time, original_time, original_jerk)

    return pd.DataFrame({'time(s)': new_time, 'jerk_equivalent': new_jerk})


def random_cut_and_interpolate(df, target_length, cut_duration):
    """
    Performs a random cut on a DataFrame and interpolates the result to a fixed length.

    Parameters:
        df (pd.DataFrame): DataFrame containing 'time(s)_n' and 'jerk_equivalent' columns.
        target_length (int): Desired number of points after interpolation.
        cut_duration (float): Duration of the segment to cut in seconds.

    Returns:
        pd.DataFrame: A new DataFrame containing the cut and interpolated segment.
    """
    max_time = df['time(s)_n'].max()

    if max_time < cut_duration:
        raise ValueError("The DataFrame duration is less than the requested cut duration.")

    start_time = np.random.uniform(0, max_time - cut_duration)
    end_time = start_time + cut_duration

    cut_df = df[(df['time(s)_n'] >= start_time) & (df['time(s)_n'] <= end_time)]

    new_time = np.linspace(start_time, end_time, target_length)
    interpolated_jerk = np.interp(new_time, cut_df['time(s)_n'], cut_df['jerk_equivalent'])

    return pd.DataFrame({'time(s)': new_time, 'jerk_equivalent': interpolated_jerk})

def convert_model(pathname, filename):
    """
    Converts a Keras model to TensorFlow Lite format and saves the converted model.

    Parameters:
        pathname (str): Path to the saved Keras model (.h5 format).
        filename (str): Path where the converted TFLite model will be saved.

    Returns:
        None
    """
    model = tf.keras.models.load_model(pathname)
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.experimental_enable_resource_variables = True
    converter.target_spec.supported_ops = [
        tf.lite.OpsSet.TFLITE_BUILTINS,
        tf.lite.OpsSet.SELECT_TF_OPS
    ]
    converter._experimental_lower_tensor_list_ops = False
    tflite_model = converter.convert()
    with open(filename, "wb") as f:
        f.write(tflite_model)
    print("Model converted")

def classify_events_by_batches(data_frame, batch_size, threshold=0.5, collision_range=(300, 12363.40), bump_threshold=(12364, 45435)):
    """
    Classifies events in a DataFrame by dividing it into batches and analyzing each batch.

    Parameters:
        data_frame (pd.DataFrame): Input DataFrame with 'normalized_time' and 'jerk_equivalent'.
        batch_size (int): Number of rows per batch.
        threshold (float): Percentage (0-1) to determine left and right bounds around the peak jerk.
        collision_range (tuple): Range of 'jerk_equivalent' for collision events. Default is (40, 12363.40).
        bump_threshold (tuple): Range of 'jerk_equivalent' for bump events. Default is (12364, 45435).

    Returns:
        list of dict: List of classifications and statistics for each batch.
    """
    batch_results = []

    for i in range(0, len(data_frame), batch_size):
        batch = data_frame.iloc[i:i + batch_size]
        max_jerk = batch['jerk_equivalent'].max()
        max_index = batch['jerk_equivalent'].idxmax()

        jerk_threshold = threshold * max_jerk

        left_bound = max_index
        previous_max = max_jerk
        while left_bound > batch.index[0]:
            current_value = batch['jerk_equivalent'].iloc[left_bound - batch.index[0]]
            if current_value >= jerk_threshold and current_value <= previous_max:
                previous_max = current_value
                left_bound -= 1
            else:
                break

        right_bound = max_index
        previous_max = max_jerk
        while right_bound < batch.index[-1]:
            current_value = batch['jerk_equivalent'].iloc[right_bound - batch.index[0]]
            if current_value >= jerk_threshold and current_value <= previous_max:
                previous_max = current_value
                right_bound += 1
            else:
                break

        start_time = batch['normalized_time'].iloc[left_bound - batch.index[0]]
        end_time = batch['normalized_time'].iloc[right_bound - batch.index[0]]

        duration = end_time - start_time

        mean_jerk = batch['jerk_equivalent'].mean()
        max_jerk_adjusted = (max_jerk - mean_jerk)

        peak_to_duration = max_jerk_adjusted / duration if duration > 0 else 0

        if (collision_range[0] <= peak_to_duration) and (peak_to_duration <= collision_range[1]) and (duration >0.1):
            batch_type = "collision"
        elif bump_threshold[0] <= peak_to_duration:
            batch_type = "bump"
        else:
            batch_type = "normal"

        batch_results.append({
            'batch_index': i // batch_size,
            'batch_type': batch_type,
            'max_jerk': max_jerk,
            'mean_jerk': mean_jerk,
            'peak/time': peak_to_duration,
            'duration': duration,
            'start_time': start_time,
            'end_time': end_time
        })

    return batch_results

def calculate_peak_to_event_time_percentage(data_frame, batch_size, percentage=0.8, min_duration=0.1):
    """
    Calculates event durations and peak-to-duration ratios based on percentage changes in 'jerk_equivalent',
    analyzed in batches. Includes the start and end times for each detected event.

    Parameters:
        data_frame (pd.DataFrame): Input DataFrame with 'normalized_time' and 'jerk_equivalent'.
        batch_size (int): Number of rows per batch for processing.
        percentage (float): Percentage change threshold for detecting significant increases or decreases (e.g., 0.5 for 50%).
        min_duration (float): Minimum event duration to consider valid. Default is 0.1.

    Returns:
        list of dict: List of detected events with start and end times, duration, peak value, and peak-to-duration ratio for each batch.
    """
    batch_results = []

    for i in range(0, len(data_frame), batch_size):
        batch = data_frame.iloc[i:i + batch_size]

        events = []
        peaks = []
        start_times = []
        end_times = []
        start_time = None

        for j in range(1, len(batch)):
            jerk_prev = batch.iloc[j - 1]['jerk_equivalent']
            jerk_curr = batch.iloc[j]['jerk_equivalent']

            if jerk_prev == 0:  # Avoid division by zero
                continue

            jerk_diff = (jerk_curr - jerk_prev) / abs(jerk_prev)

            if jerk_diff > percentage and start_time is None:
                start_time = batch.iloc[j - 1]['normalized_time']

            elif jerk_diff < -percentage and start_time is not None:
                end_time = batch.iloc[j]['normalized_time']

                event_duration = end_time - start_time
                if event_duration >= min_duration:
                    peak = batch.iloc[j - 1]['jerk_equivalent']
                    events.append(event_duration)
                    peaks.append(peak)
                    start_times.append(start_time)
                    end_times.append(end_time)

                start_time = None

        if events:
            avg_peak_to_duration = sum(peaks) / sum(events)
        else:
            avg_peak_to_duration = 0

        batch_results.append({
            'batch_index': i // batch_size,
            'average_peak_to_duration': avg_peak_to_duration,
            'num_events': len(events),
            'events': [
                {
                    'start_time': start_times[k],
                    'end_time': end_times[k],
                    'duration': events[k],
                    'peak': peaks[k]
                }
                for k in range(len(events))
            ]
        })

    return batch_results
