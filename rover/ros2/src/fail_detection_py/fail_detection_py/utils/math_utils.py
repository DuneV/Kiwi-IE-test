#!/usr/bin/env python3

# For the logic behind it look for the file processing.ipynb on the /workspace/analysis/notebooks

import numpy as np
import math


def classify_events_with_secondary_opinion(
    jerk_deque,
    threshold=0.5,
    collision_range=(300, 12363.40),
    bump_threshold=(12364, 45435),
):
    """
    Classifies events using a secondary opinion by analyzing jerk magnitudes in the deque.

    Parameters:
        jerk_deque (deque): Queue containing jerk magnitudes.
        threshold (float): Percentage (0-1) to determine left and right bounds around the peak jerk.
        collision_range (tuple): Range of jerk magnitudes for collision events.
        bump_threshold (tuple): Range of jerk magnitudes for bump events.

    Returns:
        dict: Classification and statistics for the event.
    """
    if len(jerk_deque) < 50:
        return {
            "event_type": "insufficient_data",
        }

    jerk_array = np.array(jerk_deque)

    max_jerk = jerk_array.max()
    # mean_jerk = jerk_array.mean()
    max_index = jerk_array.argmax()

    jerk_threshold = threshold * max_jerk

    left_bound = max_index
    while left_bound > 0 and jerk_array[left_bound] >= jerk_threshold:
        left_bound -= 1

    right_bound = max_index
    while (
        right_bound < len(jerk_array) - 1 and jerk_array[right_bound] >= jerk_threshold
    ):
        right_bound += 1

    duration = right_bound - left_bound

    peak_to_duration = max_jerk / duration if duration > 0 else math.inf

    if collision_range[0] <= peak_to_duration <= collision_range[1]:
        event_type = "collision"
    elif bump_threshold[0] <= peak_to_duration <= bump_threshold[1]:
        event_type = "bump"
    else:
        event_type = "normal"

    return {"event_type": event_type}


# Truth Table:
#     Event1 | Event2 | Output
#     -------|--------|-------
#         0  |    0   |   0
#         1  |    0   |   1
#         0  |    1   |   0
#         1  |    1   |   1


def process_both_methods(event1, event2):
    """
    Processes two events based on a truth table logic using INFO_DICT values.

    Truth Table:
    INFO_DICT = {"Collision": 0, "Bump": 1, "Normal": 2}

        Parameters:
            event1 (int): First event (0, 1, or 2).
            event2 (int): Second event (0, 1, or 2).
        Returns:
            int: Output based on the truth table logic.
    """
    if event1 == 2 or event2 == 2:
        # Default case
        return 2

    if event1 == 0 and event2 == 0:
        return 0
    elif event1 == 1 and event2 == 0:
        return 1
    elif event1 == 0 and event2 == 1:
        return 0
    elif event1 == 1 and event2 == 1:
        return 1
