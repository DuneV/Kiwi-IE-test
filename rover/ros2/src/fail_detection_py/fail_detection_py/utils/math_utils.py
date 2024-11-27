#!/usr/bin/env python3

# For the logic behind it look for the file processing.ipynb on the /workspace/analysis/notebooks

import numpy as np
import math


def classify_event_with_threshold(jerk_deque, threshold=300 * 0.5) -> bool:
    """
    Classifies events based on a simple threshold check of the jerk magnitude.

    Parameters:
        jerk_deque (deque): Queue containing jerk magnitudes.
        threshold (float): The threshold value to classify the event.

    Returns:
        dict: Classification of the event based on the threshold.
    """
    jerk_array = np.array(jerk_deque)

    # Find the maximum jerk value in the deque
    max_jerk = jerk_array.max()

    # Classify based on the threshold
    if max_jerk >= threshold:
        event_type = 1
    else:
        event_type = 0

    return event_type


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
