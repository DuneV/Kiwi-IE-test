from fail_detection_py.fail_detection_node import FailDetectionLogic

def test_detect_failure():
    logic = FailDetectionLogic()
    assert logic.detect_failure("ERROR: Something went wrong") == True
    assert logic.detect_failure("All good") == False
