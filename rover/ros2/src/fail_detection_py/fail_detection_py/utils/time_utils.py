from std_msgs.msg import Header


def headers2dt(header1: Header, header2: Header):
    """Calculate time difference between two message headers in nanoseconds"""
    dt_ns = (header1.stamp.nanosec - header2.stamp.nanosec) + (
        header1.stamp.sec - header2.stamp.sec
    ) * 1e9
    return dt_ns
