import logging
import os
from datetime import datetime

# Build the directory if it is not created before
LOG_DIR = os.path.join(os.path.dirname(__file__), "../logs")
os.makedirs(LOG_DIR, exist_ok=True)

# Name of the log file based on the time stamp

LOG_FILE = os.path.join(LOG_DIR, f"log_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.log")

# Basic config of logger

logging.basicConfig(
    level=logging.INFO,  # (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    format="%(asctime)s [%(levelname)s] %(message)s",  # Format of the message
    handlers=[
        logging.FileHandler(LOG_FILE),  # Save log file
        logging.StreamHandler()  # Show the log on console
    ]
)

def get_logger(name):
    """
    Create and return a logger with a specific name
    
    Args:
        name (str): Name of the logger

    Returns:
        logging.Logger: Instance of the logger
    """
    return logging.getLogger(name)
