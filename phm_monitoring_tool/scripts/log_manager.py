import datetime
import gzip
import shutil
import os
import logging
import re

# File paths
log_file = "/src/healthmanagement/logs/health_report.log"
report_file = "/src/healthmanagement/logs/summary_report.txt"
compressed_file = "/src/healthmanagement/logs/summary_report_compressed.gz"

# Configure the logger
logging.basicConfig(filename=log_file,
                    level=logging.INFO,
                    format='%(asctime)s ---- %(message)s',
                    datefmt='%Y-%m-%d %H:%M:%S')

# Function to log messages
def log_message(message):
    logging.info(message)

# Function to generate the summary report
def generate_report():
    max_index = 10  # Define the maximum index size for the array
    status_array = [-1] * max_index  # Initialize array with -1 to indicate no data

    # Regex pattern to extract the number in parentheses and the status
    pattern = r"(SUCCESS|FAILURE): \((\d+)\)"

    with open(log_file, 'r') as f:
        lines = f.readlines()

    # Process the log lines to update the status array
    for line in lines:
        match = re.search(pattern, line)
        if match:
            status, index_str = match.groups()
            index = int(index_str) - 1  # Convert to 0-based index
            if 0 <= index < max_index:
                status_array[index] = 1 if status == "SUCCESS" else 0

    # Calculate successes and failures based on the current status array
    success_count = status_array.count(1)
    failure_count = status_array.count(0)

    # Write the summary report to a file
    with open(report_file, 'a') as f:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        f.write(f"{timestamp} --- Successes: {success_count}, Failures: {failure_count}, Status Array: {status_array}\n")
        print(f"{timestamp} --- Successes: {success_count}, Failures: {failure_count}, Status Array: {status_array}")

    # Log the start of a new report
    log_message("Start of a new report")

# Function to compress the report
def compress_report():
    with open(report_file, 'rb') as f_in:
        with gzip.open(compressed_file, 'wb') as f_out:
            shutil.copyfileobj(f_in, f_out)
    print(f"Compressed report saved as {compressed_file}")

# Function to ensure log files exist
def ensure_log_files_exist(log_file, report_file, compressed_file):
    log_dir = os.path.dirname(log_file)

    # Create the logs directory if it doesn't exist
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
        print(f"Created directory: {log_dir}")
    
    # Create empty log files if they don't exist
    for file in [log_file, report_file, compressed_file]:
        if not os.path.exists(file):
            with open(file, 'w') as f:
                f.write('')  # Create an empty file
            print(f"Created file: {file}")

# Generate the summary report
ensure_log_files_exist(log_file, report_file, compressed_file)
generate_report()
compress_report()
print(f"Report generated and saved as {report_file}")
