import datetime
import gzip
import shutil
import os
import logging

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
    success_count = 0
    failure_count = 0
    compressed_report = []  # To store compressed binary entries (0 for success, 1 for failure)
    last_start_report_index = 0

    # Find the last occurrence of "---- Start of Report" in the log file
    with open(log_file, 'r') as f:
        lines = f.readlines()
        for i, line in enumerate(lines):
            if "---- Start of a new report" in line:
                last_start_report_index = i

    # Process logs after the last "---- End of Report"
    for line in lines[last_start_report_index + 1:]:
        if "SUCCESS" in line:
            success_count += 1
            compressed_report.append('0')
        elif "FAILURE" in line:
            failure_count += 1
            compressed_report.append('1')

    # Write the summary report to a file with timestamps, counts, and compressed report
    with open(report_file, 'w') as f:
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # Write the date, success/failure counts, and the compressed binary list
        f.write(f"{timestamp} --- Successes: {success_count}, Failures: {failure_count} --- Compressed: {','.join(compressed_report)}\n")

    # Log the start of a new report
    log_message("Start of a new report")

def compress_report():
    with open(report_file, 'rb') as f_in:
        with gzip.open(compressed_file, 'wb') as f_out:
            shutil.copyfileobj(f_in, f_out)
    print(f"Compressed report saved as {compressed_file}")

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
print(f"Report generated saved as {report_file}")
