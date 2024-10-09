import datetime
import gzip
import shutil

log_file = "/src/health_management_container/health_report.log"
report_file = "/src/health_management_container/summary_report.txt"
compressed_file = "/src/health_management_container/summary_report_compressed.gz"

# Function to log messages to a file
def log_message(message):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open(log_file, 'a') as f:
        f.write(f"{timestamp} ---- {message}\n")

# Function to generate the summary report
def generate_report():
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
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

    with open(log_file, 'a') as f:
            f.write(f"{timestamp} ---- Start of a new report\n")

def compress_report():
    with open(report_file, 'rb') as f_in:
        with gzip.open(compressed_file, 'wb') as f_out:
            shutil.copyfileobj(f_in, f_out)
    print(f"Compressed report saved as {compressed_file}")


# Generate the summary report
generate_report()
compress_report()
print(f"Report generated saved as {report_file}")
