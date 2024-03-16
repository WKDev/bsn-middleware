import subprocess
import platform
import time

def ping_with_timeout(host, timeout=1):
    """
    Pings a host and returns False if there is no response within the specified timeout.
    :param host: The hostname or IP to ping.
    :param timeout: Timeout in seconds.
    :return: True if the host responds within the timeout, otherwise False.
    """
    # Construct the ping command based on the operating system
    command = ['ping', '-c', '1', host]
    if platform.system().lower() == "windows":
        command = ['ping', '-n', '1', host]
    
    # Start the ping process
    start_time = time.time()
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    try:
        # Wait for the ping process to complete or timeout
        process.communicate(timeout=timeout)
        return process.returncode == 0
    except subprocess.TimeoutExpired:
        return False
    finally:
        # Ensure the process is terminated
        process.kill()
        # Check if the response was within the timeout
        return (time.time() - start_time) <= timeout
    
print(ping_with_timeout("192.168.11.1"))