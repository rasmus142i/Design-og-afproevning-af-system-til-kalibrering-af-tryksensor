
import serial
import numpy as np
import time
import os
import serial.tools.list_ports
import threading as th
from queue import Queue, Empty

current_polarity = None
current_peltier_power = None


# STM IO safety context manager is defined below


#
# The philosophy for this iteration will be:
# 1. the STM is continuously sending ADC data. it will be interrupted when we send it a command, which it will then process, but then it goes back to sending data.
# 1.1 the data should have some kind of structure so that we can easily parse it. maybe just "T:xx.x V:xx.x" or something like that, where T is temperature and V is voltage. maybe also a timestamp if we want to be fancy.
# 1.2 the data will be read immediately to a csv file by a background thread
# 1.3 we obviously need helper functions that let the controller read the csv file and get the latest values
# 2. the fluke will also continuously send data. we never configure the fluke, so it just needs to go straight to a csv file.
# 2.1 we also need helper functions for the fluke data
# 3. the usb relay will be controlled by helper functions that send the right commands to the right ports. we don't need to read anything from the usb relay, we just need to know that the command was sent.
# 3.1 these will also be logged

lock = th.Lock()
stm_lock = th.Lock()
qFlukeVal = Queue()
qADSReads = Queue()
# Holds the most recent parsed STM values by key (e.g. V, T0, T1, ...).
stm_latest = {}
stm_latest_lock = th.Lock()
stm_stop_event = th.Event()
fluke_stop_event = th.Event()

##
#   Constants
##

temperature_mode_command = "T"
"""Command for entering ADC temperature mode."""
temperature_channel_mode_command = "t"
"""Command for entering ADC temperature mode with channel selection."""
voltage_mode_command = "V"
"""Command for entering ADC voltage mode."""
read_command = "R"
"""Command for reading ADC value."""

relay_12V_pin = "7"
"""STM Digital GPIO number which controls the 12V power supply MFET."""
peltier_polarity_pin = "6"
"""STM Digital GPIO number which controls the polarity switcher relays for the peltier."""

USBRelay_CompressorPower = 1
"""USB relay number which controls the compressor power."""
USBRelay_MagnetVentilOut = magnet_ventil_out_relay = 3
"""USB relay number which controls the magnet ventil for letting air out."""
CONTROL_EVENT_LOG_FILE = "control_events.csv"
"""CSV file for timestamped control events such as polarity/relay switching."""
DATA_COLLECTION_DEBUG_LOG_FILE = os.path.join(os.path.dirname(__file__), "data_collection_debug.txt")
"""Text log file for verbose state/target/polarity debug messages during data_collection."""

data_folder = os.path.join(os.path.dirname(__file__), f"data{time.strftime('-%Y-%m-%d-%H-%M-%S')}")
os.makedirs(data_folder, exist_ok=True)

##
#   Serial functions
##

def print_serial_ports():
    """Prints all active serial COM ports to the console. Helpful for debugging USB snuff"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        print(port.device)

def get_stm_port():
    """Returns the COM port (as a string) associated with "STM". Raises error if not worky."""
    ports = serial.tools.list_ports.comports()
    for port in ports:  
        if "STM" in port.description:
            return port.device
    raise Exception("STM port not found")

def get_relay_port():
    """Returns the COM port (as a string) associated with "Numato" (this is the description for the USB relay). Raises error if not worky."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Numato" in port.description:
            return port.device
    raise Exception("Relay port not found")

def get_FLUKE_port():
    """Returns the COM port (as a string) associated with "USB Serial Port" (this is the description for the FLUKE device). Raises error if not worky."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "USB Serial" in port.description:
            return port.device
    raise Exception("FLUKE port not found")

# in this new paradigm, serial ports will remain open... so lets just let them be global variables that we can open

ser_stm, ser_fluke, ser_relay = None, None, None

# Getter functions for accessing serial objects from outside (e.g., Jupyter notebook)
def get_ser_stm():
    global ser_stm
    return ser_stm

def get_ser_fluke():
    global ser_fluke
    return ser_fluke

defsers = ["STM", "FLUKE", "RELAY"]
def open_serial_ports(ports=defsers):
    """Opens the serial ports for the STM, the FLUKE and the USB relay and returns them as a tuple. Order is (STM, FLUKE, USB relay)."""
    global ser_stm, ser_fluke, ser_relay
    if "STM" in ports: stm_port = get_stm_port()
    if "RELAY" in ports: relay_port = get_relay_port()
    if "FLUKE" in ports: fluke_port = get_FLUKE_port()
    try: 
        if "STM" in ports: ser_stm = serial.Serial(port=stm_port, baudrate=115200, timeout=2)
        if "RELAY" in ports: ser_relay = serial.Serial(port=relay_port, baudrate=19200, timeout=1)
        if "FLUKE" in ports: ser_fluke = serial.Serial(port=fluke_port, baudrate=9600, timeout=0.1)
    except Exception as e:
        print(f"Error opening serial ports: {e}")
        raise e

def close_serial_ports(ports = defsers):
    """Closes the serial ports for the STM, the FLUKE and the USB relay."""
    global ser_stm, ser_fluke, ser_relay
    if "STM" in ports and ser_stm is not None:
        ser_stm.close()
        ser_stm = None
    if "FLUKE" in ports and ser_fluke is not None:
        ser_fluke.close()
        ser_fluke = None
    if "RELAY" in ports and ser_relay is not None:
        ser_relay.close()
        ser_relay = None

##
#   FLUKE Commands
##

def FLUKE_communicate(command):
    """Opens the FLUKE comport, sends a command, reads it, returns it. Closes comport."""
    ser = serial.Serial(port=get_FLUKE_port(), baudrate=9600, timeout=2)
    ser.write(f"{command}\n".encode('utf-8'))
    response = ser.read(12)
    clean_response = response.decode('utf-8').strip()
    ser.close()
    return str(clean_response)

def FLUKE_temp(): #temperature in celsius
    """Returns the temperature reading from the FLUKE device as a float. (usually celsius)"""
    temp = FLUKE_communicate("TEMP?")
    return float(temp)

def FLUKE_val(): #pressure value in fluke unit
    """Returns the pressure value from the FLUKE device as a float. Unit is whatever its set to last (probably bar)"""
    val = FLUKE_communicate("VAL?")
    return float(val)

# new continuous read to a csv



def _fluke_val_ser():
    """RESERVED BY CONT PROCESS Returns the pressure value from the FLUKE device as a float. Unit is whatever its set to last (probably bar). This version uses the global serial connection instead of opening and closing a new one."""
    global ser_fluke
    try:
        ser_fluke.write("VAL?\n".encode('utf-8'))
        buffer = ""
        while True:
            oneByte = ser_fluke.read(1)
            if oneByte == b"\r":    #method should returns bytes
                break
            else:
                buffer += oneByte.decode()
    except serial.SerialException as e:
        print(f"(ser) Error communicating with FLUKE device: {e}")
        raise e
    except AttributeError:
        print("Serial connection to FLUKE device not established. Please call open_serial_ports() first.")
        raise Exception("Serial connection to FLUKE device not established.")
    clean_response = buffer.strip()
    return clean_response

def thread_FLUKE_read(queue, filepath=f"{data_folder}/FLUKE_data.csv", stop_event=None):
    """Continuously reads the FLUKE device and appends the values to a csv file with a timestamp. This is meant to be run in a background thread during data collection."""
    global ser_fluke
    if stop_event is None:
        stop_event = fluke_stop_event

    iters = 0
    with open(filepath, "a") as f:
        while not stop_event.is_set():
            try:
                with lock:
                    val = _fluke_val_ser()
                    #print(f"FLUKE value read: {val}") # debug print
            except serial.SerialException as e:
                print(f"Error (cont) reading FLUKE device: {e}")
                break
            queue.put(val)
            timestamp = time.time()
            f.write(f"{timestamp},{val}\n")
            iters += 1
            if iters >= 10:
                f.flush()
                iters = 0
            #time.sleep(0.002) # adjust this for how fast you want to read the FLUKE. 0.2 is probably fine for most purposes, but you can go faster if you want.

flukethread = th.Thread(target=thread_FLUKE_read, args=(qFlukeVal, f"{data_folder}/FLUKE_data.csv", fluke_stop_event))

##
#   STM commands
##

STM_ALLOWED_COMMANDS = {"H", "D", "d", "T", "t", "R", "U", "V", "c", "C", "f", "F"}
STM_MAX_COMMAND_BODY_LEN = 7  # C-side parser uses local_buf[8] including null terminator


def _ensure_stm_ready():
    """Raises if STM serial is not opened and ready for I/O."""
    global ser_stm
    if ser_stm is None:
        raise RuntimeError("STM serial connection not established. Call open_serial_ports(['STM']) first.")
    if not ser_stm.is_open:
        raise RuntimeError("STM serial port is closed.")


def _validate_stm_command(command):
    """Validate command format before sending to avoid malformed/unsafe STM input."""
    if not isinstance(command, str):
        raise TypeError("STM command must be a string")

    cmd = command.strip()
    if not cmd:
        raise ValueError("STM command cannot be empty")
    if len(cmd) > STM_MAX_COMMAND_BODY_LEN:
        raise ValueError(
            f"STM command too long ({len(cmd)}). Max supported length is {STM_MAX_COMMAND_BODY_LEN}."
        )
    if "\n" in cmd or "\r" in cmd:
        raise ValueError("STM command must not contain line breaks")

    command_letter = cmd[0]
    if command_letter not in STM_ALLOWED_COMMANDS:
        raise ValueError(f"Unsupported STM command '{command_letter}'")

    if command_letter in {"D", "d"}:
        if len(cmd) != 2 or cmd[1] not in {"3", "4", "5", "6", "7"}:
            raise ValueError("Pin command must be D/d followed by one of: 3,4,5,6,7")

    if command_letter in {"t", "c", "C", "f", "F"}:
        if len(cmd) != 2 or not cmd[1].isdigit():
            raise ValueError(f"Command '{command_letter}' must include one numeric argument")

    return cmd


def _stm_write_readline(command, flush_after=True):
    """Thread-safe STM write/read helper with guarded decode and optional buffer flush."""
    global ser_stm
    cmd = _validate_stm_command(command)
    _ensure_stm_ready()

    with stm_lock:
        ser_stm.write(f"{cmd}\n".encode("utf-8"))
        response = ser_stm.readline()
        clean_response = response.decode("utf-8", errors="replace").strip()
        if flush_after:
            stm_flush()
        return clean_response

def stm_communicate(command):
    """Opens the STM comport, sends a command, reads it, returns it. Closes comport."""
    return _stm_write_readline(command, flush_after=True)

def handshake():
    """Picks a random number, sends it with "H" in front and sees if the STM understands that it has to return just the number. Returns True if it works, False if it doesn't."""
    message = np.random.randint(low=0, high=10, size=3)
    return stm_communicate(f"H{message[0]}{message[1]}{message[2]}") == f"{message[0]}{message[1]}{message[2]}"

def set_polarity(polarity):
    """
    Sets the polarity of the Peltier device. 'heat' or 1 for heating, 'cool' or 0 for cooling. 
    NOTE: Leaves peltier power off!
    """
    global current_polarity
    normalized_polarity = "heat" if polarity in ("heat", 1) else "cool" if polarity in ("cool", 0) else None
    if normalized_polarity is None:
        raise ValueError("polarity must be 'heat'/'cool' or 1/0")

    if current_polarity == normalized_polarity:
        return

    if normalized_polarity == "heat":
        set_peltier_power(False, reason="set_polarity_heat")
        time.sleep(3)           # wait a little bit
        _stm_set_pin(peltier_polarity_pin, True)
        current_polarity = normalized_polarity
        log_control_event("POLARITY", "heat", details=f"pin={peltier_polarity_pin}")
    elif normalized_polarity == "cool":
        set_peltier_power(False, reason="set_polarity_cool")
        time.sleep(3)
        _stm_set_pin(peltier_polarity_pin, False)
        current_polarity = normalized_polarity
        log_control_event("POLARITY", "cool", details=f"pin={peltier_polarity_pin}")

STM_ACTIVE_COMMAND = "F1"
STM_INACTIVE_COMMAND = "f1"

def cont_STM_read(queue, filepath = f"{data_folder}/STM_data.csv", stop_event=None):
    global STM_ACTIVE_COMMAND, STM_INACTIVE_COMMAND
    global ser_stm
    if stop_event is None:
        stop_event = stm_stop_event

    _ensure_stm_ready()
    stm_communicate(STM_ACTIVE_COMMAND)
    flushcounter = 0
    with open(filepath, "a") as f:
        while not stop_event.is_set():
            try:
                with stm_lock:
                    line = ser_stm.readline()
                if not line:
                    continue
                decoded_line = line.decode("utf-8", errors="replace").rstrip("\r\n")
                goof = decoded_line.split(",")[-1]
                f.write(f"{time.time()},{decoded_line}\n")
                flushcounter += 1
                if flushcounter >= 10:
                    f.flush()
                    flushcounter = 0
                if ":" in goof:
                    key, val = decoded_line.split(":", 1)
                    key_for_real = key.split(",")[-1]
                    with stm_latest_lock:
                        stm_latest[key_for_real] = val
            except Exception as e:
                print(f"Error reading STM device: {e}")
                break
    try:
        stm_communicate(STM_INACTIVE_COMMAND)
        stm_flush()
    except Exception:
        pass

def get_stm_latest(key):
    """Returns the latest parsed STM value for key (for example 'V' or 'T0')."""
    with stm_latest_lock:
        return stm_latest.get(key, None)

def stm_flush():
    global ser_stm
    try:
        _ensure_stm_ready()
        ser_stm.reset_input_buffer()
    except Exception as e:
        print(f"Error flushing STM serial port: {e}")

stmthread = th.Thread(target=cont_STM_read, args=(qADSReads, f"{data_folder}/STM_data.csv", stm_stop_event))
defthreads = [flukethread, stmthread]

def start_threads(threads=None):
    global flukethread, stmthread, defthreads
    if threads is None:
        threads = defthreads

    # Allow restarting STM reader after a previous stop.
    stm_stop_event.clear()
    fluke_stop_event.clear()

    requested_ports = []
    for thread in threads:
        if thread is stmthread and (ser_stm is None or not ser_stm.is_open):
            requested_ports.append("STM")
        if thread is flukethread and (ser_fluke is None or not ser_fluke.is_open):
            requested_ports.append("FLUKE")

    if requested_ports:
        open_serial_ports(list(dict.fromkeys(requested_ports)))

    # Python threads cannot be started twice; recreate them if needed.
    if flukethread.ident is not None:
        flukethread = th.Thread(target=thread_FLUKE_read, args=(qFlukeVal, "fluke_data.csv", fluke_stop_event))
    if stmthread.ident is not None:
        stmthread = th.Thread(target=cont_STM_read, args=(qADSReads, "STM_data.csv", stm_stop_event))
    defthreads = [flukethread, stmthread]

    # Avoid stale references from previously used thread objects.
    launch_threads = []
    if any(t.name == flukethread.name for t in threads) or threads is defthreads:
        launch_threads.append(flukethread)
    if any(t.name == stmthread.name for t in threads) or threads is defthreads:
        launch_threads.append(stmthread)
    if not launch_threads:
        launch_threads = [flukethread, stmthread]

    for thread in launch_threads:
        try: 
            thread.start()
            print(f"Started thread {thread.name}")
        except Exception as e:
            print(f"Error starting thread {thread.name}: {e}")

def stop_threads(threads=None):
    if threads is None:
        threads = defthreads

    stm_stop_event.set()
    fluke_stop_event.set()

    # Attempt to stop threads gracefully using a stop event if available
    for thread in threads:
        # If thread has a 'stop_event' attribute, set it
        stop_event = getattr(thread, 'stop_event', None)
        if stop_event is not None:
            stop_event.set()
    # Now join threads, but only if they are alive
    for thread in threads:
        try:
            if thread.is_alive():
                thread.join(timeout=5)
                if thread.is_alive():
                    print(f"Warning: Thread {thread.name} did not terminate after join timeout.")
                else:
                    print(f"Joined thread {thread.name}")
            else:
                print(f"Thread {thread.name} was not alive.")
        except Exception as e:
            print(f"Error joining thread {thread.name}: {e}")

##
#   ADC
## 

def _hex_payload_to_adc_ratio(hex_payload):
    """Convert 12-hex-char ADS payload (6 bytes) to the legacy normalized ADC ratio."""
    cleaned = str(hex_payload).strip()
    if len(cleaned) < 12:
        raise ValueError(f"ADC payload too short: '{cleaned}'")
    if len(cleaned) % 2 != 0:
        raise ValueError(f"ADC payload has odd number of hex characters: '{cleaned}'")

    data_bytes = [int(cleaned[i:i + 2], 16) for i in range(0, min(len(cleaned), 12), 2)]
    if len(data_bytes) < 6:
        raise ValueError(f"ADC payload has insufficient bytes: '{cleaned}'")

    raw24 = (data_bytes[3] << 16) | (data_bytes[4] << 8) | data_bytes[5]
    return raw24 / (2**23)


def _wait_stm_latest(key, timeout=1.5):
    """Wait for a streamed STM key (for example V, T0) and return its latest payload."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        value = get_stm_latest(key)
        if value is not None:
            return value
        time.sleep(0.01)
    raise TimeoutError(f"Timed out waiting for STM stream key '{key}'")


def _latest_fluke_value(timeout=1.5):
    """Read latest FLUKE value from queue or active serial as fallback."""
    latest = None
    while True:
        try:
            latest = qFlukeVal.get_nowait()
        except Empty:
            break

    if latest is not None:
        # latest format: '2.400000E-02,BAR'
        try:
            format_latest = float(latest.split(",")[0])  # Extract the numeric part before the comma
        except (ValueError, IndexError) as e:
            time.sleep(0.2)
            return _latest_fluke_value()
        return format_latest

    # Fallback for when the continuous reader has not started.
    if ser_fluke is not None and ser_fluke.is_open:
        with lock:
            return float(_fluke_val_ser())

    # Last fallback: one-shot query on a fresh connection.
    return float(FLUKE_communicate("VAL?"))


def latest_fluke_queue_value(timeout=2.0):
    """Read latest FLUKE value from queue only, without touching the FLUKE serial port."""
    deadline = time.time() + timeout
    latest_valid = None

    def _parse_fluke_item(item):
        if item is None:
            return None
        if isinstance(item, (int, float)):
            return float(item)

        s = str(item).strip()
        if not s:
            return None

        # expected format: '2.400000E-02,BAR'
        return float(s.split(",")[0].strip())

    while time.time() < deadline:
        drained_any = False
        while True:
            try:
                # Drain queue fully without blocking to get the newest sample.
                latest = qFlukeVal.get_nowait()
                drained_any = True
                try:
                    parsed = _parse_fluke_item(latest)
                except (ValueError, IndexError, TypeError):
                    continue
                if parsed is not None:
                    latest_valid = parsed
            except Empty:
                break

        if drained_any and latest_valid is not None:
            return latest_valid

        time.sleep(0.01)

    raise TimeoutError("Timed out waiting for FLUKE queue value. Is the FLUKE thread running?")


def FLUKE_val():
    """Returns the most recent FLUKE pressure value as float using the new continuous reader path."""
    return _latest_fluke_value()

def adc_comunicate(command):
    """Works just like the STM communicate function but with a little bit of extra time for the ADC to do its thing. Used for all ADC commands."""
    # ADC commands use the same STM command channel and should be validated the same way.
    return _stm_write_readline(command, flush_after=False)

def adc_temp_mode(): return adc_comunicate(temperature_mode_command)
"""Helper function for entering ADC temperature mode. Just calls the adc_communicate function with the right command."""

def adc_temp_channel_mode(channel): return adc_comunicate(f"{temperature_channel_mode_command}{channel}")
"""Helper function for entering ADC temperature mode with channel selection between 0 and 7. Just calls the adc_communicate function with the right command."""

def adc_volt_mode(): return adc_comunicate(voltage_mode_command)
"""Helper function for entering ADC voltage mode. Just calls the adc_communicate function with the right command."""

def adc_read(): return adc_comunicate(read_command)
"""Helper function for reading the ADC value. Just calls the adc_communicate function with the right command."""

def adc_convert(channel_key="V", timeout=10):
    """Converts streamed STM payload for a key (V, T0..T7) to normalized ADC ratio."""
    payload = _wait_stm_latest(channel_key, timeout=timeout)
    return _hex_payload_to_adc_ratio(payload)

def get_volt(excitement_voltage = 5, PGA_gain = 32):
    """Reads the ADC output, returns it converted to a voltage."""
    V = adc_convert("V") * (excitement_voltage/PGA_gain)
    return V

def get_temp(
    ratiometric_resistor = 2000 # Ohm. 2k on new board, 3.3k on old board
    ):
    """Reads the ADC output, returns it converted to a temperature in celsius."""
    R1 = adc_convert("T2") * ratiometric_resistor
    temp1 = -(np.sqrt(-0.00232*R1 + 17.59246) - 3.908) / 0.00116
    R2 = adc_convert("T3") * ratiometric_resistor
    temp2 = -(np.sqrt(-0.00232*R2 + 17.59246) - 3.908) / 0.00116
    return np.mean([temp1, temp2])

def get_temp_channel(channel, ratiometric_resistor = 2000):
    """Reads the ADC output from a specific channel, returns it converted to a temperature in celsius."""
    if not isinstance(channel, int) or not 0 <= channel <= 7:
        raise ValueError("channel must be an integer from 0 to 7")
    R = adc_convert(f"T{channel}") * ratiometric_resistor
    temp = -(np.sqrt(-0.00232*R + 17.59246) - 3.908) / 0.00116
    return temp

def get_temp_all_channels(maxchannel=4, ratiometric_resistor = 2000):
    """Reads the ADC output from all channels up to maxchannel, returns it converted to a list of temperatures in celsius."""
    if not isinstance(maxchannel, int) or not 0 <= maxchannel <= 7:
        raise ValueError("maxchannel must be an integer from 0 to 7")
    temps = []
    for channel in range(maxchannel+1):
        R = adc_convert(f"T{channel}") * ratiometric_resistor
        temp = -(np.sqrt(-0.00232*R + 17.59246) - 3.908) / 0.00116
        temps.append(temp)
    return temps

##
#   USB Relay Commands
##

def relay_control(relay_number, action):
    """
    Arguments:
    1. Relay number (int 0;3). See documentation for which relay is what.
    2. Action (str or int): 'on' or 1 for turning on, 'off' or 0 for turning off.
    
    """
    global ser_relay
    if ser_relay is None or not ser_relay.is_open:
        open_serial_ports(["RELAY"])

    if action == "on" or action == "ON" or action == "On" or action == 1:
        command = f"relay on {relay_number}\r"
        normalized_action = "on"
    elif action == "off" or action == "OFF" or action == "Off" or action == 0:
        command = f"relay off {relay_number}\r"
        normalized_action = "off"
    else:
        print("Invalid action. Please use 'on' or 'off'.")
        return

    if isinstance(relay_number, int) and 0 <= relay_number <= 3:
        ser_relay.write(command.encode())
        response = ser_relay.read(9).decode()
        log_control_event(
            source="USB_RELAY",
            action=normalized_action,
            details=f"relay={relay_number};response={response.strip()}"
        )
        return response
    else:
        print("Error: relay_number must be one of the digits between 0 and 3.")
        return None
    
def pressurize(button_time = 0):
    """Helper function for USB relay: Turns on the compressor button for a set amount of time (in seconds)."""
    relay_control(USBRelay_CompressorPower, 1)
    time.sleep(button_time)
    relay_control(USBRelay_CompressorPower, 0)

##
#   Data management
##


def log_control_event(source, action, details="", filepath=f"{data_folder}/CONTROL_EVENT_LOG_FILE.csv"):
    """Append a timestamped control event (STM pin or USB relay action) to CSV."""
    needs_header = not os.path.exists(filepath)
    with open(filepath, "a", encoding="utf-8") as f:
        if needs_header:
            f.write("timestamp,source,action,details\n")
        sanitized_details = str(details).replace("\n", " ").replace("\r", " ")
        f.write(f"{time.time()},{source},{action},{sanitized_details}\n")
        f.flush()


def log_data_collection_debug(message, filepath=f"{data_folder}/DATA_COLLECTION_DEBUG_LOG_FILE.txt"):
    """Append a timestamped debug line to a text file for notebook-friendly tracing."""
    try:
        with open(filepath, "a", encoding="utf-8") as f:
            f.write(f"{time.time()} | {message}\n")
            f.flush()
    except Exception as e:
        # Debug logging must never break control flow.
        print(f"[data_collection_debug] Failed to write log: {e}")


def _stm_set_pin(pin_number, state):
    """Set an STM pin high/low and log the switching timestamp."""
    pin_str = str(pin_number)
    if state:
        stm_communicate(f"D{pin_str}")
        log_control_event("STM_PIN", "high", details=f"pin={pin_str}")
    else:
        stm_communicate(f"d{pin_str}")
        log_control_event("STM_PIN", "low", details=f"pin={pin_str}")

def set_peltier_power(state, reason=""):
    """Set peltier power switch (12V pin), with explicit on/off logging and state tracking."""
    global current_peltier_power
    desired_state = bool(state)

    if current_peltier_power is desired_state:
        log_data_collection_debug(
            f"State=peltier_switch_unchanged | TargetState=power:{'on' if desired_state else 'off'}, reason:{reason} | "
            f"Polarity={current_polarity}"
        )
        return

    _stm_set_pin(relay_12V_pin, desired_state)
    current_peltier_power = desired_state
    switch_action = "on" if desired_state else "off"
    log_control_event("PELTIER_SWITCH", switch_action, details=f"pin={relay_12V_pin};reason={reason}")
    log_data_collection_debug(
        f"State=peltier_switch_{switch_action} | TargetState=reason:{reason} | Polarity={current_polarity}"
    )

MASTERLIST = [[],[],[], []] #list with 4 lists, one for temp, one for volt, one for fluke, one for time. Used for storing data during collection, then converted to dataframe at the end.
TEMPLIST = [[], []] #list with 2 lists, one for temp, one for time. Used for storing temperature data during the set_temp function, which is then used to calculate the time constant of the system.

def get_data(): #list with 4 lists
    """Returns one live sample tuple; continuous process logging is handled by background CSV writers."""
    return (get_temp(), get_volt(), latest_fluke_queue_value(), time.time())
    
##
#   Main
##

def Data_Matrix_Generator(low_pressure, high_pressure, low_temp, high_temp, pressure_points, temperature_points):
    pressures = np.linspace(high_pressure, low_pressure, pressure_points) # inverted cuz science
    temperatures1 = np.linspace(low_temp, high_temp, temperature_points)
    temperatures2 = np.linspace(high_temp, low_temp, temperature_points)
    temperatures = [temperatures1, temperatures2]
    data_matrix = np.array([(p, t) for i,p in enumerate(pressures) for t in temperatures[i%2]])
    return data_matrix

def Data_Matrix_Generator(low_pressure, high_pressure, low_temp, high_temp, pressure_points):
    pressures = np.linspace(high_pressure, low_pressure, pressure_points) # inverted cuz science
    temperatures1 = [low_temp, high_temp]
    temperatures2 = [high_temp, low_temp]
    temperatures = [temperatures1, temperatures2]
    data_matrix = np.array([(p, t) for i,p in enumerate(pressures) for t in temperatures[i%2]])
    return data_matrix

def set_temp(temp_wish):
    return set_temp_stream(temp_wish)


def set_temp_stream(temp_wish, tolerance=0.05, timeout_s=6000, sample_period_s=0.2):
    """Drive temperature to target using streamed STM temperatures and logged control pin switching."""
    temp_now = get_temp()
    niter = 0
    while ((temp_now > 60) or (temp_now < -10)):
        temp_now = get_temp()
        log_data_collection_debug(
            f"State=set_temp_waiting_for_valid_reading | TargetState=temp:{temp_wish:.5f} | "
            f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
        )
    log_data_collection_debug(
        f"State=set_temp_start | TargetState=temp:{temp_wish:.5f}, tolerance:{tolerance:.5f} | "
        f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
    )
    if abs(temp_wish - temp_now) <= tolerance:
        log_data_collection_debug(
            f"State=set_temp_already_in_tolerance | TargetState=temp:{temp_wish:.5f} | "
            f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
        )
        return temp_now

    heating = temp_wish > temp_now
    set_polarity(1 if heating else 0)
    log_data_collection_debug(
        f"State=set_temp_polarity_set | TargetState=mode:{'heating' if heating else 'cooling'}, temp:{temp_wish:.5f} | "
        f"Polarity={current_polarity}"
    )
    set_peltier_power(True, reason="set_temp_stream")
    log_data_collection_debug(
        f"State=set_temp_power_on | TargetState=temp:{temp_wish:.5f} | Polarity={current_polarity}"
    )

    start = time.time()
    try:
        while True:
            temp_now = get_temp()
            while ((temp_now > 60) or (temp_now < -10)):
                temp_now = get_temp()
                log_data_collection_debug(
                    f"State=set_temp_waiting_for_valid_reading | TargetState=temp:{temp_wish:.5f} | "
                    f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
                )
            log_data_collection_debug(
                f"State=set_temp_tracking | TargetState=temp:{temp_wish:.5f} | "
                f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
            )
            if heating and temp_now >= (temp_wish - tolerance):
                log_data_collection_debug(
                    f"State=set_temp_reached | TargetState=temp:{temp_wish:.5f} | "
                    f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
                )
                return temp_now
            if (not heating) and temp_now <= (temp_wish + tolerance):
                log_data_collection_debug(
                    f"State=set_temp_reached | TargetState=temp:{temp_wish:.5f} | "
                    f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
                )
                return temp_now
            if (time.time() - start) > timeout_s:
                log_data_collection_debug(
                    f"State=set_temp_timeout | TargetState=temp:{temp_wish:.5f}, timeout_s:{timeout_s} | "
                    f"Polarity={current_polarity} | CurrentTemp={temp_now:.5f}"
                )
                raise TimeoutError(f"Temperature target {temp_wish} not reached in {timeout_s}s")
            time.sleep(sample_period_s)
    finally:
        set_peltier_power(False, reason="set_temp_stream_done")
        log_data_collection_debug(
            f"State=set_temp_power_off | TargetState=temp:{temp_wish:.5f} | Polarity={current_polarity}"
        )


def data_collection(data_matrix, pressure_tolerance = 0, vent_pulse_s=0.01, pressure_settle_s=3.0,
                    temp_tolerance=0.05, temp_timeout_s=6000, sample_pause_s=0.05):
    """Run target points using stream-based STM/FLUKE readings; no in-memory process logging."""
    global current_polarity
    try:
        log_data_collection_debug(
            f"State=run_start | TargetState=points:{len(data_matrix)} | Polarity={current_polarity}"
        )
        for n in range(len(data_matrix)):
            target_pressure = data_matrix[n,0]
            target_temp = data_matrix[n,1]

            log_data_collection_debug(
                f"State=point_start ({n+1}/{len(data_matrix)}) | "
                f"TargetState=pressure:{target_pressure:.5f}, temp:{target_temp:.5f} | "
                f"Polarity={current_polarity}"
            )

            # Pressure-control phase must run with peltier power disabled.
            set_peltier_power(False, reason="pressure_control")
            current_pressure = latest_fluke_queue_value()
            if (target_pressure == 0):
                while current_pressure > 0.06:
                    log_data_collection_debug(
                        f"State=venting(susamogus)_to_atmospheric_pressure | "
                        f"TargetState=pressure:{target_pressure:.5f} | "
                        f"Polarity={current_polarity} | "
                        f"PeltierPower={'on' if current_peltier_power else 'off'} | "
                        f"CurrentPressure={current_pressure:.5f}"
                    )
                    set_peltier_power(False, reason="pressure_control")
                    relay_control(magnet_ventil_out_relay, 1) #magnetventil åben
                    time.sleep(5) # sussy wussy
                    relay_control(magnet_ventil_out_relay, 0) #magnetventil lukket
                    time.sleep(pressure_settle_s)
                    current_pressure = latest_fluke_queue_value()
            else:
                while target_pressure < (current_pressure + pressure_tolerance):
                    log_data_collection_debug(
                        f"State=venting_pressure | "
                        f"TargetState=pressure:{target_pressure:.5f} | "
                        f"Polarity={current_polarity} | "
                        f"PeltierPower={'on' if current_peltier_power else 'off'} | "
                        f"CurrentPressure={current_pressure:.5f}"
                    )
                    set_peltier_power(False, reason="pressure_control")
                    relay_control(magnet_ventil_out_relay, 1) #magnetventil åben
                    time.sleep(vent_pulse_s)
                    relay_control(magnet_ventil_out_relay, 0) #magnetventil lukket
                    time.sleep(pressure_settle_s)
                    current_pressure = latest_fluke_queue_value()
            log_data_collection_debug(
                f"State=temperature_control | "
                f"TargetState=temp:{target_temp:.5f} | "
                f"Polarity={current_polarity}"
            )
            set_temp_stream(
                target_temp,
                tolerance=temp_tolerance,
                timeout_s=temp_timeout_s
            )
            time.sleep(sample_pause_s)
            log_data_collection_debug(
                f"State=point_done ({n+1}/{len(data_matrix)}) | "
                f"TargetState=pressure:{target_pressure:.5f}, temp:{target_temp:.5f} | "
                f"Polarity={current_polarity}"
            )
        log_data_collection_debug(
            f"State=run_done | TargetState=points:{len(data_matrix)} | Polarity={current_polarity}"
        )
        set_peltier_power(False, reason="data_collection_done")
        # we have to do another sus
        log_data_collection_debug(
            f"State=venting(susamogus)_to_atmospheric_pressure | "
            f"TargetState=pressure:{target_pressure:.5f} | "
            f"Polarity={current_polarity} | "
            f"PeltierPower={'on' if current_peltier_power else 'off'} | "
            f"CurrentPressure={current_pressure:.5f}"
        )
        set_peltier_power(False, reason="pressure_control")
        relay_control(magnet_ventil_out_relay, 1) #magnetventil åben
        time.sleep(5) # sussy wussy
        relay_control(magnet_ventil_out_relay, 0) #magnetventil lukket
        time.sleep(pressure_settle_s)
        current_pressure = latest_fluke_queue_value()
        stop_threads()
        
    except KeyboardInterrupt:
        log_data_collection_debug(
            f"State=keyboard_interrupt | TargetState=aborted | Polarity={current_polarity}"
        )
        set_peltier_power(False, reason="data_collection_interrupt")
        time.sleep(0.1)
        close_serial_ports(["STM", "FLUKE", "RELAY"])
        raise
    except Exception as e:
        log_data_collection_debug(
            f"State=error | TargetState=exception | Polarity={current_polarity} | Error={type(e).__name__}: {e}"
        )
        try:
            set_peltier_power(False, reason="data_collection_error")
        except Exception:
            pass
        raise