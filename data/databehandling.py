import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import scipy as sci
import os

#region Defines/Defaults
defFlukeFilename = "FLUKE_data.csv"
defSTMFilename = "STM_data.csv"
defControlFilename = "CONTROL_EVENT_LOG_FILE.csv"
#endregion

#region Fluke Functions

def read_fluke(folderpath, filename=defFlukeFilename):
    filepath = f"{folderpath}/{filename}"
    try:
        return pd.read_csv(filepath, names=["Time", "Pressure", "Unit"])
    except FileNotFoundError:
        print(f"File not found: {filepath}")
        return None
    except TypeError:
        with open(filepath, 'r') as file:
            for line in file:
                print(line.strip())

def fix_fluke_data(folderpath, filename=defFlukeFilename, overwrite=False):
    filepath = f"{folderpath}/{filename}"
    if overwrite:
        with open(filepath, 'r') as file:
            lines = file.readlines()
    
        fixed_lines = []
        for line in lines:
            parts = line.strip().split(',')
            if len(parts) == 3:
                fixed_lines.append(line)
            else:
                print(f"Skipping malformed line: {line.strip()}")

        F = len(lines)
        f = len(fixed_lines)
        with open(filepath, 'w') as file:
            file.writelines(fixed_lines)
            if f == F:
                print("No malformed lines found. No changes made.")
            else:
                print(f"Fixed data written (n = {f}, dn = {F-f}) back to '{filepath}' succesfully.")
    else:
        f = 0
        F = 0
        with open(filepath, 'r') as file:
            with open(f'{filepath}.fixed', 'w') as fixed_file:
                for line in file:
                    F += 1
                    parts = line.strip().split(',')
                    if len(parts) == 3:
                        fixed_file.write(line)
                        f += 1
                    else:
                        print(f"Skipping malformed line: {line.strip()}")
        print(f"Fixed data written (n = {f}, dn = {F-f}) to '{filepath}.fixed' successfully.")
        
def read_and_fix_fluke(folderpath, filename=defFlukeFilename, overwrite=False):
    # first check if .fixed already exists, if it does, read that instead of fixing again
    if overwrite:
        fix_fluke_data(folderpath, filename, overwrite=True)
        return read_fluke(folderpath, filename)

    fixed_filename = f"{filename}.fixed"
    fixed_filepath = f"{folderpath}/{fixed_filename}"
    if not os.path.exists(fixed_filepath):
        fix_fluke_data(folderpath, filename, overwrite=False)
    return read_fluke(folderpath, fixed_filename)
#endregion
#region STM Functions

def _stm_adc_convert(short_hex):
    # Format: XXXXXX string.
    # Conversion table: 0x000000 = 0, 0x7FFFFF = 1, 0x800000 = -1, 0xFFFFFF = -2.
    if len(short_hex) != 6:
        return None
    try:
        int_value = int(short_hex, 16)
        if int_value >= 0x800000:
            return int_value - 0x1000000
        else:
            return int_value
    except ValueError:
        return None

def _stm_convert_hex_to_list(hex_string):
    # six parts to the hex string: AA, BB, CC, DD, EE, FF. AA, BB and CC will be appended separately, DD-FF is the data and should be converted.
    if len(hex_string) != 12:
        return None
    try:
        AA = hex_string[0:2]
        BB = hex_string[2:4]
        CC = hex_string[4:6]
        data_hex = hex_string[6:12]
        data_int = _stm_adc_convert(data_hex)
        return [AA, BB, CC, data_int]
    except ValueError:
        return None

def stm_getRaw(folderpath, filename=defSTMFilename):
    """Reads the STM data from the specified CSV file and organizes it into a dictionary of DataFrames, where each key is a unique signature and the value is a DataFrame containing the corresponding data entries. The function handles malformed lines and values gracefully, skipping them and providing feedback on the number of valid entries processed for each signature."""
    filepath = f"{folderpath}/{filename}"
    outputdict = {} # structure: {signature: list((timestamp, status, value), (timestamp, status, value))...}
    with open(filepath, 'r') as file:
        data = []
        for line_number, line in enumerate(file):
            parts = line.strip("\r\n").split(',')
            if len(parts) == 3:
                timestamp, status, value = parts
                try:
                    signature, output = [x.strip() for x in value.split(":")]
                    if signature not in outputdict.keys():
                        outputdict[signature] = [] # if it does not yet exist, create a new entry
                    else: # if it does exist append to the existing entry
                        convert_output = _stm_convert_hex_to_list(output)
                        outputdict[signature].append((float(timestamp), *convert_output))
                except ValueError:
                    #print(f"Skipping malformed value at line {line_number + 1}: {value}")
                    continue
            #else:
                #print(f"Skipping malformed line at line {line_number + 1}: {line.strip()}")
    for signature, data in outputdict.items():
        print(f"Signature: {signature}, Number of entries: {len(data)}")
        if len(data[0]) == 5:
            outputdict[signature] = pd.DataFrame(data, columns=["Time", "High", "Echo", "Status", "Data"])
        else:
            outputdict[signature] = pd.DataFrame(data, columns=["Time", "Status"])
    return outputdict

def integer_to_temperature(integer_value):
    """Converts an integer value to temperature in Celsius, based on the characteristics of the STM32's ADC and a 2k reference resistor. The conversion assumes that the integer value is derived from a HEX representation where the least significant bit (LSB) corresponds to 0x000000, the positive full scale (PFS) corresponds to 0x7FFFFF, the negative full scale (NSF) corresponds to 0x800000, and -1 LSB corresponds to 0xFFFFFF. The function normalizes the integer value to a range of (-1, 1), calculates the corresponding resistance, and then applies a formula to convert that resistance to temperature in Celsius."""
    #
    # integer was represented by a HEX, which has LSB at 0x000000, PFS at 0x7FFFFF, and NSF at 0x800000, and -1 LSB at 0xFFFFFF.
    
    # Normalize to (-1,1):
    ratio = integer_value / 0x7FFFFF
    resistance = ratio * 2000 # 2k reference
    temp = -((-0.00232*resistance + 17.59246)**0.5 - 3.908) / 0.00116
    return temp

def integer_to_voltage(integer_value, gain=1):
    """Converts an integer value to mV, taking into account the gain."""
    # integer was represented by a HEX, which has LSB at 0x000000, PFS at 0x7FFFFF, and NSF at 0x800000, and -1 LSB at 0xFFFFFF.
    
    # Normalize to (-1,1):
    ratio = integer_value / 0x7FFFFF
    voltage = ratio * 5000 / gain # 5V reference
    return voltage

def stm_GetClean(folderpath, filename=defSTMFilename, gain=1):
    raw_data = stm_getRaw(folderpath, filename)
    clean_data = {}
    for signature, df in raw_data.items():
        if "Data" in df.columns:
            if "T" in signature:
                df["Temperature"] = df["Data"].apply(integer_to_temperature)
            else:
                df["Voltage"] = df["Data"].apply(lambda x: integer_to_voltage(x, gain))
            clean_data[signature] = df
        else:
            clean_data[signature] = df
    return clean_data

def stm_GetCleanWithoutOutliers(folderpath, filename=defSTMFilename, gain=1):
    clean_data = stm_GetClean(folderpath, filename, gain)
    for signature, df in clean_data.items():
        if "Temperature" in df.columns:
            df["Temperature"] = df["Temperature"][(df["Temperature"] > -10) & (df["Temperature"] < 100)]
        elif "Voltage" in df.columns:
            df["Voltage"] = df["Voltage"][(df["Voltage"] > -1000) & (df["Voltage"] < 1000)]
    return clean_data

#endregion

#region Control Events
def read_control_events(folderpath, filename=defControlFilename):
    filepath = f"{folderpath}/{filename}"
    try:
        return pd.read_csv(filepath)
    except FileNotFoundError:
        print(f"File not found: {filepath}")
        return None
    except pd.errors.ParserError:
        print(f"Error parsing CSV file: {filepath}")
        return None
    
#endregion

#region Data Collection Events
def get_all_target_points(
    folderpath,
    filename="DATA_COLLECTION_DEBUG_LOG_FILE.txt"
):
    """Output format: List of dictionaries, where each dictionary has the format: {'timestamp': float, 'pressure': float, 'temperature': float}"""
    filepath = f"{folderpath}/{filename}"
    with open(filepath, 'r') as file:
        lines = file.readlines()
        target_points = []
        for line in lines:
            if "State=point_done" in line:
                # line format here is: '1777626153.2633305 | State=point_done (1/30) | TargetState=pressure:60.00000, temp:0.00000 | Polarity=cool'
                parts = line.split('|')
                timestamp = float(parts[0].strip())
                pressure = float(parts[2].split(',')[0].split(':')[1].strip())
                temperature = float(parts[2].split(',')[1].split(':')[1].strip())
                target_points.append({
                    'timestamp': timestamp,
                    'pressure': pressure,
                    'temperature': temperature
                })
        return target_points