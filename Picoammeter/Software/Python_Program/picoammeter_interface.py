import time
import math
import serial
import matplotlib.pyplot as plt
from serial import SerialException
import scipy
from scipy import signal
import numpy as np
from matplotlib.animation import FuncAnimation

logfile = "C:\\Users\\100kV\\Documents\\Python_Scripts\\picoammeter_testing_log.txt"

COM_PORT1 = "COM30"
COM_PORT2 = "COM31"
COM_PORT3 = "COM32"
PICO_NAME1 = "PicoOne"
PICO_NAME2 = "PicoTwo"
PICO_NAME3 = "PicoThree"
SAMP_FREQ1 = 1500
SAMP_FREQ2 = 1500
SAMP_FREQ3 = 1500
MAX_ATTEMPTS = 3
STORED_VALUES = 1000

BROADCAST_PERIOD = 0.1 #seconds
VERBOSE = 0

b50 = [0.9196148449933444, -5.401852667947308, 13.335740723304264, -17.706937730118, 13.335740723304268, -5.40185266794731, 0.9196148449933449]
a50 = [1.0, -5.7100517233890855, 13.704928811150559, -17.694285505137817, 12.960090866896783, -5.106305837485728, 0.845691458547884]

class IIR:
    def __init__(self, b, a):
        self.b = b
        self.a = a
        self.P = len(b)
        self.Q = len(a)
        self.x_values = [0] * self.P
        self.y_values = [0] * self.Q
        self.index = 0
        if (VERBOSE == 1):
            print("New filter created. Coefficients are... ")
            self.print_coefficients()
        
    def reset_filter(self):
        self.x_values = [0] * self.P
        self.y_values = [0] * self.Q
        self.index = 0
        
    def set_coefficients(self, b, a):
        self.b = b
        self.a = a
        self.P = len(b)
        self.Q = len(a)
        self.reset_filter()
    
    def print_coefficients(self):
        print("b = [", end = '')
        for val in self.b[:-1]:
            print("{},".format(val), end='')
        print("{}]".format(self.b[-1]))
        
        print("a = [", end = '')
        for val in self.a[:-1]:
            print("{},".format(val), end='')
        print("{}]".format(self.a[-1]))
        
    def inc_index(self):
        self.index += 1
        if (self.index > (self.P - 1)):
            self.index = 0
            
    def dec_index(self):
        self.index -= 1
        if (self.index < 0):
            self.index = (self.P - 1)
            
    def filter_single(self, new_input):
        self.x_values[self.index] = new_input
        new_output = 0
        
        for i in range(self.P):
            new_output += self.b[i] * self.x_values[self.index]
            self.dec_index()
            
        for i in range(1, self.Q):
            self.dec_index()
            new_output -= self.a[i] * self.y_values[self.index]
        
        self.dec_index()
        new_output = new_output / self.a[0]
        
        self.y_values[self.index] = new_output
        self.inc_index()
        
        return new_output
    
    def filter_multiple(self, array):
        filtered_results = []
        for val in array:
            filtered_result = self.filter_single(val)
            filtered_results.append(filtered_result)
        return filtered_results

class Picoammeter:
    def __init__(self, com, name, fs, offset):
        self.com = com
        self.connected = 0
        self.ser_port = None
        self.name = name
        self.attempt_connect(1)
        self.filters = []
        self.f_s = fs
        self.check_connection()
        self.unsentValues = []
        self.lastSentValues = []
        self.bad_transmissions = 0
        self.lastFiltered = [None] * STORED_VALUES
        self.lastFilteredIndex = 0
        self.lastUnfiltered = [None] * STORED_VALUES
        self.lastUnfilteredIndex = 0        
        
        # Calibration Values
        self.gains = [1e9, 0.97e7, 1e5, 1e3]
        #self.offsets = [1.8e-12,110e-12,12.3e-9,960e-9]
        self.offsets = [offset,0,0,0]
        self.vref = 4.096
 
    def update_offset(self, offset, rang):
        self.offsets[rang] = offset

    def say_hello(self):
        print("Hello, I am {}".format(self.name), end='')
        if self.check_connection():
            print(", connected to {}.".format(self.ser_port.name))
        else:
            print(", but I have lost my serial connection.")
    
    def check_connection(self):
        try:
            self.ser_port.in_waiting
            self.connected = 1
            return 1
        except (SerialException, AttributeError):
            self.connected = 0
            return 0
            
    def show_dsp_freq_resp(self):
        if (len(self.filters) == 0):
            if (VERBOSE >= 2):
                print("No filters assigned to {}".format(self.name))
            return;
        freq = 0
        hsum = 0
        for filt in self.filters:
            freq, h = signal.freqz(filt.b, filt.a, fs=self.f_s)
            hsum += 20*np.log10(abs(h))

        # Plot
        fig, ax = plt.subplots(1, 1, figsize=(8, 6))
        ax.plot(freq, hsum, color='blue')
        ax.set_title("Total Frequency Response of {}'s DSP".format(self.name))
        ax.set_ylabel("Amplitude (dB)", color='blue')
        ax.grid(True)
        plt.show()
    
    def add_iir(self, b, a):
        new_iir = IIR(b, a)
        self.filters.append(new_iir)
     
    def retrieveProcessData(self):
        receivedData = self.retrieveData()
        if (receivedData != [[]]):
            newCurrents = self.convertDataArray(receivedData)
            newFilteredCurrents = self.filterCurrents(newCurrents)
            self.unsentValues += newFilteredCurrents
    
    def attempt_connect(self, sleep_time = 0):
        attempts = 0
        while (self.connected == 0):
            try:
                ser = serial.Serial(self.com, 74880, timeout=0)
                if (ser.name == self.com):
                    if (VERBOSE >= 1):
                        print("Picoammeter {} connected at {}".format(self.name, self.com))
                    self.connected = 1
                    self.ser_port = ser
                    self.ser_port.reset_input_buffer()
                    return True
                else:
                    ser.close()
            except SerialException:
                pass
                
            if (self.connected == 0):
                if (VERBOSE >= 1):
                    print("Failed to connect to {}. Retrying...".format(self.com))
                time.sleep(sleep_time)
                attempts += 1
                if ((attempts >= MAX_ATTEMPTS) and (MAX_ATTEMPTS > 0)):
                    if (VERBOSE >= 1):
                        print("Reached maximum number of attempts whilst connecting to {}".format(self.com))
                        print("Exiting function")
                    return False;
                
    def retrieveData(self):
    
        if (self.connected == 0):
            self.attempt_connect()
            
        if (self.connected == 1):
            try:
                if (self.ser_port.in_waiting > 0):
                
                    #Read incoming data
                    data = self.ser_port.read(3)
                    
                    # Check sync
                    try:
                        rang = data[0]
                        val = (data[1] * 256) + data[2]
                    except IndexError:
                        data = self.ser_port.read()
                        return [[]]
                    if ((0 <= rang <= 3) and (4096 <= val <= 61439)):
                        if (VERBOSE >= 2):
                            if ((len(data) > 0) or (VERBOSE >= 3)):
                                print("new data: {}".format(data))
                        #return [[rang, val]]
                        return [data]
                    else:
                        # Read one byte to attempt to correct sync
                        data = self.ser_port.read(1)
            except SerialException:
                self.connected = 0
                
        return [[]]
        
    def convertDataArray(self, datas):
        newCurrents = []
        for data in datas:
            rng = data[0]
            val = (data[1] * 256) + data[2]
            current = self.convertDataSingle([rng,val])
            if (current != None):
                newCurrents.append(current)
            
        #apply fixes if possible
        for i in range(len(newCurrents)):
            if (newCurrents[i] == 1):   #should be fixed
                backup_values = self.lastSentValues + self.unsentValues
                if ((i > 0) and (i < (len(newCurrents) - 1))):    #replace with midpoint of measurements either side
                    prev = newCurrents[i - 1]
                    post = newCurrents[i + 1]
                    mid = (prev + post) / 2
                    newCurrents[i] = mid
                
                elif (len(backup_values) > 0):
                    newCurrents[i] = np.mean(backup_values) #replace with mean of unsentValues
            
            if (self.lastUnfilteredIndex >= 1000):
                self.lastUnfilteredIndex = 0
            self.lastUnfiltered[self.lastUnfilteredIndex] = newCurrents[i]
            self.lastUnfilteredIndex += 1
            if (self.lastUnfilteredIndex >= 1000):
                self.lastUnfilteredIndex = 0
        
        if (VERBOSE >= 2):
            if ((len(newCurrents) > 0) or (VERBOSE >= 3)):
                print("newCurrents: {}".format(newCurrents))
        return newCurrents
    
    # Converts a data value into a current measurement
    # Returns value in Amps, or 1 if failure
    def convertDataSingle(self, data):
        current = 0.0
        try:
            tia_range = data[0]
            adc_value = data[1]
            
            if (VERBOSE >= 2): print("TRAN {}, RANGE {}, ADC {}".format(data, tia_range, adc_value))
            try:
                gain = self.gains[tia_range]
                offset = self.offsets[tia_range]
                vref = self.vref
            except IndexError:
                self.bad_transmissions += 1
                current = 1
            if ((tia_range < 0) or (tia_range > 3)) or ((adc_value < 0) or (adc_value > 65535)):
                self.bad_transmissions += 1
                current = 1
            else:
                current = (vref/gain) * ((65535 - (2 * adc_value)) / 65535)
                current = current - offset
        except ValueError:
            if ((data == b'\r\n') or (data == b'\r') or (data == b'\n')):
                return None
            self.bad_transmissions += 1
            current = 1
        return current
        
    def filterCurrents(self, currents):
        for filt in self.filters:
            currents = filt.filter_multiple(currents)
            
        if (VERBOSE >= 3):
            if ((len(currents) > 0) or (VERBOSE >= 3)):
                print("filtered_currents: {}".format(currents))
        
        for value in currents:
            self.lastFiltered[self.lastFilteredIndex] = value
            self.lastFilteredIndex += 1
            if (self.lastFilteredIndex >= 1000):
                self.lastFilteredIndex = 0
            
        return currents
  
    def returnLastMeasurements(self, length=STORED_VALUES):
        # make the arrays start from most recent
        unfiltered = []
        for i in range(self.lastUnfilteredIndex, self.lastUnfilteredIndex + length):
            unfiltered.append(self.lastUnfiltered[i % STORED_VALUES])
        filtered = []
        for i in range(self.lastFilteredIndex, self.lastFilteredIndex + length):
            filtered.append(self.lastFiltered[i % STORED_VALUES])
        return [unfiltered, filtered]
           
    def process_unsent(self):
        data = []
        num_datapoints = len(self.unsentValues)
        mean = 0
        stdev = 0
        if (num_datapoints > 0):
            mean = np.mean(self.unsentValues)
            stdev = np.std(self.unsentValues)
        
        status = self.check_connection()
        data = [mean, stdev, num_datapoints, self.bad_transmissions, (self.connected & (num_datapoints > 0))]
        
        self.lastSentValues = self.unsentValues
        self.unsentValues = []
        self.bad_transmissions = 0
        return data
           
def init_picos(coms, names, samp_freqs, offsets):
    picos = []
    attempts = 0
    for i in range(len(coms)):
        com = coms[i]
        name = names[i]
        samp_freq = samp_freqs[i]
        offset = offsets[i]
        pico = Picoammeter(com, name, samp_freq, offset)
        picos.append(pico)
    return picos           

def to_si(d, unit, sep=' '):
    inc_prefixes = ['k', 'M', 'G', 'T', 'P', 'E', 'Z', 'Y']
    dec_prefixes = ['m', 'µ', 'n', 'p', 'f', 'a', 'z', 'y']
    if d == 0:
        return str(0)
    degree = int(math.floor(math.log10(math.fabs(d)) / 3))
    prefix = ''
    if degree != 0:
        ds = degree / math.fabs(degree)
        if ds == 1:
            if degree - 1 < len(inc_prefixes):
                prefix = inc_prefixes[degree - 1]
            else:
                prefix = inc_prefixes[-1]
                degree = len(inc_prefixes)
        elif ds == -1:
            if -degree - 1 < len(dec_prefixes):
                prefix = dec_prefixes[-degree - 1]
            else:
                prefix = dec_prefixes[-1]
                degree = -len(dec_prefixes)
        scaled = float(d * math.pow(1000, -degree))
        s = "{scaled}{sep}{prefix}".format(scaled=scaled,
                                           sep=sep,
                                           prefix=prefix)
    else:
        s = "{d}".format(d=d)
    return s+str(unit)

def broadcast(devices):
    file_entry = ''
    for device in devices:
        data = device.process_unsent()
        fs = data[2]/BROADCAST_PERIOD
        file_entry += str(data[0]) + ',' + str(data[1]) + "," + str(data[4]) + "," + str(fs) + '\n'
        #print(to_si(data[0], 'A'))
    #print(file_entry)
    with open(logfile, 'a') as file:
        file.write(file_entry)
    return data[0]

com_ports = [COM_PORT1]#, COM_PORT2, COM_PORT3]
pico_names = [PICO_NAME1, PICO_NAME2, PICO_NAME3]
samp_freqs = [SAMP_FREQ1, SAMP_FREQ2, SAMP_FREQ3]
offsets = [111.25e-12,0,0,0]
picos = init_picos(com_ports, pico_names, samp_freqs, offsets)

# add IIR filters to each picoammeter
for pico in picos:
    pico.say_hello()
    pico.add_iir(b50, a50)
    pico.show_dsp_freq_resp()
    
time.sleep(1)
lastTime = time.time()
i = 0
records = []

while True:
    for pico in picos:
        pico.retrieveProcessData()
        
    newTime = time.time()
    if (newTime - lastTime > BROADCAST_PERIOD):
        records.append(broadcast(picos))
        lastTime = newTime
        
        if (len(records) > 200):
            records_ = []
            for value in records:
                if (value != 0):
                    records_.append(value)
            print(records_)
            plt.plot(records_)
            plt.show()
            while True:
                time.sleep(1)
        
       