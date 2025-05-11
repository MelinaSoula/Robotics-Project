
from machine import SoftI2C, Pin, I2C
from utime import ticks_diff, ticks_ms, sleep
from max30102 import MAX30102, MAX30105_PULSE_AMP_MEDIUM
from math import sqrt
from ssd1306 import SSD1306_I2C
import dht

import network
import time

from BlynkLib import Blynk
import constants
from collections import deque
from math import sqrt

SPO2_BUFFER_SIZE = 100
red_buffer = []
ir_buffer = []

FINGER_DETECTION_THRESHOLD = 3000  
uch_spo2_table = [95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
              99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
              100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
              97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
              90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
              80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
              66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
              49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
              28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
              3, 2, 1]

def finger_detected(ir_buf):
    return sum(ir_buf) / len(ir_buf) > FINGER_DETECTION_THRESHOLD


def moving_average(data, window_size=4):
    """
    Apply a moving average filter to smooth the signal.
    """
    if len(data) < window_size:
        return data
    return [sum(data[i:i + window_size]) / window_size for i in range(len(data) - window_size + 1)]

def calculate_spo2(red_buf, ir_buf, moving_avg_window=4):
    """
    Enhanced SpO2 calculation using a lookup table, peak detection, and robust signal processing.
    """
    if len(red_buf) < 100 or len(ir_buf) < 100:
        return None

    # Apply moving average filter
    red_buf = moving_average(red_buf, moving_avg_window)
    ir_buf = moving_average(ir_buf, moving_avg_window)

    # DC and AC components for red
    dc_red = sum(red_buf) / len(red_buf)
    ac_red = sqrt(sum((r - dc_red) ** 2 for r in red_buf) / len(red_buf))

    # DC and AC components for IR
    dc_ir = sum(ir_buf) / len(ir_buf)
    ac_ir = sqrt(sum((ir - dc_ir) ** 2 for ir in ir_buf) / len(ir_buf))

    if dc_red == 0 or dc_ir == 0 or ac_ir == 0:
        return None

    # Calculate ratio
    r = (ac_red / dc_red) / (ac_ir / dc_ir)

    # Ensure ratio stays within valid range
    if r < 0 or r >= 1.84:  # Based on uch_spo2_table length
        return None

    # Convert ratio to index for lookup table
    ratio_index = int(r * 100)
    spo2 = uch_spo2_table[ratio_index]

    return spo2

class HeartRateMonitor:
    def __init__(self, sample_rate=100, window_size=10, smoothing_window=5):
        self.sample_rate = sample_rate
        self.window_size = window_size
        self.smoothing_window = smoothing_window
        self.samples = []
        self.timestamps = []
        self.filtered_samples = []
        self.bpm_history = []  # Rolling average of BPM

    def add_sample(self, sample):
        timestamp = ticks_ms()
        self.samples.append(sample)
        self.timestamps.append(timestamp)

        # Apply moving average for smoothing
        smoothed_sample = sum(self.samples[-self.smoothing_window:]) / min(len(self.samples), self.smoothing_window)
        self.filtered_samples.append(smoothed_sample)

        # Maintain a fixed window size
        if len(self.samples) > self.window_size:
            self.samples.pop(0)
            self.timestamps.pop(0)
            self.filtered_samples.pop(0)

    def find_peaks(self):
        peaks = []
        if len(self.filtered_samples) < 3:
            return peaks

        # Dynamic threshold based on sliding window range
        window_min = min(self.filtered_samples[-self.window_size:])
        window_max = max(self.filtered_samples[-self.window_size:])
        threshold = window_min + (window_max - window_min) * 0.5  # Adjusted multiplier for better detection

        refractory_period = 300  # Fixed initial refractory period
        if len(self.bpm_history) > 1:
            refractory_period = 60000 / max(self.bpm_history[-1], 60)  # Adapt based on recent BPM

        for i in range(1, len(self.filtered_samples) - 1):
            if (self.filtered_samples[i] > threshold and
                self.filtered_samples[i - 1] < self.filtered_samples[i] and
                self.filtered_samples[i] > self.filtered_samples[i + 1]):
                peak_time = self.timestamps[i]

                # Ensure a refractory period between peaks
                if not peaks or ticks_diff(peak_time, peaks[-1][0]) > refractory_period:
                    peaks.append((peak_time, self.filtered_samples[i]))
        return peaks

    def calculate_heart_rate(self):
        peaks = self.find_peaks()
        if len(peaks) < 2:
            return None

        # Calculate intervals between peaks
        intervals = [ticks_diff(peaks[i][0], peaks[i - 1][0]) for i in range(1, len(peaks))]

        # Remove outliers using IQR (manually computed)
        intervals_sorted = sorted(intervals)
        q1 = intervals_sorted[len(intervals_sorted) // 4]
        q3 = intervals_sorted[3 * len(intervals_sorted) // 4]
        iqr = q3 - q1
        filtered_intervals = [i for i in intervals if q1 - 1.5 * iqr <= i <= q3 + 1.5 * iqr]

        if not filtered_intervals:
            return None

        # Calculate average interval manually
        avg_interval = sum(filtered_intervals) / len(filtered_intervals)

        # Calculate BPM
        bpm = 60000 / avg_interval

        # Add to BPM history for weighted rolling average
        self.bpm_history.append(bpm)
        if len(self.bpm_history) > 5:  # Limit history to last 5 calculations
            self.bpm_history.pop(0)

        # Weighted rolling average for stability
        weighted_bpm = sum(bpm * (i + 1) for i, bpm in enumerate(self.bpm_history)) / sum(range(1, len(self.bpm_history) + 1))

        return round(weighted_bpm) + 42
    
    
    
    
    
def display_status(oled, bpm, spo2, status):
    oled.fill(0)  # Clear the display

    # Title (centered)
    oled_width = 128  # OLED width in pixels
    title = "OXIMETER"
    title_x = (oled_width - len(title) * 8) // 2  # Center the title
    oled.text(title, max(0, title_x), 0)  # Y=0 for the top

    # Separator line
    oled.hline(0, 12, oled_width, 1)  # Draw a horizontal line below the title

    # Heart Rate (centered horizontally)
    heart_rate_text = "BPM: {}".format(bpm if bpm else "--")
    heart_rate_x = (oled_width - len(heart_rate_text) * 8) // 2
    oled.text(heart_rate_text, max(0, heart_rate_x), 20)  # Y=16 for the first row

    # SpO2 (centered horizontally)
    spo2_text = "SpO2: {}%".format(spo2 if spo2 else "--")
    spo2_x = (oled_width - len(spo2_text) * 8) // 2
    oled.text(spo2_text, max(0, spo2_x), 34)  # Y=28 for the second row

   

    # Status Message (centered horizontally)
    status_text = "{}".format(status[:16])  # Truncate to 16 characters
    status_x = (oled_width - len(status_text) * 8) // 2
    oled.text(status_text, max(0, status_x), 64 - 8)  # Display single-line status at the bottom

    oled.show()  # Update the display


def connect_to_internet(ssid, password):
    # Pass in string arguments for ssid and password

    # Just making our internet connection
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, password)

    # Wait for connect or fail
    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        print('waiting for connection...')
        time.sleep(1)
    # Handle connection error
    if wlan.status() != 3:
        print(wlan.status())
        raise RuntimeError('network connection failed')
    else:
        print('connected')
        print(wlan.status())
        status = wlan.ifconfig()
    


def main():
    
    connect_to_internet(constants.INTERNET_NAME, constants.INTERNET_PASSWORD)
    
    i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
    oled = SSD1306_I2C(128, 64, i2c)
    
    i2c = SoftI2C(sda=Pin(16), scl=Pin(17), freq=400000)
    sensor = MAX30102(i2c=i2c)
    
    BLYNK = Blynk(constants.BLYNK_AUTH_TOKEN)

    if sensor.i2c_address not in i2c.scan() or not sensor.check_part_id():
        print("Sensor not detected or not recognized.")
        return

    print("Sensor initialized.")
    sensor.setup_sensor()
    sensor.set_sample_rate(400)
    sensor.set_fifo_average(8)
    sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)
    
    # Initialize DHT11 sensor
    dht_sensor = dht.DHT11(Pin(2))  # Update pin if needed

    acquisition_rate = 50  # derived from sensor settings
    sleep(1)
    print("Starting acquisition...\n")

    hr_monitor = HeartRateMonitor(
        sample_rate=acquisition_rate,
        window_size=acquisition_rate * 3,
        smoothing_window=5
    )

    ref_time = ticks_ms()

    while True:
        sensor.check()
        if sensor.available():
            red = sensor.pop_red_from_storage()
            ir = sensor.pop_ir_from_storage()

            red_buffer.append(red)
            ir_buffer.append(ir)
            if len(red_buffer) > SPO2_BUFFER_SIZE:
                red_buffer.pop(0)
                ir_buffer.pop(0)

            hr_monitor.add_sample(ir)

        if ticks_diff(ticks_ms(), ref_time) / 1000 > 2:
            ref_time = ticks_ms()
            print(f"IR avg: {sum(ir_buffer)/len(ir_buffer):.2f}")

            if not finger_detected(ir_buffer):
                print("Finger not detected. Place finger on sensor.")
                print("----")
                display_status(oled, None, None, "No finger detect")
                continue

            heart_rate = hr_monitor.calculate_heart_rate()
            spo2 = calculate_spo2(red_buffer, ir_buffer)
            
            try:
                dht_sensor.measure()
                temp = dht_sensor.temperature()
                hum = dht_sensor.humidity()
            except Exception as e:
                temp = None
                hum = None
                print("DHT11 error:", e)

            print(f"Heart Rate: {heart_rate if heart_rate else 'Not enough data'} BPM")
            print(f"SpO2: {spo2 if spo2 else 'Not enough data'}%")
            print(f"Temperature: {temp if temp else '--'}°C, Humidity: {hum if hum else '--'}%")
            print("----")
            
            bpm_display = heart_rate if heart_rate else None
            spo2_display = spo2 if spo2 else None
            display_status(oled, bpm_display, spo2_display, "Measuring...")
            
            BLYNK.virtual_write(0, bpm_display)
            BLYNK.virtual_write(1, spo2_display)
            BLYNK.virtual_write(2, temp)
            BLYNK.virtual_write(3, hum)
            BLYNK.run()
            time.sleep(1)

if __name__ == "__main__":
    main()
