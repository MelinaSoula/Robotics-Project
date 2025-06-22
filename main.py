# Imports
from machine import SoftI2C, Pin, I2C
from utime import ticks_diff, ticks_ms, sleep
from ssd1306 import SSD1306_I2C
from max30102 import MAX30102, MAX30105_PULSE_AMP_MEDIUM
from math import sqrt
from collections import deque
import framebuf
import dht
import network
import time
from BlynkLib import Blynk
import constants
from sgp30 import Adafruit_SGP30
from time import time, sleep_ms
from machine import PWM

TEST_MODE = False
TEMP = 22          
HUM = 100         
CO2 = 3000        
TVOC = 300

HEART_RATE = 150   
SPO2 = 100

'''
Only BPM abnormal Doctor Short 2x beep (e.g. beep–beep)
Only Environment abnormal Nurse 1 long beep (e.g. beeeeeep)
Both BPM and Environment abnormal Nurse 3 quick beeps (e.g. beep-beep-beep)
'''

# Environmental Thresholds
CO2_THRESHOLD = 5000  # ppm
TEMP_HIGH_THRESHOLD = 30  # °C
TEMP_LOW_THRESHOLD = 10  # °C
HUMIDITY_THRESHOLD = 80  # %
TVOC_THRESHOLD = 500  # ppb

# Heart Related Thresholds
BPM_HIGH_THRESHOLD = 100  # bpm (Tachycardia)
BPM_LOW_THRESHOLD = 60  # bpm (Bradycardia)
SPO2_LOW_THRESHOLD = 95  # % (Low SpO2)

# Icons
thermo_icon = bytearray([
    0x18, 0x18, 0x18, 0x18, 0x18, 0x3C, 0x3C, 0x18
])
thermo_fb = framebuf.FrameBuffer(thermo_icon, 8, 8, framebuf.MONO_HLSB)

water_icon = bytearray([
    0x18, 0x18, 0x18, 0x3C, 0x3C, 0x3C, 0x18, 0x00
])
water_fb = framebuf.FrameBuffer(water_icon, 8, 8, framebuf.MONO_HLSB)

co2_icon = bytearray([
    0x3C, 0x42, 0x40, 0x40, 0x42, 0x3C, 0x04, 0x38
])
co2_fb = framebuf.FrameBuffer(co2_icon, 8, 8, framebuf.MONO_HLSB)

warning_icon = bytearray([
    0x10, 0x38, 0x6C, 0xC6, 0x82, 0x82, 0xFE, 0x00
])
warning_fb = framebuf.FrameBuffer(warning_icon, 8, 8, framebuf.MONO_HLSB)

heart_icon = bytearray([
    0x0C, 0x1E, 0x3F, 0x3F, 0x3F,0x1E, 0x0C, 0x00, 
])
heart_fb = framebuf.FrameBuffer(heart_icon, 8, 8, framebuf.MONO_HLSB)

spo2_icon = bytearray([
    0x08, 0x1C, 0x3E, 0x7F, 0x7F, 0x3E, 0x1C, 0x08  
])
spo2_fb = framebuf.FrameBuffer(spo2_icon, 8, 8, framebuf.MONO_HLSB)


# MAX30102
SPO2_BUFFER_SIZE = 100

FINGER_DETECTION_THRESHOLD = 15000
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

def buzzer_beep(alert_type):
    if alert_type == 'doctor':
        buzzer.freq(4400)
        for _ in range(2):
            buzzer.duty_u16(32768) #Turn sound ON (50% duty cycle)
            sleep_ms(150)  #Beep for 150ms
            buzzer.duty_u16(0) # Turn sound OFF
            sleep_ms(150)  #Pause

    elif alert_type == 'nurse_env':
        buzzer.freq(1000)
        buzzer.duty_u16(32768)
        sleep_ms(600)
        buzzer.duty_u16(0)

    elif alert_type == 'nurse_both':
        buzzer.freq(1000)
        for _ in range(3):
            buzzer.duty_u16(32768)
            sleep_ms(120)
            buzzer.duty_u16(0)
            sleep_ms(120)



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

# OLED Display
def display_status(oled, bpm, spo2, co2, tvoc, temp, hum, screen_state):
    oled.fill(0)
    oled_width = 128

    if screen_state == 1:
        # Screen 1: BPM and SpO2
        title = "VITALS"
        title_x = (oled_width - len(title) * 8) // 2
        oled.text(title, max(0, title_x), 0)
        oled.hline(0, 12, oled_width, 1)

        oled.blit(heart_fb, 0, 24)
        oled.text(f"BPM: {bpm}" if bpm is not None else "BPM: --", 12, 24)

        oled.blit(spo2_fb, 0, 34)
        oled.text(f"SpO2: {spo2}%" if spo2 is not None else "SpO2: --%", 12, 34)

    elif screen_state == 2:
        # Screen 2: Temperature, Humidity, CO₂, TVOC
        title = "ENVIRONMENT"
        title_x = (oled_width - len(title) * 8) // 2
        oled.text(title, max(0, title_x), 0)
        oled.hline(0, 12, oled_width, 1)

        oled.blit(thermo_fb, 0, 24)
        oled.text(f"T: {temp}C" if temp is not None else "T: --C", 12, 24)

        oled.blit(water_fb, 0, 34)
        oled.text(f"H: {hum}%" if hum is not None else "H: --%", 12, 34)

        oled.blit(co2_fb, 0, 44)
        oled.text(f"CO2: {co2}ppm" if co2 is not None else "CO2: --ppm", 12, 44)

        oled.blit(warning_fb, 0, 54)
        oled.text(f"TVOC: {tvoc}ppb" if tvoc is not None else "TVOC: --ppb", 12, 54)

    oled.show()



# Internet Connection
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

# Globlas for main function
ALERT_COOLDOWN = 60  
ALERT_STABLE_DURATION = 20  

last_alert_times = {
    "room_conditions": 0,
    "patient_condition": 0,
    "patient_discomfort": 0
}

abnormal_start_times = {
    "room_conditions": None,
    "patient_condition": None,
    "patient_discomfort": None
}

buzzer = PWM(Pin(15))
buzzer.freq(4400)
buzzer.duty_u16(0)


co2_alert_state = {
    "start_time": None,
    "last_alert_time": 0,
    "mode": None,  # 'beep' or 'continuous'
    "active": False,
    "buzzer_start_time": None,
    "last_beep_time": None
}

  
def initialize_display():
    i2c_oled = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    oled_address = 0x3C  # Common default address for SSD1306

    if oled_address not in i2c_oled.scan():
        print("OLED not detected or not recognized.")
        return None

    oled = SSD1306_I2C(128, 64, i2c_oled)
    print("OLED initialized.")
    return oled


def initialize_max30102():
    i2c_max = SoftI2C(sda=Pin(16), scl=Pin(17), freq=400000)
    sensor = MAX30102(i2c=i2c_max)

    if sensor.i2c_address not in i2c_max.scan():
        print("MAX30102 not detected on I2C bus.")
        return None, None

    if not sensor.check_part_id():
        print("MAX30102 detected, but part ID check failed.")
        return None, None

    print("MAX30102 sensor initialized.")
    sensor.setup_sensor()
    sensor.set_sample_rate(400)
    sensor.set_fifo_average(8)
    sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)

    acquisition_rate = 50  # Hz
    return sensor, acquisition_rate

def dht_and_sgp30():
    i2c_sgp30 = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
    try:
        dht_sensor = dht.DHT11(Pin(2))
        print("DHT11 sensor initialized.")
    except Exception as e:
        print("DHT11 sensor not detected or not recognized.")
        dht_sensor = None

    try:
        sgp30 = Adafruit_SGP30(i2c_sgp30)
        print("SGP30 sensor initialized.")
    except Exception as e:
        print("SGP30 sensor not detected or not recognized.")
        sgp30 = None
    return dht_sensor, sgp30

def acquire_max_data(sensor, red_buffer, ir_buffer, hr_monitor, buffer_size):
    try:
        if sensor:
            sensor.check()
            if sensor.available():
                red = sensor.pop_red_from_storage()
                ir = sensor.pop_ir_from_storage()

                red_buffer.append(red)
                ir_buffer.append(ir)

                if len(red_buffer) > buffer_size:
                    red_buffer.pop(0)
                    ir_buffer.pop(0)

                hr_monitor.add_sample(ir)
    except Exception as e:
        print(f"Error during sensor data acquisition: {e}")
        
def read_environmental_data(dht_sensor, sgp30):
    """Reads temperature, humidity, CO2, and TVOC from sensors."""
    
    temp, hum, co2, tvoc = None, None, None, None

    # Read data from DHT sensor
    try:
        if dht_sensor:
            dht_sensor.measure()
            temp = dht_sensor.temperature()
            hum = dht_sensor.humidity()
    except Exception:
        pass  # Keeps temp and hum as None if reading fails

    # Read data from SGP30 sensor
    try:
        if sgp30:
            co2, tvoc = sgp30.iaq_measure()
    except Exception:
        pass  # Keeps co2 and tvoc as None if reading fails

    return temp, hum, co2, tvoc

def read_dht_sensor(dht_sensor):
    """Reads temperature and humidity from the DHT sensor."""
    temp, hum = None, None
    try:
        if dht_sensor:
            dht_sensor.measure()
            temp = dht_sensor.temperature()
            hum = dht_sensor.humidity()
    except Exception as e:
        print(f"Error reading DHT sensor: {e}")
    return temp, hum

def read_sgp30_sensor(sgp30):
    """Reads CO₂ and TVOC levels from the SGP30 sensor."""
    co2, tvoc = None, None
    try:
        if sgp30:
            co2, tvoc = sgp30.iaq_measure()
    except Exception as e:
        print(f"Error reading SGP30 sensor: {e}")
    return co2, tvoc
def display_status1(oled, bpm=None, spo2=None, status="", temp=None, hum=None, co2=None, tvoc=None):
    oled.fill(0)  # Clear screen
    oled_width = 128

    # Title (centered)
    title = "OXIMETER"
    title_x = (oled_width - len(title) * 8) // 2
    oled.text(title, title_x, 0)

    # Horizontal separator
    oled.hline(0, 12, oled_width, 1)

    if bpm is not None and spo2 is not None and bpm != "Measuring":
        # Show vitals (centered)
        heart_text = "BPM: {}".format(bpm)
        spo2_text = "SpO2: {}%".format(spo2)
        oled.text(heart_text, (oled_width - len(heart_text) * 8) // 2, 20)
        oled.text(spo2_text, (oled_width - len(spo2_text) * 8) // 2, 34)

    else:
        # Show environmental data
        lines = [
            "T: {}C".format(temp if temp is not None else "--"),
            "H: {}%".format(hum if hum is not None else "--"),
            "CO2: {}".format(co2 if co2 is not None else "--"),
            "TVOC: {}".format(tvoc if tvoc is not None else "--")
        ]
        for i, line in enumerate(lines):
            oled.text(line, 0, 16 + (i * 10))

    # Status at bottom
    status = status[:16]  # Truncate if too long
    oled.text(status, (oled_width - len(status) * 8) // 2, 56)

    oled.show()

def main():
    # Initialize sensor buffers
    red_buffer = []
    ir_buffer = []
    # Internet connection
    connect_to_internet(constants.INTERNET_NAME, constants.INTERNET_PASSWORD)

    '''Initialize OLED and MAX30102'''
    oled = initialize_display()
    sensor, acquisition_rate = initialize_max30102()
    ''' Initialize DHT and SGP30 '''
    dht_sensor, sgp30 = dht_and_sgp30()
    
    ''' Initialize Blynk '''
    BLYNK = Blynk(constants.BLYNK_AUTH_TOKEN)
    
    '''Initialize hr_monitor'''
    hr_monitor = HeartRateMonitor(
        sample_rate=acquisition_rate,
        window_size=acquisition_rate * 3,
        smoothing_window=5
    )

    '''Variables for screen switching'''
    screen_state = 1
    last_screen_switch = ticks_ms()

    ''' Initialize vital and environmental variables'''
    temp, hum = None, None
    co2, tvoc = None, None
    heart_rate = None
    spo2 = None
    
    finger_on_sensor = False
    ref_time = ticks_ms()
    
    ''' reverse timer'''
    timer = 2
    
    

    while True:
        acquire_max_data(sensor, red_buffer, ir_buffer, hr_monitor, SPO2_BUFFER_SIZE)

        ''' Data processing every 2 seconds'''
        if ticks_diff(ticks_ms(), ref_time) / 1000 > 2:
            ref_time = ticks_ms()
            finger_on_sensor = finger_detected(ir_buffer)

            '''Environmental Data - always updated'''
            if TEST_MODE:
                temp = TEMP           # 15°C to 30°C
                hum = HUM          # 30% to 70%
                co2 = CO2         # 400 to 3400 ppm
                tvoc = TVOC         # 100 to 1100 ppb
            else:
                temp, hum = read_dht_sensor(dht_sensor)
                co2, tvoc = read_sgp30_sensor(sgp30)
                
            if not finger_detected(ir_buffer) and not TEST_MODE :
                
               finger_present = False
               print("Finger not detected. Place finger on sensor.")
               print("Finger not detected. Place finger on sensor.")
               print("Environmental Data:")
               print(f"Temperature: {temp if temp is not None else '--'} °C")
               print(f"Humidity: {hum if hum is not None else '--'} %")
               print(f"CO₂: {co2 if co2 is not None else '--'} ppm")
               print(f"TVOC: {tvoc if tvoc is not None else '--'} ppb")
               print("----")
               
               red_buffer = []
               ir_buffer = []
             
               timer = 2
               
               display_status1(oled, None, None, "No finger", temp, hum, co2, tvoc)
               
               BLYNK.virtual_write(0, temp)
               BLYNK.virtual_write(1, hum)
               BLYNK.virtual_write(5, co2)
               BLYNK.virtual_write(4, tvoc)
               BLYNK.virtual_write(9,   0)
               BLYNK.virtual_write(8,   0)
                
               continue

            if TEST_MODE:
                finger_on_sensor = True # Always pretend finger is on
                heart_rate = HEART_RATE    # 40-110 BPM
                spo2 = SPO2           # 85-95%
            else:
                heart_rate = hr_monitor.calculate_heart_rate()
                spo2 = calculate_spo2(red_buffer, ir_buffer)
            
            if(timer==0 ):
                print(f"Heart Rate: {heart_rate if heart_rate else 'Not enough data'} BPM")
                print(f"SpO2: {spo2 if spo2 else 'Not enough data'}%")
                print("----")
                
                
            else:
                print(f"Not enough data. Wait more {timer} sec")
            
            # Display and Blynk Updates
            if not finger_on_sensor:
                bpm_display = "No finger"
                spo2_display = "No finger"
                heart_rate = None
                spo2 = None
            elif timer > 0:
                bpm_display= "Measuring"
                spo2_display= "Measuring"
                status = f"Wait {timer} sec"
                timer -= 1
            else:
                bpm_display = heart_rate if heart_rate else "--"
                spo2_display = spo2 if spo2 else "--"
                status = "Measuring..."

            print(f"BPM: {bpm_display}")
            print(f"SpO₂: {spo2_display}")
            print(f"Temperature: {temp if temp is not None else '--'} °C")
            print(f"Humidity: {hum if hum is not None else '--'} %")
            print(f"CO₂: {co2 if co2 is not None else '--'} ppm")
            print(f"TVOC: {tvoc if tvoc is not None else '--'} ppb")
            print("-" * 40)
            
            display_status1(
            oled,
            bpm=bpm_display,
            spo2=spo2_display,
            status=status,
            temp=temp,
            hum=hum,
            co2=co2,
            tvoc=tvoc
            )

            if ticks_diff(ticks_ms(), last_screen_switch) / 1000 >= 3:
                screen_state = 1 if screen_state == 2 else 2
                last_screen_switch = ticks_ms()

            #display_status(oled, bpm_display, spo2_display, co2, tvoc, temp, hum, screen_state)
            
            BLYNK.virtual_write(0, temp)
            BLYNK.virtual_write(1, hum)
            BLYNK.virtual_write(5, co2)
            BLYNK.virtual_write(4, tvoc)
            BLYNK.virtual_write(9, heart_rate if heart_rate  else 0)
            BLYNK.virtual_write(8, spo2 if spo2  else 0)
            
        # Only process alerts if both heart_rate and spo2 are available
        if heart_rate is not None and spo2 is not None:
            now = time()

            # 🟡 1. Detect environmental abnormalities
            env_alerts = []
            if co2 is not None and co2 > CO2_THRESHOLD:
                env_alerts.append(f"⚠️ CO2 High: {co2} ppm")
            if temp is not None:
                if temp > TEMP_HIGH_THRESHOLD:
                    env_alerts.append(f"🔥 High Temp: {temp}°C")
                elif temp < TEMP_LOW_THRESHOLD:
                    env_alerts.append(f"❄️ Low Temp: {temp}°C")
            if hum is not None and hum > HUMIDITY_THRESHOLD:
                env_alerts.append(f"💧 High Humidity: {hum}%")
            if tvoc is not None and tvoc > TVOC_THRESHOLD:
                env_alerts.append(f"⚠️ High TVOC: {tvoc} ppb")

            # 🔴 2. Detect heart condition alerts
            heart_alerts = []
            if heart_rate > 100:
                heart_alerts.append(f"⚠️ High BPM: {heart_rate} bpm (Tachycardia)")
            elif heart_rate < 60:
                heart_alerts.append(f"⚠️ Low BPM: {heart_rate} bpm (Bradycardia)")
            if spo2 < 95:
                heart_alerts.append(f"⚠️ Low SpO₂: {spo2}%")

            # 🔵 3. Decide who to alert and beep appropriately
            if heart_alerts and not env_alerts:
                if abnormal_start_times["patient_condition"] is None:
                    abnormal_start_times["patient_condition"] = now
                elif (now - abnormal_start_times["patient_condition"] >= ALERT_STABLE_DURATION and
                      now - last_alert_times["patient_condition"] >= ALERT_COOLDOWN):
                    BLYNK.log_event("patient_condition", "\n".join(heart_alerts))
                    buzzer_beep('doctor')
                    last_alert_times["patient_condition"] = now

            elif env_alerts and not heart_alerts:
                if abnormal_start_times["room_conditions"] is None:
                    abnormal_start_times["room_conditions"] = now
                elif (now - abnormal_start_times["room_conditions"] >= ALERT_STABLE_DURATION and
                      now - last_alert_times["room_conditions"] >= ALERT_COOLDOWN):
                    BLYNK.log_event("room_conditions", "\n".join(env_alerts))
                    buzzer_beep('nurse_env')
                    last_alert_times["room_conditions"] = now

            elif env_alerts and heart_alerts:
                if abnormal_start_times["patient_discomfort"] is None:
                    abnormal_start_times["patient_discomfort"] = now
                elif (now - abnormal_start_times["patient_discomfort"] >= ALERT_STABLE_DURATION and
                      now - last_alert_times["patient_discomfort"] >= ALERT_COOLDOWN):
                    all_alerts = heart_alerts + env_alerts
                    BLYNK.log_event("patient_discomfort", "\n".join(all_alerts))
                    buzzer_beep('nurse_both')
                    last_alert_times["patient_discomfort"] = now

            # 🔘 Reset timers when conditions are safe again
            if not heart_alerts:
                abnormal_start_times["patient_condition"] = None
            if not env_alerts:
                abnormal_start_times["room_conditions"] = None
            if not heart_alerts or not env_alerts:
                abnormal_start_times["patient_discomfort"] = None



            BLYNK.run()
            

if __name__ == "__main__":
    main()

