from machine import SoftI2C, Pin, I2C
from utime import ticks_diff, ticks_ms, sleep
from max30102 import MAX30102, MAX30105_PULSE_AMP_MEDIUM
from math import sqrt
from ssd1306 import SSD1306_I2C


SPO2_BUFFER_SIZE = 100
red_buffer = []
ir_buffer = []

FINGER_DETECTION_THRESHOLD = 3000  # Tune depending on sensor position + ambient light

def finger_detected(ir_buf):
    return sum(ir_buf) / len(ir_buf) > FINGER_DETECTION_THRESHOLD

def calculate_spo2(red_buf, ir_buf):
    if len(red_buf) < SPO2_BUFFER_SIZE or len(ir_buf) < SPO2_BUFFER_SIZE:
        return None

    dc_red = sum(red_buf) / len(red_buf)
    dc_ir = sum(ir_buf) / len(ir_buf)
    ac_red = sqrt(sum((r - dc_red) ** 2 for r in red_buf) / len(red_buf))
    ac_ir = sqrt(sum((ir - dc_ir) ** 2 for ir in ir_buf) / len(ir_buf))

    if dc_red == 0 or dc_ir == 0 or ac_ir == 0:
        return None

    r = (ac_red / dc_red) / (ac_ir / dc_ir)
    spo2 = 110 - 25 * r

    if spo2 < 70 or spo2 > 100:
        return None

    return round(spo2, 1)

class HeartRateMonitor:
    def __init__(self, sample_rate=100, window_size=10, smoothing_window=5):
        self.sample_rate = sample_rate
        self.window_size = window_size
        self.smoothing_window = smoothing_window
        self.samples = []
        self.timestamps = []
        self.filtered_samples = []

    def add_sample(self, sample):
        timestamp = ticks_ms()
        self.samples.append(sample)
        self.timestamps.append(timestamp)

        smoothed_sample = sum(self.samples[-self.smoothing_window:]) / min(len(self.samples), self.smoothing_window)
        self.filtered_samples.append(smoothed_sample)

        if len(self.samples) > self.window_size:
            self.samples.pop(0)
            self.timestamps.pop(0)
            self.filtered_samples.pop(0)

    def find_peaks(self):
        peaks = []
        if len(self.filtered_samples) < 3:
            return peaks

        min_val = min(self.filtered_samples)
        max_val = max(self.filtered_samples)
        threshold = min_val + (max_val - min_val) * 0.6

        for i in range(1, len(self.filtered_samples) - 1):
            if (self.filtered_samples[i] > threshold and
                self.filtered_samples[i - 1] < self.filtered_samples[i] and
                self.filtered_samples[i] > self.filtered_samples[i + 1]):
                peak_time = self.timestamps[i]
                if not peaks or ticks_diff(peak_time, peaks[-1][0]) > 300:
                    peaks.append((peak_time, self.filtered_samples[i]))
        return peaks

    def calculate_heart_rate(self):
        peaks = self.find_peaks()
        if len(peaks) < 2:
            return None

        intervals = [ticks_diff(peaks[i][0], peaks[i - 1][0]) for i in range(1, len(peaks))]
        avg_interval = sum(intervals) / len(intervals)
        bpm = 60000 / avg_interval
        return round(bpm)
    
    
    
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
    oled.text(heart_rate_text, max(0, heart_rate_x), 20)  # Y=20 for the first row

    # SpO2 (centered horizontally)
    spo2_text = "SpO2: {}%".format(spo2 if spo2 else "--")
    spo2_x = (oled_width - len(spo2_text) * 8) // 2
    oled.text(spo2_text, max(0, spo2_x), 36)  # Y=36 for the second row

    # Status Message (handle long messages)
    max_chars = oled_width // 8  # Maximum characters per line (128 pixels / 8 per char = 16 chars)
    if len(status) > max_chars:  # If the status is too long, wrap it
        status_line_1 = status[:max_chars]  # First line
        status_line_2 = status[max_chars:]  # Remaining text for the second line
        oled.text(status_line_1, 0, 52)  # Display first line at Y=52
        oled.text(status_line_2, 0, 60)  # Display second line at Y=60
    else:
        status_x = (oled_width - len(status) * 8) // 2  # Center the status
        oled.text(status, max(0, status_x), 52)  # Display single-line status at Y=52

    oled.show()  # Update the display


def main():
    
    i2c=I2C(0,sda=Pin(0), scl=Pin(1), freq=400000)
    oled = SSD1306_I2C(128, 64, i2c)
    
    i2c = SoftI2C(sda=Pin(16), scl=Pin(17), freq=400000)
    sensor = MAX30102(i2c=i2c)

    if sensor.i2c_address not in i2c.scan() or not sensor.check_part_id():
        print("Sensor not detected or not recognized.")
        return

    print("Sensor initialized.")
    sensor.setup_sensor()
    sensor.set_sample_rate(400)
    sensor.set_fifo_average(8)
    sensor.set_active_leds_amplitude(MAX30105_PULSE_AMP_MEDIUM)

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

            print(f"Heart Rate: {heart_rate if heart_rate else 'Not enough data'} BPM")
            print(f"SpO2: {spo2 if spo2 else 'Not enough data'}%")
            print("----")
            
            bpm_display = heart_rate if heart_rate else None
            spo2_display = spo2 if spo2 else None
            display_status(oled, bpm_display, spo2_display, "Measuring...")

if __name__ == "__main__":
    main()



