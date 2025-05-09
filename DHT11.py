import machine
import dht
import time

sensor = dht.DHT11(machine.Pin(2))  # Update pin if needed

while True:
    try:
        time.sleep(2)  # Let the sensor settle
        sensor.measure()
        temp = sensor.temperature()
        hum = sensor.humidity()
        print("Temperature: {:.2f}°C, Humidity: {:.2f}%".format(temp, hum))
    except Exception as e:
        print("Failed to read sensor:", e)

    time.sleep(3)  # DHT22 needs delay between reads

