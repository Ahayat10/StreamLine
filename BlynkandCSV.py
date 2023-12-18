# Import Python System Libraries
import time
from BlynkTimer import BlynkTimer
import BlynkLib
import csv 
from datetime import datetime

# Import Blinka Libraries
import busio
from digitalio import DigitalInOut, Direction, Pull
import board

# Import RFM9x
import adafruit_rfm9x

BLYNK_AUTH_TOKEN = 'YOURAUTENTICATIONTOKEN'

# Initialize Blynk
blynk = BlynkLib.Blynk(BLYNK_AUTH_TOKEN)

# Create BlynkTimer Instance
timer = BlynkTimer()

# Configure LoRa Radio
CS = DigitalInOut(board.CE1)
RESET = DigitalInOut(board.D25)
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, 915.0)
rfm9x.tx_power = 23
rfm9x.sync_word = 0xF3

# Create BlynkTimer Instance
timer = BlynkTimer()

@blynk.on("connected")
def blynk_connected():
    print("Raspberry Pi Connected to New Blynk") 
    time.sleep(2)
    
@blynk.on("V0")
def v0_write_handler(value):
    message = None
    if int(value[0]) == 0:
        print('Received OFF')
        message = "OFF"
        sendDataEsp(message)
    if int(value[0]) == 1:
        message = "ON"
        print('Received ON')
        sendDataEsp(message)
    # print("SENT TO ESP")
    
def sendApp():
    humidity, temperature, moisture = receiveData()
    #print("app" , humidity, temperature, moisture, valve_state)
    if humidity is not None and temperature is not None and moisture is not None:
        blynk.virtual_write(3, humidity)
        blynk.virtual_write(2, temperature)
        blynk.virtual_write(4, moisture)
        print("sent to app")

def sendDataEsp(message, repeat=10):
    for _ in range(repeat):
        if rfm9x.send(message.encode("utf-8")):
            print(f"Sent {message}")
        else:
            print(f"Failed to send {message}")

def receiveData():
    # global send_counter  # Declare send_counter as a global variable
    humidity = None # Initialize humidity variable as None
    temperature = None # Initialize temperature variable as None
    moisture = None  # Initialize moisture variable as None
    #valve_state = None  # Initialize valve_state variable as None
    
    packet = rfm9x.receive()
    if packet is not None:
        try:
            # Try to decode the packet data as UTF-8
            packet_text = packet.decode("utf-8")
            print("Received LoRa packet:", packet_text)

            # Parse the received LoRa packet
            key_value_pairs = packet_text.split(',')
            print("keyValue:" , key_value_pairs)
            parsed_data = {}

            for pair in key_value_pairs:
                parts = pair.strip().split(':', 1)  # Split at the first colon
                #print("parts:" , parts)
                #print("length:" , len(parts))
                if len(parts) == 2:
                    key, value = parts[0].strip(), parts[1].strip()
                    parsed_data[key] = value
            
            if 'dity' in parsed_data:
                humidity = parsed_data['dity']
                
            if 'Temp' in parsed_data:
                temperature = parsed_data['Temp']

            if 'Moisture Level' in parsed_data:
                moisture = float(parsed_data['Moisture Level'].rstrip('%'))

            print("humidity:" , humidity)
            print("Temp: " , temperature)
            print("Moisture:", moisture)
            
        except UnicodeDecodeError:
            print("Failed to decode the packet as UTF-8")
    
    return humidity, temperature, moisture

send_counter = 0  # Packet counter
filename = "SensorData.csv"

with open(filename, mode='a', newline='') as csvfile:
    csv_writer = csv.writer(csvfile)
    if csvfile.tell() == 0:
        csv_writer.writerow(["Timestamp", "Humidity", "Temperature", "Moisture"])

while True:
    blynk.run()
    humidity, temperature, moisture = receiveData()
    sendApp()


    if humidity is not None and temperature is not None and moisture is not None:
        # Get the current timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")

        # Write data with timestamp to the CSV file
        with open(filename, mode='a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([timestamp, humidity, temperature, moisture])

        time.sleep(2)  # Adjust sleep duration based on your requirements
