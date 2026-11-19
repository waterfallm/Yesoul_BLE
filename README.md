# Yesoul_BLE

# Summary
This project uses an ESP32 board to receive data from a Yesoul smart bike (S3, G1M Plus, G1M Max) and rebroadcast it in a format that works with a Garmin Edge cycle computer or Garmin Watch that supports BLE sensors.

# Required Libraries
* [NimBLE-Arduino](https://github.com/h2zero/NimBLE-Arduino)
* Arduino

# Hardware
* ESP32 board of some type, no headers needed

# Details
The goal here is to allow the connection of an indoor bike (AKA spin bike, exercise bike) to a Garmin Edge Cycling computer. This is needed as the Yesoul bike broadcasts under the [Fitness Machine Service](https://www.bluetooth.com/specifications/specs/fitness-machine-service-1-0/) and the Garmin Edge/watch will not receive this profile. So this project receives the Fitness Machine data, converts the cadence, power, and speed data as needed, and rebroadcasts this data under the [Cycling Power Service](https://www.bluetooth.com/specifications/specs/cycling-power-service-1-1/) and [Cycling Speed & Cadence Service](https://www.bluetooth.com/specifications/specs/cycling-speed-and-cadence-service-1-1/).

## Supported Bikes
* **Yesoul S3** (advertised name: `YS_G1_...`)
* **Yesoul G1M Plus** (advertised name: `YS_G1MPLUS...`)
* **Yesoul G1M Max** (advertised name: `YS_G1MMAX...`)

The bike model is automatically detected from the advertised BLE name. Each model may use a slightly different power scaling factor, which is applied automatically.

## How It Works
The Yesoul bike exposes its live metrics through the FTMS Indoor Bike Data characteristic (0x2AD2), which is a sequence of optional fields selected by a 16-bit flags word. This project parses the flags and walks the fields in order, so it works correctly regardless of which fields each bike model chooses to send. The parsed data is then re-broadcast under:
* **Cycling Power Service (0x1818)** – instantaneous power + crank revolutions
* **Cycling Speed & Cadence Service (0x181C)** – cadence + wheel revolutions (enables distance tracking on Garmin)

# Known Limitations
* Currently this connects to the first device it sees with the Fitness Machine Service UUID (0x1826) and Indoor Bike characteristic UUID (0x2AD2). If there are two Yesoul bikes in range, the first one found will be used.
* The wheel circumference for distance calculation is approximated at 2.5 m. Adjust `wheelCircumferenceM` in `main.cpp` if your bike has a different flywheel size.

# References
This is a great write up on understanding the Cycling Power Service, [Arduino BLE Cycling Power Service Blog Post](https://teaandtechtime.com/arduino-ble-cycling-power-service)