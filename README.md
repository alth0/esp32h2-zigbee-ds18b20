# ESP32-H2 Zigbee DS18B20 Temperature Sensor

This project configures an ESP32-H2 into a battery-friendly Zigbee End Device that monitors ambient conditions using a DS18B20 temperature sensor. By default, the firmware automatically initializes the 1-Wire protocol on GPIO 4 using the ESP-IDF RMT peripheral backend. The device unconditionally transmits a Zigbee cluster report every 5 minutes.

To flash the firmware, download the combined factory binary from the [Releases](https://github.com/alth0/esp32h2-zigbee-ds18b20/releases) tab. Connect your ESP32-H2 to your computer, hold the BOOT button while resetting the board to enter download mode, and run the following command using esptool.py:

`esptool.py --chip esp32h2 --port COM3 --baud 460800 write_flash 0x0 esp32h2-zigbee-*-factory-*.bin`

(Be sure to replace COM3 with your actual serial port and match the exact downloaded filename).
