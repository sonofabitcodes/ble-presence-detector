ESP32 Bluetooth Low Energie presence detector
========================

Scans nearby BLE devices, and publishes them over http and/or mqtt.  
>[There is a example node backend as well](https://github.com/sonofabitcodes/ble-presence-detector-backend-example)

#### Hardware requirements
* ESP32
* 5V Power Supply
* Micro-USB cable to flash

#### Software requirements
* Computer running Linux, Windows or Mac
* [ESP-IDF (Espressif IoT Development Framework)](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/)

## How to use

#### Configure the device

```bash
make menuconfig
```

* Set serial port under Serial Flasher Options.
* Set Wifi and configure the device under BLE Presence Detector Configuration > WIFI

#### Build and Flash

```bash
make flash
```

#### Monitor device for debugging purposes

```bash
make monitor
```

> To exit the serial monitor, type Ctrl+]
