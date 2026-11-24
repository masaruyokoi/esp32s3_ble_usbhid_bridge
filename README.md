#  BLE-to-USB HID Keyboard Input Bridge

## Purpose and Architecture
This project provides a device that sends keyboard input from a source PC to a target PC or smartphone, allowing passwords generated or retrieved by a password manager to be used for login or initial account setup.

## Intended Use Case

1. Create a complex password and save it in a password manager.
2. When entering the saved password on a different PC or smartphone, the user would otherwise have to type it manually on a keyboard, which can lead to mistakes.
3. To prevent mistakes, copy and paste the password from the PC running the password manager into a settings or login screen on the target device. This is accomplished using bridge hardware and software that communicates with it.

## Devices
- Target device: A device that accepts text input from a USB keyboard. Windows, Mac, and Android devices are supported. It connects to the USB HID Bridge via a USB cable.
- Source device: A Windows, Mac, or Linux PC running Google Chrome. It sends the text to be entered on the target device to the USB HID Bridge.
- USB HID Bridge: Receives text from the source device and sends key events to the target device using the USB HID keyboard protocol. It communicates with the source device over Bluetooth and sends key events to the target device through its USB OTG port. The bridge supports the ESP32-S3 only.

On the source PC, the user specifies the text to enter through a web browser interface. The bridge connects to the target PC or smartphone over USB and acts as a USB HID keyboard.

```text
[ Source PC (Chrome extension / Web Bluetooth API) ]
    ↓  BLE GATT write (wireless)
[ ESP32-S3 (NimBLE reception ➔ TinyUSB HID conversion) ]
    ↓  Single USB OTG cable (wired)
[ Target PC (recognized as a standard USB keyboard) ]
```

## Language and Framework
- Uses ESP-IDF
- Implemented in C


## Requirements
- Encrypt Bluetooth communication between the source PC and the ESP32-S3 using BLE pairing.
- Include the MAC address in the Bluetooth device name to identify the individual device.
- Report return values in the web UI to notify the user of errors, such as the device not being mounted as a USB HID device.

## Preparing the Build Environment

First, set up the ESP-IDF development environment. See the [ESP-IDF IDE setup instructions](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html#build-in-ide).

After setup, use the following commands to build the firmware and flash it to the bridge device:

```
idf.py build
idf.py flash
```

