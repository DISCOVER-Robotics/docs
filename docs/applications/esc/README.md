# Instructions for ESC Firmware Upgrade Program

## Download Link
[empty](https://empty)

## Dependencies Installation
```bash
pip3 install can-python rich
```

## Basic Usage
```bash
python iap.py -i [ESC ID] -m [CAN interface] [firmware path]
```

## Example
Assuming the firmware and "iap.py" are in the same directory, the file name is "BLDC_4_ChibiOS.bin", the ESC ID to be upgraded is "1", and the CAN interface used is "can0", then execute the following command:
```bash
python iap.py BLDC_4_ChibiOS.bin -i 1 -m can0
```
## Program Output
```
❯ python iap.py BLDC_4_ChibiOS.bin -i 2 -m can0
Device name: vesc-motor-control
Device detected.
Version: 2.7.0
Request successful. Starting data transmission.
Transmitting data... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╸ 100% 0:00:00
End request successful. Data transmit finished.
Please wait for the program auto close and don't shutdown the power supply now.
Burning firmware... ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━ 100% 0:00:00
Burn finished.
Version: 2.7.0
Time elapsed: 40.14004611968994 seconds
```

If the above output appears, it means the firmware upgrade was successful.

## Notes
Currently, only ESC firmware upgrades are supported, and baseboard firmware upgrades are not supported.