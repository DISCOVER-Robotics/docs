# Instructions for Electronic Speed Controller(ESC) Firmware Upgrade Program

## Download Upgrade Script and Firmware
- [Upgrade script](https://git.qiuzhi.tech:20000/airbot-play/control/sdk/-/raw/develop/examples/python/iap_burn.py?ref_type=heads&inline=false)
- [Firmware](https://127.0.0.1)

## Dependencies Installation
```bash
pip3 install can-python rich
```

## Basic Usage
```bash
python [Upgrade script] -i [ESC ID] -m [CAN interface] [Firmware path]
```

## Example
Assuming the Upgrade script, named "iap.py", and the Firmware, named "BLDC_4_ChibiOS.bin" , are in the same directory, the ESC ID to be upgraded is "1", and the CAN interface used is "can0", then execute the following command:
```bash
python iap.py BLDC_4_ChibiOS.bin -i 1 -m can0
```
## Program Output
```
❯ python iap.py BLDC_4_ChibiOS.bin -i 1 -m can0
Device name: vesc-motor-control
Device detected.
Version: 2.6.0
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
Currently, the IAP script only supports ESC firmware upgrades, and baseboard firmware upgrades are not yet supported.