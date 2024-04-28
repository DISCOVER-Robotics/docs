# 电调固件升级程序使用说明

## 下载链接
[empty](https://empty)

## 依赖安装
```bash
pip3 install can-python rich
```

## 基础用法
```bash
python iap.py -i [电调板ID] -m [CAN接口] [固件路径]
```
### 示例
假设固件和iap.py在同一个目录下，文件名为BLDC_4_ChibiOS.bin，待升级的电调板ID为1，使用的CAN接口为can0，则执行如下命令：
```bash
python iap.py BLDC_4_ChibiOS.bin -i 1 -m can0
```
### 程序输出
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
如果出现以上输出，则表示固件升级成功。


## 注意事项
目前只支持电调板固件升级，不支持底座板固件升级。
