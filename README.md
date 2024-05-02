# 老年人跌倒检测系统 （FDS）

## 已有功能
- UART0 代码调试
- LED 闪烁 task 指示系统运行

## 待开发
- log
- IMU 六轴参数处理
- TOF 距离数据处理
- GPS 定位（室外）
- tiny-ML 学习模型部署 
- 蓝牙室内定位  

## ISSUES
1. GPIO 初始化不能放在 task 中进行初始化，否则会造成 " Core  1 panic'ed (LoadProhibited). Exception was unhandled. "   
**Implement**： No  

2. SysRunningLedTask 堆栈深度必须大于 512 ，否则会造成 " A stack overflow in task SysRunningLedTa has been detected "  
**Implement**： No   
                猜测是 HAL 库的调用太深
