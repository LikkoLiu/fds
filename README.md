# 老年人跌倒检测系统 （FDS）

## 已有功能
- UART0 代码调试
- LED 闪烁 task 指示系统运行
- IMU 六轴参数处理
- TOF 距离数据处理
- BLE5.0
- log 格式规范

## 待开发
- GPS 定位 （×） 
- tiny-ML 学习模型部署 
- 蓝牙室内定位  

## ISSUES
1. GPIO 初始化不能放在 task 中进行初始化，否则会造成 " Core  1 panic'ed (LoadProhibited). Exception was unhandled. "   
**Implement**： No  

2. SysRunningLedTask 堆栈深度必须大于 512 ，否则会造成 " A stack overflow in task SysRunningLedTa has been detected "  
**Implement**： No  -   猜测是 HAL 库的调用太深

3. git 配置 github 远程仓库存在 " undenide " 问题   
**Implement**： remote 配置将 http 改为 git 形式 

4. 配置 timer.h 文件时，出现 " fatal error: esp_timer.h: No such file or directory "    
**Implement**： 顶层 cmakelists 中 idf_component_register 加入 PRIV_REQUIRES driver esp_timer 

5. IMU 算法 yaw 值漂移问题   
**Implement**： 暂时无法解决，需加入磁力计  
![alt text](https://img-blog.csdnimg.cn/20210117115154114.png)

6. 在 gps 读取数据，进行字符串查找时，出现     
" esp32Core  0 register dump:     
PC      : 0x40057143  PS      : 0x00060930  A0      : 0x82009c89  A1      : 0x3fca5360   0x40057143: strchr in ROM "
**Implement**： 将 data 、 dest 字符串缓存的命名从全局 改为 在任务函数开始处命名

## Optimization
1. 蓝牙 GAP 广播间隔修改 MIN：0x0800(1.28Sec) MAX：0x0F00   
**res**：蓝牙未连接时，功耗由 35mA 降低为 24mA；但蓝牙连接时，功耗为 28mA 
```shell
I (16429) GATTS_FDS: update connection params status = 0, min_int = 0, max_int = 0,conn_int = 32,latency = 0, timeout = 400
```
**Influence**：蓝牙连接速度变慢    

2. 将 SPI Flash 由 DIO 切换为 QIO          
**res**：蓝牙未连接时，功耗由 24mA 降低为 23mA；但蓝牙连接时，功耗为 27mA    
**Influence**：暂无   
