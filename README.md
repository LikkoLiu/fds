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

7. 在初始化 FFT 时，" Not possible to initialize FFT2R INVAILD LENGTH"    
**Implement**： 将 `N_SAMPLES` 由 3000 更换为 2048

8. TOF 在热熔胶固定后，发生初始化失败问题
**Implement**： 热吹风吹了下，怀疑虚焊

9. AlgorithmTask 堆栈设置为 1024 * 64，发生初始化失败问题
```
I (11659) i2c-tof: Previous distance:    0 , CurrentmmDetected distance: 2520 
sudden change in distance from ground!

assert failed: vTaskResume tasks.c:2097 (xTaskToResume)


Backtrace: 0x40375ca6:0x3fce3070 0x403810b1:0x3fce3090 0x40388219:0x3fce30b0 0x40382f1e:0x3fce31d0 0x4200d0a5:0x3fce31f0 0x40381be1:0x3fce3240
0x40375ca6: panic_abort at /home/likko/esp/v5.2.1/esp-idf/components/esp_system/panic.c:472
0x403810b1: esp_system_abort at /home/likko/esp/v5.2.1/esp-idf/components/esp_system/port/esp_system_chip.c:93
0x40388219: __assert_func at /home/likko/esp/v5.2.1/esp-idf/components/newlib/assert.c:81
0x40382f1e: vTaskResume at /home/likko/esp/v5.2.1/esp-idf/components/freertos/FreeRTOS-Kernel/tasks.c:2097 (discriminator 1)
0x4200d0a5: TofTask at /home/likko/Graduation/fds/components/tof/src/i2c_tof.c:112
0x40381be1: vPortTaskWrapper at /home/likko/esp/v5.2.1/esp-idf/components/freertos/FreeRTOS-Kernel/portable/xtensa/port.c:134
```
**Implement**： 减小为 1024*48

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
