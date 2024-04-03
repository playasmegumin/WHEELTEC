# 基于WHEELTEC B570 平衡小车的嵌入式作业代码

2024.4.4更新：增加了Swing模式。
## 最高注意事项：
我所有的注释都是在utf-8编码下写的。如果出现乱码请更改文件编码为utf-8，同时避免在GTK编码下保存文件，否则会产生不可逆的注释丢失。
## 值得关注的文件和函数：
### WHEELTEC/control.c
这是小车控制流程的主文件，其中的HAL_GPIO_EXTI_Callback是包含了小车控制主要逻辑的函数。
Core/main.c里面曾经提到过：由于delay的精度差强人意，本项目采用mpu6050自带的时间中断构成delay50（可以开摆50ms）。
本项目的主要驱动方式是每5ms接收来自mpu6050的中断请求，然后利用HAL_GPIO_EXTI_Callback回调函数进行主逻辑的处理。
这个函数就是个小小的状态机。
标识模式的核心变量是Flag_avoid和Flag_follow。如果Flag_avoid==1就是避障模式，如果Flag_follow==1就是跟随模式。因此在这里我加了一个Flag_swing表示摇摆模式。
此外，我还添加了sin100[]数组和sin100_counter。前者是我用matlab得到的100个采样点的sin函数，后者是一个全局变量，标示小车在此时此刻所处的相位（差不多这个意思）。根据相位可以得到此时小车应有的目标速度。只要速度是三角函数，位移也将是三角函数，借此实现小车的摇摆。

### WHEELTEC/show.c
这是用于设计OLED屏幕的文件。
我修改了里面的部分内容以在OLED上显示sin100[]当前的数值。
### Core/Inc/main.h
我在里面添加了相关的变量声明以令上述行为生效。