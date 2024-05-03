/*
 * control.c
 *
 *  Created on: Jun 14, 2022
 *      Author: lf
 */
#include "control.h"
#include "usart.h"

// GUAHOOK
// sin100函数为具有一个完整周期、有100个采样点的sin函数
float sin100[100] = {0,0.0627,0.1253,0.1873,0.2486,0.0309,0.3681,0.4257,0.4817,0.5358,0.5877,0.6374,0.6845,0.7289,0.7705,0.8090,0.8443,0.8763,0.9048,0.9297,0.9510,0.9685,0.9822,0.9921,0.9980,1,0.9980,0.9921,0.9822,0.9685,0.9510,0.9297,0.9048,0.8763,0.8443,0.8090,0.7705,0.7289,0.6845,0.6374,0.5877,0.5358,0.4817,0.4257,0.3681,0.3090,0.2486,0.1873,0.1253,0.0627,0,-0.0627,-0.1253,-0.1873,-0.2486,-0.3090,-0.3681,-0.4257,-0.4817,-0.5358,-0.5877,-0.6374,-0.6845,-0.7289,-0.7705,-0.8090,-0.8443,-0.8763,-0.9048,-0.9297,-0.9510,-0.9685,-0.9822,-0.9921,-0.9980,-1,-0.9980,-0.9921,-0.9822,-0.9685,-0.9510,-0.9297,-0.9048,-0.8763,-0.8443,-0.8090,-0.7705,-0.7289,-0.6845,-0.6374,-0.5877,-0.5358,-0.4817,-0.4257,-0.3681,-0.3090,-0.2486,-0.1873,-0.1253,-0.0627};
// sin100_counter为当前的sin100数组下标，如果现在为swing模式并且Flag_Stop==0，sin100_counter会在0-100之间循环变化
u8 sin100_counter = 0;

/**************************************************************************
Function: Control function
Input   : none
Output  : none
*********锟杰ｏ拷***锟叫的匡拷锟狡达拷锟诫都************
         5ms锟解部锟叫讹拷***MPU6050***INT***锟脚达拷***
         锟较革拷证************锟捷达拷******时***同***
***诓*********锟�
******  值******
**************************************************************************/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) // 用于处理来自MPU6050的时钟中断，这里是实际上的主函数
{
	static int Voltage_Temp,Voltage_Count,Voltage_All;
	static u8 Flag_Target;											// 这个Flag_Target扮演一个1位计数器的角色
	static u8 Flag_Quatre;
	static u8 Swing_period = 2;										// Swing_period单位为2
	int Encoder_Left,Encoder_Right;
	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
	if(GPIO_Pin==MPU6050_INT_Pin)									// 这里避免非时钟中断触发这里的逻辑
	{
		Flag_Target=!Flag_Target;									// Flag_target翻转，其实是对时钟做了一个二分时

		Get_Angle(Way_Angle);                     					// 这里是采用Way_Angle指定的方式进行角度解算，这里写死了Way_Angle==2用卡尔曼滤波
		Encoder_Left=-Read_Encoder(2);								// 这里的2和4指的是不同通道，编码器的数据包含TIM2/3/4，TIM3是超声波编码器
		Encoder_Right=-Read_Encoder(4);								// 关于TIM等每个信号的含义参见开发手册的4.1节

		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);		// 单位mm/s；把数值读到Velocity_left里面去了，这个是全局变量

		//--------------------------------------------------------
		if(delay_flag==1)											// delay_flag为正的时候主循环会卡住
		{
			if(++delay_50==10)	 delay_50=0,delay_flag=0;  			// 时钟中断的间隔是5ms，因此这里一次delay是50ms（delay_50计数10次）
		}
		//--------------------------------------------------------

		//********************************************************
		Flag_Quatre++;										 // GUAHOOK: 分时器
	//	if(Flag_swing==1 && Flag_Quatre == 4) 				 // 本来想写这个，但为了方便调整摇摆周期还是加了个Swing_period，在其声明处赋值。
		if(Flag_swing==1 && Flag_Quatre == Swing_period*2-1) // Swing_period是摇摆周期，暂定Swing_period==2s
		{													 // sin100[]为100个采样点、一个完整周期的sin函数；若摇摆周期为2s，则每20ms过一个采样点
			if(Flag_Stop==1 || sin100_counter == 99) 	sin100_counter=0; // 当小车停止或到达99时将计数器置0，这是为了保证每次开始运动时都从0开始
			else 										sin100_counter++;

			Flag_Quatre = 0;								 // 重置Flag_Quatre
		}
		//********************************************************

		if(Flag_Target==1)                        					// 每次Flag_Target为1的时候触发，所以这段逻辑触发的间隔是10ms
		{
			Voltage_Temp=Get_battery_volt();
			Voltage_Count++;
			Voltage_All+=Voltage_Temp;
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;
			__HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);				// 清除PR寄存器内容，这样这次中断请求就算正式搞定了（不然过会这个中断请求还会再叫一次）
			return;
		}



		// 下面的逻辑是Flag_Target==0时触发的，这是个二选一的并列关系
		Read_Distane();
		if(Flag_follow==0&&Flag_avoid==0)	Led_Flash(100);			// 没搞出来LED灯还有什么玄妙之处
		if(Flag_follow==1||Flag_avoid==1)	Led_Flash(0);

		// 做个按键检测，主要是检测功能键的状态（就是按了会停止平衡状态那个）
		Key();

		// 平衡环、速度环、转向环全部触发一遍
		Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);
		Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);
		Turn_Pwm=Turn(Gyro_Turn);									// Gyro是角速度

		Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;				// 左右电机的输出力度就是上面这三个PWM的数值叠加
		Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;				// 有意思的是，左右轮的差别就在转向PWM

		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);				// 过一个限制，避免爆了，这玩意是不是上节课写进Simulink里了
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);

		if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right)) //这玩意总感觉没写好，我拿起来的时候并不会停止
			Flag_Stop=1;
		if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))
			Flag_Stop=0;

		Choose(Encoder_Left,Encoder_Right);							// 当小车的Flag_stop=1的时候可以选择模式，否则无事发生

		if(Turn_Off(Angle_Balance,Voltage)==0)     					// 等于0是正常状态，非0为异常状态
			Set_Pwm(Motor_Left,Motor_Right);
		__HAL_GPIO_EXTI_CLEAR_IT(MPU6050_INT_Pin);					// 宣告本次中断请求结束
	 }
}

/**************************************************************************
Function: Vertical PD control
Input   : Angle是角度；Gyro是角速度
Output  : balance所需要的Vertical control PWM数值

PD控制器，用于控制直立环平衡。倾角决定此时此刻车轮需要的加速度。
**************************************************************************/
int Balance(float Angle,float Gyro)
{
   float Angle_bias,Gyro_bias;
	 int balance;
	 Angle_bias=Middle_angle-Angle;
	 Gyro_bias=0-Gyro;
	 balance=-Balance_Kp/100*Angle_bias-Gyro_bias*Balance_Kd/100; 	// 直立环；搜索一下Balance_Kd在main.c中的声明可以看到初始PID数值
	 return balance;												// 这几个Kp/Ki/Kd都会被来自串口（蓝牙）的数据更新
}

/**************************************************************************
Function: Speed PI control
Input   : encoder_left即Left wheel encoder reading；encoder_right即Right wheel encoder reading
Output  : Speed control PWM

PI控制器实现的速度环；输出速度环需要的PWM数值。
最好笑的一集，control.c所有的注释从商家手里出来的时候就已经烂掉了，无法恢复。这一部分的相关注释参照开发手册的4.3节。
**************************************************************************/
int Velocity(int encoder_left,int encoder_right)
{
    static float velocity,Encoder_Least,Encoder_bias,Movement,total_v;
	static float Encoder_Integral,Target_Velocity;

	//================目标速度设置=================================//
	if(Flag_follow==1||Flag_avoid==1) 		Target_Velocity = 30;	// 这里是避障和跟随模式下的目标速度
	//else if(Flag_swing==1)				Target_Velocity = sin100[sin100_counter] * 100; // 本来打算在这里加，但是效果过于不明显了
	else 									Target_Velocity = 50;	// 如果是普通模式的话会减速


	//================这是蓝牙遥控部分，开发手册里没有，看半天没看出来，有一说一很想删了这部分====================//
	if(Flag_front==1)    					Movement = Target_Velocity/Flag_velocity;
	else if(Flag_back==1)					Movement = -Target_Velocity/Flag_velocity;
	else  									Movement = 0;

	//=============我猜是避障和跟随模式所需要施加的“摄动”==================//
	if(Flag_follow==1&&(Distance>200&&Distance<500)&&Flag_Left!=1&&Flag_Right!=1) // Flag_Left和Flag_Right是蓝牙对左右轮的控制信号
		Movement=Target_Velocity/Flag_velocity;
	if(Flag_follow==1&&Distance<200&&Flag_Left!=1&&Flag_Right!=1)	// 这里会把距离控在Distance==200，是不是稍微有点粗糙了
		Movement=-Target_Velocity/Flag_velocity;

	if(Flag_avoid==1&&Distance<450&&Flag_Left!=1&&Flag_Right!=1)	// 前进，直到Distance<450的时候退退退
		Movement=-Target_Velocity/Flag_velocity;

	//================这部分就是纯血的速度环PI控制器了=====================//
	total_v = encoder_left+encoder_right;										// GUAHOOK: 修改速度环
	Encoder_Least = sin100[sin100_counter]*10 - total_v;                    	// 获取最新速度偏差 = 目标速度-测量速度（左右编码器之和）
																				// 这里原代码的目标速度为0。如果现在不是swing模式，那么sin100_counter==0，得到的目标速度也为0
																				// 如果为swing模式，则令目标速度按sin函数变化
																				// sin100[]*10里面的10是峰值速度，可调，影响摇摆的幅度
																				// 直接把基于sin计算出来的速度作为目标速度，积分得到的位移即-cos曲线
	Encoder_bias *= 0.84;		                                          		// 这个Encoder_bias的数值是继承的，是真正用于计算的“速度差值”
	Encoder_bias += Encoder_Least*0.16;											// 相当于上次偏差的 0.84 + 本次偏差的 0.16，减缓速度差值，减少对直立的干扰
	Encoder_Integral += Encoder_bias;                                  			// 积分出位移 积分时间：10ms
	Encoder_Integral =  Encoder_Integral + Movement;                       		// 这里其实是把Movement当速度偏值用，因此返回去看，Movement是速度
	if(Encoder_Integral>10000)  	Encoder_Integral =  10000;             		// 积分限幅
	if(Encoder_Integral<-10000)		Encoder_Integral = -10000;

	// 这里耍了个把戏，Encoder_bias取反就是现在的左右编码器之和（就是现在的速度），因此对其积分的Encoder_Intergral取反就是位移
	// 因此原来的速度Vo实际上等于0-Encoder_bias
	// velocity = -Encoder_bias * Velocity_Kp/100 - Encoder_Integral * Velocity_Ki/100;
	velocity = -Encoder_bias * Velocity_Kp/100 - Encoder_Integral * Velocity_Ki/100;

	if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) Encoder_Integral=0;	// 电机关闭后清除速率积分
	return velocity;
}
/**************************************************************************
Function: Turn control
Input   : Z-axis angular velocity
Output  : Turn control PWM
*********锟杰ｏ拷转******锟�
***诓******锟絑************
******  值***转******锟絇WM
***    锟竭ｏ拷***趣锟狡硷拷******莞******锟睫癸拷司
**************************************************************************/
int Turn(float gyro)
{
	static float Turn_Target,turn,Turn_Amplitude=54;
	float Kp=Turn_Kp,Kd;			//锟睫革拷转***锟劫度ｏ拷***锟睫革拷Turn_Amplitude******
	//===================遥************转******=================//
	if(1==Flag_Left)		Turn_Target=-Turn_Amplitude/Flag_velocity;
	else if(1==Flag_Right)	Turn_Target=Turn_Amplitude/Flag_velocity;
	else Turn_Target=0;
	if(1==Flag_front||1==Flag_back)		Kd=Turn_Kd;
	else 								Kd=0;   //转***锟绞憋拷锟饺★拷*********堑木***锟� 锟叫碉拷模***PID***思***
	//===================转***PD*********=================//
	turn=Turn_Target*Kp/100+gyro*Kd/100;//***锟絑*********锟角斤拷***PD******
	return turn;								 				 //转***PWM***转为*********转为***
}

/**************************************************************************
Function: Assign to PWM register
Input   : motor_left***Left wheel PWM***motor_right***Right wheel PWM
Output  : none
*********锟杰ｏ拷***值***PWM锟侥达拷***
***诓************锟絇WM*********PWM
******  值******
**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
  if(motor_left>0)	{BIN1_SET;		BIN2_RESET;	} //前***
  else				{BIN1_RESET;	BIN2_SET;	} //******
  PWMB=myabs(motor_left);
  if(motor_right>0)	{AIN2_SET;		AIN1_RESET;	}	//前***
  else 				{AIN2_RESET;	AIN1_SET;	}   //******
  PWMA=myabs(motor_right);
}
/**************************************************************************
Function: PWM limiting range
Input   : IN***Input  max***Maximum value  min***Minimum value
Output  : Output
*********锟杰ｏ拷******PWM***值
***诓******锟絀N************锟�  max***锟睫凤拷***锟街�  min***锟睫凤拷***小值
******  值***锟睫凤拷***锟街�
**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
/**************************************************************************
Function: Press the key to modify the car running state
Input   : none
Output  : none
*********锟杰ｏ拷******锟睫革拷小*********状态
***诓*********锟�
******  值******
**************************************************************************/
void Key(void)
{
	u8 tmp,tmp2;
	tmp=click_N_Double(50);						// 检测按键状态，看看是单击了还是双击，双击了不管
	if(tmp==1)									// 这里是单击的情况，看来双击有扩展点；如果单击就翻转停止位
	{
		Flag_Stop=!Flag_Stop;
	}
	tmp2=Long_Press();							// 看看是不是长按；长按的话翻转显示位（显示屏可以停止变化）
	if(tmp2==1)
	{
		Flag_Show=!Flag_Show;

		if(Flag_Show) RetargetInit(&huart1);
		else 		  RetargetInit(&huart3);
	}
}
/**************************************************************************
Function: If abnormal, turn off the motor
Input   : angle***Car inclination***voltage***Voltage
Output  : 1***abnormal***0***normal
*********锟杰ｏ拷锟届常锟截闭碉拷锟�
***诓******锟絘ngle***小******牵锟絭oltage******压
******  值***1***锟届常  0*********
**************************************************************************/
u8 Turn_Off(float angle, int voltage)
{
	u8 temp;
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1000)//***氐锟窖癸拷***锟�10V锟截闭碉拷锟�
	{	                                                 //***谴***锟�40锟饺关闭碉拷锟�
		temp=1;                                          //Flag_Stop***1***************锟狡关闭碉拷锟�
		AIN1_RESET;
		AIN2_RESET;
		BIN1_RESET;
		BIN2_RESET;
	}
	else
		temp=0;
	return temp;
}

///**************************************************************************
//Function: Get angle
//Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
//Output  : none
//函数功能：获取角度
//入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
//返回  值：无
//**************************************************************************/
void Get_Angle(u8 way)
{
	short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
  	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	//Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		accel_x=Accel_X/1671.84;
		accel_y=Accel_Y/1671.84;
		accel_z=Accel_Z/1671.84;
		gyro_x=Gyro_X/939.8;                              //陀螺仪量程转换
		gyro_y=Gyro_Y/939.8;                              //陀螺仪量程转换
		if(Way_Angle==2)
		{
			 Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//卡尔曼滤波
			 Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;
		}
		else if(Way_Angle==3)
		{
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计
	}
}
/**************************************************************************
Function: Absolute value function
Input   : a***Number to be converted
Output  : unsigned int
*********锟杰ｏ拷******值******
***诓******锟絘******要*********锟街碉拷***锟�
******  值***锟睫凤拷*********
**************************************************************************/
int myabs(int a)
{
	int temp;
	if(a<0)  temp=-a;
	else temp=a;
	return temp;
}
/**************************************************************************
Function: Check whether the car is picked up
Input   : Acceleration***Z-axis acceleration***Angle***The angle of balance***encoder_left***Left encoder count***encoder_right***Right encoder count
Output  : 1***picked up  0***No action
*********锟杰ｏ拷***锟叫★拷***欠******锟�
***诓******锟紸cceleration***z******俣龋锟紸ngle***平***慕嵌龋锟絜ncoder_left*********************锟絜ncoder_right***锟揭憋拷************
******  值***1:小************  0***小***未*********
**************************************************************************/
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //***一***
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //******1***小***锟接斤拷***止
			count0++;
			else
			count0=0;
			if(count0>10)
			flag=1,count0=0;
	 }
	 if(flag==1)                                                      //******诙***锟�
	 {
			if(++count1>200)       count1=0,flag=0;                       //***时***锟劫等达拷2000ms******锟截碉拷一***
			if(Acceleration>26000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //******2***小*********0锟饺革拷************
			flag=2;
	 }
	 if(flag==2)                                                       //*********
	 {
		  if(++count2>100)       count2=0,flag=0;                        //***时***锟劫等达拷1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //******3***小*********胎***为*********锟斤到******转***
      {
				flag=0;
				return 1;                                                    //***獾叫★拷*********锟�
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance***Left encoder count***Right encoder count
Output  : 1***put down  0***No action
*********锟杰ｏ拷***锟叫★拷***欠癖环***锟�
***诓******锟狡斤拷锟角度ｏ拷*********************冶************锟�
******  值***1***小*********   0***小***未******
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{
	 static u16 flag,count;
	 if(Flag_Stop==0)                     //***止***锟�
			return 0;
	 if(flag==0)
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //******1***小*********0锟饺革拷******
			flag=1;
	 }
	 if(flag==1)
	 {
		  if(++count>50)                     //***时***锟劫等达拷 500ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_left>3&&encoder_right>3&&encoder_left<40&&encoder_right<40) //******2***小*********胎***未锟较碉拷锟绞憋拷***锟轿拷锟�
      {
				flag=0;
				flag=0;
				return 1;                         //***獾叫★拷*********锟�
			}
	 }
	return 0;
}
/**************************************************************************
Function: Encoder reading is converted to speed (mm/s)
Input   : none
Output  : none
*********锟杰ｏ拷***************转***为锟劫度ｏ拷mm/s***
***诓*********锟�
******  值******
**************************************************************************/
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	float Rotation_Speed_L,Rotation_Speed_R;						//***锟阶拷锟�  转***=******************5ms每锟轿ｏ拷****取频***/***频***/***锟劫憋拷/***************
	Rotation_Speed_L = encoder_left*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Left = Rotation_Speed_L*PI*Diameter_67;		//***************俣锟�=转****锟杰筹拷
	Rotation_Speed_R = encoder_right*Control_Frequency/EncoderMultiples/Reduction_Ratio/Encoder_precision;
	Velocity_Right = Rotation_Speed_R*PI*Diameter_67;		//***************俣锟�=转****锟杰筹拷
}
/**************************************************************************
Function: Select car running mode
Input   : encoder_left***Left wheel encoder reading***encoder_right***Right wheel encoder reading
Output  : none
*********锟杰ｏ拷选***小*********模式
***诓******锟絜ncoder_left******************锟�  encoder_right***锟揭憋拷************
******  值******
**************************************************************************/
void Choose(int encoder_left,int encoder_right)
{
	static int count;
	if(Flag_Stop==0)
		count = 0;
	if((Flag_Stop==1)&&(encoder_left<10))		// 这个函数每次中断都会被执行，因此Flag赋值必须写全
	{
		count += myabs(encoder_right);
		if(count>6&&count<135)		// 普通模式
		{
			Flag_follow = 0;
			Flag_avoid = 0;
			Flag_swing = 0;
		}
		if(count>135&&count<270)	// 避障模式
		{
			Flag_avoid = 1;
			Flag_follow = 0;
			Flag_swing = 0;
		}
		if(count>270&&count<405)	// 跟随模式
		{
			Flag_avoid = 0;
			Flag_follow = 1;
			Flag_swing = 0;
		}
		if(count>405&&count<540)	// 摇摆模式
		{
			Flag_avoid = 0;
			Flag_follow = 0;
			Flag_swing = 1;
		}
		if(count>540)
			count = 0;
	}
}



