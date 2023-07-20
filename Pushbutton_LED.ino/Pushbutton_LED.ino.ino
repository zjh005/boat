#include <Servo.h> 

#define SERVOPINH  5 //水平舵机
#define SERVOPINV  6 //垂直舵机

#define DTIME   50  //延时参数，数值越小相应速度越快，反之相应慢   单位毫秒 一般取值（10~100） 
#define TOL   50     //照度的相应范围，越小越敏感，反之迟缓  （取值10~100 根据环境光强度不同敏感度也不同，室内光源变化幅度大，阳光下变化小）
/*以上2参数太小，会对光线的细微变化极其敏感，进而产生抖动，
  为消除抖动，可以使用滤波处理，或者调节参数，减慢反应时间或敏感度来应对。 */

          
// 水平舵机的设置
Servo horizontal; // 水平舵机
int servoh = 90;   // 默认角度

int servohLimitHigh = 145;  //左右角度
int servohLimitLow = 5;     //左右角度

// 垂直舵机的设置
Servo vertical;     // 垂直舵机
int servov = 90;     // 默认角度

int servovLimitHigh = 80;  //
int servovLimitLow = 1 ;    //最大仰角 不易过大，传感器可能顶住机架


//
// 4个传感器的接线口   
const int ldrlt = A0; //左上 
const int ldrrt = A1; //右上
const int ldrld = A2; //左下
const int ldrrd = A3; //右下

Servo upservo ;                 //建立两个舵机对象：投喂系统上、下
#include<SoftwareSerial.h>               //蓝牙连接的头文件
SoftwareSerial BT(10,11);                //RX接D11,TX接D10 
String BtOrder = "";                     //接收蓝牙指令
char serialData;
int pos=0,num=0;

void setup()
{
  Serial.begin(9600);
  Serial.println("Serial is ready");
  horizontal.attach(SERVOPINH); 
  vertical.attach(SERVOPINV);
  horizontal.write(servoh);
  vertical.write(servov);
  delay(100);

  //测试运行情况
  //测试垂直轴的运行情况，注意检测一下是否有卡住（或者导线缠绕）的情况。
//  for(int i=servovLimitLow;i<servovLimitHigh;i+=10)
//  {  vertical.write(i);
//     delay(150);
//  }
  vertical.write((servovLimitLow + servovLimitHigh)/2);
  delay(100);
//  //测试水平
//  for(int i=0;i<180;i+=10)
//   {  horizontal.write(i);
//     delay(150);
//   }
  BT.begin(9600);  
  BT.println("Bluetooch is ready");  
  upservo.attach(3);               //开关投喂舵机接引脚5，代号为’U‘
  Serial.print("Please input serial date");

}

void loop() 
{
  int lt = analogRead(ldrlt); //左上 
  int rt = analogRead(ldrrt); //右上
  int ld = analogRead(ldrld); //左下
  int rd = analogRead(ldrrd); //分别读取4个传感器的照度值
  

        
  int avt = (lt + rt) / 2; // 
  int avd = (ld + rd) / 2; // 
  int avl = (lt + ld) / 2; // 
  int avr = (rt + rd) / 2; //将邻近的读数平均
  
  int dvert = avt - avd; // 
  int dhoriz = avl - avr;//再计算上下行和左右排平均值的平均值

  Serial.print(lt);
  Serial.print(",");
  Serial.print(rt);
  Serial.print(",");
  Serial.print(ld);
  Serial.print(",");
  Serial.print(rd);
  Serial.print ("      |    ");    
  
  Serial.print(avt);
  Serial.print(",");
  Serial.print(avd);
  Serial.print(",");
  Serial.print(avl);
  Serial.print(",");
  Serial.print(avr);
  Serial.print(",   ");
  Serial.print(DTIME);
  Serial.print(",   ");
  //Serial.println(TOL);    //串口输出读数值，调试正常后，可以删除此段代码，可以提高相应速度。
  
  
//检查差异是否在公差范围内，否则改变垂直角度    
  if (-1*TOL > dvert || dvert > TOL){
    if (avt < avd){
      servov = ++servov;
      if (servov > servovLimitHigh){ 
        servov = servovLimitHigh;
      }
    }
    else if (avt > avd){
      servov= --servov;
      if (servov < servovLimitLow){
      servov = servovLimitLow;
      }
    }
    vertical.write(180- servov); //舵机旋转角度和光线相反的话 用(180- servov)或 (servov) 调换方向即可
  }

  //检查差异是否在公差范围内，否则改变水平角度  
  if (-1*TOL > dhoriz || dhoriz > TOL) {
    if (avl < avr){
      servoh = --servoh;
      if (servoh < servohLimitLow){
        servoh = servohLimitLow;
      }
    }
    else if (avl > avr){
      servoh = ++servoh;
      if (servoh > servohLimitHigh){
       servoh = servohLimitHigh;
       }
    }
    else if (avl = avr){}// nothing
    horizontal.write(180- servoh); //舵机旋转角度和光线相反的话 用(180- servoh) 或 (servoh) 调换方向即可
  }
  delay(DTIME);
  delay(1000);
  BT.print("测试计数");
  BT.println(num);                 //蓝牙发出信息
  Serial.print("测试计数");
  Serial.println(num);             //测试串口接收信息
  num+=1;
    if(BT.available()>0){           //检查串口缓存是否有数据等待传输
      serialData = (char)BT.read(); 
      if(serialData == '1')
      {
         upservo.write(180);           //上舵机旋转九十度打开投喂口
      }else{
         upservo.write(0);           //返回初始状态
      }
    }
}