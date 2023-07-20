#include "dht11.h"
/*
 * DHT11驱动
 */
//复位DHT11
void Dht11_DATA_OUT(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  /*Configure GPIO pin : PB12 */
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

// 输入input
void Dht11_DATA_IN(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct = {0};

	  /*Configure GPIO pin : PB12 */
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Delay_us(uint32_t us)
{
    uint32_t delay = (HAL_RCC_GetHCLKFreq() / 4000000 * us);
    while (delay--)
	{
		;
	}
}
void DHT11_Rst(void)
{
	Dht11_DATA_OUT();                                          //设置为输出
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);     //拉低引脚
	HAL_Delay(20);                                             //延迟20ms
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);       //拉高引脚
	Delay_us(30);                                              //延迟30us
}

//等待DHT11回应
//返回值：1代表未检测到存在，0代表检测到存在
uint8_t DHT11_Check(void)
{
	uint8_t retry=0;
	Dht11_DATA_IN();                                           //设置为输入
	while(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) && retry<100)
	{
		retry++;
		Delay_us(1);
	}
	if(retry>=100)
		return 1;
	else
		retry=0;

	while(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) && retry<100)
	{
		retry++;
		Delay_us(1);
	}
	if(retry>=100)
		return 1;
	return 0;
}

//从DHT11读取一个位
//返回值：1/0
uint8_t DHT11_Read_Bit(void)
{
	uint8_t retry=0;
	while(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) && retry<100)
	{
		retry++;
		Delay_us(1);
	}
	retry=0;

	while(GPIO_PIN_RESET==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) && retry<100)
	{
		retry++;
		Delay_us(1);
	}
	Delay_us(40);

	if(GPIO_PIN_SET==HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
		return 1;
	else
		return 0;
}

//从DHT11读取一个字节
//返回值：读取到的字节数据(8位)
uint8_t DHT11_Read_Byte(void)
{
	uint8_t dat=0;
	for(uint8_t i=0;i<8;i++)
	{
		dat <<= 1;
		dat |= DHT11_Read_Bit();
	}
	return dat;
}

//从DHT11读取一次数据
//humi：湿度值，temp：温度值
//返回值：0代表正常，1代表读取失败
uint8_t DHT11_Read_Data(uint8_t* humi,uint8_t* temp)
{
	uint8_t buf[5];
	DHT11_Rst();
	if(DHT11_Check() == 0)
	{
		for(uint8_t i=0;i<5;i++)
			buf[i]=DHT11_Read_Byte();
		if((buf[0]+buf[1]+buf[2]+buf[3])==buf[4])
		{
			*humi=buf[0];       //这里省略小数部分
			*temp=buf[2];
		}
	}
	else
		return 1;
	return 0;
}
