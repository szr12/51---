/**********************�۾���������С������************************
*  ƽ̨��HJ-C51 HJ-1G HL-1 HJ-3G HJ-C52  + Keil uVision4 + STC89C52 + AT89S52
*  ���ƣ�HL-1����С������������
*  ��˾���۾����ӿƼ����޹�˾
*  �Ա���http://shop37031453.taobao.com
*  ��վ��www.hjmcu.com    www.hlmcu.com
*  ��д?H
*  ���ڣ�2008-8-08
*  �޸����ݣ�������IO�ڷֱ�ӵ�P3.2 P3.3 �ڣ�ÿ��С������ο����ܲ�ͬ�����Լ������ο����Ը��ȶ��ߺ���
*   QQ : 398115088 121350852
*  ����:11.0592MHZ
*  ˵������ѿ�Դ�����ṩԴ���������������ֱ�ӵ��۾���̳����
*  ��̳��
*  Ӳ�����ã�Ҫ���Լ������������������ʵ��
*  ʹ��˵������������IO���Լ��öŰ������Ӹ���ģ�飬�����Լ��޸ĸ���ģ��IO��
*  ��Ҫ����۾���������С�����ģ�������ʵ��--��ֱ�ӵ��۾���վ����
*  ѧϰ���飺�뵽�۾�����ѧϰ��WWW.HJMCU.COM�������52�Ρ����㵽��Ŀ����ѧ51��Ƭ����C���Լ�����Ƶ�̡̳� BT����3.63G
   �ص���ʾ��������ֻ���ο������ṩ����֧�֣����Լ��о����ա�
   *  ͬʱҪ�Ӻ�LCD1602 ע�����W1�Աȶȣ��õ���USB����ʱ����һ�㣬�õ�ع���ʱ��Сһ��Աȶȡ�

   HL-1��HL-1����С�����̽��߷�������һ��������������ܲ����������ջ�С����

   J3
   IN1--�ӵ�--ʵ����ϵ�P1.2
   IN2--�ӵ�--ʵ����ϵ�P1.3
   EN1--�ӵ�--ʵ����ϵ�P1.4
   EN2--�ӵ�--ʵ����ϵ�P1.5
   IN3--�ӵ�--ʵ����ϵ�P1.6
   IN4--�ӵ�--ʵ����ϵ�P1.7
   J4
   IN5--�ӵ�--ʵ����ϵ�P2.1
   IN6--�ӵ�--ʵ����ϵ�P2.0
   J5
   OUT1--�ӵ�--ʵ����ϵ�P3.3
   OUT2--�ӵ�--ʵ����ϵ�P3.4
   OUT3--�ӵ�--ʵ����ϵ�P3.5
   OUT4--�ӵ�--ʵ����ϵ�P3.6

   ��Դ�ӷ�----��ע��˶ԣ�һ��Ҫ100%��ȷ��������ջ�HL-1ʵ���
   J17
   VCC--�ӵ�--ʵ����ϵ� TTL V ����
   GND--�ӵ�--ʵ����ϵ� TTL G ����
******************************************************************/
// ˵�������û��LCD1602Һ����ʾ����ͬѧ�����öŰ��P0.7���Žӵ�GND��Ч��һ����

#include <intrins.h>
#include "LCD1602display.h"
#include <AT89X52.H>
#include <HJ-4WD_PWM.H>

// #include "STC12C5A60S2_PWM.h"
#define TX P2_1
#define RX P2_0
sbit DU = P2 ^ 6;
sbit WE = P2 ^ 7;
#define Forward_L_DATA 190 // ��ǰ��������ֱ�ߵ�ʱ������������������������ʱ����100,100�����256����С0��0��ʱ��������256��ʱ�����
#define Forward_R_DATA 190 // ����С��ǰ����ʱ���е�����գ�˵���ұ����ת�ٹ��죬�ǿ���ȡһ��ֵ��һ�㣬����һ��ֵСһ�㣬���� 200  190
						   // ֱ�������Ϊ�����ϵ���ͬһ��������Ҳ��һ���ٶ�һ�µģ���Ҫ�Լ��ֶ�����

// sbit P4_0=0xc0;	//P4�ڵ�ַ

/*****����ԭͼ���߶���******/
sbit L293D_IN1 = P1 ^ 2;
sbit L293D_IN2 = P1 ^ 3;
sbit L293D_IN3 = P1 ^ 6;
sbit L293D_IN4 = P1 ^ 7;
sbit L293D_EN1 = P1 ^ 4;
sbit L293D_EN2 = P1 ^ 5;

sbit BUZZ = P2 ^ 3;

// void cmg88()//������ܣ�������
//{
// DU=1;
// P0=0X00;
// DU=0;
// }

void Delay400Ms(void); // ��ʱ400���뺯��

unsigned char code Range[] = "songzirui666"; // LCD1602��ʾ��ʽ
unsigned char code ASCII[13] = "0123456789.-M";
unsigned char code table[] = "Distance:000.0cm";
unsigned char code table1[] = "!!! Out of range";

unsigned char disbuff[4] = {0, 0, 0, 0}; // ���ڷֱ��ž����ֵ0.1mm��mm��cm��m��ֵ

void Count(void); // ������㺯��

// unsigned int  time=0;//���ڴ�Ŷ�ʱ��ʱ��ֵ
unsigned long S = 0; // ���ڴ�ž����ֵ
bit flag = 0;		 // ���������־λ

#define left 'C'
#define right 'D'
#define up 'A'
#define down 'B'
#define stop 'F'

char code str[] = "�յ�ָ���ǰ!\n";
char code str1[] = "�յ�ָ����!\n";
char code str2[] = "�յ�ָ�����!\n";
char code str3[] = "�յ�ָ�����!\n";
char code str4[] = "�յ�ָ�ֹͣ!\n";

bit flag_REC = 0;
bit flag1 = 0;
int flage2 = 0;

unsigned char i = 0;
unsigned char dat = 0;
unsigned char buff[5] = 0; // ���ջ����ֽ�
void Timer_Count(void);

/************************************************************************/
// �ַ������ͺ���
void send_str()
// �����ִ�
{
	unsigned char i = 0;
	while (str[i] != '\0')
	{
		SBUF = str[i];
		while (!TI)
			;	// �������ݴ���
		TI = 0; // ������ݴ��ͱ�־
		i++;	// ��һ���ַ�
	}
}

void send_str1()
// �����ִ�
{
	unsigned char i = 0;
	while (str1[i] != '\0')
	{
		SBUF = str1[i];
		while (!TI)
			;	// �������ݴ���
		TI = 0; // ������ݴ��ͱ�־
		i++;	// ��һ���ַ�
	}
}

void send_str2()
// �����ִ�
{
	unsigned char i = 0;
	while (str2[i] != '\0')
	{
		SBUF = str2[i];
		while (!TI)
			;	// �������ݴ���
		TI = 0; // ������ݴ��ͱ�־
		i++;	// ��һ���ַ�
	}
}

void send_str3()
// �����ִ�
{
	unsigned char i = 0;
	while (str3[i] != '\0')
	{
		SBUF = str3[i];
		while (!TI)
			;	// �������ݴ���
		TI = 0; // ������ݴ��ͱ�־
		i++;	// ��һ���ַ�
	}
}

void send_str4()
// �����ִ�
{
	unsigned char i = 0;
	while (str4[i] != '\0')
	{
		SBUF = str4[i];
		while (!TI)
			;	// �������ݴ���
		TI = 0; // ������ݴ��ͱ�־
		i++;	// ��һ���ַ�
	}
}

void sint() interrupt 4 // �жϽ���3���ֽ�
{

	if (RI) // �Ƿ�����ж�
	{
		RI = 0;
		dat = SBUF;
		if (dat == 'O' && (i == 0)) // �������ݵ�һ֡
		{
			buff[i] = dat;
			flag1 = 1; // ��ʼ��������
		}
		else if (flag1 == 1)
		{
			i++;
			buff[i] = dat;
			if (i >= 2)
			{
				i = 0;
				flag1 = 0;
				flag_REC = 1;
			} // ֹͣ����
		}
	}
}

//=========================================================================================================================
/********����������***************/
void Conut(void)
{
	time = TH1 * 256 + TL1;
	TH1 = 0;
	TL1 = 0;
	// 0.00034��/1000=0.34����  Ҳ����1us����0.34����
	// ���ǣ��������ڼ�����Ǵӳ��������䵽������յ�˫·�̣�
	// �������ǽ�����Ľ������2����ʵ�ʵ�·��
	S = time;	  // �����һ����ʱ���Ƕ���΢�롣
	S = S * 0.17; // ��ʱ���㵽�Ľ��Ϊ���ף������Ǿ�ȷ�����׵ĺ���λ�ˣ�������С����
				  // 0.17=3.4/2
	//=======================================
	if ((S >= 5000) ) // ����������Χ
	{BUZZ = 0; //
						Delay1ms(50);
						BUZZ = 1;
		flag = 0;
	}
	else
	{
		if (S == 0)
		{

			DisplayListChar(0, 1, table1);
		}

		else
		{

			disbuff[0] = S % 10;
			disbuff[1] = S / 10 % 10;
			disbuff[2] = S / 100 % 10;
			disbuff[3] = S / 1000;
			DisplayListChar(0, 1, table);
			DisplayOneChar(9, 1, ASCII[disbuff[3]]);
			DisplayOneChar(10, 1, ASCII[disbuff[2]]);
			DisplayOneChar(11, 1, ASCII[disbuff[1]]);
			DisplayOneChar(12, 1, ASCII[10]);
			DisplayOneChar(13, 1, ASCII[disbuff[0]]);
		}
	}
}

/********************************************************/
void zd0() interrupt 3 // T0�ж��������������,������෶Χ`
{
	if (flage2 == 2)
	{		
		
	}
}

/********�������ߵ�ƽ�����ȼ������***************/
void Timer_Count(void)
{
	TR1 = 1; // ��������
	while (RX)
		;	 // ��RXΪ1�������ȴ�
	TR1 = 0; // �رռ���
	Conut(); // ����
}
/********************************************************/
void StartModule() // ����ģ��
{

	TX = 1; // ����һ��ģ��
	Delay10us(2);
	TX = 0;
}
/********************************************************/

/*************������********************/
void main(void)
{
	unsigned int a;
	// cmg88();//�������
	Delay1ms(400); // �����ȴ�����LCM���빤��״̬
	LCMInit();	   // LCM��ʼ��
	Delay1ms(5);   // ��ʱƬ��

	DisplayListChar(0, 0, Range);
	DisplayListChar(0, 1, table);

	//===============================
	// PWM_ini();
	//===============================
	TMOD = 0x20;
	TH1 = 0xFd; // 11.0592M����9600������
	TL1 = 0xFd;
	SCON = 0x50;
	PCON = 0x00;
	TR1 = 1;
	ES = 1;
	EA = 1;
	//=================================

A:
	if (flage2 == 1)
	{			  // ��������һ��
		BUZZ = 0; //
		Delay1ms(50);
		BUZZ = 1;			// ��50ms��رշ�����
		TMOD  = 0x10; // ��T0Ϊ��ʽ1��GATE=1��
		EA = 1;				// �������ж�
		TH1 = 0;
		TL1 = 0;
		ET1 = 1; // ����T0�ж�

		TH0 = 0XFc; // 1ms��ʱ
		TL0 = 0X18;
		TR0 = 1;
		ET0 = 1;
		flage2 = 2;
		while (flage2 == 2)
		{

			if (Left_1_led == 0 && Right_1_led == 0)
			{
				run(); // ����ǰ������
			}
			if (Right_2_led == 0 && Left_2_led == 0) // �������ߴ�����ͬʱ��⵽����
			{
				stoprun(); // ���õ��ֹͣ����
				RX = 1;
				StartModule(); // ����ģ��

				for (a = 951; a > 0; a--)
				{

					if (RX == 1)
					{
						Timer_Count(); // �������ߵ�ƽ�����ȼ��㺯��
					}
				}
				delay(100);	
				
			}
			else
			{
				if (Left_1_led == 1 && Right_1_led == 0) // ��߼�⵽����
				{
					leftrun(); // ����С����ת  ����
					run();
				}

				if (Right_1_led == 1 && Left_1_led == 0) // �ұ߼�⵽����
				{
					rightrun();
					run();
				}
			}
		}
	}

	//=======================================================================================================================
	while (flage2 != 1)
	{

		if (flag_REC == 1)
		{
			flag_REC = 0;
			if (buff[0] == 'O' && buff[1] == 'N') // ��һ���ֽ�ΪO���ڶ����ֽ�ΪN���������ֽ�Ϊ������
				switch (buff[2])
				{
				case up: // ǰ��
					flage2 = 1;
					goto A;
					break;
				}
		}
	}
	// ���ź�Ϊ0  û���ź�Ϊ1
}
