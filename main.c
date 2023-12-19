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
#define Forward_R_DATA 190 // ����С��ǰ����ʱ���е�����գ�˵���ұ�����ת�ٹ��죬�ǿ���ȡһ��ֵ��һ�㣬����һ��ֵСһ�㣬���� 200  190
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

//=========================================================================================================================
/********����������***************/
void Conut(void)
{
	time = TH1 * 256 + TL1;
	TH1 = 0;
	TL1 = 0;

	// ��ʱtime��ʱ�䵥λ�����ھ�����ٶȣ���Ӿ���Ϊ11.0592MHZʱ��
	// time��ֵΪ0.54us*time����λΪ΢��
	// ��ô1us�������߶�Զ�ľ����أ�1s=1000ms=1000000us
	//  340/1000000=0.00034��
	// 0.00034��/1000=0.34����  Ҳ����1us����0.34����
	// ���ǣ��������ڼ�����Ǵӳ��������䵽������յ�˫·�̣�
	// �������ǽ�����Ľ������2����ʵ�ʵ�·��

	S = time ; // �����һ����ʱ���Ƕ���΢�롣
	S = S * 0.17; // ��ʱ���㵽�Ľ��Ϊ���ף������Ǿ�ȷ�����׵ĺ���λ�ˣ�������С����

	//=======================================
	if ((S >= 5000) || flag == 1) // ����������Χ
	{
		flag = 0;
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

/********************************************************/
void zd0() interrupt 3 // T0�ж��������������,������෶Χ
{
	flag = 1; // �ж������־
	RX = 0;
}

/********�������ߵ�ƽ������ȼ������***************/
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
	unsigned char i;
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

	//=================================
B:
	for (i = 0; i < 50; i++) // �ж�K4�Ƿ���
	{
		Delay1ms(1);   // 1ms���ж�50�Σ����������һ�α��жϵ�K4(S4)û���£������¼��
		if (P3_7 != 0) // ��K4��S4������ʱ������С��
			goto B;	   // ��ת�����B�����¼��
	}
	// ��������һ��
	BUZZ = 0; // 50�μ��K4(S4)ȷ���ǰ���֮�󣬷������������Ρ����죬Ȼ������С����
	Delay1ms(50);
	BUZZ = 1;			// ��50ms��رշ�����
	TMOD = TMOD | 0x10; // ��T0Ϊ��ʽ1��GATE=1��
	EA = 1;				// �������ж�
	TH1 = 0;
	TL1 = 0;
	ET1 = 1; // ����T0�ж�

	TH0 = 0XFc; // 1ms��ʱ
	TL0 = 0X18;
	TR0 = 1;
	ET0 = 1;

	//=======================================================================================================================
	while (1)
	{

		// ���ź�Ϊ0  û���ź�Ϊ1

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
					Timer_Count(); // �������ߵ�ƽ������ȼ��㺯��
				}
			}
			delay(1000); 
			for (a = 951; a > 0; a--)
			{

				if (RX == 1)
				{
					Timer_Count(); // �������ߵ�ƽ������ȼ��㺯��
				}
			}
			delay(1000); 
			return;
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