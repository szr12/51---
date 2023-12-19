/*****************HJ-4WD�۾�������������С��PWMר��ͷ�ļ�*****************
 *  ƽ̨��HJ-4WD/HJ-E/HL-1����С�� + Keil U4 + STC89C52
 *  ���ƣ�
 *  ��˾���۾����ӿƼ����޹�˾
 *  �Ա���http://shop37031453.taobao.com
 *  ��վ��www.hjmcu.com	����С��ר����վ��WWW.HLMCU.COM
 *  ��д���۾�
 *  ���ڣ�2008-8-08
 *  ����:���ܳ�QQ:1526248688  ������QQ:398115088
 *  ����:11.0592MHZ
 *  ˵������ѿ�Դ�����ṩԴ���������������ֱ�ӵ��۾���̳����
 *  ��̳��http://hjdz.haotui.com
 *  ��ѽ�Ե��5�Ǻ�������18�����ϰ���׷�����ۺ��ټ���20G���ϣ�������������
 *  ��Ƶ�̳̣���С��������ϸ��Ƶ�̳̣�����Ҫ�뵽�۾���վ�������
 ******************************************************************/
#ifndef _LED_H_
#define _LED_H_

// ����С������ģ������IO��
sbit IN1 = P1 ^ 2;
sbit IN2 = P1 ^ 3;
sbit IN3 = P1 ^ 6;
sbit IN4 = P1 ^ 7;
sbit EN1 = P1 ^ 4;
sbit EN2 = P1 ^ 5;

/***���������߶���*****/

#define Left_1_led P3_4	 // �ڰ��ߴ�����
#define Right_1_led P3_3 // �ڰ��ߴ�����
#define Left_2_led P3_5	 // ���ϴ�����
#define Right_2_led P3_6 // ���ϴ�����

#define Left_moto_pwm P1_5	// PWM�źŶ�
#define Right_moto_pwm P1_4 // PWM�źŶ�

#define Left_moto_go        \
	{                       \
		P1_2 = 0, P1_3 = 1; \
	} // ������ǰ��
#define Left_moto_back      \
	{                       \
		P1_2 = 1, P1_3 = 0; \
	} // ��ߵ�����ת
#define Left_moto_Stop \
	{                  \
		P1_5 = 0;      \
	} // ��ߵ��ͣת
#define Right_moto_go       \
	{                       \
		P1_6 = 1, P1_7 = 0; \
	} // �ұߵ����ǰ��
#define Right_moto_back     \
	{                       \
		P1_6 = 0, P1_7 = 1; \
	} // �ұߵ�������
#define Right_moto_Stop \
	{                   \
		P1_4 = 0;       \
	} // �ұߵ��ͣת

unsigned char pwm_val_left = 0;	 // ��������
unsigned char push_val_left = 0; // ����ռ�ձ�N/20
unsigned char pwm_val_right = 0;
unsigned char push_val_right = 0; // �ҵ��ռ�ձ�N/20
bit Right_moto_stop = 1;
bit Left_moto_stop = 1;
unsigned int time = 0;

/************************************************************************/
// ��ʱ����
void delay(unsigned int k)
{
	unsigned int x, y;
	for (x = 0; x < k; x++)
		for (y = 0; y < 2000; y++)
			;
}
/************************************************************************/
// ǰ��ǰ��
void run(void)
{
	push_val_left = 8; // �ٶȵ��ڱ��� 0-20������0��С��20���
	push_val_right = 8;
	Left_moto_go;  // ������ǰ��
	Right_moto_go; // �ҵ����ǰ��
}

// ���˺��� ����������ף��뿴�۾�����51����С����Ƶ�̳�
void backrun(void)
{
	push_val_left = 8; // �ٶȵ��ڱ��� 0-20������0��С��20���
	push_val_right = 8;
	Left_moto_back;	 // ����������
	Right_moto_back; // �ҵ��������
}

// ��ת
void leftrun(void)
{
	push_val_left =15;
	push_val_right = 15;
	Right_moto_go;	// �ҵ����ǰ��
	Left_moto_back; // ����ֹͣ
}

// ��ת
void rightrun(void)
{
	push_val_left = 13;
	push_val_right = 13;
	Left_moto_go;	 // ������ǰ��
	Right_moto_back; // �ҵ��ֹͣ
}

// С��ֹͣ����
void stoprun(void)
{
	IN1 = 0; // ����ֹͣ
	IN2 = 0;
	IN3 = 0; // �ҵ��ֹͣ
	IN4 = 0;
}

/************************************************************************/
/*                    PWM���Ƶ��ת��                                   */
/************************************************************************/
/*                    ��������                                        */
/*����push_val_left��ֵ�ı���ת��,ռ�ձ�            */
void pwm_out_left_moto(void)
{
	if (Left_moto_stop)
	{
		//push_val_left:����ռ�ձ�N/20���������ò�ͬ��ֵ������ռ�ձȣ��Ӷ��ı����ٶ�
		if (pwm_val_left <= push_val_left)
		{
			//pwm_val_left:��������ֵ
			Left_moto_pwm = 1;
		}
		else
		{
			Left_moto_pwm = 0;
		}
		if (pwm_val_left >= 20)
			pwm_val_left = 0;
	}
	else
	{
		Left_moto_pwm = 0;
	}
}
/******************************************************************/
/*                    �ҵ������                                  */
void pwm_out_right_moto(void)
{
	if (Right_moto_stop)
	{
		if (pwm_val_right <= push_val_right)
		{
			Right_moto_pwm = 1;
		}
		else
		{
			Right_moto_pwm = 0;
		}
		if (pwm_val_right >= 20)
			pwm_val_right = 0;
	}
	else
	{
		Right_moto_pwm = 0;
	}
}

/***************************************************/
///*TIMER0�жϷ����Ӻ�������PWM�ź�*/
void timer0() interrupt 1 using 2
{
	TH0 = 0XFc; // 1Ms��ʱ
	TL0 = 0X18;
	time++;
	pwm_val_left++;
	pwm_val_right++;
	pwm_out_left_moto();
	pwm_out_right_moto();
}

/*********************************************************************/

#endif