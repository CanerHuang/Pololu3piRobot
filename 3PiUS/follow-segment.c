/*
 * follow_segment.c
 *
 * This file just contains the follow_segment() function, which causes
 * 3pi to follow a segment of the maze until it detects an
 * intersection, a dead end, or the finish.
 *
 */

#include <pololu/3pi.h>
#include <pololu/orangutan.h>
const unsigned char pulseInPins[] = { IO_D0 };
const unsigned char trigPins[] = { IO_D1 };
unsigned long echotime = 0;
unsigned char USenable = 1;   // ******************************* 103/10/14
struct PulseInputStruct pulse_info;   // ******************************* 103/10/14

/**
 *
 * Obstacle_Avoidance();
 * ���׻ٰƵ{��
 * �d�ҡG
 * Obstacle_Avoidance(�t��,���s�ɶ�,�u��ɶ�,����ɶ�);
 *
 */
void Obstacle_Avoidance(int max,int normal_true,int long_line,int short_line)
{
	set_motors(max,-max);
	delay_ms(normal_true);
	
	set_motors(max,max);
	delay_ms(long_line);

	set_motors(-max,max);
	delay_ms(normal_true);
	
	set_motors(max,max);
	delay_ms(short_line);
	
	set_motors(-max,max);
	delay_ms(normal_true);
	
	set_motors(max,max);
	delay_ms(long_line);

	
	set_motors(max,-max);
	delay_ms(normal_true);
}


/**
 *
 * Ultrasonic_Sensor(int &Sensor_Value);
 * �O�@�Ӱ򥻪��W���i���Ƶ{��
 * �i�H�z�L�I�s�o�ӰƵ{���Ө��o�W���i�ƭ�
 * �d�ҡG
 * Ultrasonic_Sensor(�P�����s���});
 *
 */

void Ultrasonic_Sensor(int *US_Sensor)
{
   // pulse_in_start(pulseInPins, 1);		// start measuring pulses on PD0    // ******************************* 103/10/14
//##########################  �W���i�ˬd  #############################

    if (USenable < 1)
	{
	   USenable = 1;
       pulse_in_start(pulseInPins, 1);		// start measuring pulses on PD0
	   unsigned long time = get_ticks();
	   // generate our servo pulse output on PD1.
	   // high 10 us at least, 1 tick = 0.4us
	   set_digital_output(IO_D1, HIGH);
	   while ((get_ticks() - time) <= 30UL)
		{
		//	wait for 12us;
		
		}

       set_digital_output(IO_D1,LOW );
	   echotime = get_ticks();
	}
	else
	{
	   if (pulse_to_microseconds((get_ticks() - echotime) ) > 30000)  // delay 30ms
    	{
	       USenable = 0;
		   get_pulse_info(0, &pulse_info);  // get pulse info for D0
		   if (pulse_to_microseconds(pulse_info.lastHighPulse) > 28000)
			{
				*US_Sensor = 255;
			}
			else
			{
	            int dis = pulse_to_microseconds(pulse_info.lastHighPulse)*34000/2000000;
				*US_Sensor = dis;
			}

    	}
	}

//##########################  �W���i�ˬd  #############################
}
/**
 * Reduce_Interference(unsigned long stime,int *US_Sensor_Value);
 * ��֤z�Z���Ƶ{��
 * �d�ҡG
 * Reduce_Interference(��֤z�Z�ɶ�,�P�����s���});
 */
 void Reduce_Interference(unsigned long stime,int *US_Sensor_Value)
 {
	unsigned long stoptime2 = get_ms() + stime;
	while(get_ms()<stoptime2)
	{
		set_motors(0,0);
		Ultrasonic_Sensor(&US_Sensor_Value);
	}
 }
/**
 * follow_line(int start_sleep,int max_sleep,unsigned long stime, double kp, double ki, double kd, unsigned char distance,unsigned long Reduce_Interference_Time);
 * �O�@�Ӱ򥻪��`�u���Ƶ{��
 * �d�ҡG
 * follow_line(�_�l�t��/�̤p�ƫ�,�̰��t��,�٨��ɶ�,��Ҷ�,�n����,�L����,�׻ٶZ��,��֤z�Z�ɶ�);
 */

void follow_line(int start_sleep,int max_sleep,unsigned long stime, int kp, int ki, double kd, unsigned char distance,unsigned long Reduce_Interference_Time)
{
	int US_Sensor_Value=0;
	int last_proportional = 0;
	long integral=0;
	clear();  // clear LCD     // ******************************* 103/10/14
	int max = start_sleep;
	unsigned long stoptime = get_ms() + stime;
	while(1)
	{
		// Normally, we will be following a line.  The code below is
		// similar to the 3pi-linefollower-pid example, but the maximum
		// speed is turned down to 60 for reliability.

		// Get the position of the line.
		if( get_ms() >= stoptime )
		{
			max = start_sleep;
		}
		else
		{
			max = max_sleep;
		}
		unsigned int sensors[5];
		unsigned int position = read_line_white(sensors,IR_EMITTERS_ON);

		// The "proportional" term should be 0 when we are on the line.
		int proportional = ((int)position) - 2000;

		// Compute the derivative (change) and integral (sum) of the
		// position.
		int derivative = proportional - last_proportional;
		integral += proportional;

		// Remember the last position.
		last_proportional = proportional;

		// Compute the difference between the two motor power settings,
		// m1 - m2.  If this is a positive number the robot will turn
		// to the left.  If it is a negative number, the robot will
		// turn to the right, and the magnitude of the number determines
		// the sharpness of the turn.
		int power_difference = proportional/kp + integral/ki + (double)derivative*kd;

		// Compute the actual motor settings.  We never set either motor
		// to a negative value.
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;
		
		if(power_difference < 0)
			set_motors(max+power_difference,max);
		else
			set_motors(max,max-power_difference);
			
    	Ultrasonic_Sensor(&US_Sensor_Value);
        if(US_Sensor_Value < distance && US_Sensor_Value != 0)
		{
			Reduce_Interference(Reduce_Interference_Time,&US_Sensor_Value);
			if(US_Sensor_Value < distance && US_Sensor_Value != 0)
			{
				//Obstacle_Avoidance(�t��,���s�ɶ�,�u��ɶ�,����ɶ�);
				Obstacle_Avoidance(30,280,600,750);//�׻ٰƵ{�������I
			}
			Reduce_Interference(Reduce_Interference_Time,&US_Sensor_Value);
		}
	}
}
void follow_segment()
{

	int US_Sensor_Value=0;


	while(1)
	{

		Ultrasonic_Sensor(&US_Sensor_Value);
		set_motors(50,50);//����
		if(US_Sensor_Value != 0 && US_Sensor_Value <30)//�p��30cm�}�l�׻�
		{
			//�׻ٵ{��
			set_motors(50,-50);
			delay_ms(500);
			set_motors(50,50);
			delay_ms(800);
			set_motors(-50,50);
			delay_ms(500);
			set_motors(50,-50);
			delay_ms(500);
		}
	}

}

// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
