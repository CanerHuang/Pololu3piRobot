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
void follow_segment()
{
	int last_proportional = 0;
	long integral=0;

	while(1)
	{
		// Normally, we will be following a line.  The code below is
		// similar to the 3pi-linefollower-pid example, but the maximum
		// speed is turned down to 60 for reliability.

		// Get the position of the line.
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
		int power_difference = proportional/20 + integral/10000 + derivative*3/2;

		// Compute the actual motor settings.  We never set either motor
		// to a negative value.
		const int max = 60; // the maximum speed
		if(power_difference > max)
			power_difference = max;
		if(power_difference < -max)
			power_difference = -max;
		
		if(power_difference < 0)
			set_motors(max+power_difference,max);
		else
			set_motors(max,max-power_difference);

//##########################  超音波檢查  #############################

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
				      clear();  // clear LCD
					  lcd_goto_xy(0, 0);
					  print("Too far");
				}
				else
				{
					lcd_goto_xy(0, 0);		// go to start of first LCD row
					// print the length (in us) of the most recently received high pulse
					print_unsigned_long(pulse_to_microseconds(pulse_info.lastHighPulse));
					print(" us      ");
					lcd_goto_xy(0, 1);		// go to start of second LCD row
					// print the length (in us) of the most recently received low pulse
		            int dis = pulse_to_microseconds(pulse_info.lastHighPulse)*34000/2000000;
					print_unsigned_long(dis);
					print(" cm      ");
				}

	    	}
		}

//##########################  超音波檢查  #############################

	}
}

// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
