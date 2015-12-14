/*
 * 3pi-US - demo code for the Pololu 3pi Robot
 * 
 * http://www.pololu.com/docs/0J21
 * http://www.pololu.com
 * http://forum.pololu.com
 *
 */
#include <pololu/3pi.h>
#include <avr/pgmspace.h>

#include "bargraph.h"
const char welcome_line1[] PROGMEM = " UCH";
const char welcome_line2[] PROGMEM = "3\xf7 Robot";
const char demo_name_line1[] PROGMEM = "Line";
const char demo_name_line2[] PROGMEM = "Follow";
const char welcome[] PROGMEM = ">g32>>c32";
const char go[] PROGMEM = "L16 cdegreg4";

void initialize()
{
	unsigned int sensors[5]; // an array to hold sensor values
	pololu_3pi_init(2000);
	load_custom_characters(); // load the custom characters
	print_from_program_space(welcome_line1);
	lcd_goto_xy(0,1);
	print_from_program_space(welcome_line2);
	play_from_program_space(welcome);
	delay_ms(2000);

	clear();
	print_from_program_space(demo_name_line1);
	lcd_goto_xy(0,1);
	print_from_program_space(demo_name_line2);
	delay_ms(2000);
	set_digital_input(IO_D0, PULL_UP_ENABLED);	// enable internal pull-up on PD0
	while(!button_is_pressed(BUTTON_B))
	{
		int bat = read_battery_millivolts();
		clear();
		print_long(bat);
		print("mV");
		lcd_goto_xy(0,1);
		print("Press B");
		delay_ms(100);
	}
	wait_for_button_release(BUTTON_B);
	delay_ms(1000);

	while(!button_is_pressed(BUTTON_B))
	{
		unsigned int position = read_line_white(sensors,IR_EMITTERS_ON);
		clear();
		print_long(position);
		lcd_goto_xy(0,1);
		display_readings(sensors);

		delay_ms(100);
	}
	wait_for_button_release(BUTTON_B);

	clear();

	print("Go!");		

	play_from_program_space(go);
	while(is_playing());
}

int main()
{
	initialize();
	follow_segment();
	while(1);
}

// Local Variables: **
// mode: C **
// c-basic-offset: 4 **
// tab-width: 4 **
// indent-tabs-mode: t **
// end: **
