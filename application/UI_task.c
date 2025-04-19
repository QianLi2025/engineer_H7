#include "UI_task.h"

char str1[]="hello";

uint8_t temp_id=0;

uint8_t is_inited;

void UI_INIT(void)
{
	//ËÑË÷ID
	//Çå¿ÕUI
	
	//³õ´Î»æÖÆ
	if(dianguan_cmd.robot_status.robot_id==RED_2)
	{
		temp_id=RED_2;
	}	
		if(dianguan_cmd.robot_status.robot_id==BLUE_2)
	{
		temp_id=BLUE_2;
	}	
	
}
	
	
void UI_TASK(void)
{
	if(temp_id==0)
	{
		UI_INIT();
	}
	else
	{
	 rf_ui_write_string(&dianguan_cmd, "sd1", str1,  strlen(str1),  15,  5,  250,  250,  1,  temp_id);
	 rf_ui_string_upgrade(&dianguan_cmd);
	}
	
}