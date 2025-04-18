#include "UI_task.h"

char str1[]="hello";


void UI_INIT(void)
{
	//ËÑË÷ID
	//Çå¿ÕUI
	
	//³õ´Î»æÖÆ

}
	
	
void UI_TASK(void)
{
	 rf_ui_write_string(&dianguan_cmd,  str1,  strlen(str1),  15,  5,  250,  250,  1,  RED_2);
	 rf_ui_upgrade(&dianguan_cmd);
	
}