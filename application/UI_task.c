#include "UI_task.h"

ROBOT_STATE_e ROBOT_STATE;//机器人状态

char str1[]="hello";

uint8_t temp_id=2;

uint8_t is_inited;

int height_test;
void UI_INIT(void)
{
	//搜索ID
	//清空UI
	
	//初次绘制
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
//	if(temp_id==0)
//	{
//		UI_INIT();
//	}
//	else
//	{
	 rf_ui_write_string(&dianguan_cmd, "sd1", str1,  strlen(str1),  15,  5,  250,  250,  1,  temp_id);
	 rf_ui_string_upgrade(&dianguan_cmd);
//	}
	
}


void UI_UPLOAD(void)//上传
{
	if(ROBOT_STATE==NORMAL){
	strcpy(ui_g_Ungroup_ROBOT_STATE->string, "NORMAL");}
	if(ROBOT_STATE==CUSTOM){
	strcpy(ui_g_Ungroup_ROBOT_STATE->string, "CUSTOM");}
	
	
	ui_g_Ungroup_REAL_HEIGHT->number =height;//高度真实
	
	
	ui_g_Ungroup_heigth_index->start_y =(uint32_t)(430+ height*(826-430)/(MAX_HEIGHT-0));//高度索引
	
	
	
	if(ARM_CMD_data.sucker_mode==SUCKER_OFF){
	strcpy(ui_g_Ungroup_SUCKER_STATE->string, "OFF");}
		if(ARM_CMD_data.sucker_mode==SUCKER_ON){
	strcpy(ui_g_Ungroup_SUCKER_STATE->string, "ON ");}
		
	
	
	  ui_g_Ungroup_rxy_index->start_x = (uint32_t)(105+ rc_mode_xy_after_check[1]*(463-113)/(-228-400));
    ui_g_Ungroup_rxy_index->start_y = (uint32_t)(610+ rc_mode_xy_after_check[0]*(860-610)/(500-0));
	

}
