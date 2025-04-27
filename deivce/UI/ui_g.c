//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 13
#define TOTAL_STRING 5

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
uint8_t ui_g_dirty_string[TOTAL_STRING];
#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

void ui_init_g() {
    ui_g_Ungroup_HEIGHT_LINE->figure_tpye = 0;
    ui_g_Ungroup_HEIGHT_LINE->layer = 0;
    ui_g_Ungroup_HEIGHT_LINE->start_x = 1343;
    ui_g_Ungroup_HEIGHT_LINE->start_y = 430;
    ui_g_Ungroup_HEIGHT_LINE->end_x = 1345;
    ui_g_Ungroup_HEIGHT_LINE->end_y = 844;
    ui_g_Ungroup_HEIGHT_LINE->color = 2;
    ui_g_Ungroup_HEIGHT_LINE->width = 3;

    ui_g_Ungroup_ARROW1->figure_tpye = 0;
    ui_g_Ungroup_ARROW1->layer = 0;
    ui_g_Ungroup_ARROW1->start_x = 1313;
    ui_g_Ungroup_ARROW1->start_y = 810;
    ui_g_Ungroup_ARROW1->end_x = 1347;
    ui_g_Ungroup_ARROW1->end_y = 844;
    ui_g_Ungroup_ARROW1->color = 2;
    ui_g_Ungroup_ARROW1->width = 3;

    ui_g_Ungroup_ARROW2->figure_tpye = 0;
    ui_g_Ungroup_ARROW2->layer = 0;
    ui_g_Ungroup_ARROW2->start_x = 1344;
    ui_g_Ungroup_ARROW2->start_y = 845;
    ui_g_Ungroup_ARROW2->end_x = 1378;
    ui_g_Ungroup_ARROW2->end_y = 813;
    ui_g_Ungroup_ARROW2->color = 2;
    ui_g_Ungroup_ARROW2->width = 3;

    ui_g_Ungroup_heigth_index->figure_tpye = 2;
    ui_g_Ungroup_heigth_index->layer = 0;
    ui_g_Ungroup_heigth_index->r = 10;
    ui_g_Ungroup_heigth_index->start_x = 1340;
    ui_g_Ungroup_heigth_index->start_y = 430;
    ui_g_Ungroup_heigth_index->color = 4;
    ui_g_Ungroup_heigth_index->width = 12;

    ui_g_Ungroup_REAL_HEIGHT->figure_tpye = 6;
    ui_g_Ungroup_REAL_HEIGHT->layer = 0;
    ui_g_Ungroup_REAL_HEIGHT->font_size = 20;
    ui_g_Ungroup_REAL_HEIGHT->start_x = 1018;
    ui_g_Ungroup_REAL_HEIGHT->start_y = 810;
    ui_g_Ungroup_REAL_HEIGHT->color = 0;
    ui_g_Ungroup_REAL_HEIGHT->number = 0;
    ui_g_Ungroup_REAL_HEIGHT->width = 2;

    ui_g_Ungroup_SILVER2_LOCATION->figure_tpye = 0;
    ui_g_Ungroup_SILVER2_LOCATION->layer = 0;
    ui_g_Ungroup_SILVER2_LOCATION->start_x = 889;
    ui_g_Ungroup_SILVER2_LOCATION->start_y = 435;
    ui_g_Ungroup_SILVER2_LOCATION->end_x = 1006;
    ui_g_Ungroup_SILVER2_LOCATION->end_y = 435;
    ui_g_Ungroup_SILVER2_LOCATION->color = 1;
    ui_g_Ungroup_SILVER2_LOCATION->width = 4;

    ui_g_Ungroup_NewRound->figure_tpye = 2;
    ui_g_Ungroup_NewRound->layer = 0;
    ui_g_Ungroup_NewRound->r = 22;
    ui_g_Ungroup_NewRound->start_x = 959;
    ui_g_Ungroup_NewRound->start_y = 536;
    ui_g_Ungroup_NewRound->color = 0;
    ui_g_Ungroup_NewRound->width = 3;

    ui_g_Ungroup_SOURCE_HEIGHT->figure_tpye = 0;
    ui_g_Ungroup_SOURCE_HEIGHT->layer = 0;
    ui_g_Ungroup_SOURCE_HEIGHT->start_x = 820;
    ui_g_Ungroup_SOURCE_HEIGHT->start_y = 663;
    ui_g_Ungroup_SOURCE_HEIGHT->end_x = 1097;
    ui_g_Ungroup_SOURCE_HEIGHT->end_y = 663;
    ui_g_Ungroup_SOURCE_HEIGHT->color = 0;
    ui_g_Ungroup_SOURCE_HEIGHT->width = 4;

    ui_g_Ungroup_BOUNDARY_rec->figure_tpye = 1;
    ui_g_Ungroup_BOUNDARY_rec->layer = 0;
    ui_g_Ungroup_BOUNDARY_rec->start_x = 106;
    ui_g_Ungroup_BOUNDARY_rec->start_y = 607;
    ui_g_Ungroup_BOUNDARY_rec->color = 1;
    ui_g_Ungroup_BOUNDARY_rec->width = 1;
    ui_g_Ungroup_BOUNDARY_rec->end_x = 466;
    ui_g_Ungroup_BOUNDARY_rec->end_y = 863;

    ui_g_Ungroup_rxy_index->figure_tpye = 2;
    ui_g_Ungroup_rxy_index->layer = 0;
    ui_g_Ungroup_rxy_index->r = 7;
    ui_g_Ungroup_rxy_index->start_x = 286;
    ui_g_Ungroup_rxy_index->start_y = 622;
    ui_g_Ungroup_rxy_index->color = 6;
    ui_g_Ungroup_rxy_index->width = 10;

    ui_g_Ungroup_SOURCELine1->figure_tpye = 0;
    ui_g_Ungroup_SOURCELine1->layer = 0;
    ui_g_Ungroup_SOURCELine1->start_x = 850;
    ui_g_Ungroup_SOURCELine1->start_y = 650;
    ui_g_Ungroup_SOURCELine1->end_x = 850;
    ui_g_Ungroup_SOURCELine1->end_y = 731;
    ui_g_Ungroup_SOURCELine1->color = 0;
    ui_g_Ungroup_SOURCELine1->width = 3;

    ui_g_Ungroup_SOURCELine2->figure_tpye = 0;
    ui_g_Ungroup_SOURCELine2->layer = 0;
    ui_g_Ungroup_SOURCELine2->start_x = 1055;
    ui_g_Ungroup_SOURCELine2->start_y = 654;
    ui_g_Ungroup_SOURCELine2->end_x = 1055;
    ui_g_Ungroup_SOURCELine2->end_y = 735;
    ui_g_Ungroup_SOURCELine2->color = 0;
    ui_g_Ungroup_SOURCELine2->width = 3;

    ui_g_Ungroup_SILVERLine->figure_tpye = 0;
    ui_g_Ungroup_SILVERLine->layer = 0;
    ui_g_Ungroup_SILVERLine->start_x = 969;
    ui_g_Ungroup_SILVERLine->start_y = 478;
    ui_g_Ungroup_SILVERLine->end_x = 973;
    ui_g_Ungroup_SILVERLine->end_y = 572;
    ui_g_Ungroup_SILVERLine->color = 6;
    ui_g_Ungroup_SILVERLine->width = 1;

    ui_g_Ungroup_SUCKER->figure_tpye = 7;
    ui_g_Ungroup_SUCKER->layer = 0;
    ui_g_Ungroup_SUCKER->font_size = 20;
    ui_g_Ungroup_SUCKER->start_x = 837;
    ui_g_Ungroup_SUCKER->start_y = 902;
    ui_g_Ungroup_SUCKER->color = 3;
    ui_g_Ungroup_SUCKER->str_length = 6;
    ui_g_Ungroup_SUCKER->width = 2;
    strcpy(ui_g_Ungroup_SUCKER->string, "SUCKER");

    ui_g_Ungroup_SUCKER_STATE->figure_tpye = 7;
    ui_g_Ungroup_SUCKER_STATE->layer = 0;
    ui_g_Ungroup_SUCKER_STATE->font_size = 20;
    ui_g_Ungroup_SUCKER_STATE->start_x = 1018;
    ui_g_Ungroup_SUCKER_STATE->start_y = 902;
    ui_g_Ungroup_SUCKER_STATE->color = 3;
    ui_g_Ungroup_SUCKER_STATE->str_length = 3;
    ui_g_Ungroup_SUCKER_STATE->width = 2;
    strcpy(ui_g_Ungroup_SUCKER_STATE->string, "OFF");

    ui_g_Ungroup_ROBOT->figure_tpye = 7;
    ui_g_Ungroup_ROBOT->layer = 0;
    ui_g_Ungroup_ROBOT->font_size = 20;
    ui_g_Ungroup_ROBOT->start_x = 743;
    ui_g_Ungroup_ROBOT->start_y = 855;
    ui_g_Ungroup_ROBOT->color = 3;
    ui_g_Ungroup_ROBOT->str_length = 11;
    ui_g_Ungroup_ROBOT->width = 2;
    strcpy(ui_g_Ungroup_ROBOT->string, "ROBOT_STATE");

    ui_g_Ungroup_ROBOT_STATE->figure_tpye = 7;
    ui_g_Ungroup_ROBOT_STATE->layer = 0;
    ui_g_Ungroup_ROBOT_STATE->font_size = 20;
    ui_g_Ungroup_ROBOT_STATE->start_x = 1016;
    ui_g_Ungroup_ROBOT_STATE->start_y = 857;
    ui_g_Ungroup_ROBOT_STATE->color = 3;
    ui_g_Ungroup_ROBOT_STATE->str_length = 6;
    ui_g_Ungroup_ROBOT_STATE->width = 2;
    strcpy(ui_g_Ungroup_ROBOT_STATE->string, "NORMAL");

    ui_g_Ungroup_HEIGHT_NAME->figure_tpye = 7;
    ui_g_Ungroup_HEIGHT_NAME->layer = 0;
    ui_g_Ungroup_HEIGHT_NAME->font_size = 20;
    ui_g_Ungroup_HEIGHT_NAME->start_x = 842;
    ui_g_Ungroup_HEIGHT_NAME->start_y = 812;
    ui_g_Ungroup_HEIGHT_NAME->color = 0;
    ui_g_Ungroup_HEIGHT_NAME->str_length = 6;
    ui_g_Ungroup_HEIGHT_NAME->width = 2;
    strcpy(ui_g_Ungroup_HEIGHT_NAME->string, "HEIGHT");


    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_tpyel = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_tpyel = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING);

    for (int i = 0; i < TOTAL_FIGURE; i++) {
        ui_g_now_figures[i].operate_tpyel = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        ui_g_now_strings[i].operate_tpyel = 2;
    }
}

void ui_update_g() {
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++) {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0) {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++) {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0) {
            ui_g_dirty_string[i] = 1;
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING);
}
