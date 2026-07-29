#ifndef CC_CAR_MENU_H
#define CC_CAR_MENU_H

#include <stdint.h>

/*
 * This is the only menu-facing interface used by cpu0_main.c.
 * Input handling, parameter application and screen rendering stay inside
 * the menu module so the vehicle loop does not depend on UI details.
 */
void car_menu_init(void);
void car_menu_frame_accepted(uint32_t frame_period_ms);
void car_menu_result_accepted(void);
void car_menu_task(void);
void car_menu_display(void);

#endif
