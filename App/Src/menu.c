/*
 * menu.c
 *
 *  Created on: 23 mar 2021
 *      Author: Administrator
 */
/* === private includes === */
#include "time_table.h"
#include "buttons.h"
#include "lcd_by_expander.h"
#include "check_macros.h"
#include "menu.h"

/* === private defines === */

/* coordinates of values on LCD display */
#define LCD_OPEN_TIME_ROW       0
#define LCD_OPEN_TIME_COL      17
#define LCD_INJECTION_TIME_ROW  1
#define LCD_INJECTION_TIME_COL  17
#define LCD_COOLING_TIME_ROW    2
#define LCD_COOLING_TIME_COL    17

#define NUM_BUTTONS             3

/* button indices in base.buttons[] tab  */
#define ENCODER_BUTTON          0
#define CNT_RES_BUTTON          1
#define SAVE_BUTTON             2


/* === private types === */

/* user menu entries */
enum menu_entry {
    NONE,

    CYCLE_CNT_THOUSANDS,
    CYCLE_CNT_HUNDREDS,
    CYCLE_CNT_TENS,
    CYCLE_CNT_UNITS,

    OPEN_TIME,
    INJECTION_TIME,
    COOLING_TIME,
};


/* user menu */
static struct Menu{
    button_id                   buttons[NUM_BUTTONS];
    struct Encoder;
    struct Time_table           *cycle_times;
    struct Status_tab           *status_tab;
    enum menu_entry             entry;
} base;

/* === private functions === */

static enum GPIO_expander_status print_static_lcd_txt()
{
    enum GPIO_expander_status status;
    status = lcd_write("czas otwarcia:", LCD_OPEN_TIME_ROW, LCD_OPEN_TIME_COL);
    CHECK(status == GPIO_EXPANDER_OK);

    lcd_write("czas wtrysku:", LCD_INJECTION_TIME_ROW, LCD_INJECTION_TIME_COL);
    CHECK(status == GPIO_EXPANDER_OK);

    lcd_write("czas chlodzenia:", LCD_COOLING_TIME_ROW, LCD_COOLING_TIME_COL);
    CHECK(status == GPIO_EXPANDER_OK);

    error:
    return status;
}

/* === API functions === */

void menu_init(struct Time_table *time_tab, struct Status_tab *status_tab)
{
    /* init base struct */
    base.cycle_times = time_tab;
    base.status_tab = status_tab;

    /* initialize buttons */
    base.buttons[ENCODER_BUTTON] = Button_init(CP_ENC_BUTTON_GPIO_Port, CP_ENC_BUTTON_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);
    base.buttons[CNT_RES_BUTTON] = Button_init(CP_CNT_RESET_GPIO_Port, CP_CNT_RESET_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);
    base.buttons[SAVE_BUTTON] = Button_init(CP_SAVE_SETUPS_GPIO_Port, CP_SAVE_SETUPS_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);

    base.entry = NONE;
    /* initialize lcd display */

    /* print static part of menu text on lcd */
    enum GPIO_expander_status temp_exp_status;
    status = print_static_lcd_txt();
    CHECK(status == GPIO_EXPANDER_OK);


    // TODO print cycle times values

    return;

    error:
    if(temp_exp_status != GPIO_EXPANDER_OK) base.status_tab->GPIO_exp_stat = temp_exp_status;
    if(temp_)
        return;
}


void menu_process()
{
    Buttons_process();
    // TODO check which button been push and do actions accordingly
}
