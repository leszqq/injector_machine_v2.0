/*
 * menu.c
 *
 *  Created on: 23 mar 2021
 *      Author: Administrator
 */
/* === private includes === */

#include "buttons.h"
#include "lcd_by_expander.h"
#include "check_macros.h"
#include "menu.h"
#include <assert.h>
#include <stdio.h>

/* === private defines === */

/* coordinates of values on LCD display */
#define LCD_OPEN_TIME_ROW       0
#define LCD_OPEN_TIME_COL      17
#define LCD_INJECTION_TIME_ROW  1
#define LCD_INJECTION_TIME_COL  17
#define LCD_COOLING_TIME_ROW    2
#define LCD_COOLING_TIME_COL    17

#define LCD_ROW_LEN            21

#define UPDATE_PERIOD           20          // period of reading buttons and updating displayed values in ms
#define NUM_BUTTONS             3

/* button indices in base.buttons[] tab  */
#define ENCODER_BUTTON          0
#define CNT_RES_BUTTON          1
#define SAVE_BUTTON             2

#define TIME_STEP               100
#define MAX_TIME_VALUE          40000
#define MAX_MAX_CYCLE_CNT_VALUE 9999

/* === private macros === */
/* format value in miliseconds to print as seconds */
#define FORMAT_TIME(TIME) (TIME > 10000 ? "%d.%d" : " %d.%d")

/* === private types === */

/* user menu entries */
enum menu_entry {
    NO_ENTRY,

    OPEN_TIME,
    INJECTION_TIME,
    COOLING_TIME,

    CYCLE_CNT_THOUSANDS,
    CYCLE_CNT_HUNDREDS,
    CYCLE_CNT_TENS,
    CYCLE_CNT_UNITS,
};


/* user menu */
static struct Menu{
    button_id                   buttons[NUM_BUTTONS];
    struct Time_table           *cycle_times;
    struct Status_tab           *status_tab;
    enum fsm_state              *machine_state;
    struct Counter              *counter;
    uint32_t                    timestamp;

    char                        aux_text[LCD_ROW_LEN];              // auxiliary char buffer
    enum menu_entry             entry;                              // selected menu entry
    bool                        setups_saved;                       // true if setups saved in EEPROM are consistent with these on display
} base;

/* === private functions === */

static enum GPIO_expander_status print_static_lcd_txt()
{
    enum GPIO_expander_status status;

    snprintf(base.aux_text, LCD_ROW_LEN, "cz. otwarcia:    %d.%d", base.cycle_times->open_time / 1000, base.cycle_times->open_time % 1000);
    status = Lcd_write(base.aux_text, LCD_OPEN_TIME_ROW, 0);
    CHECK(status == GPIO_EXPANDER_OK);

    snprintf(base.aux_text, LCD_ROW_LEN, "cz. wtrysku:     %d.%d", base.cycle_times->injection_time / 1000, base.cycle_times->injection_time % 1000);
    status = Lcd_write(base.aux_text, LCD_INJECTION_TIME_ROW, 0);
    CHECK(status == GPIO_EXPANDER_OK);

    snprintf(base.aux_text, LCD_ROW_LEN, "cz. chlodzenia:  %d.%d", base.cycle_times->cooling_time / 1000, base.cycle_times->cooling_time % 1000);
    status = Lcd_write(base.aux_text, LCD_COOLING_TIME_ROW, 0);
    CHECK(status == GPIO_EXPANDER_OK);

    return status;

    error:
    return status;
}

static uint16_t limit_val(uint16_t val, uint16_t low_limit, uint16_t high_val)
{
    if(val < 0) return 0;
    if(val > high_val) return high_val;
    return val;
}
/* === API functions === */

void Menu_init(struct Time_table *time_tab, struct Status_tab *status_tab, enum fsm_state *machine_state, struct Counter *counter)
{
    enum sev_seg_status temp_sev_seg_status = SEV_SEG_OK;
    enum GPIO_expander_status temp_exp_status = GPIO_EXPANDER_OK;

    /* init base struct */
    assert(time_tab != NULL);
    assert(status_tab != NULL);
    assert(machine_state != NULL);
    assert(counter != NULL);

    base.cycle_times = time_tab;
    base.status_tab = status_tab;
    base.machine_state = machine_state;
    base.counter = counter;
    base.entry = NO_ENTRY;
    base.setups_saved = true;

    /* initialize buttons */
    base.buttons[ENCODER_BUTTON] = Button_init(CP_ENC_BUTTON_GPIO_Port, CP_ENC_BUTTON_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);
    base.buttons[CNT_RES_BUTTON] = Button_init(CP_CNT_RESET_GPIO_Port, CP_CNT_RESET_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);
    base.buttons[SAVE_BUTTON] = Button_init(CP_SAVE_SETUPS_GPIO_Port, CP_SAVE_SETUPS_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);

    /* print static part of menu text on lcd */

    temp_exp_status = print_static_lcd_txt();
    CHECK(temp_exp_status == GPIO_EXPANDER_OK);

    /* display data on seven segment display */

    temp_sev_seg_status = Sev_seg_write(0);
    CHECK(temp_sev_seg_status == SEV_SEG_OK);

    /* temp code */
    base.timestamp = HAL_GetTick();
    return;

    error:
    if(temp_exp_status != GPIO_EXPANDER_OK) base.status_tab->GPIO_exp_stat = temp_exp_status;
    if(temp_sev_seg_status != SEV_SEG_OK) base.status_tab->sev_seg_stat = temp_sev_seg_status;
        return;
}


void Menu_process()
{
    Buttons_process();

    /* Every UPDATE_PERIOD, handle user interface */
    if(HAL_GetTick() > base.timestamp + UPDATE_PERIOD){
        base.timestamp = HAL_GetTick();

        /* encoder button. Handle order of menu entries depending on if the setups are saved or no. */
        if(Button_been_push(base.buttons[ENCODER_BUTTON])){
            if(base.entry == NO_ENTRY){
                /* display maximum cycle count on seven segment display when entering menu */
                Sev_seg_write(base.counter->max_value);
                base.entry = OPEN_TIME;
            } else if(base.entry == CYCLE_CNT_UNITS) {
                /* After last menu entry, continue iterating through options and display maximum cycle count if setups not saved. If saved, exit setup menu and display actual cycle count. */
                if(base.setups_saved){
                    base.entry = NO_ENTRY;
                    Sev_seg_write(base.counter->value);
                } else {
                    base.entry = OPEN_TIME;
                }
            } else {
                /* Iterate through menu entries on encoder button push */
                base.entry++;
            }

            /* Make blinking fields at current menu entry */
            switch (base.entry) {
                case NO_ENTRY:
                    Sev_seg_blink(NONE);
                    break;
                case OPEN_TIME:
                    Sev_seg_blink(NONE);
                    Lcd_blink_on(LCD_OPEN_TIME_ROW, 1);
                    break;
                case INJECTION_TIME:
                    Lcd_blink_on(LCD_INJECTION_TIME_ROW, 1);
                    break;
                case COOLING_TIME:
                    Lcd_blink_on(LCD_COOLING_TIME_ROW, 1);
                    break;
                case CYCLE_CNT_THOUSANDS:
                    Lcd_blink_off();
                    Sev_seg_blink(THOUSANDS);
                    break;
                case CYCLE_CNT_HUNDREDS:
                    Sev_seg_blink(HUNDREDS);
                    break;
                case CYCLE_CNT_TENS:
                    Sev_seg_blink(TENS);
                    break;
                case CYCLE_CNT_UNITS:
                    Sev_seg_blink(UNITS);
                    break;
                default:
                    assert(0);
            }
        }

        /* cnt reset button. Reset number of counted cycles to zero.*/
        if(Button_been_push(base.buttons[CNT_RES_BUTTON])){
            base.counter->value = 0;
            Sev_seg_write(0);
        }

        /* setup save button. Save setups value to EEPROM */
        if(Button_been_push(base.buttons[SAVE_BUTTON]) && !base.setups_saved){
            base.setups_saved = true;
            base.status_tab->eeprom_stat = EEPROM_write_setups(base.cycle_times, base.counter->max_value);
        }

        /* modify cycle times and cycle counter maximum value by encoder */

        uint8_t enc_transitions = Encoder_get_transitions();
        uint8_t new_val;
        if(enc_transitions != 0){
            base.setups_saved = false;
            if(base.entry == OPEN_TIME || base.entry == INJECTION_TIME || base.entry == COOLING_TIME){

            }

            switch(base.entry){
            case OPEN_TIME:
                base.cycle_times->open_time = limit_val( enc_transitions + base.cycle_times->open_time, 0, MAX_TIME_VALUE);
                /* lcd blink in here */
                Lcd_blink_on(LCD_OPEN_TIME_ROW, LCD_OPEN_TIME_COL-1);
            }
        }
//            base.setups_saved = false;

//        switch(base.entry){
//        case NONE:
//        default:
//            Sev_seg_blink(NONE);
//            break;
//        case OPEN_TIME:
//            Sev_seg_write(base.counter->max_value);
//            base.cycle_times->open_time = limit_val( enc_transitions + base.cycle_times->open_time, 0, MAX_TIME_VALUE);
//            snprintf(base.aux_text, 4, FORMAT_TIME(), base.cycle_times->open_time / 1000, base.cycle_times->open_time % 1000);
//            Lcd_write(base.aux_text, LCD_OPEN_TIME_ROW, LCD_OPEN_TIME_COL-1);
//            break;
//        case INJECTION_TIME:
//            base.cycle_times->injection_time = limit_val( enc_transitions + base.cycle_times->injection_time, 0, MAX_TIME_VALUE);
//            break;
//        case COOLING_TIME:
//            base.cycle_times->cooling_time = limit_val(enc_transitions + base.cycle_times->cooling_time, 0, MAX_TIME_VALUE);
//            break;
//        case CYCLE_CNT_THOUSANDS:
//            Sev_seg_write(base.counter->max_value);
//            Sev_seg_blink(THOUSANDS);
//            base.counter->max_value = limit_val(enc_transitions*1000 + base.counter->max_value, 0, MAX_MAX_CYCLE_CNT_VALUE);
//            break;
//        case CYCLE_CNT_HUNDREDS:
//            Sev_seg_blink(HUNDREDS);
//            base.counter-> max_value = limit_val(enc_transitions*100 + base.counter->max_value, 0, MAX_MAX_CYCLE_CNT_VALUE);
//            break;
//        case CYCLE_CNT_TENS:
//            Sev_seg_blink(TENS);
//            base.counter-> max_value = limit_val(enc_transitions*10 + base.counter->max_value, 0, MAX_MAX_CYCLE_CNT_VALUE);
//            break;
//        case CYCLE_CNT_UNITS:
//            Sev_seg_blink(UNITS);
//            base.counter-> max_value = limit_val(enc_transitions + base.counter->max_value, 0, MAX_MAX_CYCLE_CNT_VALUE);
//            break;
//        }

//        static uint16_t temp;
//        Sev_seg_write(temp++);
//        static char text[5];
//        snprintf(text, 5, "%d", temp);
//        Lcd_write(text, LCD_OPEN_TIME_ROW, LCD_OPEN_TIME_COL - 2);
    }
    Lcd_process();
    Sev_seg_process();



    // TODO check which button been push and do actions accordingly
}
