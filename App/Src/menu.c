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
#define LCD_OPEN_TIME_COL       16
#define LCD_INJECTION_TIME_ROW  1
#define LCD_INJECTION_TIME_COL  16
#define LCD_COOLING_TIME_ROW    2
#define LCD_COOLING_TIME_COL    16

#define LCD_ROW_LEN            21

#define UPDATE_PERIOD           20          // period of reading buttons and updating displayed values in ms
#define NUM_BUTTONS             3

/* button indices in base.buttons[] tab  */
#define ENCODER_BUTTON          0
#define CNT_RES_BUTTON          1
#define SAVE_BUTTON             2

#define TIME_STEP               100
#define MAX_TIME_VALUE          32000
#define MAX_MAX_CYCLE_CNT_VALUE 9999

/* === private macros === */
/* format value in miliseconds to print as seconds */
#define FORMAT_TIME(TIME)   (TIME > 10000 ? "%d.%d" : " %d.%d")

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
    enum fsm_state              *machine_state;
    struct Counter              *counter;
    uint32_t                    timestamp;

    char                        aux_text[LCD_ROW_LEN];              // auxiliary char buffer
    enum menu_entry             entry;                              // selected menu entry
    bool                        setups_saved;                       // true if setups saved in EEPROM are consistent with these on display

    enum EEPROM_status eeprom_stat;
    enum GPIO_expander_status gpio_exp_stat;
    enum sev_seg_status sev_seg_stat;
} base;

/* === private functions === */

static enum GPIO_expander_status print_static_lcd_txt()
{
    snprintf(base.aux_text, LCD_ROW_LEN, "czas otwarcia:   %d.%d", base.cycle_times->open_time / 1000, base.cycle_times->open_time % 1000);
    base.gpio_exp_stat = Lcd_write(base.aux_text, LCD_OPEN_TIME_ROW, 0);
    CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "");

    snprintf(base.aux_text, LCD_ROW_LEN, "czas wtrysku:    %d.%d", base.cycle_times->injection_time / 1000, base.cycle_times->injection_time % 1000);
    base.gpio_exp_stat = Lcd_write(base.aux_text, LCD_INJECTION_TIME_ROW, 0);
    CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "");

    snprintf(base.aux_text, LCD_ROW_LEN, "czas chlodzenia: %d.%d", base.cycle_times->cooling_time / 1000, base.cycle_times->cooling_time % 1000);
    base.gpio_exp_stat = Lcd_write(base.aux_text, LCD_COOLING_TIME_ROW, 0);
    CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "");

    DBG_LOG("Exiting with gpio_exp_stat = %d", base.gpio_exp_stat);
    return base.gpio_exp_stat;

    error:
    return base.gpio_exp_stat;
}

static uint16_t limit_val(int16_t val, uint16_t low_limit, uint16_t high_val)
{
    if(val < 0) return 0;
    if(val > high_val) return high_val;
    return val;
}

/* update displayed time value on lcd and allgin blinking cursor*/
static enum GPIO_expander_status update_lcd_time(uint16_t val, uint8_t row, uint8_t col)
{
    snprintf(base.aux_text, 5, FORMAT_TIME(val), val / 1000, val % 1000);
    base.gpio_exp_stat = Lcd_write(base.aux_text, row, col);
    CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "gpio_exp_status: %d", base.gpio_exp_stat);

    base.gpio_exp_stat = Lcd_blink_on(row, col + 3);
    CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "gpio_exp_status: %d", base.gpio_exp_stat);

    return GPIO_EXPANDER_OK;
    error:
    return base.gpio_exp_stat;
}
/* === API functions === */

void Menu_init(struct Time_table *time_tab, enum fsm_state *machine_state, struct Counter *counter)
{
    enum sev_seg_status sev_seg_status = SEV_SEG_OK;
    enum GPIO_expander_status gpio_exp_status = GPIO_EXPANDER_OK;

    /* init base struct */
    CHECK(time_tab != NULL, "");
    CHECK(machine_state != NULL, "");
    CHECK(counter != NULL, "");

    base.cycle_times = time_tab;
    base.machine_state = machine_state;
    base.counter = counter;
    base.entry = NO_ENTRY;
    base.setups_saved = true;

    /* initialize buttons */
    base.buttons[ENCODER_BUTTON] = Button_init(CP_ENC_BUTTON_GPIO_Port, CP_ENC_BUTTON_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);
    base.buttons[CNT_RES_BUTTON] = Button_init(CP_CNT_RESET_GPIO_Port, CP_CNT_RESET_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);
    base.buttons[SAVE_BUTTON] = Button_init(CP_SAVE_SETUPS_GPIO_Port, CP_SAVE_SETUPS_Pin, DEBOUNCING_TIME, GPIO_PIN_RESET);

    /* print static part of menu text on lcd */

    gpio_exp_status = print_static_lcd_txt();
    CHECK(gpio_exp_status == GPIO_EXPANDER_OK, "");

    /* display data on seven segment display */
    sev_seg_status = Sev_seg_write(0, SEV_SEG_NO_REFRESH);
    CHECK(sev_seg_status == SEV_SEG_OK, "");

    /* temp code */
    base.timestamp = HAL_GetTick();
    return;

    error:
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
                Sev_seg_write(base.counter->max_value, SEV_SEG_NO_REFRESH);
                base.entry = OPEN_TIME;
            } else if(base.entry == CYCLE_CNT_UNITS) {
                /* After last menu entry, continue iterating through options and display maximum cycle count if setups not saved. If saved, exit setup menu and display actual cycle count. */
                if(base.setups_saved){
                    base.entry = NO_ENTRY;
                    Sev_seg_write(base.counter->value, SEV_SEG_REFRESH);
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
                    Lcd_blink_on(LCD_OPEN_TIME_ROW, LCD_OPEN_TIME_COL + 3);
                    break;
                case INJECTION_TIME:
                    Lcd_blink_on(LCD_INJECTION_TIME_ROW, LCD_INJECTION_TIME_COL + 3);
                    break;
                case COOLING_TIME:
                    Lcd_blink_on(LCD_COOLING_TIME_ROW, LCD_COOLING_TIME_COL + 3);
                    break;
                case CYCLE_CNT_THOUSANDS:
                    Lcd_blink_off();
                    Sev_seg_blink(THOUSANDS);
                    break;
                case CYCLE_CNT_HUNDREDS:
                    Sev_seg_write(base.counter->max_value, SEV_SEG_REFRESH);
                    Sev_seg_blink(HUNDREDS);
                    break;
                case CYCLE_CNT_TENS:
                    Sev_seg_write(base.counter->max_value, SEV_SEG_REFRESH);
                    Sev_seg_blink(TENS);
                    break;
                case CYCLE_CNT_UNITS:
                    Sev_seg_write(base.counter->max_value, SEV_SEG_REFRESH);
                    Sev_seg_blink(UNITS);
                    break;
                default:
                    CHECK(0, "");
            }
        }

        /* cnt reset button. Reset number of counted cycles to zero.*/
        if(Button_been_push(base.buttons[CNT_RES_BUTTON])){
            base.counter->value = 0;
            Sev_seg_write(0, SEV_SEG_REFRESH);
        }

        /* setup save button. Save setups value to EEPROM */
        if(Button_been_push(base.buttons[SAVE_BUTTON]) && !base.setups_saved){
            base.setups_saved = true;
            base.eeprom_stat = EEPROM_write_setups(base.cycle_times, base.counter->max_value);
            CHECK(base.eeprom_stat == EEPROM_ERROR, "");

            base.sev_seg_stat = Sev_seg_write(base.counter->value, SEV_SEG_REFRESH);
            CHECK(base.sev_seg_stat == SEV_SEG_OK, "0");

            base.entry = NO_ENTRY;
            base.gpio_exp_stat = Lcd_blink_off();
            CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "");
        }

        /* Encoder. Modify cycle times and cycle counter maximum value based on encoder transitions */

        int16_t enc_trans = Encoder_get_transitions();
        if(enc_trans != 0){
            base.setups_saved = false;
            switch(base.entry){
            /* update value in memory and on display*/
            case NO_ENTRY:
                break;
            case OPEN_TIME:
                base.cycle_times->open_time = limit_val(base.cycle_times->open_time + TIME_STEP*enc_trans, 0, MAX_TIME_VALUE);
                update_lcd_time(base.cycle_times->open_time, LCD_OPEN_TIME_ROW, LCD_OPEN_TIME_COL);
                break;
            case INJECTION_TIME:
                base.cycle_times->injection_time = limit_val(base.cycle_times->injection_time + TIME_STEP*enc_trans, 0 , MAX_TIME_VALUE);
                update_lcd_time(base.cycle_times->injection_time, LCD_INJECTION_TIME_ROW, LCD_INJECTION_TIME_COL);
                break;
            case COOLING_TIME:
                base.cycle_times->cooling_time = limit_val(base.cycle_times->cooling_time + TIME_STEP*enc_trans, 0, MAX_TIME_VALUE);
                update_lcd_time(base.cycle_times->cooling_time, LCD_COOLING_TIME_ROW, LCD_COOLING_TIME_COL);
                break;
            case CYCLE_CNT_THOUSANDS:
                base.counter->max_value = limit_val(base.counter->max_value + 1000*enc_trans, 0, MAX_MAX_CYCLE_CNT_VALUE);
                Sev_seg_write(base.counter->max_value, SEV_SEG_REFRESH);
                break;
            case CYCLE_CNT_HUNDREDS:
                base.counter->max_value = limit_val(base.counter->max_value + 100*enc_trans, 0, MAX_MAX_CYCLE_CNT_VALUE);
                Sev_seg_write(base.counter->max_value, SEV_SEG_REFRESH);
                break;
            case CYCLE_CNT_TENS:
                base.counter->max_value = limit_val(base.counter->max_value + 10*enc_trans, 0, MAX_MAX_CYCLE_CNT_VALUE);
                Sev_seg_write(base.counter->max_value, SEV_SEG_REFRESH);
                break;
            case CYCLE_CNT_UNITS:
                base.counter->max_value = limit_val(base.counter->max_value + 1*enc_trans, 0, MAX_MAX_CYCLE_CNT_VALUE);
                Sev_seg_write(base.counter->max_value, SEV_SEG_REFRESH);
                break;
            default:
                CHECK(0, "Shouldn't get here");
            }

            CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "");
        }
    }

    base.sev_seg_stat = Sev_seg_process();
    CHECK(base.sev_seg_stat == SEV_SEG_OK, "");
    base.gpio_exp_stat = Lcd_process();
    CHECK(base.gpio_exp_stat == GPIO_EXPANDER_OK, "");


    return;
    error:
    return;
}
