/**
 * @file batt_monitor.h
 * @brief Battery monitor
 *
 * @date 	12/03/21
 * @author 	Julian Botello <jlnbotello@gmail.com>
 */

/*==================[INCLUSIONS]=============================================*/
#include "batt_monitor.h"
#include "srv_dashboard_eh.h"
#include "max1726x.h"
#include "batt_archiver.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

/*==================[MACROS AND DEFINITIONS]=================================*/
#define NEW_BATT_FLAG_GPIO  5                       /* on GiveMove IoT v1.0.0 board is CS0 */
#define BATT_MON_SDA_GPIO   21
#define BATT_MON_SCL_GPIO   22

/*==================[INTERNAL FUNCTIONS DECLARATION]=========================*/
static void on_batt_lvl_read(esp_ble_gatts_cb_param_t *param, esp_gatt_rsp_t *rsp);

/*==================[INTERNAL DATA DEFINITION]===============================*/
dashboard_srv_cb_desc_t batt_monitor_cb_desc = {
    .idx = IDX_DS_BATTERY_LEVEL_VAL,
    .w_cb = NULL,
    .r_cb = on_batt_lvl_read
};

/*==================[INTERNAL FUNCTIONS DEFINITION]==========================*/
static void on_batt_lvl_read(esp_ble_gatts_cb_param_t *param, esp_gatt_rsp_t *rsp){
    ESP_LOGI("BATT", "Read batt level");
    rsp->attr_value.len = sizeof(dashboard_srv_data.battery_level);
    memcpy(rsp->attr_value.value, &dashboard_srv_data.battery_level, rsp->attr_value.len);
}

/*==================[EXTERNAL FUNCTIONS DEFINITION]==========================*/
void batt_monitor_init(SemaphoreHandle_t mutex)
{
    // init max17261 i2c init
    i2c_iface_init_desc(0, MAX1726X_I2C_ADDR, BATT_MON_SDA_GPIO, BATT_MON_SCL_GPIO, mutex);
    
    // gpio to set a new batettery
    gpio_reset_pin(NEW_BATT_FLAG_GPIO);
    gpio_set_direction(NEW_BATT_FLAG_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(NEW_BATT_FLAG_GPIO, GPIO_PULLUP_ONLY);

    dashboard_service_register_callback(&batt_monitor_cb_desc);
}

static bool is_new_battery()
{
    return (0 == gpio_get_level(NEW_BATT_FLAG_GPIO));
}

void batt_monitor_task(void *pvP)
{

    uint16_t temp;
    max1726x_ez_config_t ez;
    max1726x_learned_parameters_t lp;

    // Wait until device is connected.
    while (RET_OK != max1726x_read_reg(MAX1726X_STATUS_REG, &temp))
    {
        ESP_LOGW("BATT_MONITOR", "Battery monitor disconnected.\nIt's OK while your are configuring the device from PC.");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }

    if (max1726x_check_por())
        max1726x_wait_dnr();

    if (is_new_battery())
    {
        ESP_LOGI("BATT", "New battery");
        // Read ez and lp from device. They were previously loaded from PC GUI through the adapter.
        if (RET_OK == max1726x_read_ez_config(&ez))
        {
            ESP_LOGI("BATT_EZ", "OK");
            batt_archiver_save_ez(&ez);
        }
        if (RET_OK == max1726x_get_learned_parameters(&lp))
        {
            ESP_LOGI("BATT_LP", "OK");
            batt_archiver_save_lp(&lp);            
        }
    }
    else
    { 
        ESP_LOGI("BATT", "Known battery");
        // Read from archive
        batt_archiver_open_ez(&ez);
        batt_archiver_open_lp(&lp);
        
    }
    ESP_LOGI("BATT_EZ", "designcap: %d ichgterm: %d modelcfg: %d vempty: %d",ez.designcap,ez.ichgterm,ez.modelcfg,ez.vempty);
    ESP_LOGI("BATT_LP", "cycles: %d fullcapnom: %d fullcaprep: %d rcomp0: %d tempco: %d",lp.saved_cycles,lp.saved_fullcapnom,lp.saved_fullcaprep,lp.saved_rcomp0,lp.saved_tempco);
        
    max1726x_initialize_ez_config(&ez);
    max1726x_clear_por();    
    max1726x_restore_learned_parameters(&lp);
    

    uint8_t soc=0;
    const uint16_t trigger_value = (uint16_t) 60000/SOC_READING_PERIOD_MS*30; // 30 min
    uint16_t counter = trigger_value;

    while (1)
    {
        soc = (uint8_t) max1726x_get_repsoc();
        ds_set_battery_level(soc);
        if(counter == 0){
             if (RET_OK == max1726x_get_learned_parameters(&lp))
            {
                ESP_LOGI("BATT_LP", "Saving Batt Params");
                ESP_LOGI("BATT_LP", "cycles: %d fullcapnom: %d fullcaprep: %d rcomp0: %d tempco: %d",lp.saved_cycles,lp.saved_fullcapnom,lp.saved_fullcaprep,lp.saved_rcomp0,lp.saved_tempco);
                batt_archiver_save_lp(&lp);            
            }
            counter = (uint16_t) trigger_value;
        }
        counter--;
        ESP_LOGI("BATT", "Battery level: %d %%", soc);
        vTaskDelay(SOC_READING_PERIOD_MS / portTICK_PERIOD_MS);
    }
}

/*==================[END OF FILE]============================================*/