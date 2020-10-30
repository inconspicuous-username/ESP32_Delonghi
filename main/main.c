#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "owb.h"
#include "owb_rmt.h"

#include "ds18b20.h"

#define GPIO_DS18B20_0       (23)
#define MAX_DEVICES          (1)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_10_BIT)
#define SAMPLE_PERIOD        (1000)   // milliseconds
#define SSR_PINOUT           (2)

float currentTemp;
float setpoint;
int powerLevel;
bool controller_enable;
float p_coeff = 6.0;
float i_p_coeff = 0.005;
float i_n_coeff = 0.0005;


void measureTempTask( void *pvParameters ){
    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;
    owb = owb_rmt_initialize(&rmt_driver_info, GPIO_DS18B20_0, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb_use_crc(owb, true);  // enable CRC check for ROM code

    // Find all connected devices
    printf("Find devices:\n");
    OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
    int num_devices = 0;
    OneWireBus_SearchState search_state = {0};
    bool found = false;
    owb_search_first(owb, &search_state, &found);
    while (found)
    {
        char rom_code_s[17];
        owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
        printf("  %d : %s\n", num_devices, rom_code_s);
        device_rom_codes[num_devices] = search_state.rom_code;
        ++num_devices;
        owb_search_next(owb, &search_state, &found);
    }
    printf("Found %d device%s\n", num_devices, num_devices == 1 ? "" : "s");
    // For a single device only:
    OneWireBus_ROMCode rom_code;
    owb_status status = owb_read_rom(owb, &rom_code);
    if (status == OWB_STATUS_OK){
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(rom_code, rom_code_s, sizeof(rom_code_s));
            printf("Succesfully connected to device: %s\n", rom_code_s);
        }
        else
        {
            printf("An error occurred reading ROM code: %d", status);
        }
    // Create DS18B20 devices on the 1-Wire bus
    DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
    printf("Single device optimisations enabled\n");
    ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
    ds18b20_use_crc(ds18b20_info, true);           // enable CRC check on all reads
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    printf("Sensor intialized\n");

    ds18b20_convert(ds18b20_info);
    vTaskDelay(250 / portTICK_PERIOD_MS);
    float output = 0;
    float i_part = 0;
    float delta = 0;
    // Enable the controller by default.
    controller_enable = false;
    while (1)
    {
        ds18b20_read_temp(ds18b20_info, &currentTemp);
        ds18b20_convert(ds18b20_info);
        delta = setpoint - currentTemp;
        /* Start of the controller section*/
        if (controller_enable){
            if (delta > 5){
            } else{
                if (delta > 0) // too cold
                {
                    i_part += delta * i_p_coeff;
                }
                else
                {
                    i_part += delta * i_n_coeff;
                }
                
            }

            output = delta * p_coeff + i_part;
            // ensure valudes lie within 0-100 range
            if (output<0){
                output = 0;
            }else if (output > 100){
                output =100;
            }
            powerLevel = (int) output;
            /* End of controlelr section*/
        } else{
            powerLevel = 0;
            i_part = 0;
        }

        printf("temp %.3f target %.3f power %d i_part %.3f p %.0f ip %.7f in %.7f\n", currentTemp, setpoint, powerLevel, i_part, p_coeff, i_p_coeff, i_n_coeff);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
 }

void powerControlTask( void *pvParameters ){
    printf("Configuring GPIO for SSR\n");
    gpio_set_level(SSR_PINOUT, 0); // Make sure the output is disabled before 
    gpio_set_direction(SSR_PINOUT, GPIO_MODE_OUTPUT);
    while(1){
        for (int i=0;i<100;i++){
            if (i > powerLevel){
                gpio_set_level(SSR_PINOUT, 0);    
            }
            else{
                gpio_set_level(SSR_PINOUT, 1);
            }
            vTaskDelay(20/ portTICK_PERIOD_MS);
        }
    }

}

void ParamControlTask( void *pvParameters ){
    
    char cmd;
    while(1){
        // printf("Current values: \t P = %f \t ");
        // printf("Reading msgs...\n");
        scanf("%c", &cmd);
        switch (cmd)
        {
        case 'p':
            // printf("setting P value: ");
            scanf("%f", &p_coeff);
            break;
        case '+':
            // printf("setting i_p value: ");
            scanf("%f", &i_p_coeff);
            break;
        case '-':
            // printf("setting i_n value: ");
            scanf("%f", &i_n_coeff);
            break;
        case 'e':
            // printf("enabling controller!\n");
            controller_enable = true;
            break;
        case 'd':
            // printf("disabling controller!\n");
            controller_enable = false;
            break;
        
        default:
            break;
        }
        vTaskDelay(20/ portTICK_PERIOD_MS);
    }
}


void app_main(void)
{
    powerLevel = 0;
    setpoint = 91;
    // powerControlTask(NULL);
    printf("pre-registering task \n");
    BaseType_t xReturned;
    TaskHandle_t tempTaskHandle = NULL;
    TaskHandle_t powerControlHandle = NULL;
    TaskHandle_t paramControlHandle = NULL;

    /* Create the task for reading the temperature. */
    xReturned = xTaskCreate(
                    measureTempTask,       /* Function that implements the task. */
                    "Temp Task",          /* Text name for the task. */
                    2048,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &tempTaskHandle);      /* Used to pass out the created task's handle. */

    /* Create the task for controlloing the relay */
    xReturned = xTaskCreate(
                    powerControlTask,       /* Function that implements the task. */
                    "Power control Task",          /* Text name for the task. */
                    2048,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &powerControlHandle);      /* Used to pass out the created task's handle. */

    /* Create the task for controlloing the paramters */
    xReturned = xTaskCreate(
                    ParamControlTask,       /* Function that implements the task. */
                    "Param control Task",          /* Text name for the task. */
                    2048,      /* Stack size in words, not bytes. */
                    ( void * ) 1,    /* Parameter passed into the task. */
                    2,/* Priority at which the task is created. */
                    &paramControlHandle);      /* Used to pass out the created task's handle. */

    configASSERT( tempTaskHandle );
    configASSERT( powerControlHandle );
    configASSERT( paramControlHandle );

    while(1){
       vTaskDelay(1000 / portTICK_PERIOD_MS);
    //    printf("Still alive \n");
    }
}
