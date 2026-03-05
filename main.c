#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "tkjhat/sdk.h"

// Choosing a safe stack size for the tasks
#define DEFAULT_STACK_SIZE 2048 

// FreeRTOS queue for communication
QueueHandle_t xMorseQueue;

// Structure to read IMU data
struct imu_data {
    float ax; 
    float ay; 
    float az;
    float gx; 
    float gy; 
    float gz;
    float t;
} imuData;

// Creating prototypes for tasks 
static void sensor_task(void *pvParameters);
static void serial_task(void *pvParameters);
void add_to_queue(char mark);

// Main function
int main() {
    // Initializing USB
    stdio_init_all();
    sleep_ms(2000); // Waiting for the USB to wake up
    printf("USB is starting\n");

    // Initializing JTKJ Hat
    init_hat_sdk();
    sleep_ms(100);

    // Initializing IMU sensor
    if (init_ICM42670() == 0) {
        printf("IMU initializing is good\n");
        ICM42670_start_with_default_values();
    } else {
        printf(IMU initializing failed!\n");
    }

    // Creating queue
    xMorseQueue = xQueueCreate(10, sizeof(char));

    // Creating FreeRTOS tasks
    TaskHandle_t sensorTaskHandle = NULL;
    TaskHandle_t serialTaskHandle = NULL;

    xTaskCreate(sensor_task,       
                "Sensor_Task",     
                DEFAULT_STACK_SIZE,
                NULL,              
                2,                 // Setting a higher priority to the sensor
                &sensorTaskHandle);
    
    xTaskCreate(serial_task, 
                "Serial_Task", 
                DEFAULT_STACK_SIZE, 
                NULL, 
                1,                 // Lower priority to the serial
                &serialTaskHandle);

    // Starting the system
    printf("Starting FreeRTOS Scheduler\n");
    vTaskStartScheduler();

    while(1){};
    return 0;
}


// Task to detect physical movements and that creates morse code marks.
static void sensor_task(void *pvParameters) {
    (void)pvParameters;
    struct imu_data *data = &imuData;
    
    // Counters
    static int dot_counter = 0;
    static int dash_counter = 0;

    while (1) {
        // Reading sensor
        if (ICM42670_read_sensor_data(&data->ax, &data->ay, &data->az, 
                                      &data->gx, &data->gy, &data->gz, 
                                      &data->t) == 0) {
            
            float gx = data->gx;

            // Positive GX
            if (gx > 100) {
                dot_counter++;
                dash_counter = 0; // reset opposite counter

                if (dot_counter >= 3) { // Require 3 consecutive measurements to detect a dot
                    add_to_queue('.'); 
                    dot_counter = 0; // Reset so that we dont have many dots
                    vTaskDelay(pdMS_TO_TICKS(500)); // Prevents multiple detections from a single movement
                }
            }
            // Negative GX
            else if (gx < -100) {
                dash_counter++;
                dot_counter = 0;    

                if (dash_counter >= 3) {
                    add_to_queue('-'); 
                    dash_counter = 0;
                    vTaskDelay(pdMS_TO_TICKS(500));
                }
            }
            // No movement equals that we reset counters
            else {
                dot_counter = 0;
                dash_counter = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
          
// Task for handling USB serial communication
static void serial_task(void *pvParameters) {
    (void)pvParameters;
    char received_char;

    while (1) {
        // Waiting for data from the queue forever
        if (xQueueReceive(xMorseQueue, &received_char, portMAX_DELAY) == pdPASS) {
            
            printf("%c\n", received_char);
        }
    }
}

void add_to_queue(char mark) {
    // Sending to the queue but not waiting if the queue is full
    xQueueSend(xMorseQueue, &mark, 0);
}