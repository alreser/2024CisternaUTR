#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <esp_log.h>


//Declaración de variables globales e incializar funciones
static const char*  TAG = "2024CisternaUTRs";
TaskHandle_t handleATarea1, handleATarea2, handleATarea3; //aquí recibo el manejador para la tarea una vez creada
#define PinMotor 23  




//función que ejecutará la Tarea 1 
void Tarea1(void *param)
{// aquí está el códigode la tarea 
unsigned long ulContador=0;

for (;;)
{
    /* code */
    ulContador++;
    printf("Estoy dentro de la tarea 1. Vuelta [%lu] y ejecuta en el nucleo [%lu]\n", ulContador, (unsigned long) xTaskGetCoreID(handleATarea1) ); 
    for ( long l = 0; l < 1000000; l++)
    {
        /* code */
        xTaskGetCoreID(handleATarea1);

    }
    

    vTaskDelay(30/ portTICK_PERIOD_MS); 
}


}


//funcion que eejecuta la tarea 2 
void Tarea2(void *param)
{// aquí está el códigode la tarea 
unsigned long ulContador=0;

for (;;)
{
    /* code */
    ulContador++;
    printf("Estoy dentro de la tarea 2. Vuelta [%lu] y ejecuta en el nucleo [%lu]\n", ulContador, (unsigned long) xTaskGetCoreID(handleATarea2) ); 
    for ( long l = 0; l < 1000000; l++)
    {
        /* code */
        xTaskGetCoreID(handleATarea2);

    }
    

    vTaskDelay(30/ portTICK_PERIOD_MS); 
}


}

//funcion de la tarea 3
void Tarea3(void *param)
{// aquí está el códigode la tarea 
unsigned long ulContador=0;

for (;;)
{
    /* code */
    ulContador++;
    printf("Estoy dentro de la tarea 3. Vuelta [%lu] y ejecuta en el nucleo [%lu]\n", ulContador, (unsigned long) xTaskGetCoreID(handleATarea3) ); 
    for ( long l = 0; l < 1000000; l++)
    {
        /* code */
        xTaskGetCoreID(handleATarea3);

    }
    

    vTaskDelay(30/ portTICK_PERIOD_MS); 
}


}



void app_main() 
{


//creo 3 tareas 
//xTaskCreate(Tarea1, "Tarea 1", 2048, NULL,1, &handleATarea1 );
// Creo la tarea 1 y la asigno al segundo nucleo
//xTaskCreatePinnedToCore(Tarea1, "Tarea 1", 2048, NULL,1, &handleATarea1,0);

// Creo la tarea 2 y la asigno al segundo nucleo
//xTaskCreatePinnedToCore(Tarea2, "Tarea 2", 2048, NULL,1, &handleATarea2,0);

// Creo la tarea 3 y la asigno al segundo nucleo
//xTaskCreatePinnedToCore(Tarea3, "Tarea 3", 2048, NULL,1, &handleATarea3,0);

//Iniclizamos las propiedades del GPIO 23 
gpio_reset_pin(PinMotor); 
gpio_set_direction(PinMotor, GPIO_MODE_DEF_OUTPUT) ; 



    while (1)
    {
    
    printf("Ejecutando el programa principaL ejecuta nucleo [%lu]\n", (unsigned long) xTaskGetCoreID(xTaskGetCurrentTaskHandle()));
    vTaskDelay(30/ portTICK_PERIOD_MS);



    }
}