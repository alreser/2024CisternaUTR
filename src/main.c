/* Ejemplo programación de Tareas en cada nucleo del procesador 
 Ejemplo de progración de Interrupciones por hardware desde un GPIO
Cátedra: Sistemas Informáticos Industriales 
Año: 2024
Plataforma: ESP-IDF (https://docs.espressif.com/projects/esp-idf)
IDE: VsCode con plantilla de PlatformIO
SoC: ESP32 
Notas:  En este ejemplo se crean 3 tarea que serán ejecutadas según su prioridad. También se crea una 
        interrupción por hardware ISR que se recibe como un flanco ascendente en el GPIO especificado para 
        tal fin. La interrupción se lanza al cerrar el circuito de un interruptor de tipo Final de Carrera.
  
*/


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <esp_log.h>

//Declaración de variables globales e inicializar funciones
static const char*  TAG = "2024CisternaUTRs";
#define PinMotor 23  


//Declaracion de variables globales 
int contador=0; 
long caudaltotal =0; //Indica la cantidad de litros que salieron del tanque 
long niveltanque=0; // indica la cantida de litros dentro del tanqie. Nivel. 


//#pragma region Interrupciones 
 

#define botonISR 27 //asigmo el pin 27 para recibir una interrupción externa. 
static int icontISR = 0; // contador de veces que se ejecutó la interrupción.  
static TickType_t ultimaISR; //tiempo en que se ejecutó por última vez la interrupción 


//función que ejecuta la rutina de interrrupción
void ISRAlarma(void *args)
{
 // Controlo que hayan pasado al menos 20 milisengudo entre cada interrupción. 
 // ya que el switch de final de carrera no tiene demasiada sensibilidad y produce varias ISR simultaneas.
 if (xTaskGetTickCountFromISR() > (ultimaISR + 20))
 {
    ultimaISR = xTaskGetTickCountFromISR() ;
    icontISR++; 
 };

}

/// @brief Función que inicializa la interrupción externa 
void InitISR()
{
    //Configuramos las propiedades de GPIO donde conectaremos la interrupción
    gpio_config_t gpioISR;
    gpioISR.pin_bit_mask = (1ULL << botonISR); 
    gpioISR.mode  = GPIO_MODE_DEF_INPUT;
    gpioISR.pull_up_en = GPIO_PULLUP_DISABLE;
    gpioISR.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpioISR.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&gpioISR);

    //llamo a iniciar el servicio de interrupciones en Freertos 
    gpio_install_isr_service(0);
    gpio_isr_handler_add(botonISR, ISRAlarma, NULL);

//Nota: generar dos estructuras 

}

//#pragma endregion Interrupciones

//#pragma region Tareas

TaskHandle_t handleATarea1; //aquí recibo el manejador para la tarea una vez creada

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


//#pragma endregion Tareas




void app_main() 
{

//creo 1 tarea 
//xTaskCreate(Tarea1, "Tarea 1", 2048, NULL,1, &handleATarea1 );
// Creo la tarea 1 y la asigno al segundo nucleo
//xTaskCreatePinnedToCore(Tarea1, "Tarea 1", 2048, NULL,1, &handleATarea1,0);


//Creamos una interrupción externa asociada al puerto 27. 
InitISR();

//Iniclizamos las propiedades del GPIO 23 
gpio_reset_pin(PinMotor); 
gpio_set_direction(PinMotor, GPIO_MODE_DEF_OUTPUT) ; 


bool bandera = true; 


    while (1)
    {

// aqui genero la señal de eco para luego recibir la interrupcion 

    if (bandera) 
    {
        bandera = !bandera;
        gpio_set_level(PinMotor, 0);

    }
    else
    {
        bandera = !bandera;
        gpio_set_level(PinMotor, 1);

         /* 
         senalgenerada = ahora(); 
        ecorecibido = ahora(); 
        duracionEco = ecorecibido  - senalgenerada; 
        altura = ...   
        */ 
    }
    
    

    //printf("Ejecutando el programa principaL ejecuta nucleo [%lu]\n", (unsigned long) xTaskGetCoreID(xTaskGetCurrentTaskHandle()));
    printf("Caudal =[%ld], Nivel=[%ld]\n", caudaltotal, niveltanque);
    vTaskDelay(1000/ portTICK_PERIOD_MS);



    }
}