/* 
Cátedra: Sistemas Informáticos Industriales 
Año: 2024
Plataforma: ESP-IDF (https://docs.espressif.com/projects/esp-idf)
IDE: VsCode con plantilla de PlatformIO
SoC: ESP32 
*/

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>

//Declaración de variables globales e inicializar funciones
static const char*  TAG = "2024CisternaUTRs";


#define PinCaudalimetro 35
#define PinTrigger 0
#define PinEcho 4 

//Declaracion de variables globales 
long caudalContador=0; 
float caudaltemporal=0;
float caudalHora=0; //Indica la cantidad de litros que salieron del tanque 

volatile int tiempoInicio=0;
volatile int tiempoFin=0;
float distancia=0; // indica la cantida de litros dentro del tanqie. Nivel. 

/*
#define PinMotor 23  //DELETE
#define botonISR 27 //asigmo el pin 27 para recibir una interrupción externa.   DELETE
static int icontISR = 0; // contador de veces que se ejecutó la interrupción.   DELETE
static TickType_t ultimaISR; //tiempo en que se ejecutó por última vez la interrupción  DELETE 


//función que ejecuta la rutina de interrrupción
void ISRAlarma(void *args) //DELETE
{
 // Controlo que hayan pasado al menos 20 milisengudo entre cada interrupción. 
 // ya que el switch de final de carrera no tiene demasiada sensibilidad y produce varias ISR simultaneas.
 if (xTaskGetTickCountFromISR() > (ultimaISR + 20))
 {
    ultimaISR = xTaskGetTickCountFromISR() ;
    icontISR++; 
 };

}

void InitISR()  //DELETE
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
 

//TaskHandle_t handleATarea1; //aquí recibo el manejador para la tarea una vez creada   DELETE

//función que ejecutará la Tarea 1 
void Tarea1(void *param) //DELETE
{
    unsigned long ulContador=0;

    for (;;)
    {
        ulContador++;
        printf("Estoy dentro de la tarea 1. Vuelta [%lu] y ejecuta en el nucleo [%lu]\n", ulContador, (unsigned long) xTaskGetCoreID(handleATarea1) ); 
        for ( long l = 0; l < 1000000; l++)
        {
            xTaskGetCoreID(handleATarea1);
        }
    

        vTaskDelay(30/ portTICK_PERIOD_MS); 
    }
}
*/

void ISRCaudalimetro(void *args)
{
    caudalContador++;
}

void CalcularCaudal(void *args)
{
    while(1){

        static long indiceSegundo = 0;
        float pulsosPorSegundo[3600] = {0};
        float caudalAuxiliar = 0;

        caudaltemporal = caudalContador * 2.25 / 1000; //2,25ml por pulso
        
        caudalContador = 0; //reset contador
        
        pulsosPorSegundo[indiceSegundo] = caudaltemporal;  //sumatoria de L en la ultima hora

        indiceSegundo = (indiceSegundo + 1) % 3600;  // buffer temporal


        for (int i = 0 ; i < 3600 ; i++){
            caudalAuxiliar += pulsosPorSegundo[i];  //auxiliar para evitar que caudalhora alguna vez sea 0
        }

        caudalHora = caudalAuxiliar;
        
        caudalAuxiliar = 0;

        vTaskDelay(1000/ portTICK_PERIOD_MS); 
    }
}

void IRAM_ATTR ISREcho(void *args) //IRAM_ATTR lo ejecuta en RAM en vez de ROM
{
    if (gpio_get_level(PinEcho)) { // Flanco ascendente
        tiempoInicio = esp_timer_get_time(); // Guarda el tiempo de inicio

    } else { // Flanco descendente
        tiempoFin = esp_timer_get_time(); // Guarda el tiempo de finalización

    }
}

void CalculoDistancia(void *args)
{
    while (1) {
        gpio_set_level(PinTrigger, 1); // Activa el Trigger
        vTaskDelay(pdMS_TO_TICKS(10)); // Mantiene el Trigger activo por 10 microsegundos
        gpio_set_level(PinTrigger, 0); // Desactiva el Trigger
        

        if (tiempoFin > tiempoInicio) {
            int duracion = tiempoFin - tiempoInicio; // Duración del pulso en microsegundos
            distancia = duracion / 1000000 * 343000 / 2; // Distancia (milimetros) = duracion en segundos * 343mm/s /2
        }

        vTaskDelay(1000/ portTICK_PERIOD_MS); // Espera 1 segundo antes de la siguiente medición

    }
}

void InitCaudalimetroISR()
{
    gpio_config_t gpioCaudalimetro;
    gpioCaudalimetro.pin_bit_mask = (1ULL << PinCaudalimetro); 
    gpioCaudalimetro.mode = GPIO_MODE_INPUT;
    gpioCaudalimetro.pull_up_en = GPIO_PULLUP_ENABLE;
    gpioCaudalimetro.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioCaudalimetro.intr_type = GPIO_INTR_NEGEDGE; // Detecta flanco de bajada
    gpio_config(&gpioCaudalimetro);

    gpio_install_isr_service(0); 
    gpio_isr_handler_add(PinCaudalimetro, ISRCaudalimetro, NULL);
}
 
void InitUltrasonico() {
    // Configura el pin Trigger como salida
    gpio_config_t triggerConfig;
    triggerConfig.pin_bit_mask = (1ULL << PinTrigger);
    triggerConfig.mode = GPIO_MODE_OUTPUT;
    triggerConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    triggerConfig.pull_down_en = GPIO_PULLDOWN_DISABLE; 
    gpio_config(&triggerConfig);

    // Configura el pin Echo como entrada
    gpio_config_t echoConfig;
    echoConfig.pin_bit_mask = (1ULL << PinEcho);
    echoConfig.mode = GPIO_MODE_INPUT;
    echoConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    echoConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    echoConfig.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&echoConfig);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PinEcho, ISREcho, NULL);
}

void app_main() 
{
    /*
    //creo 1 tarea 
    //xTaskCreate(Tarea1, "Tarea 1", 2048, NULL,1, &handleATarea1 );
    // Creo la tarea 1 y la asigno al segundo nucleo
    //xTaskCreatePinnedToCore(Tarea1, "Tarea 1", 2048, NULL,1, &handleATarea1,0);


    //Creamos una interrupción externa asociada al puerto 27. 
    InitISR(); //DELETE

    //Iniclizamos las propiedades del GPIO 23 
    gpio_reset_pin(PinMotor); //DELETE
    gpio_set_direction(PinMotor, GPIO_MODE_DEF_OUTPUT); //DELETE 
    */

    InitCaudalimetroISR();
    InitUltrasonico();
    gpio_reset_pin(PinCaudalimetro);

    gpio_set_direction(PinCaudalimetro, GPIO_MODE_DEF_OUTPUT); 

    gpio_reset_pin(PinTrigger);
    gpio_set_direction(PinTrigger, GPIO_MODE_DEF_OUTPUT); 
    gpio_reset_pin(PinEcho);
    gpio_set_direction(PinEcho, GPIO_MODE_DEF_OUTPUT); 

    xTaskCreate(CalcularCaudal, "CalcularCaudal", 2048, NULL, 1, NULL);
    xTaskCreate(CalculoDistancia, "TareaUltrasonido", 2048, NULL, 1, NULL);


    //bool bandera = true; //DELETE


    while (1)
    {
        // aqui genero la señal de eco para luego recibir la interrupcion 

    
        /*if (bandera){ //DELETE
        bandera = !bandera;
        gpio_set_level(PinMotor, 0);

        }else{
        bandera = !bandera;
        gpio_set_level(PinMotor, 1);

        
        senalgenerada = ahora(); 
        ecorecibido = ahora(); 
        duracionEco = ecorecibido  - senalgenerada; 
        altura = ...   
        
        }
    

        //printf("Ejecutando el programa principaL ejecuta nucleo [%lu]\n", (unsigned long) xTaskGetCoreID(xTaskGetCurrentTaskHandle()));
        */

        printf("Caudal =[%f], Distancia=[%f]\n", caudalHora, distancia);
        vTaskDelay(1000/ portTICK_PERIOD_MS);

    }
}