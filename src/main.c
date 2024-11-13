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
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_system.h>
#include <string.h>
#include <esp_log.h>


//Declaración de variables globales e inicializar funciones
static const char*  TAG = "2024CisternaUTRs";


#define PinCaudalimetro 35

#define PinTrigger 32
#define PinEcho 33
#define DistanciaMaximaCM 900 // 800 + ~10%
#define UART_Numero UART_NUM_0   // Deberian ser los pines 1 y 3. 
#define UART_TiempoEnvioMS 500 // frecuencia de envio de datos en milisegundos
 

//Declaracion de variables globales 
long caudalContador=0; 
float caudaltemporal=0;
float caudalHora=0; //Indica la cantidad de litros que salieron del tanque 
static long indiceSegundo = 0;
float pulsosPorSegundo[3600] = {0}; // Asignación dinámica

volatile uint64_t  tiempoInicio=0;
volatile uint64_t  tiempoFin=0;
bool triggerDisponible = true;
float distancia=0; // indica la cantida de litros dentro del tanque. Nivel. 
uint64_t duracion=0;
float timeout = 0; //microsegundos
float tiempoLimite = 0; // Tiempo límite para el timeout

void ISRCaudalimetro(void *args)
{
    caudalContador++;
}

void CalcularCaudal(void *args)
{

    float caudalAuxiliar = 0;    

    while(1)
    {

        caudaltemporal = caudalContador * 2.25 / 1000; //2,25ml por pulso
        
        //printf("CaudalContador =[%li]\n", caudalContador);
        //printf("CaudalTemporal =[%f]\n", caudaltemporal);

        //portENTER_CRITICAL(&mux);
        caudalContador = 0; //reset contador
        //portEXIT_CRITICAL(&mux);

        pulsosPorSegundo[indiceSegundo] = caudaltemporal;  //sumatoria de L en la ultima hora

        indiceSegundo = (indiceSegundo + 1) % 3600;  // buffer temporal
        //printf("indiceSegundo =[%li]\n", indiceSegundo);

        caudalAuxiliar = 0;
        
        for (int i = 0 ; i < 3600 ; i++){
            caudalAuxiliar += pulsosPorSegundo[i];  //auxiliar para evitar que caudalhora alguna vez sea 0
        }

        caudalHora = caudalAuxiliar;

        //printf("Caudal en ultima Hora =[%f]\n", caudalHora);

        vTaskDelay(1000/ portTICK_PERIOD_MS); 
    }
}

void IRAM_ATTR ISREcho(void *args) //IRAM_ATTR lo ejecuta en RAM en vez de ROM
{
    //printf("interrupcion\n");
    if (gpio_get_level(PinEcho)) { // Flanco ascendente
        tiempoInicio = esp_timer_get_time(); // Guarda el tiempo de inicio


    } else { // Flanco descendente
        tiempoFin = esp_timer_get_time(); // Guarda el tiempo de finalización

        if (tiempoFin > tiempoInicio) {
            duracion = tiempoFin - tiempoInicio; // Duración del pulso en microsegundos
            distancia = duracion * 0.343 / 2; // Distancia (milimetros) = duracion en segundos * 343000mm/s /2
            //printf("duracion=[%li]  Distancia=[%f]\n", caudalContador,distancia);
        }
    }
}

void CalculoDistancia(void *args)
{
    timeout = DistanciaMaximaCM * 2 * 10000 / 343;  // cm * 2 *m/100cm *s/343m *1000000us/s
    
    while (1)
    {    
        //ESP_LOGW(TAG,"EMPIEZA CICLO. Trigger: %d. Echo: %d\n", gpio_get_level(PinTrigger), gpio_get_level(PinEcho));
        //printf("EMPIEZA CICLO. Trigger: %d. Echo: %d\n", gpio_get_level(PinTrigger), gpio_get_level(PinEcho));
        //printf("Tiempo Inicio =[%"PRIu64"]. Tiempo Fin =[%"PRIu64"]. Duracion =[%"PRIu64"]\n", tiempoInicio, tiempoFin, duracion);


        if(gpio_get_level(!PinEcho)){

            gpio_set_level(PinTrigger, 0); // Desactiva el Trigger
            esp_rom_delay_us(2);   //aseguro reseteo
            gpio_set_level(PinTrigger, 1); // Activa el Trigger
            esp_rom_delay_us(10); // Mantiene el Trigger activo por al menos 10 microsegundos
            gpio_set_level(PinTrigger, 0); // Desactiva el Trigger

           // printf("Trigger: %d. Echo: %d\n", gpio_get_level(PinTrigger), gpio_get_level(PinEcho));
            
            
            tiempoLimite = esp_timer_get_time() + timeout; // Timeout en microsegundos

            // Espera a que el Echo regrese o a que se alcance el tiempo máximo
            while (gpio_get_level(PinEcho) && esp_timer_get_time() < tiempoLimite) {
                //printf("Tiempo Limite =[%f]. Esperando al Echo o al Timeout\n", tiempoLimite);
                // Espera hasta que el Echo vuelva a 0 o se haya alcanzado el timeout
            }

            if (esp_timer_get_time() >= tiempoLimite) {
                distancia = -1; // Resolver que hacer si tenemos un timeout
                //printf("Tiempo Limite sobrepasado =[%f]\n", tiempoLimite);
            }
            
        }

        vTaskDelay(1000/ portTICK_PERIOD_MS); // Espera un segundo antes de la siguiente medición
        //printf("Distancia=[%f]\n", distancia);

    }
}

void InitCaudalimetroISR()
{
    gpio_config_t gpioCaudalimetro;
    gpioCaudalimetro.pin_bit_mask = (1ULL << PinCaudalimetro); 
    gpioCaudalimetro.mode = GPIO_MODE_INPUT;
    gpioCaudalimetro.pull_up_en = GPIO_PULLUP_ENABLE;
    gpioCaudalimetro.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpioCaudalimetro.intr_type = GPIO_INTR_ANYEDGE; // Detecta flanco de subida. y de bajada
    gpio_config(&gpioCaudalimetro);

    gpio_isr_handler_add(PinCaudalimetro, ISRCaudalimetro, NULL);
}
 
void InitUltrasonico() {
    // Configura el pin Echo como entrada
    gpio_config_t echoConfig;
    echoConfig.pin_bit_mask = (1ULL << PinEcho);
    echoConfig.mode = GPIO_MODE_INPUT;
    echoConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    echoConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    echoConfig.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&echoConfig);

    gpio_isr_handler_add(PinEcho, ISREcho, NULL);
}

/// @brief funcion de inicialización de los parámetros de Universal Asynchronous Receiver and Trasnmitter
void InitUART(){

 //inicializo el puerto UART   
 uart_config_t uart_configt = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_Numero, &uart_configt);
    uart_set_pin(UART_Numero, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE); // TX: 17, RX: 16
    uart_driver_install(UART_Numero, 1024 * 2, 0, 0, NULL, 0);



}

void UART_EnviarLecturas(void *args){


    //Envio los valores sensados y estados del RTU 
    char snivel[30]; 
    char scaudalH[30];

    while (1)
    {
        /* code */
    
    //Envio el Nivel    
    sprintf(snivel,"VAL_NIVEL=%.0f", distancia );
    uart_write_bytes(UART_Numero, snivel, strlen(snivel));
    uart_write_bytes(UART_Numero, "\n", 1); // Agregar salto de línea para separar los comandos
    

        //Envio el Caudal    
    sprintf(scaudalH,"VAL_CAUDALHORA=%.0f", caudalHora );
    uart_write_bytes(UART_Numero, scaudalH, strlen(scaudalH));
    uart_write_bytes(UART_Numero, "\n", 1); // Agregar salto de línea para separar los comandos
    
    vTaskDelay( UART_TiempoEnvioMS / portTICK_PERIOD_MS );

    };
    

}

void app_main() 
{
    gpio_reset_pin(PinCaudalimetro);
    gpio_reset_pin(PinTrigger);
    gpio_reset_pin(PinEcho);

    gpio_install_isr_service(0);

    InitCaudalimetroISR();
    InitUltrasonico();
    InitUART(); //inicializo el puerto UART para enviar y recibir datos hacia y desde el ESP32 remoto. 

    gpio_set_direction(PinCaudalimetro, GPIO_MODE_INPUT); 


    gpio_set_direction(PinTrigger, GPIO_MODE_DEF_OUTPUT); 
    gpio_set_direction(PinEcho, GPIO_MODE_DEF_INPUT); 


    xTaskCreate(CalcularCaudal, "CalcularCaudal", 2048, NULL, 1, NULL);
    xTaskCreate(CalculoDistancia, "TareaUltrasonido", 2048, NULL, 1, NULL);
    //Creo la tarea que envia los datos a la terminal central con las lecturas de censores. 
    xTaskCreate(UART_EnviarLecturas, "TareaUART_EnviarLecturas", 2048, NULL, 1, NULL);

    /*while (1)
    {

        printf("Caudal =[%f], Distancia=[%f]\n", caudalHora, distancia);
        vTaskDelay(1000/ portTICK_PERIOD_MS);

    }
    */
}