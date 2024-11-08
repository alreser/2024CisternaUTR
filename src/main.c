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
#include <esp_system.h>

//Declaración de variables globales e inicializar funciones
static const char*  TAG = "2024CisternaUTRs";


#define PinCaudalimetro 35
#define PinTrigger 32
#define PinEcho 33
#define DistanciaMaximaCM 500 // 450 + ~10%

//Declaracion de variables globales 
long caudalContador=1; 
float caudaltemporal=0;
float caudalHora=0; //Indica la cantidad de litros que salieron del tanque 
static long indiceSegundo = 0;
uint64_t pulsosPorSegundo[3600] = {0}; // Asignación dinámica

volatile uint64_t  tiempoInicio=0;
volatile uint64_t  tiempoFin=0;
bool triggerDisponible = true;
float distancia=0; // indica la cantida de litros dentro del tanque. Nivel. 
uint64_t duracion=0;
float timeout = 0; //microsegundos

void ISRCaudalimetro(void *args)
{
    caudalContador++;
    //printf("CaudalContador =[%li]\n", caudalContador);
}

void CalcularCaudal(void *args)
{

        float caudalAuxiliar = 0;    

        while(1){

        caudaltemporal = caudalContador * 2.25 / 1000; //2,25ml por pulso
        
        caudalContador = 0; //reset contador
        
        pulsosPorSegundo[indiceSegundo] = caudaltemporal;  //sumatoria de L en la ultima hora

        indiceSegundo = (indiceSegundo + 1) % 3600;  // buffer temporal
        //printf("indiceSegundo =[%li]\n", indiceSegundo);

        for (int i = 0 ; i < 3600 ; i++){
            caudalAuxiliar += pulsosPorSegundo[i];  //auxiliar para evitar que caudalhora alguna vez sea 0
        }

        caudalHora = caudalAuxiliar;
        
        caudalAuxiliar = 0;

        //printf("Caudal =[%f]\n", caudalHora);

        vTaskDelay(2000/ portTICK_PERIOD_MS); 
    }
}

void IRAM_ATTR ISREcho(void *args) //IRAM_ATTR lo ejecuta en RAM en vez de ROM
{
    printf("3\n");
    if (gpio_get_level(PinEcho)) { // Flanco ascendente
        tiempoInicio = esp_timer_get_time(); // Guarda el tiempo de inicio


    } else { // Flanco descendente
        tiempoFin = esp_timer_get_time(); // Guarda el tiempo de finalización

        if (tiempoFin > tiempoInicio) {
            duracion = tiempoFin - tiempoInicio; // Duración del pulso en microsegundos
            distancia = (duracion / 1000000) * 343000 / 2; // Distancia (milimetros) = duracion en segundos * 343000mm/s /2
            printf("duracion=[%li]  Distancia=[%f]\n", caudalContador,distancia);
        }
    }
}

void CalculoDistancia(void *args)
{
    timeout = DistanciaMaximaCM * 2 * 10000 / 343;  // cm * 2 *m/100cm *s/343m *1000000us/s
    float tiempoLimite = 0; // Tiempo límite para el timeout

    while (1)
    {    
        
        if(!gpio_get_level(PinEcho)){

            gpio_set_level(PinTrigger, 1); // Activa el Trigger
            esp_rom_delay_us(15); // Mantiene el Trigger activo por al menos 10 microsegundos
            gpio_set_level(PinTrigger, 0); // Desactiva el Trigger

            printf("Trigger: %d. Echo: %d\n", gpio_get_level(PinTrigger), gpio_get_level(PinEcho));
            
            tiempoLimite = esp_timer_get_time() + timeout; // Timeout en microsegundos

            // Espera a que el Echo regrese o a que se alcance el tiempo máximo
            while (gpio_get_level(PinEcho) && esp_timer_get_time() < tiempoLimite) {
                printf("Tiempo Limite =[%f]. Yo atrapado en while\n", tiempoLimite);
                // Espera hasta que el Echo vuelva a 0 o se haya alcanzado el timeout
            }

            if (esp_timer_get_time() >= tiempoLimite) {
                distancia = -1; // Resolver que hacer si tenemos un timeout
                printf("Tiempo Limite sobrepasado =[%f]\n", tiempoLimite);
            }

        }

        vTaskDelay(500/ portTICK_PERIOD_MS); // Espera medio segundo antes de la siguiente medición
    
    }
}

void InitCaudalimetroISR()
{
    gpio_config_t gpioCaudalimetro;
    gpioCaudalimetro.pin_bit_mask = (1ULL << PinCaudalimetro); 
    gpioCaudalimetro.mode = GPIO_MODE_INPUT;
    gpioCaudalimetro.pull_up_en = GPIO_PULLUP_ENABLE;
    gpioCaudalimetro.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpioCaudalimetro.intr_type = GPIO_INTR_ANYEDGE; // Detecta flanco de subida. y de bajada
    gpio_config(&gpioCaudalimetro);

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
    echoConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    echoConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    echoConfig.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&echoConfig);


    gpio_isr_handler_add(PinEcho, ISREcho, NULL);
}

void app_main() 
{
    gpio_install_isr_service(0);
    InitCaudalimetroISR();
    InitUltrasonico();


    gpio_reset_pin(PinCaudalimetro);
    gpio_set_direction(PinCaudalimetro, GPIO_MODE_INPUT); 

    gpio_reset_pin(PinTrigger);
    gpio_set_direction(PinTrigger, GPIO_MODE_DEF_OUTPUT); 
    gpio_reset_pin(PinEcho);
    gpio_set_direction(PinEcho, GPIO_MODE_DEF_INPUT); 

    xTaskCreate(CalcularCaudal, "CalcularCaudal", 2048, NULL, 1, NULL);
    xTaskCreate(CalculoDistancia, "TareaUltrasonido", 2048, NULL, 1, NULL);


    /*while (1)
    {

        printf("Caudal =[%f], Distancia=[%f]\n", caudalHora, distancia);
        vTaskDelay(1000/ portTICK_PERIOD_MS);

    }
    */
}