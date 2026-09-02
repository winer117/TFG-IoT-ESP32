/*
   
  NODO AMBIENTAL - ESP32 #3
   
  
  DESCRIPCION GENERAL:
  Este nodo funciona como un vigía a batería. Pasa la mayor parte de su vida 
  en un estado de hibernación profunda (Deep Sleep) para ahorrar energía. 
  Solo despierta por dos razones:
  1. El sensor PIR detecta que alguien ha entrado a la habitación.
  2. Han pasado 15 minutos y es su turno programado para reportar el clima.
  
  CARACTERISTICAS PRINCIPALES:
  - Autoprotección (Watchdog): Si el sensor de temperatura falla y congela el 
  - sistema, un temporizador interno reinicia la placa automáticamente a los 10s.
  - Memoria RTC (Real Time Clock): Guarda el último clima válido y el número de 
  - mensaje en una memoria especial que no se borra mientras el procesador duerme.
  - Monitor de Batería: Lee su propio voltaje para avisar al Gateway cuándo necesita carga.
  - Transmisión Efímera: Se despierta, lee, envía los datos por ESP-NOW y 
  - se vuelve a dormir en una fracción de segundo.
   
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include <DHT.h>
#include <esp_task_wdt.h> // Librería para el Watchdog (evita bloqueos del código)

//    CONFIGURACION DE TIEMPOS Y PROTECCION   
#define INTERVALO_SLEEP_SEGUNDOS 900 // El nodo dormirá por 15 minutos (900 segundos)
#define uS_TO_S_FACTOR 1000000ULL    // Factor matemático para convertir microsegundos a segundos
#define WDT_TIMEOUT 10               // Si el procesador se traba más de 10 segundos, se reinicia solo

//    MEMORIA QUE SOBREVIVE AL SUEÑO (RTC)   
// Las variables normales se borran al entrar en Deep Sleep. Al ponerles "RTC_DATA_ATTR", 
// logramos que su valor se conserve intacto para la próxima vez que despierte.
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR float ultimaTempValida = 25.0;
RTC_DATA_ATTR float ultimoVoltaje = 3.7;
RTC_DATA_ATTR int numeroMensaje = 0; 

//    CONFIGURACION DE LA BATERIA   
#define PIN_BATERIA 34
#define V_MAX 4.2 // Voltaje de una celda de litio completamente cargada
#define V_MIN 3.3 // Voltaje mínimo de seguridad antes de apagar el equipo
#define FACTOR_CALIBRACION 1.008 

//    PARAMETROS DE COMUNICACION Y SENSORES   
#define FIXED_CHANNEL 6
#define ID_SENSOR 1
#define DHTPIN 4
#define PIR_PIN 27
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

// Dirección física (MAC) del Bridge receptor al que enviaremos los datos
uint8_t bridgeMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0};

// Estructura o "caja" donde guardaremos todos los datos antes de enviarlos por el aire (Esp-NOW)
struct DatosSensor {
    int idSensor;
    float temperatura;
    int movimiento;
    float voltajeBateria;
    int porcentajeBateria;
    unsigned long tiempo;
    int msgID;
};

DatosSensor datos;

// Banderas para confirmar si el mensaje llegó a su destino
volatile bool envioCompletado = false;
volatile bool envioExitoso = false;

//    ACUSES DE RECIBO (CALLBACKS)   
// Esta función se ejecuta automáticamente en cuanto el mensaje sale de la antena.
// Nos avisa si el Bridge lo recibió correctamente o si se perdió en el camino.
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    envioExitoso = (status == ESP_NOW_SEND_SUCCESS);
    envioCompletado = true;
}

// Función vacía requerida por la librería, aunque este nodo solo envía datos, no los recibe
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {}

//    LECTURA DEL SENSOR DE PRESENCIA (PIR)   
void leerPIR() {
    int estadoPIR = digitalRead(PIR_PIN);
    
    // Le preguntamos al procesador cuál fue el motivo por el que se despertó
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    
    // Si el pin está en HIGH o si la causa del despertar fue precisamente este pin (EXT0)
    if (estadoPIR == HIGH || cause == ESP_SLEEP_WAKEUP_EXT0) {
        datos.movimiento = 1;
        Serial.println(" - Deteccion de Movimiento: SI");
    } else {
        datos.movimiento = 0;
        Serial.println(" - Deteccion de Movimiento: NO");
    }
}

//    LECTURA DEL CLIMA (PROTEGIDA CONTRA FALLOS)   
bool leerDHT22() {
    delay(2000); // El DHT22 necesita al menos 2 segundos para encender y estabilizarse
    
    unsigned long inicioDHT = millis();
    
    // Le damos 3 oportunidades al sensor en caso de que la lectura falle
    for (int i = 0; i < 3; i++) {
        // Acariciamos al perro guardián para que no nos reinicie mientras esperamos al sensor
        esp_task_wdt_reset(); 
        
        float temp = dht.readTemperature();
        
        // Verificamos que el valor sea un número válido y esté dentro de rangos lógicos terrestres
        if (!isnan(temp) && temp > -40 && temp < 80) {
            datos.temperatura = temp;
            ultimaTempValida = temp; // Guardamos este dato en la memoria RTC como respaldo seguro
            Serial.printf(" - Temperatura: %.1f C\n", temp);
            return true;
        }
        delay(100); // Pequeña pausa antes del próximo intento
    }
    
    // Si los 3 intentos fallaron, usamos el último clima conocido para no enviar basura a la nube
    datos.temperatura = ultimaTempValida;
    Serial.println(" [AVISO] Fallo de lectura DHT. Rescatando ultima temperatura valida de la memoria RTC.");
    return false;
}

//    LECTURA Y CALCULO DE BATERIA   
float leerVoltajeBateria() {
    long suma_mV = 0;
    
    // Tomamos 10 lecturas rápidas y las promediamos para eliminar ruidos eléctricos
    for (int i = 0; i < 10; i++) { 
        suma_mV += analogReadMilliVolts(PIN_BATERIA); 
        delay(1); 
    }
    
    float pinVoltage = (suma_mV / 10) / 1000.0;
    
    // Multiplicamos por el valor de nuestro divisor de voltaje físico para conocer el voltaje real
    float bateriaVoltage = pinVoltage  4.7037  FACTOR_CALIBRACION;
    
    // Ponemos un tope matemático para evitar que un pico de voltaje nos dé "105%" de batería
    if (bateriaVoltage > 4.25) bateriaVoltage = 4.2; 
    
    ultimoVoltaje = bateriaVoltage;
    return bateriaVoltage;
}

int calcularPorcentajeBateria(float voltaje) {
    if (voltaje >= V_MAX) return 100;
    if (voltaje <= V_MIN) return 0;
    
    // Fórmula de regla de tres simple para calcular el porcentaje restante
    return (int)((voltaje - V_MIN) / (V_MAX - V_MIN)  100);
}

//    INICIALIZACION DE LA RED INALAMBRICA   
void configurarESPNow() {
    // Para ESP-NOW debemos configurar el módulo WiFi en modo Estación, aunque no nos conectemos a un router
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println(" [ERROR] Fallo critico en la inicializacion de ESP-NOW. Reiniciando...");
        ESP.restart();
    }
    
    esp_now_register_send_cb(OnDataSent);
    
    // Vinculamos la dirección MAC de nuestro destino (Bridge)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, bridgeMAC, 6);
    peerInfo.channel = FIXED_CHANNEL;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

//    RUTINA DE TRANSMISION HACIA EL BRIDGE   
void enviarDatos() {
    numeroMensaje++;
    datos.idSensor = ID_SENSOR;
    datos.tiempo = millis();
    datos.msgID = numeroMensaje;
    
    // El sistema intentará enviar el paquete hasta 3 veces si no recibe confirmación
    for (int i = 1; i <= 3; i++) {
        esp_task_wdt_reset(); // Le decimos al watchdog que seguimos vivos
        
        envioCompletado = false;
        envioExitoso = false;
        
        esp_now_send(bridgeMAC, (uint8_t*)&datos, sizeof(datos));
        
        // Esperamos un máximo de 150 milisegundos para ver si el mensaje llegó
        unsigned long inicio = millis();
        while (!envioCompletado && (millis() - inicio < 150)) {
            delay(1);
        }
        
        if (envioExitoso) break; // Si ya llegó, salimos del ciclo de reintentos
        if (i < 3) delay(150);   // Si falló, esperamos un instante antes de reintentar
    }
}

//    MOTOR DE ARRANQUE Y EJECUCION   
// En los nodos de Deep Sleep, TODO el trabajo se hace en el setup().
void setup() {
    Serial.begin(115200);
    bootCount++;

    // Activamos el "perro guardián". Si el código se traba en alguna parte, reiniciará la placa a los 10s.
    esp_task_wdt_init(WDT_TIMEOUT, true); 
    esp_task_wdt_add(NULL); 

    pinMode(PIR_PIN, INPUT);
    dht.begin();
    
    Serial.println("\n[SISTEMA] Nodo Ambiental activado. Obteniendo telemetria...");
    
    // Realizamos las lecturas
    leerPIR(); 
    leerDHT22();
    datos.voltajeBateria = leerVoltajeBateria();
    datos.porcentajeBateria = calcularPorcentajeBateria(datos.voltajeBateria);
    
    Serial.printf(" - Bateria: %.2fV (%d%%)\n", datos.voltajeBateria, datos.porcentajeBateria);
    
    // Encendemos la antena de radio, disparamos los datos y evaluamos el resultado
    configurarESPNow();
    enviarDatos();
    
    // Le indicamos al reloj interno (RTC) que nos despierte tras 15 minutos
    esp_sleep_enable_timer_wakeup(INTERVALO_SLEEP_SEGUNDOS  uS_TO_S_FACTOR);
    
    // Si la transmisión fue exitosa, configuramos el pin del PIR para que nos despierte de 
    // emergencia si detecta que alguien entra a la habitación.
    if (envioExitoso) {
        rtc_gpio_pullup_dis(GPIO_NUM_27);
        rtc_gpio_pulldown_en(GPIO_NUM_27);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_27, HIGH);
    }
    
    Serial.println("\n[SISTEMA] Ciclo completado. Desconectando red e iniciando Deep Sleep.");
    Serial.flush();
    
    // Apagamos los periféricos de radio para evitar fugas de corriente mientras duerme
    esp_now_deinit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    
    // Cortamos la energía del procesador. El sistema se detiene aquí.
    esp_deep_sleep_start();
}

// El ciclo loop() queda completamente vacío, porque la placa se "apaga" en la última línea del setup() 
// y, al despertar, volverá a ejecutar el setup() desde el principio.
void loop() {
}