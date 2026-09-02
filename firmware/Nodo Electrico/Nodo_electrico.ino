/*
  
  NODO ELÉCTRICO - ESP32 #5
  
  
  DESCRIPCION GENERAL:
  Este nodo funciona como el medidor inteligente de la casa. Su trabajo es 
  leer el voltaje y la corriente de la red eléctrica usando un chip de precisión 
  llamado ADS1115 (conectado a las pinzas SCT-013 y al transformador ZMPT101B). 
  Una vez que calcula el consumo en tiempo real, envía estos datos sin cables 
  (usando ESP-NOW) hacia el Bridge principal para que el sistema los procese.
  
  CARACTERISTICAS PRINCIPALES:
  - Sistema de autoprotección (Watchdog): Si el código llega a congelarse por 
  -  algún fallo en el bus I2C o en la red, el chip se reinicia solo para evitar 
  -  que se quede pasmado.
  - Escudo de memoria RAM: Monitorea constantemente los recursos del microcontrolador 
  -  y ejecuta una limpieza preventiva si detecta fugas de memoria.
  - Memoria RTC: Guarda el número de secuencia de los mensajes en una zona especial 
  -  que sobrevive a los reinicios, logrando que el conteo no se pierda si ocurre un apagón.
  - Lectura directa: Interpreta la señal analógica de la pinza amperimétrica sin 
  -  necesidad de soldar resistencias externas adicionales en la placa.
  
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <esp_task_wdt.h>  // Librería nativa del ESP32 para el control de bloqueos
#include <rom/rtc.h>       // Control de reinicios a nivel de hardware

// Memoria RTC: Mantiene vivo el número de mensaje aunque la placa se reinicie por completo
RTC_DATA_ATTR int rtc_bootCount = 0;
RTC_DATA_ATTR int rtc_numeroMensaje = 0;

// Definición de los pines lógicos en el ADS1115
#define CH_SCT013   0   // Canal analógico A0 para la corriente del hogar
#define CH_ZMPT101B 2   // Canal analógico A2 para el voltaje de la red

// Constantes de calibración de los sensores eléctricos
#define CORRIENTE_MAX 50.0
#define VOLT_FACTOR 550.0             
#define AMP_FACTOR 50.0             // Relación: 50 amperios por cada voltio RMS que entrega la pinza

// Factor de potencia predeterminado para estimar la potencia activa real
#define FACTOR_POTENCIA_FIJO 0.95  

// Parámetros de seguridad y rendimiento
#define WDT_TIMEOUT_SECONDS 8       // Tiempo máximo de espera antes de activar el reinicio de seguridad (8 segundos)
#define UPTIME_MAX_MS 604800000UL   // 7 días de funcionamiento continuo antes de un mantenimiento preventivo

// Parámetros de la red inalámbrica de bajo alcance (ESP-NOW)
#define FIXED_CHANNEL 6
#define ID_SENSOR 5
#define INTERVALO_ENVIO_MS 8000     // Frecuencia de transmisión de paquetes (cada 8 segundos)

// Dirección física MAC del dispositivo receptor (Bridge ESP32 #2)
uint8_t bridgeMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x0};

// Estructura que empaqueta toda la telemetría eléctrica para enviarla por radiofrecuencia
struct DatosElectricos {
  int idSensor;
  float voltajeRMS;
  float corrienteRMS;
  float potencia;
  float factorPotencia;
  unsigned long tiempo;
  int msgID;
};

DatosElectricos datosElectricos;
int numeroMensaje = 0;
bool esperandoACK = false;

// Instancia global para controlar el conversor analógico-digital de 16 bits
Adafruit_ADS1115 ads;

// Función de auditoría: Protege la estabilidad del hardware supervisando la memoria libre
void auditarSaludHardware() {
    uint32_t ramLibre = ESP.getFreeHeap();
    unsigned long uptime = millis();

    // Si la memoria RAM libre baja de 20 KB, realizamos un reinicio controlado para liberar recursos
    if (ramLibre < 20000) {
        Serial.println("\n[ALERTA DE SISTEMA] Memoria RAM en niveles críticos. Reiniciando para recuperar estabilidad.");
        delay(500);
        ESP.restart();
    }

    // Limpieza preventiva semanal para evitar que el controlador WiFi sufra degradación interna
    if (uptime > UPTIME_MAX_MS) {
        Serial.println("\n[MANTENIMIENTO PROGRAMADO] Limpieza preventiva de memoria por tiempo de actividad (7 días).");
        delay(500);
        ESP.restart();
    }
}

// Rutina principal de adquisición y cálculo de los valores eficaces (RMS)
void leerSensoresReales() {
  float voltaje_max = 0;
  float voltaje_min = 999;
  
  float corriente_max = 0;
  float corriente_min = 999;
  
  int muestras = 100;
  
  // Tomamos cien lecturas rápidas para evaluar el ciclo completo de la onda de corriente alterna
  for (int i = 0; i < muestras; i++) {
    // Leemos el voltaje instantáneo desde el canal del ZMPT101B
    int16_t adc = ads.readADC_SingleEnded(CH_ZMPT101B);
    float v = (adc  4.096) / 32768.0;
    
    if (v > voltaje_max) voltaje_max = v;
    if (v < voltaje_min) voltaje_min = v;
    
    // Leemos la corriente instantánea desde el canal del SCT-013
    int16_t adc_corriente = ads.readADC_SingleEnded(CH_SCT013);
    float voltaje_sct = (adc_corriente  4.096) / 32768.0;
    
    if (voltaje_sct > corriente_max) corriente_max = voltaje_sct;
    if (voltaje_sct < corriente_min) corriente_min = voltaje_sct;
    
    delay(1);
  }
  
  // Calculamos el voltaje RMS real utilizando la amplitud de pico detectada
  float voltaje_pico = (voltaje_max - voltaje_min) / 2;
  float voltajeRMS_calc = (voltaje_pico / sqrt(2))  VOLT_FACTOR;
  
  if (voltajeRMS_calc < 5.0) voltajeRMS_calc = 0;
  
  // Calculamos la corriente RMS aplicando el factor de conversión de la pinza
  float corriente_pico_voltaje = (corriente_max - corriente_min) / 2;
  float voltaje_sct_rms = corriente_pico_voltaje / sqrt(2);
  
  float corrienteRMS_calc = voltaje_sct_rms  AMP_FACTOR; 
  
  // Filtro de ruido: Si la corriente calculada es menor a 20 mA, la consideramos nula (cero)
  if (corrienteRMS_calc < 0.02) corrienteRMS_calc = 0;
  
  // Guardamos todos los valores calculados dentro de la estructura de datos
  datosElectricos.voltajeRMS = voltajeRMS_calc;
  datosElectricos.corrienteRMS = corrienteRMS_calc;
  datosElectricos.factorPotencia = FACTOR_POTENCIA_FIJO;

  // Calculamos la potencia activa multiplicando Voltaje por Corriente y por el Factor de Potencia
  datosElectricos.potencia = voltajeRMS_calc  corrienteRMS_calc  datosElectricos.factorPotencia;
  
  datosElectricos.idSensor = ID_SENSOR;
  datosElectricos.tiempo = millis();
}

// Funciones de acuse de recibo para confirmar que los datos llegaron al receptor
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS && esperandoACK) {
    Serial.println(" [Entregado]");
    esperandoACK = false;
  } else if (esperandoACK) {
    Serial.println(" [No entregado]");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == 4 && esperandoACK) {
    int ackID;
    memcpy(&ackID, incomingData, sizeof(ackID));
    if (ackID == datosElectricos.msgID) {
      esperandoACK = false;
    }
  }
}

// Configuración inicial del protocolo de comunicación inalámbrica ESP-NOW
void configurarESPNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("Canal de radiofrecuencia asignado: %d\n", FIXED_CHANNEL);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error crítico: No se pudo inicializar el módulo ESP-NOW.");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, bridgeMAC, 6);
  peerInfo.channel = FIXED_CHANNEL;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error: No se pudo registrar el dispositivo receptor (peer).");
    return;
  }
  
  Serial.println("Protocolo ESP-NOW configurado y enlazado con éxito.");
}

// Proceso para empaquetar, imprimir y disparar el envío de la telemetría por aire
void enviarDatos() {
  numeroMensaje++;
  
  leerSensoresReales();
  datosElectricos.msgID = numeroMensaje;
  
  Serial.printf("Paquete [#%d] | Voltaje: %.1fV | Corriente: %.2fA | Potencia: %.1fW | FP: %.2f\n",
                numeroMensaje,
                datosElectricos.voltajeRMS,
                datosElectricos.corrienteRMS,
                datosElectricos.potencia,
                datosElectricos.factorPotencia);
  
  esperandoACK = true;
  esp_now_send(bridgeMAC, (uint8_t*)&datosElectricos, sizeof(datosElectricos));
  
  unsigned long startWait = millis();
  while (esperandoACK && (millis() - startWait < 2000)) {
    // Mantenemos vivo al perro guardián mientras la placa espera la confirmación de la red
    esp_task_wdt_reset(); 
    delay(10);
  }
}

// Interfaz visual por consola para monitorear el estado actual del nodo
void mostrarEstado() {
  Serial.println("           RESUMEN OPERATIVO - NODO ELÉCTRICO            ");
  Serial.printf(" Total de reportes enviados: %d\n", numeroMensaje);
  Serial.printf(" Último nivel de voltaje: %.1f V\n", datosElectricos.voltajeRMS);
  Serial.printf(" Último nivel de corriente: %.2f A\n", datosElectricos.corrienteRMS);
  Serial.printf(" Potencia activa calculada: %.1f W\n", datosElectricos.potencia);
  Serial.printf(" Memoria RAM disponible: %u bytes\n", ESP.getFreeHeap());
  Serial.printf(" Canal inalábrico activo: %d\n", FIXED_CHANNEL);
  Serial.println("---------------------------------------------------------\n");
}

// Configuración inicial del sistema al encender o reiniciar la placa
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n--------------------------------------------------------");
  Serial.println("       NODO ELÉCTRICO - ESP32 #5 (MEDICIÓN REAL)         ");
  Serial.println("--------------------------------------------------------\n");
  
  // Verificamos si la placa se reinició debido a un fallo para recuperar el contador anterior
  esp_reset_reason_t reason = esp_reset_reason();
  bool reinicioPorFallo = (reason == ESP_RST_SW || reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT);

  if (reinicioPorFallo && rtc_bootCount > 0) {
      Serial.println("[RECUPERACIÓN] Restaurando el número de mensaje anterior desde la memoria RTC.");
      numeroMensaje = rtc_numeroMensaje;
  }
  rtc_bootCount++;

  // Inicialización del sistema de seguridad Watchdog adaptado a cualquier versión del núcleo ESP32
  Serial.println("Configurando el sistema de protección Watchdog...");
  
  #ifdef ESP_IDF_VERSION_MAJOR
    #if ESP_IDF_VERSION_MAJOR >= 5
      esp_task_wdt_config_t configWDT = {
          .timeout_ms = WDT_TIMEOUT_SECONDS  1000,
          .idle_core_mask = 0,
          .trigger_panic = true
      };
      esp_task_wdt_reconfigure(&configWDT);
    #else
      esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
    #endif
  #else
    esp_task_wdt_init(WDT_TIMEOUT_SECONDS, true);
  #endif

  esp_task_wdt_add(NULL); 
  Serial.println("Sistema Watchdog activado correctamente.");

  // Inicialización del conversor ADS1115 utilizando el bus I2C(protocolo de comunicación)
  Wire.begin();
  if (!ads.begin()) {
    Serial.println("ERROR CRÍTICO: No se encuentra conectado el chip ADS1115 en el bus I2C.");
    while (1); // El Watchdog reiniciará automáticamente el dispositivo si se queda atrapado aquí
  }
  
  ads.setGain(GAIN_ONE);  
  ads.setDataRate(RATE_ADS1115_860SPS);
  
  Serial.println("Conversor ADS1115 detectado e inicializado en la dirección 0x48.");
  
  configurarESPNow();
  
  Serial.printf("Dirección MAC de este nodo: %s\n", WiFi.macAddress().c_str());
  Serial.printf("Intervalo de transmisión configurado: cada %d segundos\n\n", INTERVALO_ENVIO_MS / 1000);
}

// Bucle repetitivo principal del microcontrolador
void loop() {
  // Acariciamos al perro guardián para demostrarle que el programa se ejecuta con normalidad
  esp_task_wdt_reset(); 
  
  // Comprobamos de manera constante que la memoria RAM y los tiempos de operación sean seguros
  auditarSaludHardware();
  
  static unsigned long lastSend = 0;
  static unsigned long lastStatus = 0;
  unsigned long ahora = millis();
  
  // Control de tiempo para disparar la lectura y el envío de datos eléctricos
  if (ahora - lastSend >= INTERVALO_ENVIO_MS) {
    lastSend = ahora;
    enviarDatos();
  }
  
  // Despliegue de un reporte resumido en la consola cada 30 segundos
  if (ahora - lastStatus >= 30000) {
    lastStatus = ahora;
    mostrarEstado();
  }
  
  // Sincronización continua del contador actual hacia la memoria RTC no volátil
  rtc_numeroMensaje = numeroMensaje;

  delay(10); 
}