/*
 EDGE CONTROLLER - ESP32-S3
 CEREBRO PRINCIPAL: INTELIGENCIA ARTIFICIAL, NUBE Y GESTION DE RED
  

 COMPONENTES Y CARACTERISTICAS:
 - Multitarea (Dual Core): Core 0 para red en tiempo real, Core 1 para IA y Telegram.
 - Inteligencia Artificial (TinyML): Inferencia de consumo electrico con TensorFlow Lite.
 - Actualizaciones Inalambricas (OTA): Descarga y actualiza el modelo neuronal automaticamente.
 - Integracion Cloud: Sincronizacion bidireccional con ThingSpeak mediante MQTT y HTTP.
 - Asistente Virtual: Interfaz conversacional (HITL) a traves de Telegram Bot.
 - Filtro Anti-Spam (Sala de Espera): Sistema de retencion en RAM para no saturar limites de la API.
 - Persistencia Segura: Almacenamiento de historial de consumo en LittleFS.
 - Watchdog Nativo: Reinicio automatico ante bloqueos de procesamiento por corrupcion de memoria.
*/

#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <map>
#include <Preferences.h>
#include <time.h>
#include <PubSubClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <LittleFS.h>
#include <esp_task_wdt.h> 

//   LIBRERIAS PARA TENSORFLOW LITE MICRO  
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"

//   VARIABLES GLOBALES PARA LA INTELIGENCIA ARTIFICIAL  
tflite::MicroErrorReporter tflErrorReporter;
tflite::ErrorReporter* errorReporter = &tflErrorReporter;
const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Tamano del Tensor Arena (Memoria RAM reservada para ejecucion del modelo)
constexpr int kTensorArenaSize = 40 * 1024;
uint8_t tensorArena[kTensorArenaSize];

// Buffer para leer el archivo desde LittleFS a la RAM
uint8_t* modelBuffer = nullptr;

//   CONFIGURACION DE RED  WIFI A LA QUE SE CONECTARA EL DISPOSITIVO 
const char* WIFI_SSID = "red_wifi";
const char* WIFI_PASS = "clave_wifi";

//   CONFIGURACION DE CANALES THINGSPEAK  
const char* THINGSPEAK_API_KEY_AMBIENTAL = "API_KEY_AMBIENTAL";
unsigned long CHANNEL_AMBIENTAL = 0000000;
const char* THINGSPEAK_API_KEY_ELECTRICO = "API_KEY_ELECTRICA";
unsigned long CHANNEL_ELECTRICO = 0000000;
const char* THINGSPEAK_API_KEY_ACTUADORES = "API_KEY_ACTUADORES";
unsigned long CHANNEL_ACTUADORES = 0000000;

//   CONFIGURACION MQTT PARA thingspeak
const char* mqtt_server = "mqtt3.thingspeak.com";
const int mqtt_port = 1883;
const char* mqtt_client_id = "MQTT_CLIENT_ID";
const char* mqtt_username  = "MQTT_USERNAME";
const char* mqtt_password  = "MQTT_PASSWORD";

const char* mqtt_publish_ambiental = "channels/0000000/publish";
const char* mqtt_subscribe_ambiental = "channels/0000000/subscribe";
const char* mqtt_publish_electrico = "channels/0000000/publish";
const char* mqtt_subscribe_electrico = "channels/0000000/subscribe";
const char* mqtt_publish_actuadores = "channels/0000000/publish";
const char* mqtt_subscribe_actuadores = "channels/0000000/subscribe";

//   LIBRERIAS Y CONFIGURACION DE TELEGRAM  
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

// Credenciales del Asistente Virtual
#define BOT_TOKEN "token_de_telegram"
#define CHAT_ID "chat_id"

// Clientes de red seguros para API de Telegram
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);

// Variables para la logica Human-in-the-Loop (Intervencion humana)
unsigned long tiempoSugerencia = 0;
bool esperandoRespuestaTelegram = false;
int accionSugerida = 0; 
const unsigned long TIMEOUT_RESPUESTA_MS = 600000; // 10 minutos de espera

String NOMBRE_USUARIO = "Usuario"; // Variable global para el perfil de Telegram

//   CONFIGURACION OTA (ACTUALIZACION DE MODELO ML)  
unsigned long tiempoAnteriorOTA = 0;
const unsigned long intervaloOTA = 604800000; // Ciclo de actualizacion cada 7 dias
const char* url_servidor_ota = "http://servidor_pythonanywhere.com/descargar-modelo";

//   PARAMETROS DE SALUD DEL SISTEMA  
#define HEARTBEAT_INTERVAL_MS 30000     // Cada 30 seg avisamos que seguimos vivos
#define NODE_TIMEOUT_MS 75000           // Si pasan 75 seg en silencio, declaramos emergencia
#define MAX_CONSECUTIVE_FAILURES 3      // Intentos maximos antes de rendirnos en un envio
#define TIEMPO_REINICIO_HORAS 4         // Reinicio preventivo para barrer la RAM y evitar cuelgues
#define TIEMPO_REINICIO_MS (TIEMPO_REINICIO_HORAS * 3600UL * 1000UL)

// La regla  de la cuenta gratuita de ThingSpeak: no subir datos mas rapido que 16 segundos
#define THINGSPEAK_MIN_INTERVAL_MS 16000

//   CONFIGURACION DEL RELOJ MUNDIAL (NTP)  
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;      // Zona horaria ajustada (Peru/Colombia)
const int daylightOffset_sec = 0;
#define HORA_INICIO_NOCHE 17            // 5 PM empieza el horario de monitoreo nocturno
#define HORA_FIN_NOCHE 2                // 2 AM termina
#define LAMPARA_TIMEOUT_MS (2UL * 60UL * 1000UL)

//   CONTROL DE FLUJO Y COMUNICACIONES  
#define TIEMPO_ESPERA_CONFIRMACION_MS 2000
#define MQTT_PUBLISH_INTERVAL 14000
#define TIEMPO_IGNORAR_ECO_MS 5000
#define SERIAL1_BAUDRATE 921600         // Velocidad extrema por cable hacia el Gateway
#define ACK_TIMEOUT_MS 3000
#define MAX_RETRIES 3

//   VARIABLES GLOBALES DE RED  
Preferences prefs;
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Banderas logicas para saber como estamos operando
bool wifiDisponible = false;
bool hayInternet = false;
bool modoLocal = false; // Se activa si el WiFi funciona pero se cae el proveedor de internet
bool mqttConectado = false;
bool horaInicializada = false;

// Cronometros para nuestras rutinas de chequeo
unsigned long lastWiFiCheck = 0;
unsigned long lastInternetCheck = 0;
unsigned long lastMQTTReconnect = 0;
unsigned long lastMQTTPublish = 0;
unsigned long lastProgrammedRestart = 0;
unsigned long lastStatus = 0;
unsigned long lastTimeSent = 0;

//   VARIABLES DEL PROTOCOLO DE DOBLE CONFIRMACION (UART ACK)  
String pendingCommand = "";             // Mensaje guardado en espera de confirmacion
unsigned long pendingCommandTime = 0;   // A que hora lo enviamos
int pendingRetries = 0;                 // Cuantos intentos llevamos
bool waitingForAck = false;             // Bandera de "estoy esperando respuesta"
bool ackReceived = false;

//   VARIABLES DEL FILTRO ANTI-ECO (MQTT)  
// Evita que el sistema reaccione dos veces a la misma orden
String ultimoMsgIdEnviado = "";
unsigned long tiempoUltimoMsgIdEnviado = 0;

// Memoria de lo ultimo que procesamos, para no enviar lo mismo si no hubo cambios
int lastProcessedField1 = -1;
int lastProcessedField2 = -1;
int lastProcessedField3 = -1;
int lastProcessedField4 = -1;
int lastProcessedField5 = -1;
int lastProcessedField6 = -1;

unsigned long lastMQTTCommandTime = 0;
String lastMQTTCommandTopic = "";

//   VARIABLES DE PRIORIDADES Y CONTROL  
bool lamparaControlManual = false;
bool ventiladorControlManual = false;
unsigned long lastManualCommand = 0;
#define MANUAL_TIMEOUT_MS 3600000 // Si activas algo manual, dura 1 hora maximo. Luego vuelve al automatico.

//   MEMORIA DE ESTADOS DE LA CASA  
int estadoLampara = 0;
int estadoVentilador = 0;

int ultimoLamparaPublicado = -1;
int ultimoVentiladorPublicado = -1;
int ultimoOrigenPublicado = -1;
unsigned long ultimoEnvioActuadores = 0;

//   MEMORIA DEL SENSOR DE CLIMA Y BATERIAS  
float ultimoTempPublicado = -999.0;
int ultimoMovPublicado = -1;
float ultimoVoltajePublicado = -1.0;
int ultimoPorcentajePublicado = -1;
unsigned long ultimoEnvioAmbiental = 0;

//   SALAS DE ESPERA RAM (FILTRO ANTI-BANEO)  
// Banderas para saber si tenemos gente esperando para subir a internet
bool ambientalPendiente = false;
float p_temp = 0.0; int p_mov = 0; float p_vBat = 0.0; int p_pBat = 0;

bool electricoPendiente = false;
float p_voltaje = 0.0; float p_corriente = 0.0; float p_potencia = 0.0; float p_fp = 0.0;

//   BUFFER CIRCULAR PARA EL HISTORIAL DE LUZ  
// Es como una libreta de 168 renglones (7 dias x 24 horas). 
// Cuando se llena, empezamos a borrar el primer renglon para anotar el nuevo.
#define HISTORIAL_SIZE 168
float consumptionHistory[HISTORIAL_SIZE];
int historyIndex = 0;
int historyCount = 0;
bool historyFull = false;

//   GUARDADO SEGURO EN DISCO (LITTLEFS)  
#define HISTORIAL_FILE "/historial.bin"
#define HISTORIAL_SAVE_INTERVAL 3600000 // Guardamos al disco duro cada hora exacta
#define HISTORIAL_SAVE_THRESHOLD 10

float lastSavedHistory[HISTORIAL_SIZE];
bool hayCambiosEnHistorial = false;
unsigned long lastHistorialSave = 0;

//   SUPERVISION DEL CABLEADO CON EL GATEWAY  
bool gatewayConectado = false;
unsigned long lastGatewayHeartbeat = 0;
#define GATEWAY_HEARTBEAT_TIMEOUT 60000 // Tolerancia maxima de silencio: 1 minuto

int estadoLamparaGateway = -1;
int estadoVentiladorGateway = -1;
int estadoLamparaActual = 0;
int estadoVentiladorActual = 0;

bool gatewayReinicioDetectado = false;
unsigned long lastGatewayReinicioLog = 0;

unsigned long ultimoEnvioElectrico = 0;

//   COLA DE REINTENTOS PARA LA NUBE  
bool hayPendiente = false;
int lamparaPendiente = -1;
int ventiladorPendiente = -1;
int origenPendiente = -1;
unsigned long tiempoPendiente = 0;
int intentosPendientes = 0;
#define MAX_INTENTOS_PENDIENTES 5

//   GESTION MULTINUCLEO (SEMAFOROS Y BUZONES)  
// Herramientas vitales para que el Core 0 y el Core 1 no colisionen al cruzar informacion
SemaphoreHandle_t xMutexVariables = NULL;      // Semaforo de control de trafico de variables
QueueHandle_t queueGatewayToCore1 = NULL;      // Buzon: Mensajes del Gateway hacia la IA
QueueHandle_t queueCore1ToGateway = NULL;      // Buzon: Ordenes de la IA hacia el Gateway
QueueHandle_t queueCore1ToMQTT = NULL;         // Buzon: Ordenes de la IA para subir a la Nube

//   INDICE DE PROTOTIPOS  
// Le avisamos al compilador todas las funciones que van a existir mas abajo en el codigo
void conectarWiFi();
void verificarWiFi();
void verificarInternet();
void configurarNTP();
void sincronizarHoraPeriodica();
void conectarMQTT();
void verificarConexionMQTT();
void suscribirMQTT();
bool publicarMQTT_Ambiental(float temp, int mov, float voltaje, int porcentaje);
bool publicarMQTT_Electrico(float voltaje, float corriente, float potencia, float fp);
void ejecutarEnvioPendiente();
void publicarThingSpeak(int lampara, int ventilador, int origen);
bool hayCambioReal(int lampara, int ventilador, int origen);
bool hayCambioAmbiental(float temp, int mov, float voltaje, int porcentaje);
void enviarAThingSpeak_Actuadores(int lampara, int ventilador, int origen);
void publicarThingSpeakDirecto(int lampara, int ventilador, int origen);
void publicarCambioModo(int estadoLampara, int estadoVentilador, int origen);
void mqttCallback(char* topic, byte* payload, unsigned int length);
void procesarDatosDesdeGateway(String datos);
void procesarACK(String datos);
void verificarReinicioProgramado();
void mostrarEstado();
void enviarHoraAlGateway();
int calcularOrigen();
void mostrarEstadoBitmask();
void updateConsumptionHistory(float consumo);
float getAverageConsumption(int hours);
void mostrarEstadoHistorial();
void saveHistorialToLittleFS();
bool loadHistorialFromLittleFS();
void autoSaveHistorial();
bool hasHistoryChanged();
void checkGatewayHeartbeat();
void procesarHeartbeatGateway(String datos);
void verificarYCorregirEstado(int lamparaEstado, int ventiladorEstado);
void sincronizarEstadoConGateway();
void registrarReinicioGateway();
bool publicarDatosElectricos(float voltaje, float corriente, float potencia, float fp);
bool descargarModeloOTA(); 

//   EL MEDICO DEL HARDWARE  
// Si empezamos a encolar muchos datos y la memoria RAM se esta asfixiando, 
// esta funcion prefiere reiniciar el equipo de forma limpia antes de que ocurra un colapso fatal.
void auditarSaludHardware() {
    uint32_t ramLibre = ESP.getFreeHeap();
    if (ramLibre < 15000) {
        Serial.printf("\n[SISTEMA CRITICO] Memoria RAM del Edge casi agotada (%u bytes). Ejecutando Reinicio Preventivo...\n", ramLibre);
        delay(500);
        ESP.restart();
    }
}

//   FUNCIONES DE LECTURA/ESCRITURA SEGURAS CON SEMAFOROS (MUTEX)  
// Todas estas funciones hacen lo mismo: antes de modificar una variable, toman el "Semaforo" (Mutex).
// Si el nucleo 0 tiene el semaforo, el nucleo 1 hace fila pacientemente y viceversa.
// Esto garantiza que jamas dos nucleos escriban en la misma variable al mismo microsegundo.

void setLamparaManual(bool estado) {
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            lamparaControlManual = estado;
            xSemaphoreGive(xMutexVariables); // Soltamos el semaforo
        }
    } else {
        lamparaControlManual = estado;
    }
}

bool getLamparaManual() {
    bool resultado = false;
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            resultado = lamparaControlManual;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        resultado = lamparaControlManual;
    }
    return resultado;
}

void setLamparaEstado(int estado) {
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            estadoLampara = estado;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        estadoLampara = estado;
    }
}

int getLamparaEstado() {
    int resultado = 0;
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            resultado = estadoLampara;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        resultado = estadoLampara;
    }
    return resultado;
}

void setVentiladorManual(bool estado) {
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            ventiladorControlManual = estado;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        ventiladorControlManual = estado;
    }
}

bool getVentiladorManual() {
    bool resultado = false;
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            resultado = ventiladorControlManual;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        resultado = ventiladorControlManual;
    }
    return resultado;
}

void setVentiladorEstado(int estado) {
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            estadoVentilador = estado;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        estadoVentilador = estado;
    }
}

int getVentiladorEstado() {
    int resultado = 0;
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            resultado = estadoVentilador;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        resultado = estadoVentilador;
    }
    return resultado;
}

void setLastManualCommand(unsigned long time) {
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            lastManualCommand = time;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        lastManualCommand = time;
    }
}

unsigned long getLastManualCommand() {
    unsigned long resultado = 0;
    if (xMutexVariables != NULL) {
        if (xSemaphoreTake(xMutexVariables, portMAX_DELAY) == pdTRUE) {
            resultado = lastManualCommand;
            xSemaphoreGive(xMutexVariables);
        }
    } else {
        resultado = lastManualCommand;
    }
    return resultado;
}

//   LOGICA COMPRIMIDA (BITMASK)  
// Esta funcion "comprime" los estados de toda la casa en un solo numero usando logica binaria
// Esto ahorra muchisimos datos de internet al subir a ThingSpeak
int calcularOrigen() {
    int lampara = getLamparaEstado();
    int ventilador = getVentiladorEstado();
    int origen = 0;
    
    if (lampara == 1) origen |= 1;
    if (ventilador == 1) origen |= 2;
    
    return origen;
}

//   LOS GUARDIANES DEL INTERNET (FILTROS DE CAMBIO)  
// Estas funciones deciden si vale la pena gastar el internet enviando un datos a la nube.
// Solo dan "luz verde" si la temperatura, bateria o algun foco cambio de verdad.
bool hayCambioReal(int lampara, int ventilador, int origen) {
    int lamparaActual = (lampara >= 0) ? lampara : getLamparaEstado();
    int ventiladorActual = (ventilador >= 0) ? ventilador : getVentiladorEstado();
    int origenActual = (origen >= 0) ? origen : calcularOrigen();
    
    // Si recien encendimos el equipo, forzamos la subida de los primeros datos
    if (ultimoLamparaPublicado == -1 && ultimoVentiladorPublicado == -1 && ultimoOrigenPublicado == -1) {
        return true;
    }
    
    bool cambioLampara = (lamparaActual != ultimoLamparaPublicado);
    bool cambioVentilador = (ventiladorActual != ultimoVentiladorPublicado);
    bool cambioOrigen = (origenActual != ultimoOrigenPublicado);
    
    return (cambioLampara || cambioVentilador || cambioOrigen);
}

bool hayCambioAmbiental(float temp, int mov, float voltaje, int porcentaje) {
    // Forzamos el primer envio tras arrancar
    if (ultimoTempPublicado == -999.0 && ultimoMovPublicado == -1 && 
        ultimoVoltajePublicado == -1.0 && ultimoPorcentajePublicado == -1) {
        return true;
    }
    
    // Solo permitimos el envio si la temperatura vario mas de 1.0 grados o el voltaje mas de 1.02V
    bool cambioTemp = (abs(temp - ultimoTempPublicado) >= 1.0);
    bool cambioMov = (mov != ultimoMovPublicado);
    bool cambioVoltaje = (abs(voltaje - ultimoVoltajePublicado) >= 1.02);
    bool cambioPorcentaje = (abs(porcentaje - ultimoPorcentajePublicado) >= 1);
    
    return (cambioTemp || cambioMov || cambioVoltaje || cambioPorcentaje);
}

//   LA LIBRETA DEL HISTORIAL DE CONSUMO  
// Cada vez que llega un dato del medidor de luz, lo anotamos aqui.
void updateConsumptionHistory(float consumo) {
    // Lo anotamos en el renglon actual
    consumptionHistory[historyIndex] = consumo;
    // Avanzamos al siguiente renglon (si llegamos a 168, volvemos a dar la vuelta al renglon 0)
    historyIndex = (historyIndex + 1) % HISTORIAL_SIZE;
    
    // Contamos cuantas paginas hemos llenado
    if (historyCount < HISTORIAL_SIZE) {
        historyCount++;
    }
    
    // Avisamos si la libreta ya se lleno por primera vez
    if (historyCount == HISTORIAL_SIZE) {
        historyFull = true;
    }
    
    hayCambiosEnHistorial = true;
    
    // Imprimimos en pantalla un reporte cada vez que cerramos 24 horas de datos
    if (historyCount % 24 == 0 && historyCount > 0) {
        Serial.printf("[HISTORIAL] %d registros locales almacenados. Buffer: %s\n", 
                      historyCount, historyFull ? "COMPLETO" : "EN PROCESO");
    }
}

// Esta funcion  recorre nuestra libreta y calcula nuestro gasto promedio
float getAverageConsumption(int hours) {
    if (historyCount == 0) return 0.0;
    
    int samples = min(hours, historyCount);
    int startIdx = (historyIndex - samples + HISTORIAL_SIZE) % HISTORIAL_SIZE;
    
    float sum = 0.0;
    for (int i = 0; i < samples; i++) {
        int idx = (startIdx + i) % HISTORIAL_SIZE;
        sum += consumptionHistory[idx];
    }
    
    return sum / samples; // Retorna el promedio exacto
}

// Sirve para depurar e imprimir el estado completo en la pantalla del monitor serie
void mostrarEstadoHistorial() {
    Serial.println("\n[INFO] Estado del Historial de Consumo:");
    Serial.printf(" - Registros almacenados: %d / %d\n", historyCount, HISTORIAL_SIZE);
    Serial.printf(" - Estado del buffer: %s\n", historyFull ? "COMPLETO" : "RECOLECTANDO");
    Serial.printf(" - Consumo promedio (24h): %.1f W\n", getAverageConsumption(24));
    Serial.printf(" - Consumo promedio (7 dias): %.1f W\n", getAverageConsumption(168));
    Serial.printf(" - Modificaciones sin guardar: %s\n", hayCambiosEnHistorial ? "SI" : "NO");
    Serial.printf(" - Ultima escritura a flash NVRAM: %lu ms\n", lastHistorialSave);
}

//   PERSISTENCIA NVRAM LITTLEFS  
bool hasHistoryChanged() {
    if (historyCount == 0) return false;
    for (int i = 0; i < historyCount; i++) {
        if (consumptionHistory[i] != lastSavedHistory[i]) {
            return true;
        }
    }
    return false;
}

void saveHistorialToLittleFS() {
    if (historyCount == 0 || !hayCambiosEnHistorial) return;
    
    File file = LittleFS.open(HISTORIAL_FILE, "w");
    if (!file) {
        Serial.println("[ERROR PERSISTENCIA] Imposible inicializar stream de escritura.");
        return;
    }
    
    file.write((uint8_t*)&historyIndex, sizeof(historyIndex));
    file.write((uint8_t*)&historyCount, sizeof(historyCount));
    file.write((uint8_t*)&historyFull, sizeof(historyFull));
    file.write((uint8_t*)consumptionHistory, sizeof(consumptionHistory));
    file.close();
    
    memcpy(lastSavedHistory, consumptionHistory, sizeof(consumptionHistory));
    hayCambiosEnHistorial = false;
    Serial.printf("[PERSISTENCIA] Volcado de historial completado: %d bloques.\n", historyCount);
}

bool loadHistorialFromLittleFS() {
    if (!LittleFS.exists(HISTORIAL_FILE)) return false;
    
    File file = LittleFS.open(HISTORIAL_FILE, "r");
    if (!file) return false;
    
    file.read((uint8_t*)&historyIndex, sizeof(historyIndex));
    file.read((uint8_t*)&historyCount, sizeof(historyCount));
    file.read((uint8_t*)&historyFull, sizeof(historyFull));
    file.read((uint8_t*)consumptionHistory, sizeof(consumptionHistory));
    file.close();
    
    memcpy(lastSavedHistory, consumptionHistory, sizeof(consumptionHistory));
    hayCambiosEnHistorial = false;
    Serial.printf("[PERSISTENCIA] Extraccion de historial completada: %d bloques recuperados.\n", historyCount);
    return true;
}

void autoSaveHistorial() {
    if (historyCount == 0) return;
    if (hasHistoryChanged() && (millis() - lastHistorialSave > HISTORIAL_SAVE_INTERVAL)) {
        saveHistorialToLittleFS();
        lastHistorialSave = millis();
    }
}

//   INTEGRIDAD DEL ENLACE UART (CONEXION POR CABLE AL GATEWAY)  

// Esta funcion vigila que el cable no se haya desconectado o el Gateway no se haya apagado.
void checkGatewayHeartbeat() {
    // Si ha pasado demasiado tiempo sin escuchar el "latido" del Gateway...
    if (millis() - lastGatewayHeartbeat > GATEWAY_HEARTBEAT_TIMEOUT) {
        if (gatewayConectado) {
            gatewayConectado = false; // Lo damos por muerto o desconectado
            Serial.println("[ALERTA UART] Senal de vida (Heartbeat) perdida. Gateway inaccesible.");
            
            // Le avisamos al nucleo 0 para que intente reconectar o lance una alerta
            char* buffer;
            asprintf(&buffer, "ALERTA:GATEWAY_REINICIADO:DESCONOCIDO");
            if (queueCore1ToMQTT != NULL) {
                if (xQueueSend(queueCore1ToMQTT, &buffer, 0) != pdTRUE) {
                    free(buffer);
                }
            } else {
                free(buffer);
            }
        }
    }
}

// Cuando el Gateway nos manda su estado actual (su latido), lo revisamos aqui
void procesarHeartbeatGateway(String datos) {
    if (datos.startsWith("HEARTBEAT:")) {
        // Desarmamos el mensaje para ver como tiene la lampara y el ventilador
        String valores = datos.substring(10);
        int primeraComa = valores.indexOf(':');
        int segundaComa = valores.indexOf(':', primeraComa + 1);
        
        if (primeraComa != -1 && segundaComa != -1) {
            int lamparaEstado = valores.substring(0, primeraComa).toInt();
            int ventiladorEstado = valores.substring(primeraComa + 1, segundaComa).toInt();
            
            // Actualizamos la hora del ultimo latido para saber que sigue vivo
            lastGatewayHeartbeat = millis();
            
            if (!gatewayConectado) {
                // Si estaba desconectado y volvio a hablar, nos alegramos y nos sincronizamos
                gatewayConectado = true;
                gatewayReinicioDetectado = true;
                lastGatewayReinicioLog = millis();
                Serial.println("[UART] Enlace con Gateway restablecido. Ejecutando apreton de manos (handshake)...");
                verificarYCorregirEstado(lamparaEstado, ventiladorEstado);
            } else {
                // Si ya estabamos conectados, verificamos que no estemos desincronizados.
                // Por ejemplo: si aqui creemos que la luz esta prendida pero el Gateway dice que esta apagada.
                if (lamparaEstado != estadoLamparaActual || ventiladorEstado != estadoVentiladorActual) {
                    Serial.printf("[ALERTA UART] Discrepancia de estados detectada: Gateway(L=%d,V=%d) | Edge(L=%d,V=%d)\n",
                                   lamparaEstado, ventiladorEstado, 
                                   estadoLamparaActual, estadoVentiladorActual);
                    // Forzamos al Gateway a que nos haga caso a nosotros (el cerebro)
                    sincronizarEstadoConGateway();
                }
            }
            estadoLamparaGateway = lamparaEstado;
            estadoVentiladorGateway = ventiladorEstado;
        }
    }
}

// Compara la realidad del hardware con lo que tenemos en memoria
void verificarYCorregirEstado(int lamparaEstado, int ventiladorEstado) {
    bool lamparaIncorrecta = (lamparaEstado != estadoLamparaActual);
    bool ventiladorIncorrecto = (ventiladorEstado != estadoVentiladorActual);
    
    // Si la lampara esta mal, le mandamos la orden correcta
    if (lamparaIncorrecta) {
        enviarComandoAlGateway("ACTUADOR:1:ESTADO:" + String(estadoLamparaActual));
    }
    // Si el ventilador esta mal, le mandamos la orden correcta
    if (ventiladorIncorrecto) {
        enviarComandoAlGateway("ACTUADOR:2:ESTADO:" + String(estadoVentiladorActual));
    }
    
    // Si tuvimos que corregir algo, lo reportamos a la nube
    if (lamparaIncorrecta || ventiladorIncorrecto) {
        char* buffer;
        asprintf(&buffer, "SINCRONIZACION:L:%d:V:%d", estadoLamparaActual, estadoVentiladorActual);
        if (queueCore1ToMQTT != NULL) {
            if (xQueueSend(queueCore1ToMQTT, &buffer, 0) != pdTRUE) {
                free(buffer);
            }
        } else {
            free(buffer);
        }
    }
}

// Impone nuestra autoridad: Le enviamos nuestros estados al Gateway para que los acate
void sincronizarEstadoConGateway() {
    enviarComandoAlGateway("ACTUADOR:1:ESTADO:" + String(estadoLamparaActual));
    enviarComandoAlGateway("ACTUADOR:2:ESTADO:" + String(estadoVentiladorActual));
    
    char* buffer;
    asprintf(&buffer, "SINCRONIZACION_FORZADA:L:%d:V:%d", estadoLamparaActual, estadoVentiladorActual);
    if (queueCore1ToMQTT != NULL) { 
        if (xQueueSend(queueCore1ToMQTT, &buffer, 0) != pdTRUE) free(buffer); 
    } else {
        free(buffer);
    }
}

// Si el Gateway se reinicio (ej. se fue la luz), le damos 60 segundos de gracia antes de limpiar la alerta
void registrarReinicioGateway() {
    if (!gatewayReinicioDetectado) return;
    if (millis() - lastGatewayReinicioLog > 60000) {
        gatewayReinicioDetectado = false;
        return;
    }
    char* buffer;
    asprintf(&buffer, "ALERTA:GATEWAY_REINICIADO:VERIFICAR_ACTUADORES");
    if (queueCore1ToMQTT != NULL) { 
        if (xQueueSend(queueCore1ToMQTT, &buffer, 0) != pdTRUE) free(buffer); 
    } else {
        free(buffer);
    }
}

//   PROTOCOLO DE TRANSMISION CON DOBLE CHECK (UART ACK)  

// Cuando le enviamos una orden al Gateway, no damos por hecho que llego. 
// Guardamos la orden y esperamos a que el Gateway nos responda "Recibido" (ACK).
void enviarComandoAlGateway(String comando) {
    if (comando.startsWith("ACTUADOR:1:ESTADO:")) {
        int estado = comando.substring(comando.lastIndexOf(":") + 1).toInt();
        estadoLamparaActual = estado;
    } else if (comando.startsWith("ACTUADOR:2:ESTADO:")) {
        int estado = comando.substring(comando.lastIndexOf(":") + 1).toInt();
        estadoVentiladorActual = estado;
    }
    
    // Si ya estabamos esperando confirmacion de otro mensaje, lo pisamos con este nuevo 
    // porque siempre damos prioridad a la instruccion mas reciente del usuario.
    if (waitingForAck) {
        Serial.println("[CONTROL] Resolucion de colision: Instruccion previa suspendida en favor del comando actual.");
    }
    
    // Guardamos el mensaje en la memoria temporal
    pendingCommand = comando;
    pendingCommandTime = millis();
    pendingRetries = 0;
    waitingForAck = true; // Levantamos la bandera de "esperando confirmacion"
    ackReceived = false;
    
    Serial.printf("[TX UART] Instruccion saliente: %s\n", comando.c_str());
    Serial1.println(comando);
    Serial1.flush(); // Empujamos los datos por el cable fisicamente
}

// Aqui revisamos las respuestas que nos llegan desde el cable
void procesarACK(String datos) {
    // Si el mensaje empieza con ACK, es el Gateway confirmando recepcion
    if (datos.startsWith("ACK:")) {
        String comandoConfirmado = datos.substring(4);
        comandoConfirmado.trim();
        
        Serial.printf("[RX UART] Confirmacion ACK recibida: %s\n", comandoConfirmado.c_str());
        
        if (waitingForAck) {
            // Verificamos que nos este confirmando el mismo mensaje que enviamos
            if (comandoConfirmado == pendingCommand) {
                waitingForAck = false; // Bajamos la bandera, todo salio bien
                ackReceived = true;
                pendingCommand = "";
                pendingRetries = 0;
            } else {
                Serial.printf("[ERROR UART] Falla en validacion. Esperabamos confirmar: %s | Nos confirmaron: %s\n", 
                              pendingCommand.c_str(), comandoConfirmado.c_str());
            }
        } else {
            // Si nos llega un ACK tardio o duplicado, lo limpiamos y seguimos
            ackReceived = true;
            pendingCommand = "";
            pendingRetries = 0;
        }
    }
}

// El vigilante de los envios: si pasa mucho tiempo sin respuesta, lo vuelve a mandar
void verificarTimeoutACK() {
    if (!waitingForAck) return;
    
    if (millis() - pendingCommandTime > ACK_TIMEOUT_MS) {
        pendingRetries++;
        
        // Si no hemos superado el limite de intentos, lo reenviamos
        if (pendingRetries <= MAX_RETRIES) {
            Serial.printf("[SISTEMA] Retransmitiendo instruccion (Intento %d de %d): %s\n", pendingRetries, MAX_RETRIES, pendingCommand.c_str());
            Serial1.println(pendingCommand);
            Serial1.flush();
            pendingCommandTime = millis(); // Reiniciamos el cronometro
        } else {
            // Si fallamos 3 veces, nos rendimos para no bloquear el sistema entero
            Serial.printf("[ERROR UART] Aborto de transmision tras %d fallos. Comando descartado: %s\n", MAX_RETRIES, pendingCommand.c_str());
            waitingForAck = false;
            pendingCommand = "";
            pendingRetries = 0;
        }
    }
}

//   GESTION DE SALA DE ESPERA (COLA MQTT PARA THINGSPEAK)  

// ThingSpeak solo permite subir datos cada 15 segundos en cuentas gratuitas.
// Esta funcion es el "portero" que manda los datos acumulados cuando ya es seguro enviarlos.
void ejecutarEnvioPendiente() {
    if (!hayPendiente) return; // Si no hay nadie en la sala de espera, no hacemos nada
    if (!wifiDisponible || !hayInternet) return;

    // Verificamos si realmente cambio algun foco o ventilador, para no gastar internet en vano
    if (!hayCambioReal(lamparaPendiente, ventiladorPendiente, origenPendiente)) {
        hayPendiente = false;
        lamparaPendiente = -1;
        ventiladorPendiente = -1;
        origenPendiente = -1;
        return;
    }

    if (!mqttClient.connected()) {
        conectarMQTT();
        if (!mqttClient.connected()) return;
    }

    // Creamos un numero de ticket unico para evitar que ThingSpeak nos cuente el mensaje dos veces (filtro anti-eco)
    String msgId = String(millis()) + "_" + String(random(1000, 9999));
    ultimoMsgIdEnviado = msgId;

    // Preparamos el paquete de datos uniendo los valores pendientes
    String payload = "";
    payload.reserve(100); // Reservamos memoria para no fragmentar la RAM
    if (lamparaPendiente >= 0) payload += "field1=" + String(lamparaPendiente);
    if (ventiladorPendiente >= 0) {
        if (payload.length() > 0) payload += "&";
        payload += "field2=" + String(ventiladorPendiente);
    }
    if (origenPendiente >= 0) {
        if (payload.length() > 0) payload += "&";
        payload += "field3=" + String(origenPendiente);
    }
    
    if (payload.length() == 0) {
        hayPendiente = false;
        return;
    }
    
    payload += "&status=" + msgId;
    Serial.printf("[MQTT TX] Procesando sala de espera. Paquete a enviar: %s\n", payload.c_str());
    
    // Disparamos la transmision hacia la nube
    bool resultado = mqttClient.publish(mqtt_publish_actuadores, payload.c_str());
    
    if (resultado) {
        // Si fue exitoso, guardamos estos valores como los ultimos conocidos
        if (lamparaPendiente >= 0) {
            ultimoLamparaPublicado = lamparaPendiente;
            lastProcessedField1 = lamparaPendiente;
        }
        if (ventiladorPendiente >= 0) {
            ultimoVentiladorPublicado = ventiladorPendiente;
            lastProcessedField2 = ventiladorPendiente;
        }
        if (origenPendiente >= 0) {
            ultimoOrigenPublicado = origenPendiente;
            lastProcessedField3 = origenPendiente;
        }
        
        // Vaciamos la sala de espera y anotamos la hora del envio
        ultimoEnvioActuadores = millis();
        hayPendiente = false;
        lamparaPendiente = -1;
        ventiladorPendiente = -1;
        origenPendiente = -1;
        Serial.println("[MQTT TX] Transmision asincrona finalizada exitosamente.");
    } else {
        // Si el puerto MQTT fallo, pasamos al Plan B: Mandarlo por HTTP clasico
        Serial.println("[MQTT TX] Fallo de comunicacion al socket. Evaluando alternativa HTTP...");
        enviarAThingSpeak_Actuadores(lamparaPendiente, ventiladorPendiente, origenPendiente);
    }
}

// Esta funcion recibe los datos pero en lugar de enviarlos al instante, 
// los sienta en la sala de espera hasta que ejecutarEnvioPendiente() los atienda.
void publicarThingSpeak(int lampara, int ventilador, int origen) {
    if (!hayCambioReal(lampara, ventilador, origen)) return;
    
    if (lampara >= 0) lamparaPendiente = lampara;
    if (ventilador >= 0) ventiladorPendiente = ventilador;
    if (origen >= 0) origenPendiente = origen;
    hayPendiente = true; // Avisamos que hay alguien esperando

    // Si ya paso el tiempo de castigo de ThingSpeak (16 segundos), los mandamos de una vez
    if (millis() - ultimoEnvioActuadores >= THINGSPEAK_MIN_INTERVAL_MS) {
        ejecutarEnvioPendiente();
    }
}

// A diferencia del anterior, esta funcion se salta la sala de espera y transmite directo.
// Solo se usa para cosas urgentes que no pueden esperar.
void publicarThingSpeakDirecto(int lampara, int ventilador, int origen) {
    if (!wifiDisponible || !hayInternet) return;

    if (!mqttClient.connected()) {
        conectarMQTT();
        if (!mqttClient.connected()) {
            // Si no conecto por MQTT, usamos el Plan B directo
            int lamparaActual = (lampara >= 0) ? lampara : getLamparaEstado();
            int ventiladorActual = (ventilador >= 0) ? ventilador : getVentiladorEstado();
            int origenActual = (origen >= 0) ? origen : calcularOrigen();
            enviarAThingSpeak_Actuadores(lamparaActual, ventiladorActual, origenActual);
            return;
        }
    }

    String msgId = String(millis()) + "_" + String(random(1000, 9999));
    ultimoMsgIdEnviado = msgId;

    int lamparaActual = (lampara >= 0) ? lampara : getLamparaEstado();
    int ventiladorActual = (ventilador >= 0) ? ventilador : getVentiladorEstado();
    int origenActual = (origen >= 0) ? origen : calcularOrigen();

    String payload;
    payload.reserve(120);
    payload = "field1=" + String(lamparaActual) +
              "&field2=" + String(ventiladorActual) +
              "&field3=" + String(origenActual) +
              "&status=" + msgId;

    bool resultado = mqttClient.publish(mqtt_publish_actuadores, payload.c_str());

    if (resultado) {
        ultimoLamparaPublicado = lamparaActual;
        lastProcessedField1 = lamparaActual;
        ultimoVentiladorPublicado = ventiladorActual;
        lastProcessedField2 = ventiladorActual;
        ultimoOrigenPublicado = origenActual;
        lastProcessedField3 = origenActual;
        
        ultimoEnvioActuadores = millis();
        Serial.println("[MQTT TX] Transmision directa (Bypass) completada.");
    } else {
        enviarAThingSpeak_Actuadores(lamparaActual, ventiladorActual, origenActual);
        ultimoEnvioActuadores = millis();
    }
}

// Sube especificamente a la nube cuando el sistema pasa a modo automatico
void publicarCambioModo(int estadoLampara, int estadoVentilador, int origen) {
    if (!wifiDisponible || !hayInternet) return;

    if (!mqttClient.connected()) {
        conectarMQTT();
        if (!mqttClient.connected()) return;
    }

    String msgId = String(millis()) + "_" + String(random(1000, 9999));
    ultimoMsgIdEnviado = msgId;

    int lamparaActual = (estadoLampara >= 0) ? estadoLampara : getLamparaEstado();
    int ventiladorActual = (estadoVentilador >= 0) ? estadoVentilador : getVentiladorEstado();
    int origenActual = (origen >= 0) ? origen : calcularOrigen();

    // Adjuntamos un texto avisando que es un cambio de modo
    String statusMsg = "Modo:AUTOMATICO|TS:" + String(millis());
    String payload;
    payload.reserve(150);
    payload = "field1=" + String(lamparaActual) +
              "&field2=" + String(ventiladorActual) +
              "&field3=" + String(origenActual) +
              "&status=" + statusMsg;

    bool resultado = mqttClient.publish(mqtt_publish_actuadores, payload.c_str());

    if (resultado) {
        ultimoLamparaPublicado = lamparaActual;
        lastProcessedField1 = lamparaActual;
        ultimoVentiladorPublicado = ventiladorActual;
        lastProcessedField2 = ventiladorActual;
        ultimoOrigenPublicado = origenActual;
        lastProcessedField3 = origenActual;
        
        ultimoEnvioActuadores = millis();
    }
}

//   CANAL HTTP SECUNDARIO (EL PLAN B POR SI FALLA MQTT)  

// A veces los routers de universidades o empresas bloquean los puertos MQTT.
// Si eso pasa, esta funcion hace exactamente lo mismo pero usando una direccion web 
// tradicional (HTTP GET), que nunca suele estar bloqueada.
void enviarAThingSpeak_Actuadores(int lampara, int ventilador, int origen) {
    if (!wifiDisponible) return;
    if (!hayCambioReal(lampara, ventilador, origen)) return; // Si no hay novedades, no gastamos internet
    
    HTTPClient http;
    // Le damos maximo 3 segundos al servidor para responder. Si se tarda mas, cancelamos 
    // la operacion para evitar que todo el procesador se quede congelado esperando.
    http.setTimeout(3000); 
    
    // Armamos la direccion web con nuestra contraseña y los datos a enviar
    String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY_ACTUADORES);
    if (lampara >= 0) url += "&field1=" + String(lampara);
    if (ventilador >= 0) url += "&field2=" + String(ventilador);
    if (origen >= 0) url += "&field3=" + String(origen);
    
    // Hacemos la llamada web
    http.begin(url);
    int httpCode = http.GET();
    http.end();
    
    // El codigo 200 en lenguaje de internet significa "OK, todo salio perfecto"
    if (httpCode == 200) { 
        if (lampara >= 0) { ultimoLamparaPublicado = lampara; lastProcessedField1 = lampara; }
        if (ventilador >= 0) { ultimoVentiladorPublicado = ventilador; lastProcessedField2 = ventilador; }
        if (origen >= 0) { ultimoOrigenPublicado = origen; lastProcessedField3 = origen; }
        
        ultimoEnvioActuadores = millis();
        // Como ya se enviaron, vaciamos la sala de espera
        hayPendiente = false;
        lamparaPendiente = -1;
        ventiladorPendiente = -1;
        origenPendiente = -1;
    } else {
        // Si la pagina nos dio error, igual reiniciamos nuestro cronometro para no 
        // bombardear al servidor de ThingSpeak y que nos terminen baneando la cuenta.
        ultimoEnvioActuadores = millis();
    }
}

void enviarAThingSpeak_Ambiental(float temp, int mov, float voltaje, int porcentaje) {
    if (!wifiDisponible) return;
    if (!hayCambioAmbiental(temp, mov, voltaje, porcentaje)) return;
    
    unsigned long ahora = millis();
    
    // Sala de espera local: Si no han pasado 16 segundos desde el ultimo envio,
    // ThingSpeak nos va a rebotar el mensaje. Asi que lo guardamos en la RAM 
    // y cortamos la funcion aqui. El ciclo principal lo enviara luego.
    if (ahora - ultimoEnvioAmbiental < THINGSPEAK_MIN_INTERVAL_MS) {
        ambientalPendiente = true;
        p_temp = temp; p_mov = mov; p_vBat = voltaje; p_pBat = porcentaje;
        return;
    }
    
    HTTPClient http;
    http.setTimeout(3000);
    String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY_AMBIENTAL);
    url += "&field1=" + String(temp, 1) + "&field2=" + String(mov) + "&field3=" + String(voltaje, 2) + "&field4=" + String(porcentaje);
    
    http.begin(url);
    int httpCode = http.GET();
    http.end();
    
    if (httpCode == 200) {
        ultimoTempPublicado = temp; ultimoMovPublicado = mov;
        ultimoVoltajePublicado = voltaje; ultimoPorcentajePublicado = porcentaje;
        ultimoEnvioAmbiental = millis();
        ambientalPendiente = false; // Vaciamos la sala de espera
    }
}

void enviarAThingSpeak_Electrico(float voltaje, float corriente, float potencia, float fp) {
    if (!wifiDisponible) return;
    
    unsigned long ahora = millis();
    
    // Misma logica de sala de espera para los datos de electricidad
    if (ahora - ultimoEnvioElectrico < THINGSPEAK_MIN_INTERVAL_MS) {
        electricoPendiente = true;
        p_voltaje = voltaje; p_corriente = corriente; p_potencia = potencia; p_fp = fp;
        return;
    }
    
    HTTPClient http;
    http.setTimeout(3000);
    String url = "http://api.thingspeak.com/update?api_key=" + String(THINGSPEAK_API_KEY_ELECTRICO);
    url += "&field1=" + String(voltaje, 1) + "&field2=" + String(corriente, 2) + "&field3=" + String(potencia, 1) + "&field4=" + String(fp, 2);
    
    http.begin(url);
    int httpCode = http.GET();
    http.end();
    
    if (httpCode == 200) {
        ultimoEnvioElectrico = millis();
        electricoPendiente = false; // Vaciamos la sala de espera
    }
}

//   CONTROL DE TRANSMISION DE DATOS AMBIENTALES Y ELECTRICOS  

bool publicarMQTT_Ambiental(float temp, int mov, float voltaje, int porcentaje) {
    if (!hayCambioAmbiental(temp, mov, voltaje, porcentaje)) {
        ultimoTempPublicado = temp; ultimoMovPublicado = mov;
        ultimoVoltajePublicado = voltaje; ultimoPorcentajePublicado = porcentaje;
        ambientalPendiente = false; 
        return true;
    }
    
    // Si no ha pasado el tiempo minimo de seguridad (16 seg) para ThingSpeak
    if (millis() - ultimoEnvioAmbiental < THINGSPEAK_MIN_INTERVAL_MS) {
        ambientalPendiente = true;
        p_temp = temp; p_mov = mov; p_vBat = voltaje; p_pBat = porcentaje;
        return true; 
    }

    if (!mqttClient.connected()) return false;
    
    String payload;
    payload.reserve(100);
    payload = "field1=" + String(temp, 1) + "&field2=" + String(mov) + "&field3=" + String(voltaje, 2) + "&field4=" + String(porcentaje);
    
    bool resultado = mqttClient.publish(mqtt_publish_ambiental, payload.c_str());
    if (resultado) {
        ultimoTempPublicado = temp; ultimoMovPublicado = mov;
        ultimoVoltajePublicado = voltaje; ultimoPorcentajePublicado = porcentaje;
        ultimoEnvioAmbiental = millis(); 
        ambientalPendiente = false;
        delay(50);
    }
    return resultado;
}

bool publicarMQTT_Electrico(float voltaje, float corriente, float potencia, float fp) {
    if (!mqttClient.connected()) return false;
    
    String payload;
    payload.reserve(100);
    payload = "field1=" + String(voltaje, 1) + "&field2=" + String(corriente, 2) + "&field3=" + String(potencia, 1) + "&field4=" + String(fp, 2);
    
    bool resultado = mqttClient.publish(mqtt_publish_electrico, payload.c_str());
    if (resultado) { delay(50); }
    return resultado;
}

bool publicarDatosElectricos(float voltaje, float corriente, float potencia, float fp) {
    if (!wifiDisponible || !hayInternet) return false;
    
    // Lo sentamos en la sala de espera si no ha pasado el limite de seguridad de ThingSpeak
    if (millis() - ultimoEnvioElectrico < THINGSPEAK_MIN_INTERVAL_MS) {
        electricoPendiente = true;
        p_voltaje = voltaje; p_corriente = corriente; p_potencia = potencia; p_fp = fp;
        return true;
    }
    
    // Comprobamos variaciones significativas para no gastar envios en vano
    static float ultimoVoltaje = -1.0; static float ultimoCorriente = -1.0; static float ultimoPotencia = -1.0;
    bool cambioVoltaje = (abs(voltaje - ultimoVoltaje) >= 1.0);     
    bool cambioCorriente = (abs(corriente - ultimoCorriente) >= 0.05); 
    bool cambioPotencia = (abs(potencia - ultimoPotencia) >= 5.0);    
    
    if (ultimoVoltaje != -1.0 && !cambioVoltaje && !cambioCorriente && !cambioPotencia) {
        electricoPendiente = false;
        return true;
    }
    
    ultimoVoltaje = voltaje; ultimoCorriente = corriente; ultimoPotencia = potencia;
    bool resultado = publicarMQTT_Electrico(voltaje, corriente, potencia, fp);
    
    if (resultado) {
        ultimoEnvioElectrico = millis();
        electricoPendiente = false;
    } else {
        // Fallback a HTTP si falla MQTT
        enviarAThingSpeak_Electrico(voltaje, corriente, potencia, fp);
        ultimoEnvioElectrico = millis(); 
        electricoPendiente = false;
        resultado = true;
    }
    return resultado;
}

//   CONDUCTOR DE MENSAJES MQTT ENTRANTES (CALLBACK)  
// Esta funcion es como el cartero. Se dispara automaticamente cada vez 
// que alguien toca un boton en la pagina web o en la aplicacion movil.
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Primero, traducimos los bytes puros que nos llegan de internet a un texto legible
    String incomingMessage = "";
    for (unsigned int i = 0; i < length; i++) {
        incomingMessage += (char)payload[i];
    }
    
    // ThingSpeak a veces nos reenvia nuestro propio mensaje (Eco). 
    // Para evitar que la casa se vuelva loca prendiendo y apagando cosas,
    // buscamos si el mensaje trae un numero de ticket ("&status=") que nosotros le pusimos.
    String msgId = "";
    String mensajeLimpio = incomingMessage;
    int idxStatus = incomingMessage.indexOf("&status=");
    if (idxStatus != -1) {
        msgId = incomingMessage.substring(idxStatus + 8);
        mensajeLimpio = incomingMessage.substring(0, idxStatus); // Cortamos la parte util
    }
    
    Serial.printf("[MQTT RX] Topico [%s] | Buffer: %s\n", topic, mensajeLimpio.c_str());
    String topicStr = String(topic);
    
    // FILTRO ANTI-ECO: Si el ticket que llego es exactamente el mismo ticket 
    // que nosotros enviamos hace un rato, lo ignoramos por completo.
    if (msgId.length() > 0 && msgId == ultimoMsgIdEnviado) return;
    
    // Si el mensaje viene por el canal de los Actuadores...
    if (topicStr.indexOf("actuadores") >= 0 || topicStr.indexOf(String(CHANNEL_ACTUADORES)) >= 0) {
        int valor = mensajeLimpio.toInt(); // Convertimos el texto a un numero entero
        unsigned long ahora = millis();
        
        // Si acabamos de enviar algo hace menos de 10 segundos, asumimos que cualquier 
        // cosa que llegue ahora es un reflejo (eco) de la red y lo marcamos.
        bool esEcoPorTiempo = (ahora - ultimoEnvioActuadores < 10000);

        lastMQTTCommandTime = ahora;
        lastMQTTCommandTopic = topicStr;
        
        // --- CASO 1: El usuario apreto el boton de la Lampara en la Web (Field1) ---
        if (topicStr.endsWith("field1")) {
            bool debeProcesar = false;
            
            // Solo lo procesamos si no es un eco y si el estado realmente cambio
            if (!esEcoPorTiempo) {
                debeProcesar = (valor != lastProcessedField1);
            }
            
            if (debeProcesar) {
                lastProcessedField1 = valor;
                Serial.printf("[MANDO REMOTO] Instruccion Lampara (Field 1): %d\n", valor);
                
                if (valor == 1) {
                    setLamparaManual(true);
                    setLastManualCommand(millis());
                    setLamparaEstado(1);
                    Serial.println("[CONTROL] Accion: ENCENDIDO (Prioridad Manual Asignada desde Nube)");
                    
                    // Le ordenamos por cable al Gateway que la prenda YA
                    Serial1.println("MODO:LAMPARA:MANUAL");
                    enviarComandoAlGateway("ACTUADOR:1:ESTADO:1");
                } else {
                    setLamparaManual(false);
                    setLamparaEstado(0);
                    Serial.println("[CONTROL] Accion: APAGADO (Retornando a Logica Automatica)");
                    
                    // Le decimos al Gateway que la apague y vuelva a escuchar a los sensores
                    Serial1.println("MODO:LAMPARA:AUTOMATICO");
                    enviarComandoAlGateway("ACTUADOR:1:ESTADO:0");
                }
                
                // Le rebotamos la confirmacion a ThingSpeak para que la interfaz web se actualice
                if (wifiDisponible && hayInternet) {
                    publicarThingSpeak(valor, -1, 0);
                }
            } else {
                if (esEcoPorTiempo) lastProcessedField1 = valor; 
            }
        }
        
        // --- CASO 2: El usuario apreto el boton del Ventilador/Calefactor en la Web (Field2) ---
        // Hace exactamente la misma logica estricta que la lampara
        else if (topicStr.endsWith("field2")) {
            bool debeProcesar = false;
            
            if (!esEcoPorTiempo) {
                debeProcesar = (valor != lastProcessedField2);
            }
            
            if (debeProcesar) {
                lastProcessedField2 = valor;
                Serial.printf("[MANDO REMOTO] Instruccion Ventilador (Field 2): %d\n", valor);
                
                if (valor == 1) {
                    setVentiladorManual(true);
                    setLastManualCommand(millis());
                    setVentiladorEstado(1);
                    Serial.println("[CONTROL] Accion: ENCENDIDO (Prioridad Manual Asignada desde Nube)");
                    Serial1.println("MODO:VENTILADOR:MANUAL");
                    enviarComandoAlGateway("ACTUADOR:2:ESTADO:1");
                } else {
                    setVentiladorManual(false);
                    setVentiladorEstado(0);
                    Serial.println("[CONTROL] Accion: APAGADO (Retornando a Logica Automatica)");
                    Serial1.println("MODO:VENTILADOR:AUTOMATICO");
                    enviarComandoAlGateway("ACTUADOR:2:ESTADO:0");
                }
                
                if (wifiDisponible && hayInternet) {
                    publicarThingSpeak(-1, valor, 0);
                }
            } else {
                if (esEcoPorTiempo) lastProcessedField2 = valor;
            }
        }
        
        // --- CASO 3: Origen de la orden (Quien lo prendio) ---
        else if (topicStr.endsWith("field3")) {
            if (!esEcoPorTiempo && valor != lastProcessedField3) {
                lastProcessedField3 = valor;
            } else {
                if (esEcoPorTiempo) lastProcessedField3 = valor;
            }
        }
        
        // --- CASO 4: Configuracion remota del Termostato (Field4) ---
        else if (topicStr.endsWith("field4")) {
            if (!esEcoPorTiempo && valor != lastProcessedField4) {
                lastProcessedField4 = valor;
                Serial.printf("[CONFIGURACION] Actualizacion de Termostato remota: %d C\n", valor);
                
                // Le pasamos por cable la nueva temperatura de corte al Gateway
                Serial1.println("CONFIG:UMBRAL:" + String(valor));
            } else if (esEcoPorTiempo) lastProcessedField4 = valor;
        }
        
        // --- CASO 5: Configuracion de inicio de ventana nocturna (Field5) ---
        else if (topicStr.endsWith("field5")) {
            if (!esEcoPorTiempo && valor != lastProcessedField5) {
                lastProcessedField5 = valor;
                Serial.printf("[CONFIGURACION] Ventana nocturna (Inicio): %d:00 hrs\n", valor);
                
                // Le avisamos al Gateway a que hora debe empezar a reaccionar al movimiento
                Serial1.println("CONFIG:HORA_INICIO:" + String(valor));
            } else if (esEcoPorTiempo) lastProcessedField5 = valor;
        }
        
        // --- CASO 6: Configuracion de fin de ventana nocturna (Field6) ---
        else if (topicStr.endsWith("field6")) {
            if (!esEcoPorTiempo && valor != lastProcessedField6) {
                lastProcessedField6 = valor;
                Serial.printf("[CONFIGURACION] Ventana nocturna (Fin): %d:00 hrs\n", valor);
                
                // Le avisamos al Gateway a que hora debe ignorar el movimiento
                Serial1.println("CONFIG:HORA_FIN:" + String(valor));
            } else if (esEcoPorTiempo) lastProcessedField6 = valor;
        }
    }
}

//   CONTROLADOR DE LA LOGICA DEL NODO GATEWAY  
// El Gateway (el otro ESP32) nos manda mensajes de texto por el cable. 
// Esta funcion lee esos textos y decide que accion tomar.
void procesarDatosDesdeGateway(String datos) {
    // 1. Si el texto empieza con "ACK", el Gateway nos esta confirmando que recibio una orden
    if (datos.startsWith("ACK:")) {
        procesarACK(datos); return;
    }
    // 2. Si nos avisa que acaba de reiniciarse por completo
    if (datos.startsWith("GATEWAY:REBOOT_COMPLETO")) {
        gatewayConectado = true; lastGatewayHeartbeat = millis(); return;
    }
    // 3. Si es su latido de rutina para decirnos "sigo vivo y estos son mis estados"
    if (datos.startsWith("HEARTBEAT:")) {
        procesarHeartbeatGateway(datos); return;
    }
    // Ignoramos el mensaje inicial de reboot hasta que se complete
    if (datos.startsWith("GATEWAY:REBOOT")) {
        return;
    }
    
    // 4. Si el Gateway nos avisa que alguien presiono el boton fisico de la Lampara
    if (datos.startsWith("MODO:LAMPARA:")) {
        String modo = datos.substring(datos.indexOf(":") + 2);
        modo.trim();
        // Si lo pasaron a manual, lo anotamos en nuestra memoria y publicamos a la nube
        if (modo == "MANUAL") {
            setLamparaManual(true);
            setLastManualCommand(millis()); 
            Serial.println("[SISTEMA] Modo logico conmutado: Lampara en estado MANUAL (Via hardware externo)");
            if (getLamparaEstado() == 1 && wifiDisponible && hayInternet) {
                publicarThingSpeakDirecto(getLamparaEstado(), -1, calcularOrigen());
            }
        } 
        // Si volvieron a ponerlo en automatico
        else if (modo == "AUTOMATICO") {
            setLamparaManual(false);
            if (wifiDisponible && hayInternet) {
                publicarCambioModo(getLamparaEstado(), -1, calcularOrigen());
            }
        }
        return;
    }
    
    // 5. Lo mismo, pero para el boton fisico del Ventilador/Calefactor
    if (datos.startsWith("MODO:VENTILADOR:")) {
        String modo = datos.substring(datos.indexOf(":") + 2);
        modo.trim();
        if (modo == "MANUAL") {
            setVentiladorManual(true);
            setLastManualCommand(millis()); 
            Serial.println("[SISTEMA] Modo logico conmutado: Ventilador en estado MANUAL (Via hardware externo)");
            if (getVentiladorEstado() == 1 && wifiDisponible && hayInternet) {
                publicarThingSpeakDirecto(-1, getVentiladorEstado(), calcularOrigen());
            }
        } else if (modo == "AUTOMATICO") {
            setVentiladorManual(false);
            if (wifiDisponible && hayInternet) {
                publicarCambioModo(-1, getVentiladorEstado(), calcularOrigen());
            }
        }
        return;
    }
    
    // 6. Si es un mensaje confirmando el estado final de un actuador
    if (datos.startsWith("CONFIRM:")) {
        int primerDigito = datos.indexOf(':') + 1;
        int segundoDigito = datos.indexOf(':', primerDigito);
        if (primerDigito != -1 && segundoDigito != -1) {
            int id = datos.substring(primerDigito, segundoDigito).toInt();
            int estado = datos.substring(segundoDigito + 1).toInt();
            if (id == 1) {
                setLamparaEstado(estado); estadoLamparaActual = estado;
            } else if (id == 2) {
                setVentiladorEstado(estado); estadoVentiladorActual = estado;
            }
        }
        return;
    }
    
    // 7. Actualizaciones rapidas de estado de la Lampara
    if (datos.startsWith("ACTUADOR:1:ESTADO:")) {
        int estado = datos.substring(datos.lastIndexOf(":") + 1).toInt();
        if (estado != getLamparaEstado()) {
            setLamparaEstado(estado);
            if (wifiDisponible && hayInternet) {
                publicarThingSpeak(estado, -1, calcularOrigen());
            }
        }
        return;
    }
    
    // 8. Actualizaciones rapidas de estado del Ventilador
    if (datos.startsWith("ACTUADOR:2:ESTADO:")) {
        int estado = datos.substring(datos.lastIndexOf(":") + 1).toInt();
        if (estado != getVentiladorEstado()) {
            setVentiladorEstado(estado);
            if (wifiDisponible && hayInternet) {
                publicarThingSpeak(-1, estado, calcularOrigen());
            }
        }
        return;
    }
    
    // 9. Si empieza con un numero, son datos puros del sensor ambiental.
    // Los mandamos al buzon del Nucleo 1 para que los procese.
    if (datos[0] >= '0' && datos[0] <= '9') {
        char* buffer = strdup(datos.c_str());
        if (queueGatewayToCore1 != NULL) { if (xQueueSend(queueGatewayToCore1, &buffer, 0) != pdTRUE) free(buffer); }
        else free(buffer);
        return;
    }
    
    // 10. Si empieza con "E,", son datos del medidor de consumo electrico.
    // Tambien los mandamos al buzon del Nucleo 1.
    if (datos.startsWith("E,")) {
        char* buffer = strdup(datos.c_str());
        if (queueGatewayToCore1 != NULL) { if (xQueueSend(queueGatewayToCore1, &buffer, 0) != pdTRUE) free(buffer); }
        else free(buffer);
        return;
    }
}

//   ESTABLECIMIENTO Y CONSERVACION DE ENLACES DE RED  

// El telefonista de MQTT. Se encarga de llamar al servidor de ThingSpeak.
void conectarMQTT() {
    if (!hayInternet || !wifiDisponible) return; // Si no hay internet, ni lo intentamos
    
    if (!mqttClient.connected()) {
        // Usamos nuestras credenciales para iniciar sesion
        if (mqttClient.connect(mqtt_client_id, mqtt_username, mqtt_password)) {
            mqttConectado = true; 
            suscribirMQTT(); // Si conectamos, nos suscribimos a los canales para escuchar ordenes
        } else {
            mqttConectado = false;
        }
    }
}

// Revisa constantemente si nos caimos del servidor MQTT y nos vuelve a conectar
void verificarConexionMQTT() {
    static unsigned long lastReconnectAttempt = 0;
    // Solo intentamos reconectar cada 5 segundos para no saturar el chip
    if (!mqttClient.connected() && (millis() - lastReconnectAttempt > 5000)) {
        lastReconnectAttempt = millis(); 
        conectarMQTT();
    }
}

// Le decimos a ThingSpeak: "Avisame si alguien cambia estos valores desde la web o la app"
void suscribirMQTT() {
    if (!mqttClient.connected()) return;
    
    String sub1 = String(mqtt_subscribe_actuadores) + "/fields/field1";
    String sub2 = String(mqtt_subscribe_actuadores) + "/fields/field2";
    String sub3 = String(mqtt_subscribe_actuadores) + "/fields/field3";
    String sub4 = String(mqtt_subscribe_actuadores) + "/fields/field4";
    String sub5 = String(mqtt_subscribe_actuadores) + "/fields/field5";
    String sub6 = String(mqtt_subscribe_actuadores) + "/fields/field6";
    
    mqttClient.subscribe(sub1.c_str());
    mqttClient.subscribe(sub2.c_str());
    mqttClient.subscribe(sub3.c_str());
    mqttClient.subscribe(sub4.c_str());
    mqttClient.subscribe(sub5.c_str());
    mqttClient.subscribe(sub6.c_str());
}

//   PROTOCOLOS DE RED ESTANDAR  

// Hacemos "ping" a un servidor de Google (8.8.8.8) para saber si de verdad tenemos 
// acceso a internet, o si solo estamos conectados al router pero sin linea.
void verificarInternet() {
    if (millis() - lastInternetCheck < 30000) return; // Solo revisamos cada 30 segundos
    lastInternetCheck = millis();
    
    if (WiFi.status() == WL_CONNECTED) {
        WiFiClient client;
        if (client.connect("8.8.8.8", 53)) {
            hayInternet = true; 
            modoLocal = false; 
            client.stop();
        } else {
            hayInternet = false; 
            modoLocal = true;
        }
    } else {
        hayInternet = false; 
        modoLocal = true;
    }
}

// Tarea para enlazar el modulo WiFi con la red de nuestra casa
void conectarWiFi() {
    WiFi.mode(WIFI_STA); // Nos configuramos como clientes (estacion)
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    // Le pasamos los certificados a Telegram para que la conexion sea segura (HTTPS)
    secured_client.setCACert(TELEGRAM_CERTIFICATE_ROOT);
    
    int intentos = 0;
    // Le damos 30 intentos (unos 15 segundos) para conectarse
    while (WiFi.status() != WL_CONNECTED && intentos < 30) { 
        delay(500); 
        intentos++; 
    }
    wifiDisponible = (WiFi.status() == WL_CONNECTED);
}

// Si en pleno funcionamiento se cae el WiFi, esta funcion intenta reconectarlo
void verificarWiFi() {
    if (millis() - lastWiFiCheck > 30000) {
        lastWiFiCheck = millis();
        if (WiFi.status() != WL_CONNECTED) {
            wifiDisponible = false; 
            WiFi.reconnect();
        } else if (!wifiDisponible) {
            wifiDisponible = true;
        }
    }
}

// Obtenemos la hora real desde los servidores NTP de internet
void configurarNTP() {
    if (WiFi.status() != WL_CONNECTED) { horaInicializada = false; return; }
    
    // Configuramos nuestra zona horaria y apuntamos al servidor de Google
    configTime(gmtOffset_sec, daylightOffset_sec, "time.google.com");
    
    struct tm timeinfo;
    int intentos = 0; 
    bool horaObtenida = false;
    
    while (intentos < 30 && !horaObtenida) {
        delay(500); 
        intentos++;
        if (getLocalTime(&timeinfo)) horaObtenida = true;
    }
    horaInicializada = horaObtenida;
}

// Si no pudimos obtener la hora al inicio, esta funcion lo seguira intentando
// silenciosamente de fondo sin congelar el procesador.
void sincronizarHoraPeriodica() {
    if (horaInicializada) return;
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 0)) {
        if (timeinfo.tm_year > 70) {
            horaInicializada = true;
            Serial.println("[NTP] Sincronizacion de reloj en capa secundaria completada.");
        }
    }
}

// Le pasamos por cable la hora actual al Gateway para que el sepa 
// cuando aplicar las reglas de dia y de noche para el sensor de movimiento.
void enviarHoraAlGateway() {
    struct tm timeinfo;
    if (horaInicializada && getLocalTime(&timeinfo)) {
        String horaStr = "TIME:" + String(timeinfo.tm_hour) + ":" + String(timeinfo.tm_min);
        Serial1.println(horaStr);
    }
}

//   INTERFAZ DE DIAGNOSTICO CLI (EN PANTALLA)  

// Imprime un resumen rapido de si las cosas estan encendidas o apagadas
void mostrarEstadoBitmask() {
    int lampara = getLamparaEstado();
    int ventilador = getVentiladorEstado();
    Serial.println("\n[INFO] Estado Logico de Actuadores:");
    Serial.printf(" - Lampara: %s\n", lampara ? "ENCENDIDA" : "APAGADA");
    Serial.printf(" - Ventilador: %s\n", ventilador ? "ENCENDIDO" : "APAGADO");
}

void mostrarEstado() {
    mostrarEstadoBitmask();
}

// Cada 4 horas reiniciamos el ESP32 a proposito. 
// Esto limpia la memoria RAM y evita que el sistema se cuelgue a largo plazo.
void verificarReinicioProgramado() {
    if (millis() - lastProgrammedRestart > TIEMPO_REINICIO_MS) {
        delay(1000); 
        ESP.restart();
    }
}

//   ACTUALIZACIONES OVER-THE-AIR (OTA) PARA TENSORFLOW  

// Descargamos el archivo del modelo predictivo desde internet hacia nuestra placa
bool descargarModeloOTA() {
    if (!wifiDisponible || !hayInternet) {
        Serial.println("[ERROR OTA] Enlace descendente inaccesible. Cancelando sincronizacion de modelo.");
        return false;
    }

    Serial.println("[OTA] Inicializando descarga de hiperparametros desde servidor central...");
    HTTPClient http;
    
    // Apuntamos al servidor donde tenemos alojado el archivo .tflite
    http.begin(url_servidor_ota);
    
    int httpCode = http.GET();
    // 200 significa que el servidor nos respondio "Todo OK, aqui tienes el archivo"
    if (httpCode == HTTP_CODE_OK) {
        // Abrimos nuestro disco interno para escribir el nuevo archivo
        File file = LittleFS.open("/model.tflite", "w");
        if (!file) {
            Serial.println("[ERROR IO] Imposible abrir flujo de escritura en Flash para modelo tflite.");
            http.end();
            return false;
        }
        
        // Guardamos todo el archivo descargado directo a la memoria
        http.writeToStream(&file);
        file.close();
        
        Serial.println("[OTA] Transaccion completada. Modelo predictivo almacenado en NVRAM.");
        http.end();
        return true;
    } else {
        Serial.printf("[ERROR OTA] Rechazo de origen. HTTP Status: %d\n", httpCode);
        http.end();
        return false;
    }
}

//   CARGA DEL CEREBRO PREDICTIVO ML  

// Sacamos el archivo del modelo desde el disco interno y lo cargamos en la RAM para usarlo
bool cargarModeloIA() {
    // Primero revisamos si el archivo existe
    if (!LittleFS.exists("/model.tflite")) {
        Serial.println("[ML Ops] Ausencia de binario tflite en bloque de memoria. Esperando pipeline OTA.");
        return false;
    }
    
    File file = LittleFS.open("/model.tflite", "r");
    size_t modelSize = file.size(); // Vemos cuanto pesa
    
    // Si ya habiamos cargado uno antes, limpiamos la memoria
    if (modelBuffer != nullptr) {
        free(modelBuffer);
    }
    
    // Reservamos un espacio en la RAM exactamente del tamano del modelo
    modelBuffer = (uint8_t*)malloc(modelSize);
    if (modelBuffer == nullptr) {
        Serial.println("[ERROR MEMORIA] Desbordamiento (Heap Overflow) al intentar reservar area de modelo IA.");
        file.close();
        return false;
    }
    
    // Leemos el archivo y lo pasamos a ese espacio reservado
    file.read(modelBuffer, modelSize);
    file.close();

    // Le decimos a TensorFlow que lea el modelo
    tflModel = tflite::GetModel(modelBuffer);
    // Verificamos que sea una version compatible
    if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
        Serial.println("[ERROR TENSORFLOW] Discrepancia en version de schema TFLite.");
        return false;
    }

    // Preparamos a los interpretes que haran los calculos matematicos
    static tflite::AllOpsResolver resolver;
    tflInterpreter = new tflite::MicroInterpreter(tflModel, resolver, tensorArena, kTensorArenaSize, errorReporter);
    
    // Les asignamos su lugar de trabajo en la memoria RAM
    if (tflInterpreter->AllocateTensors() != kTfLiteOk) {
        Serial.println("[ERROR TENSORFLOW] Fallo critico en localizacion de memoria (AllocateTensors).");
        return false;
    }

    // Dejamos listos el cajon de entrada (datos que metemos) y el de salida (la respuesta de la IA)
    tflInputTensor = tflInterpreter->input(0);
    tflOutputTensor = tflInterpreter->output(0);
    
    Serial.println("[ML Ops] Red Neuronal cargada satisfactoriamente en espacio de Tensor Arena.");
    return true;
}

//   MANEJO Y VINCULACION DE CLIENTES (MEMORIA LITTLEFS)  
// Aquí guardamos los datos del usuario de la casa para que el ESP32 no los olvide 
// si ocurre un corte de energia o un reinicio del sistema.
String chatIDCliente = ""; 

// Esta funcion guarda el "numero de telefono" (Chat ID) y el nombre del usuario en el disco duro interno
bool guardarClienteVinculado(String id, String nombre) {
    // Abrimos un archivo de texto en modo escritura ("w" = write)
    File file = LittleFS.open("/cliente_chat_id.txt", "w");
    if (!file) {
        Serial.println("[ERROR IO] Fallo al crear el archivo. No se pudo guardar el perfil del usuario.");
        return false;
    }
    
    // Escribimos el ID y el nombre separados por una coma, por ejemplo: "7993821172,Mario"
    file.print(id + "," + nombre);
    file.close(); // Cerramos el archivo para que se guarde fisicamente en la memoria
    
    // Actualizamos las variables que usa el programa para que funcionen inmediatamente
    chatIDCliente = id;
    NOMBRE_USUARIO = nombre; 
    Serial.println("[SISTEMA] Nuevo administrador autorizado y guardado con exito: " + nombre + " (ID: " + id + ")");
    return true;
}

// Esta funcion se ejecuta justo despues de encender el equipo para buscar al usuario
void cargarClienteVinculado() {
    // Preguntamos si el archivo del perfil existe en el disco duro interno
    if (LittleFS.exists("/cliente_chat_id.txt")) {
        // Lo abrimos en modo lectura ("r" = read)
        File file = LittleFS.open("/cliente_chat_id.txt", "r");
        if (file) {
            String contenido = file.readString(); // Leemos todo el texto de golpe
            contenido.trim(); // Le quitamos espacios vacios al inicio o al final
            file.close();

            // Buscamos en que posicion esta la coma que separa el ID del Nombre
            int comaIndex = contenido.indexOf(',');
            if (comaIndex != -1) {
                // Extraemos la parte izquierda (el numero de ID)
                chatIDCliente = contenido.substring(0, comaIndex);
                // Extraemos la parte derecha (el nombre del usuario)
                NOMBRE_USUARIO = contenido.substring(comaIndex + 1);
            } else {
                // Control de errores: si por algun motivo el archivo se guardo sin coma, 
                // asumimos que todo el texto es el ID y le ponemos un nombre por defecto.
                chatIDCliente = contenido;
                NOMBRE_USUARIO = "Usuario";
            }
            Serial.println("[SISTEMA] Perfil cargado desde la memoria. Administrador reconocido: " + NOMBRE_USUARIO + " (" + chatIDCliente + ")");
        }
    } else {
        // Si el archivo no existe, significa que el equipo esta de fabrica o se uso el comando /reset
        Serial.println("[SISTEMA] Equipo sin configurar. Esperando vinculacion de administrador por Telegram (/vincular).");
    }
}

//   SERVICIO DE NOTIFICACIONES HUMAN-IN-THE-LOOP (TELEGRAM)  
// Este sistema permite que el ESP32 "consulte" con el usuario antes de tomar decisiones drasticas.
unsigned long ultimaAlertaMensual = 0;
unsigned long intervaloSilencioAlerta = 0; // Tiempo que el usuario nos pidio que no le molestemos

// El sistema de Inteligencia Artificial usa esta funcion para vigilar la factura de luz
void verificarConsumoMensual(float consumoActualKWh, float umbralMaximoKWh) {
    if (chatIDCliente == "") return; // Si no hay nadie vinculado, no le avisamos a nadie

    // Verificamos si el usuario nos puso en "silencio". 
    // Si la alerta aun esta silenciada por el usuario (ej: pidio recordar en 7 dias), cortamos aqui.
    if (ultimaAlertaMensual > 0 && (millis() - ultimaAlertaMensual < intervaloSilencioAlerta)) return;

    // Si el calculo proyecta que el gasto sera mayor que nuestro limite configurado...
    if (consumoActualKWh > umbralMaximoKWh) {
        String mensajeAlerta = "⚠️ *Alerta de Consumo Energetico*\n\n";
        mensajeAlerta += "La proyeccion mensual ha ascendido a *" + String(consumoActualKWh, 1) + " kWh* ";
        mensajeAlerta += "(Limite estipulado: *" + String(umbralMaximoKWh, 1) + " kWh*).\n\n";
        mensajeAlerta += "Requiere confirmacion de operacion a continuacion.";

        // Dibujamos un teclado interactivo en Telegram para que el usuario no tenga que escribir, solo pulsar.
        String botonesOpciones = "{\"keyboard\":[[\"Activar Ahorro\"], [\"Recordar en 7 dias\", \"Silenciar\"]],\"resize_keyboard\":true,\"one_time_keyboard\":true}";
        bot.sendMessageWithReplyKeyboard(chatIDCliente, mensajeAlerta, "Markdown", botonesOpciones);
        
        accionSugerida = 3; // 3 significa "Opciones de consumo electrico"
        esperandoRespuestaTelegram = true; // Levantamos la bandera para estar atentos a la respuesta
        tiempoSugerencia = millis(); // Empezamos a contar el timeout de 10 minutos
        ultimaAlertaMensual = millis(); 
        
        // Por defecto, si el usuario nos ignora, asumimos que no quiere ser molestado 
        // con este tema durante las proximas 24 horas (86400000 milisegundos)
        intervaloSilencioAlerta = 86400000UL; 
    }
}

//   COMUNICACION CON EL MAYORDOMO VIRTUAL (TELEGRAM BOT)  
// Esta funcion es el cerebro conversacional. Lee lo que envia el usuario desde Telegram y decide que accion tomar.
void procesarMensajesTelegram(int numNuevosMensajes) {
    for (int i = 0; i < numNuevosMensajes; i++) {
        String chat_id = bot.messages[i].chat_id;
        String text = bot.messages[i].text;
        String textLower = text;
        textLower.toLowerCase(); // Pasamos todo a minusculas para que "Hola", "HOLA" o "hola" se entiendan igual.

        // CASO 1: El equipo esta "sin usuario registrado". Nadie se ha vinculado aun.
        // Aqui pedimos la llave maestra (PIN) para asignar un usuario.
        if (chatIDCliente == "") {
            if (textLower.startsWith("/vincular ")) {
                String pin = text.substring(10);
                pin.trim();
                // IMPORTANTE Esta es la contrasena de seguridad establecida para al comunicaion con telegram
                if (pin == "contraseña_telegran") {
                    String nombreTelegram = bot.messages[i].from_name;
                    if (nombreTelegram == "") nombreTelegram = "Usuario"; 
                    guardarClienteVinculado(chat_id, nombreTelegram);
                    bot.sendMessage(chat_id, "🎉 ¡Hola " + nombreTelegram + "! Dispositivo vinculado con exito. Ahora eres el administrador absoluto.", "");
                } else {
                    bot.sendMessage(chat_id, "❌ PIN incorrecto. Credenciales rechazadas.", "");
                }
            } else {
                bot.sendMessage(chat_id, "🔒 Plataforma bloqueada. El sistema espera una clave de vinculacion valida. Escribe: /vincular [TU_PIN]", "");
            }
            continue; // Finaliza aqui y pasa al siguiente mensaje si hay mas
        }

        // CASO 2: Seguridad estricta (Escudo anti-intrusos)
        // Si ya tenemos un administrador y nos escribe otra persona, le negamos el acceso rotundamente.
        if (chat_id != chatIDCliente) {
            bot.sendMessage(chat_id, "⛔ Acceso denegado. Este perfil de hardware ya pertenece a una cuenta administrativa activa.", "");
            continue;
        }

        // CASO 3: Es el usuario legitimo. Procesamos sus ordenes y comandos.
        
        // 3.1: Comandos de saludo o para pedir el reporte de consumo actual
        if (textLower == "/start" || textLower == "hola" || textLower == "/reporte") {
            struct tm timeinfo;
            String saludo = "Hola";
            // Averiguamos que hora es para saludar educadamente (Buenos dias/tardes/noches)
            if (getLocalTime(&timeinfo, 0)) {
                int hora = timeinfo.tm_hour;
                if (hora >= 5 && hora < 12) saludo = "Buenos dias";
                else if (hora >= 12 && hora < 19) saludo = "Buenas tardes";
                else saludo = "Buenas noches";
            }
            
            // Calculamos cuanto gastaria al mes si siguiera consumiendo igual que las ultimas 24h
            float consumoMensual_kWh = (getAverageConsumption(24) * 24.0 * 30.0) / 1000.0;
            
            String msj = "🤖 " + saludo + " " + NOMBRE_USUARIO + ".\n\n";
            msj += "📊 *Diagnostico de Energia*:\n";
            msj += "El gasto proyectado estimado para el mes en curso es de *" + String(consumoMensual_kWh, 1) + " kWh*.\n\n";
            msj += "Soy tu Mayordomo Virtual. El sistema predictivo está activo y todo funciona correctamente.";
            
            bot.sendMessage(chat_id, msj, "Markdown");
        } 
        
        // 3.2: Comando para formatear o "liberar" el equipo (volverlo al CASO 1)
        else if (textLower == "/reset") {
            LittleFS.remove("/cliente_chat_id.txt"); 
            chatIDCliente = "";
            NOMBRE_USUARIO = "Usuario";
            bot.sendMessage(chat_id, "🧹 Memoria de administracion purgada. Enrutador disponible para nueva asignacion.", "");
        }
        
        // 3.3: Logica interactiva con la Inteligencia Artificial (El bot nos propuso algo y espera un SI o NO)
        else if (esperandoRespuestaTelegram) {
            
            // Si la red neuronal sugirio automatizar la Lampara (1) o el Calefactor (2) segun los patrones de uso
            if (accionSugerida == 1 || accionSugerida == 2) {
                if (textLower == "si" || textLower == "sí" || textLower == "yes") {
                    if (accionSugerida == 1) {
                        Serial.println("[MAYORDOMO IA] El usuario aprobó la sugerencia. Aprendiendo el nuevo hábito de la Lámpara...");
                        char* cmd = strdup("MODO:LAMPARA:AUTOMATICO");
                        if (queueCore1ToGateway != NULL) xQueueSend(queueCore1ToGateway, &cmd, 0);
                        bot.sendMessage(chat_id, "✅ Orden confirmada, " + NOMBRE_USUARIO + ". Patrones de iluminacion automatizados.", "");
                    } 
                    else if (accionSugerida == 2) {
                        Serial.println("[MAYORDOMO IA] El usuario aprobó la sugerencia. Aprendiendo el nuevo hábito del Calefactor...");
                        char* cmd = strdup("MODO:VENTILADOR:AUTOMATICO");
                        if (queueCore1ToGateway != NULL) xQueueSend(queueCore1ToGateway, &cmd, 0);
                        bot.sendMessage(chat_id, "✅ Orden confirmada, " + NOMBRE_USUARIO + ". Patrones de calefaccion automatizados.", "");
                    }
                    esperandoRespuestaTelegram = false; accionSugerida = 0; 
                } 
                else if (textLower == "no") {
                    bot.sendMessage(chat_id, "❌ Entendido, " + NOMBRE_USUARIO + ". Ignorare este patron predictivo.", "");
                    esperandoRespuestaTelegram = false; accionSugerida = 0; 
                }
            }
            
            // Si el bot detecto que nos pasamos del limite de consumo energetico (Alerta de Consumo Mensual)
            else if (accionSugerida == 3) {
                if (textLower == "activar ahorro") {
                    // Enviamos comandos para APAGAR TODO desde aqui y dejarlo en automatico
                    char* cmd1 = strdup("ACTUADOR:1:ESTADO:0");
                    if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd1, 0);
                    char* cmd2 = strdup("MODO:LAMPARA:AUTOMATICO");
                    if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd2, 0);
                    char* cmd3 = strdup("ACTUADOR:2:ESTADO:0");
                    if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd3, 0);
                    char* cmd4 = strdup("MODO:VENTILADOR:AUTOMATICO");
                    if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd4, 0);
                    
                    bot.sendMessage(chat_id, "🔋 Equipos logicos suspendidos para ahorro energetico. Alertas silenciadas durante 30 dias.", "");
                    intervaloSilencioAlerta = 30UL * 24 * 60 * 60 * 1000UL; // Callado un mes entero
                    esperandoRespuestaTelegram = false; accionSugerida = 0;
                }
                else if (textLower == "recordar en 7 días" || textLower == "recordar en 7 dias") {
                    bot.sendMessage(chat_id, "⏳ Pospuesto. Retomaremos esta alerta dentro de una semana (168 horas).", "");
                    intervaloSilencioAlerta = 7UL * 24 * 60 * 60 * 1000UL; // Callado una semana
                    esperandoRespuestaTelegram = false; accionSugerida = 0;
                }
                else if (textLower == "silenciar") {
                    bot.sendMessage(chat_id, "🔇 Alertas termicas suspendidas. Sin supervision de limites durante 30 dias operacionales.", "");
                    intervaloSilencioAlerta = 30UL * 24 * 60 * 60 * 1000UL; // Callado un mes entero
                    esperandoRespuestaTelegram = false; accionSugerida = 0;
                }
            }
        } 
        
        // 3.4: Cualquier otro mensaje no reconocido o sin contexto
        else {
            bot.sendMessage(chat_id, "💤 Todo estable. Estado del hardware sin variaciones criticas en tiempo real.", "");
        }
    }
}

//   RUTINA DE REVISION AUTOMATICA DE TELEGRAM  
// Esta funcion es llamada continuamente por el Core 1 para no descuidar los mensajes entrantes
void revisarTelegram() {
    static unsigned long ultimaRevision = 0;
    // Solo revisamos la API de Telegram cada 10 segundos para evitar penalizaciones (Rate Limits) de la red
    if (millis() - ultimaRevision > 10000) {  
        int numNuevosMensajes = bot.getUpdates(bot.last_message_received + 1);
        while (numNuevosMensajes) {
            procesarMensajesTelegram(numNuevosMensajes);
            numNuevosMensajes = bot.getUpdates(bot.last_message_received + 1);
        }
        ultimaRevision = millis();
    }
    
    // Logica NO invasiva: Si el mayordomo nos propuso algo y llevamos 10 minutos sin responderle,
    // asume automaticamente que es una negativa, cancela la sugerencia y deja de molestar al usuario.
    if (esperandoRespuestaTelegram && (millis() - tiempoSugerencia > TIMEOUT_RESPUESTA_MS)) {
        if (chatIDCliente != "") {
            bot.sendMessage(chatIDCliente, "Tiempo agotado. Tomare tu silencio como un 'No'.", ""); 
        }
        esperandoRespuestaTelegram = false;
    }
}

// TAREA PRINCIPAL DEL NUCLEO 0: Se encarga de las comunicaciones (WiFi, Nube y Cable)
void taskCore0(void *parameter) {
    // Antes de entrar al ciclo infinito, preparamos el terreno
    conectarWiFi();
    configurarNTP(); // Pedimos la hora real por internet
    
    // Dejamos listo el cartero MQTT para que sepa a donde enviar los datos
    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setKeepAlive(120);
    
    // Si tenemos internet y la hora ya se configuro, nos conectamos de una vez
    if (wifiDisponible && hayInternet && horaInicializada) conectarMQTT();
    
    // Entramos al ciclo infinito del nucleo 0. De aqui el procesador no sale nunca.
    while (true) {
        // Le damos un toque al perro guardian para que sepa que no nos hemos colgado
        esp_task_wdt_reset(); 
        
        // Revisiones de rutina en segundo plano
        verificarWiFi();
        verificarInternet();
        if (!horaInicializada) sincronizarHoraPeriodica(); // Si fallo la hora antes, lo sigue intentando
        
        // Si la red esta perfecta, atendemos los envios a la nube
        if (hayInternet && wifiDisponible && horaInicializada) {
            verificarConexionMQTT();
            if (mqttClient.connected()) {
                // Esta funcion mantiene viva la conexion con el servidor
                mqttClient.loop();
                
                // Un pequeno truco: a los 6 segundos de encender, forzamos un envio 
                // para que la nube sepa exactamente como arrancaron los equipos.
                static bool syncArranqueHecha = false;
                if (!syncArranqueHecha && millis() > 6000) {
                    syncArranqueHecha = true;
                    Serial.println("[MQTT] Sincronizando estado raiz bidireccional (Post-Init).");
                    publicarThingSpeakDirecto(getLamparaEstado(), getVentiladorEstado(), calcularOrigen());
                }
            }
        } else {
            mqttConectado = false;
        }
        
        // Escuchamos si el Gateway nos esta hablando por el cable (UART1)
        if (Serial1.available()) {
            String datos = Serial1.readStringUntil('\n');
            datos.trim();
            // Si nos llego algo, lo mandamos a procesar para saber si es un sensor o una orden
            if (datos.length() > 0) procesarDatosDesdeGateway(datos);
        }
        
        // Revisiones de salud del cableado y confirmaciones
        verificarTimeoutACK(); // Revisamos si alguien no nos respondio un mensaje
        checkGatewayHeartbeat(); // Vemos si el Gateway sigue vivo
        registrarReinicioGateway();
        
        // Revisamos el buzon: Vemos si el Nucleo 1 (Telegram/IA) nos dejo un mensaje para el Gateway
        char* buffer;
        if (queueCore1ToGateway != NULL) {
            if (xQueueReceive(queueCore1ToGateway, &buffer, 0) == pdTRUE) {
                Serial1.println(buffer); // Lo mandamos por cable
                free(buffer); // Limpiamos la memoria para no saturar la RAM
            }
        }
        
        // Revisamos el otro buzon: Vemos si el Nucleo 1 nos dejo datos para subir a la nube
        if (queueCore1ToMQTT != NULL) {
            if (xQueueReceive(queueCore1ToMQTT, &buffer, 0) == pdTRUE) {
                String datos = String(buffer);
                free(buffer);
                
                // Filtramos el mensaje dependiendo de que nos pidan publicar
                if (datos.startsWith("PUBLICAR_LAMPARA:")) {
                    int primero = datos.indexOf(':'); int segundo = datos.indexOf(':', primero + 1);
                    int estado = datos.substring(primero + 1, segundo).toInt();
                    int origen = datos.substring(segundo + 1).toInt();
                    if (wifiDisponible && hayInternet) publicarThingSpeakDirecto(estado, -1, origen);
                    continue; // Saltamos al siguiente ciclo
                }
                
                if (datos.startsWith("PUBLICAR_VENTILADOR:")) {
                    int primero = datos.indexOf(':'); int segundo = datos.indexOf(':', primero + 1);
                    int estado = datos.substring(primero + 1, segundo).toInt();
                    int origen = datos.substring(segundo + 1).toInt();
                    if (wifiDisponible && hayInternet) publicarThingSpeakDirecto(-1, estado, origen);
                    continue;
                }
                
                // Si son datos del sensor de clima, desarmamos el texto para sacar los numeros
                if (datos.startsWith("AMBIENTAL:")) {
                    String valores = datos.substring(10);
                    int primeraComa = valores.indexOf(','); int segundaComa = valores.indexOf(',', primeraComa + 1);
                    int terceraComa = valores.indexOf(',', segundaComa + 1);
                    
                    if (primeraComa != -1 && segundaComa != -1 && terceraComa != -1) {
                        float temp = valores.substring(0, primeraComa).toFloat();
                        int mov = valores.substring(primeraComa + 1, segundaComa).toInt();
                        float voltaje = valores.substring(segundaComa + 1, terceraComa).toFloat();
                        int porcentaje = valores.substring(terceraComa + 1).toInt();
                        
                        // Solo subimos si de verdad cambio algun dato, para ahorrar internet
                        if (hayCambioAmbiental(temp, mov, voltaje, porcentaje)) {
                            if (wifiDisponible && hayInternet) {
                                bool mqttExitoso = publicarMQTT_Ambiental(temp, mov, voltaje, porcentaje);
                                // Si fallo por MQTT, intentamos mandarlo por HTTP clasico
                                if (!mqttExitoso && !ambientalPendiente) enviarAThingSpeak_Ambiental(temp, mov, voltaje, porcentaje);
                            }
                        } else {
                            // Si no cambio nada, solo guardamos el registro actual
                            ultimoTempPublicado = temp; ultimoMovPublicado = mov;
                            ultimoVoltajePublicado = voltaje; ultimoPorcentajePublicado = porcentaje;
                        }
                    }
                } 
                // Si es un dato del medidor de luz (empieza con E)
                else if (datos.startsWith("E,")) {
                    if (wifiDisponible && hayInternet) {
                        String valores = datos.substring(2);
                        int primeraComa = valores.indexOf(','); int segundaComa = valores.indexOf(',', primeraComa + 1);
                        int terceraComa = valores.indexOf(',', segundaComa + 1);
                        
                        // Formato nuevo que incluye el factor de potencia (fp)
                        if (primeraComa != -1 && segundaComa != -1 && terceraComa != -1) {
                            float voltaje = valores.substring(0, primeraComa).toFloat();
                            float corriente = valores.substring(primeraComa + 1, segundaComa).toFloat();
                            float potencia = valores.substring(segundaComa + 1, terceraComa).toFloat();
                            float fp = valores.substring(terceraComa + 1).toFloat();
                            publicarDatosElectricos(voltaje, corriente, potencia, fp);
                        } 
                        // Formato antiguo por si el sensor no manda el factor de potencia
                        else {
                            int primeraComa2 = valores.indexOf(','); int segundaComa2 = valores.indexOf(',', primeraComa2 + 1);
                            if (primeraComa2 != -1 && segundaComa2 != -1) {
                                float voltaje = valores.substring(0, primeraComa2).toFloat();
                                float corriente = valores.substring(primeraComa2 + 1, segundaComa2).toFloat();
                                float potencia = valores.substring(segundaComa2 + 1).toFloat();
                                // Le ponemos un valor aproximado de 0.90 para que no falle
                                publicarDatosElectricos(voltaje, corriente, potencia, 0.90);
                            }
                        }
                    }
                } 
                // Reportamos a la nube si un actuador encendio o apago
                else if (datos.startsWith("ACTUADOR:1:")) {
                    int estado = datos.substring(datos.lastIndexOf(":") + 1).toInt();
                    if (wifiDisponible && hayInternet) publicarThingSpeak(estado, -1, 0);
                } else if (datos.startsWith("ACTUADOR:2:")) {
                    int estado = datos.substring(datos.lastIndexOf(":") + 1).toInt();
                    if (wifiDisponible && hayInternet) publicarThingSpeak(-1, estado, 0);
                }
            }
        }
        
        // Cada minuto exacto le pasamos la hora al Gateway para que sepa si es de dia o de noche
        static unsigned long lastTimeSent = 0;
        if (horaInicializada && (millis() - lastTimeSent > 60000)) {
            lastTimeSent = millis(); enviarHoraAlGateway();
        }
        
        verificarReinicioProgramado();
        
        // SALAS DE ESPERA: Aqui revisamos si ya pasaron los 16 segundos reglamentarios de ThingSpeak
        // Si ya pasaron, mandamos los datos que teniamos retenidos
        if (hayPendiente && (millis() - ultimoEnvioActuadores >= THINGSPEAK_MIN_INTERVAL_MS)) {
            ejecutarEnvioPendiente();
        }

        if (ambientalPendiente && (millis() - ultimoEnvioAmbiental >= THINGSPEAK_MIN_INTERVAL_MS)) {
            bool mqttExitoso = publicarMQTT_Ambiental(p_temp, p_mov, p_vBat, p_pBat);
            if (!mqttExitoso && ambientalPendiente) enviarAThingSpeak_Ambiental(p_temp, p_mov, p_vBat, p_pBat);
        }

        if (electricoPendiente && (millis() - ultimoEnvioElectrico >= THINGSPEAK_MIN_INTERVAL_MS)) {
            publicarDatosElectricos(p_voltaje, p_corriente, p_potencia, p_fp);
        }
        
        // Imprimimos un resumen en pantalla cada 40 segundos
        if (millis() - lastStatus > 40000) {
            lastStatus = millis(); mostrarEstado();
        }
        
        // Pausa obligatoria de 10 milisegundos. Sin esto, podriamos ocacionar colisiones.
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// TAREA SECUNDARIA DEL NUCLEO 1: El cerebro pensante (Inteligencia Artificial y Telegram)
void taskCore1(void *parameter) {
    // Entramos al ciclo infinito del nucleo 1. Aqui hacemos el trabajo pesado 
    // sin interrumpir la conexion WiFi que maneja el nucleo 0.
    while (true) {
        // Le damos un toque al perro guardian para que sepa que no nos hemos colgado
        esp_task_wdt_reset(); 
        
        // Revisamos si el usuario nos escribio algun comando por Telegram
        revisarTelegram();
        
        // Revisamos el buzon: vemos si el nucleo 0 nos dejo datos que llegaron por cable desde el Gateway
        char* buffer;
        if (queueGatewayToCore1 != NULL) {
            if (xQueueReceive(queueGatewayToCore1, &buffer, 0) == pdTRUE) {
                String datos = String(buffer); 
                free(buffer); // Limpiamos la memoria del mensaje leido
                
                // CASO A: Si el mensaje empieza con un numero, son datos del sensor ambiental
                if (datos[0] >= '0' && datos[0] <= '9') {
                    // Desarmamos el texto buscando las comas para separar los valores
                    int primeraComa = datos.indexOf(',');
                    int segundaComa = datos.indexOf(',', primeraComa + 1);
                    int terceraComa = datos.indexOf(',', segundaComa + 1);
                    int cuartaComa = datos.indexOf(',', terceraComa + 1);
                    
                    if (primeraComa != -1 && segundaComa != -1 && terceraComa != -1) {
                        // Extraemos la temperatura y el movimiento
                        float temp = datos.substring(primeraComa + 1, segundaComa).toFloat();
                        int movimiento = datos.substring(segundaComa + 1, terceraComa).toInt();
                        float voltaje = 0.0; 
                        int porcentaje = 0;
                        
                        // Calculamos la bateria dependiendo del formato en que llego el mensaje
                        if (cuartaComa != -1) {
                            voltaje = datos.substring(terceraComa + 1, cuartaComa).toFloat();
                            porcentaje = datos.substring(cuartaComa + 1).toInt();
                        } else {
                            float valor = datos.substring(terceraComa + 1).toFloat();
                            if (valor > 10.0) {
                                voltaje = valor; 
                                porcentaje = (int)((valor - 3.0) / (4.2 - 3.0) * 100.0);
                                if (porcentaje < 0) porcentaje = 0; 
                                if (porcentaje > 100) porcentaje = 100;
                            } else {
                                porcentaje = (int)valor; 
                                voltaje = 3.0 + (porcentaje / 100.0) * (4.2 - 3.0);
                            }
                        }
                        
                        // Empaquetamos todo ordenado y se lo pasamos al nucleo 0 para que lo suba a internet
                        char* bufferEnvio;
                        asprintf(&bufferEnvio, "AMBIENTAL:%.1f,%d,%.2f,%d", temp, movimiento, voltaje, porcentaje);
                        if (queueCore1ToMQTT != NULL) { 
                            if (xQueueSend(queueCore1ToMQTT, &bufferEnvio, 0) != pdTRUE) free(bufferEnvio); 
                        } else {
                            free(bufferEnvio);
                        }
                    }
                }
                // CASO B: Si el mensaje empieza con "E,", son datos del sensor de electricidad
                else if (datos.startsWith("E,")) {
                    String valores = datos.substring(2);
                    int primeraComa = valores.indexOf(','); 
                    int segundaComa = valores.indexOf(',', primeraComa + 1);
                    
                    if (primeraComa != -1 && segundaComa != -1) {
                        float potencia = valores.substring(segundaComa + 1).toFloat();
                        
                        // Guardamos este consumo en nuestro registro historico
                        updateConsumptionHistory(potencia);
                        
                        // AQUI VIENE LA MAGIA DE LA IA: Le pasamos el consumo a TensorFlow
                        if (tflInputTensor != nullptr && tflOutputTensor != nullptr && tflInterpreter != nullptr) {
                            tflInputTensor->data.f[0] = potencia; 
                            // Ejecutamos la red neuronal
                            if (tflInterpreter->Invoke() == kTfLiteOk) {
                                float resultadoPrediccion = tflOutputTensor->data.f[0];
                                
                                // Si la IA esta mas de un 80% segura (0.8) de un patron, le avisa al usuario
                                if (resultadoPrediccion > 0.8 && !esperandoRespuestaTelegram && chatIDCliente != "") {
                                    String mensajeSugerencia = "";
                                    
                                    // Dependiendo de la temperatura actual, adivina si prendiste la luz o la estufa
                                    if (ultimoTempPublicado > 0.0 && ultimoTempPublicado < 20.0) {
                                        accionSugerida = 2; // Opcion 2 = Calefactor
                                        mensajeSugerencia = "Hola " + NOMBRE_USUARIO + ". Identificacion de correlacion de hardware: Activacion estandar de calefactor a " + String(ultimoTempPublicado, 1) + " C.\n\n¿Autorizas transferir ejecucion a red neuronal predictiva?";
                                    } else {
                                        accionSugerida = 1; // Opcion 1 = Lampara
                                        mensajeSugerencia = "Hola " + NOMBRE_USUARIO + ". He revisado registros de incidencia de lampara central y coincide en vector de tiempo actual.\n\n¿Autorizas transferir ejecucion a red neuronal predictiva?";
                                    }
                                    
                                    // Le mandamos los botones interactivos por Telegram
                                    String botonesOpciones = "{\"keyboard\":[[\"Sí\", \"No\"]],\"resize_keyboard\":true,\"one_time_keyboard\":true}";
                                    bot.sendMessageWithReplyKeyboard(chatIDCliente, mensajeSugerencia, "", botonesOpciones);
                                    
                                    // Ponemos al sistema en espera de la respuesta del usuario
                                    esperandoRespuestaTelegram = true;
                                    tiempoSugerencia = millis();
                                }
                            }
                        }

                        // Calculamos si con este ritmo de gasto se pasara de su limite a fin de mes (150 kWh)
                        float consumoKWh = (getAverageConsumption(24) * 24.0 * 30.0) / 1000.0;
                        verificarConsumoMensual(consumoKWh, 150.0);

                        // Finalmente, le pasamos los datos electricos al nucleo 0 para subirlos a internet
                        char* bufferEnvio = strdup(datos.c_str());
                        if (queueCore1ToMQTT != NULL) { 
                            if (xQueueSend(queueCore1ToMQTT, &bufferEnvio, 0) != pdTRUE) free(bufferEnvio); 
                        } else {
                            free(bufferEnvio);
                        }
                    }
                }
            }
        }
        
        // Guardamos de a ratos el historial de luz en la memoria interna por si se apaga el equipo
        autoSaveHistorial();
        
        // REVISIONES DE TIEMPO FUERA (TIMEOUTS) DE SEGURIDAD
        
        // Si el usuario prendio la lampara a mano desde el movil y paso mas de 1 hora...
        if (getLamparaManual() && (millis() - getLastManualCommand() > MANUAL_TIMEOUT_MS)) {
            // Se la apagamos y devolvemos el control a los sensores automaticos para ahorrar energia
            setLamparaManual(false);
            setLamparaEstado(0); 
            Serial.println("[INFERENCIA] Restriccion manual expirada. Liberando proceso (Apagado Lampara) al supervisor automatico.");
            
            // Avisamos al Gateway por cable
            char* cmd1 = strdup("ACTUADOR:1:ESTADO:0");
            if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd1, 0);
            char* cmd2 = strdup("MODO:LAMPARA:AUTOMATICO");
            if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd2, 0);
            
            // Avisamos a la nube MQTT
            char* bufferEnvio;
            asprintf(&bufferEnvio, "PUBLICAR_LAMPARA:0:%d", calcularOrigen()); 
            if (queueCore1ToMQTT != NULL) { 
                if (xQueueSend(queueCore1ToMQTT, &bufferEnvio, 0) != pdTRUE) free(bufferEnvio); 
            } else {
                free(bufferEnvio);
            }
        }
        
        // Hacemos exactamente lo mismo si se olvido prendido el ventilador/calefactor
        if (getVentiladorManual() && (millis() - getLastManualCommand() > MANUAL_TIMEOUT_MS)) {
            setVentiladorManual(false);
            setVentiladorEstado(0); 
            Serial.println("[INFERENCIA] Restriccion manual expirada. Liberando proceso (Apagado Ventilador) al supervisor automatico.");
            
            char* cmd1 = strdup("ACTUADOR:2:ESTADO:0");
            if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd1, 0);
            char* cmd2 = strdup("MODO:VENTILADOR:AUTOMATICO");
            if (queueCore1ToGateway) xQueueSend(queueCore1ToGateway, &cmd2, 0);
            
            char* bufferEnvio;
            asprintf(&bufferEnvio, "PUBLICAR_VENTILADOR:0:%d", calcularOrigen()); 
            if (queueCore1ToMQTT != NULL) { 
                if (xQueueSend(queueCore1ToMQTT, &bufferEnvio, 0) != pdTRUE) free(bufferEnvio); 
            } else {
                free(bufferEnvio);
            }
        }
        
        // Le damos al procesador una pequena siesta de 50 milisegundos. 
        // Esto es obligatorio en FreeRTOS para que el nucleo no se sature y se reinicie.
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//   INICIALIZACION GLOBAL (EL ARRANQUE DEL SISTEMA)  
// Esta funcion es como el motor de arranque. Se ejecuta una sola vez cuando conectamos la placa a la corriente.
void setup() {
    // 1. Iniciamos la comunicacion con el ordenador para poder ver los mensajes de diagnostico en la pantalla
    Serial.begin(115200);
    
    // 2. Iniciamos la comunicacion por cable (UART1) con nuestro nodo Gateway
    Serial1.begin(SERIAL1_BAUDRATE, SERIAL_8N1, 14, 15); 
    
    // 3. Activamos el "perro guardian" (Watchdog) del hardware.
    // Le damos un limite de 45 segundos. Si nuestro codigo se llega a bloquear por algun motivo 
    // y no le avisa al perro que seguimos vivos, este reiniciara el chip automaticamente para salvarnos.
    #ifdef ESP_IDF_VERSION_MAJOR
      #if ESP_IDF_VERSION_MAJOR >= 5
        esp_task_wdt_config_t configWDT = { .timeout_ms = 45000, .idle_core_mask = 0, .trigger_panic = true };
        esp_task_wdt_reconfigure(&configWDT);
      #else
        esp_task_wdt_init(45, true);
      #endif
    #else
      esp_task_wdt_init(45, true);
    #endif
    esp_task_wdt_add(NULL); 
    
    delay(500); // Pequeña pausa de medio segundo para que la corriente en los pines se estabilice

    // 4. Sincronizacion de arranque. Le avisamos al Gateway por cable que acabamos de encender.
    // Por seguridad, le exigimos que apague todos los actuadores y los ponga en modo automatico.
    Serial.println("[SISTEMA] Disparando bloque logico predeterminado (Gateway)");
    Serial1.println("ACTUADOR:1:ESTADO:0"); 
    delay(50);
    Serial1.println("ACTUADOR:2:ESTADO:0");
    delay(50);
    Serial1.println("MODO:LAMPARA:AUTOMATICO");
    delay(50);
    Serial1.println("MODO:VENTILADOR:AUTOMATICO");
    delay(50);
    Serial1.flush(); // Aseguramos que todos los textos salgan por el cable sin quedarse atascados
    
    // 5. Dejamos todas nuestras variables de memoria interna limpias y en cero para empezar con el pie derecho
    lamparaControlManual = false; 
    ventiladorControlManual = false;
    lastManualCommand = 0; 
    estadoLampara = 0; 
    estadoVentilador = 0;
    waitingForAck = false; 
    pendingCommand = ""; 
    pendingRetries = 0; 
    ackReceived = false;
    
    // 6. Creamos los "Semaforos" y los "Buzones" (Colas de datos).
    // Como vamos a tener dos nucleos del procesador trabajando al mismo tiempo, estas herramientas 
    // evitan que los nucleos choquen entre si si intentan modificar la misma variable a la vez.
    xMutexVariables = xSemaphoreCreateMutex();
    queueGatewayToCore1 = xQueueCreate(20, sizeof(char*));
    queueCore1ToGateway = xQueueCreate(20, sizeof(char*));
    queueCore1ToMQTT = xQueueCreate(20, sizeof(char*));
    
    // 7. Arrancamos el disco duro interno de la placa (LittleFS)
    if (!LittleFS.begin(true)) {
        Serial.println("[ERROR IO] Sector de archivos LittleFS comprometido o vacio.");
    } else {
        // Intentamos cargar el historial de consumo de luz del usuario
        if (!loadHistorialFromLittleFS()) {
            // Si no hay historial porque el equipo es nuevo, preparamos la memoria en blanco
            historyIndex = 0; 
            historyCount = 0; 
            historyFull = false;
            memset(consumptionHistory, 0, sizeof(consumptionHistory));
            memset(lastSavedHistory, 0, sizeof(lastSavedHistory));
            hayCambiosEnHistorial = false;
        }
    }

    // 8. Cargamos nuestro cerebro (la red neuronal de TensorFlow) y verificamos si hay un usuario usuario vinculado
    cargarModeloIA();
    cargarClienteVinculado();

    // 9. Sincronizamos todos nuestros cronometros internos a la hora de arranque
    lastHistorialSave = millis(); 
    lastProgrammedRestart = millis(); 
    lastStatus = millis();
    tiempoAnteriorOTA = millis(); 
    
    // 10. Prendemos la antena WiFi en modo cliente para conectarnos al router de la casa.
    // Un detalle importante: fijamos el canal de WiFi al numero 6 para evitar 
    // que haga interferencia con la red de radio que usa el Gateway.
    WiFi.mode(WIFI_STA); 
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    
    // 11. ¡EL GRAN FINAL DEL ARRANQUE! Dividimos nuestro codigo en los dos nucleos fisicos.
    // Core 0: Toma la responsabilidad de mantener el WiFi, la nube y hablar por el cable.
    // Core 1: Se queda procesando la Inteligencia Artificial y contestando por Telegram.
    xTaskCreatePinnedToCore(taskCore0, "Core0_Task", 16384, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(taskCore1, "Core1_Task", 16384, NULL, 1, NULL, 1);
}

//   LOOP PRINCIPAL ASINCRONO  
// Como ya le dimos todo el trabajo pesado a los Nucleos 0 y 1, este ciclo nativo de Arduino 
// se queda casi vacio. Lo usamos como un administrador general para tareas muy lentas.
void loop() { 
    // Le avisamos al perro guardian que la placa sigue viva
    esp_task_wdt_reset(); 
    
    // Verificamos por seguridad que no se nos este llenando la memoria RAM
    auditarSaludHardware(); 
    
    unsigned long tiempoActual = millis();

    // Revisor de Actualizaciones Semanales (OTA)
    // Si ya pasaron 7 dias exactos desde la ultima vez que revisamos...
    if (tiempoActual - tiempoAnteriorOTA >= intervaloOTA) {
        tiempoAnteriorOTA = tiempoActual;
        
        Serial.println("\n[SISTEMA] Reloj temporal de 7 dias expirado. Solicitando modelo binario remoto.");
        
        // Intentamos descargar el nuevo cerebro de IA desde el servidor que creamos
        if (descargarModeloOTA()) {
            Serial.println("[SISTEMA] Operacion OTA Exitosa. Reiniciando hardware para cargar nueva IA...");
            delay(1500); 
            // Si la descarga fue un exito, reiniciamos el equipo para que reemplace la IA antigua
            ESP.restart(); 
        } else {
            // Si no hay internet o cayo el servidor, no pasa nada, se pospone y lo intenta la proxima vez
            Serial.println("[SISTEMA] Proceso interrumpido. El ciclo OTA se pospondra por fallo de descarga hasta proximo ciclo.");
        }
    }

    // Le damos un respiro enorme de 1 segundo a este ciclo porque no tiene ningun apuro de ejecucion
    delay(1000); 
}