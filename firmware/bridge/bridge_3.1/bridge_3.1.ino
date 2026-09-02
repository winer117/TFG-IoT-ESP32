/*

 NODO BRIDGE - ESP32 #2
 CONTROLADOR DE RED IOT MESH Y CONTROL DE ACTUADORES

  
 ARQUITECTURA Y CARACTERISTICAS:
 - Arquitectura Hibrida: ESP-NOW (comunicacion de sensores) y UART (hacia el Gateway).
 - Arranque Seguro: Estado inicial en OFFLINE (desconectado) hasta confirmacion de telemetria.
 - Monitorizacion Logica: Deteccion y reporte de perdida de conexion (Timeout 30s reinio seguro)
 - Auto-Recuperacion: Restauracion automatica de estado ante reconexiones.
 - Tolerancia a Fallos: Implementacion de Watchdog Timer (perroguardian) (WDT) de hardware nativo.
 - Gestion de Memoria: Monitoreo continuo de heap RAM y reinicios preventivos.
 - Persistencia: Retencion de estado mediante RTC Memory y memoria Flash (Preferences, recuerda el ultimo estado ante un reincio).
 - Control Termico: Termostato integrado con margen de tenperatura ( mas 0.5 C).
 */


#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h> 
#include <rom/rtc.h>      
#include <Preferences.h>  

// Instancia para almacenamiento en memoria Flash No Volatil
Preferences prefs;

// Variables con retencion en memoria RTC (sobreviven a reinicios por software/o por falta de suministro de energia )
RTC_DATA_ATTR int rtc_bootCount = 0;
RTC_DATA_ATTR float rtc_tempActivacion = 18.0;
RTC_DATA_ATTR bool rtc_ventiladorActivo = false;
RTC_DATA_ATTR unsigned long rtc_tiempoRestanteVentilador = 0;
RTC_DATA_ATTR bool rtc_modoManual = false;
RTC_DATA_ATTR bool rtc_alertaAmbientalEnviada = true;
RTC_DATA_ATTR bool rtc_alertaElectricoEnviada = true;

// CONFIGURACION DE RED Y HARDWARE
#define FIXED_CHANNEL 6

// Variables dinamicas de control
float tempActivacion = 18.0; 
#define VENTILADOR_TIMEOUT_MS (20UL * 60UL * 1000UL)

// TEMPORIZADORES DE RED Y SISTEMA 
#define TIMEOUT_SENSOR_MS 30000UL      // Tiempo maximo de espera por sensor (30s)
#define WATCHDOG_TIMEOUT_MS 900000UL   // Tiempo maximo de inactividad de ESP-NOW (15m)

#define UPTIME_MAX_MS 604800000UL      // Reinicio preventivo programado (7 dias)
#define WDT_LOOP_TIMEOUT 30            // Timeout para el ciclo principal (30s)

// Direcciones MAC de nodos origen (Modificar por las MAC reales del hardware)
uint8_t sensorMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // MAC del ESP32 Ambiental (ESP 32 sensor)
uint8_t sensorElectricoMAC[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // MAC del ESP32 Electrico (ESP 32 sensor)

// ESTRUCTURAS DE DATOS
struct DatosSensor {
  int idSensor;
  float temperatura;
  int movimiento;
  float voltajeBateria;
  int porcentajeBateria;
  unsigned long tiempo;
  int msgID;
};

struct DatosElectricos {
  int idSensor;
  float voltajeRMS;
  float corrienteRMS;
  float potencia;
  float factorPotencia;
  unsigned long tiempo;
  int msgID;
};

// VARIABLES DE ESTADO Y MONITOREO
int totalMensajes = 0;
unsigned long lastSensorTime = 0;

// Variables de control de telemetria
unsigned long lastDataReceived = 0;
unsigned long lastAmbientalTime = 0;
unsigned long lastElectricoTime = 0;

// Estados de alerta de conectividad
bool alertaAmbientalEnviada = true;  
bool alertaElectricoEnviada = true;  

// Variables de actuadores
unsigned long ventiladorEncendidoHasta = 0;
bool ventiladorActivo = false;
unsigned long lastManualVentilador = 0;
#define MANUAL_VENTILADOR_TIMEOUT_MS 3600000

int ultimoEstadoVentiladorEnviado = -1;

unsigned long tiempoInicioBridge = 0;
#define TIEMPO_IGNORAR_GATEWAY_MS 10000

unsigned long tiempoUltimoApagadoManual = 0;
#define TIEMPO_IGNORAR_ENCENDIDO_MS 5000

unsigned long lastReenvioEstado = 0;
#define INTERVALO_REENVIO_MS 60000

// Variable de prioridad y sincronizacion de modos
bool modoManualVentiladorBridge = false;

//  GESTION DE RECURSOS DEL SISTEMA 
void auditarSaludHardware() {
    uint32_t ramLibre = ESP.getFreeHeap();
    unsigned long uptime = millis();

    // Prevencion de desbordamiento de memoria o fugas
    if (ramLibre < 20000) {
        Serial.printf("\n[SISTEMA CRITICO] Memoria RAM insuficiente (%u bytes). Ejecutando reinicio preventivo...\n", ramLibre);
        delay(500);
        ESP.restart();
    }

    //Mantenimiento programado para liberar fragmentacion del SDK (memoria)
    if (uptime > UPTIME_MAX_MS) {
        Serial.println("\n[MANTENIMIENTO] Reinicio programado alcanzado (Limpieza de SDK).");
        delay(500);
        ESP.restart();
    }
}

//  CONTROL DE TRANSMISION (EVITA REDUNDANCIA)
void enviarEstadoVentilador(int estado) {
    if (estado == 0) {
        ultimoEstadoVentiladorEnviado = -1;
    }
    
    if (estado != ultimoEstadoVentiladorEnviado) {
        ultimoEstadoVentiladorEnviado = estado;
        Serial2.println("STATUS:VENTILADOR:" + String(estado));
        Serial.printf("[TX] Actualizacion de estado de ventilador: %s\n", estado ? "ENCENDIDO" : "APAGADO");
    } else {
        Serial.printf("[TX] Estado de ventilador sin cambios: %s (Omitiendo transmision duplicada)\n", estado ? "ENCENDIDO" : "APAGADO");
    }
}

// ========== RUTINA DE RECEPCION ESP-NOW ==========
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    totalMensajes++;
    lastSensorTime = millis();
    lastDataReceived = millis(); 
    
    int idSensor;
    memcpy(&idSensor, incomingData, sizeof(int));
    
    if (idSensor == 1) {
        // Procesamiento Nodo Ambiental
        lastAmbientalTime = millis();
        
        if (alertaAmbientalEnviada) {
            Serial.println("[CONECTIVIDAD] Nodo Ambiental (ID:1) - Estado: ONLINE");
            Serial2.println("ALERTA:SENSOR_AMBIENTAL:ONLINE");
            alertaAmbientalEnviada = false;
        }

        DatosSensor datos;
        memcpy(&datos, incomingData, sizeof(datos));
        
        int ackID = datos.msgID;
        esp_now_send(sensorMAC, (uint8_t*)&ackID, sizeof(ackID));
        
        Serial.printf("[RX ESP-NOW] ID: %d | Temp: %.1f C | Mov: %d | Bat: %.2fV (%d%%)\n",
                      totalMensajes, datos.temperatura, 
                      datos.movimiento,
                      datos.voltajeBateria, datos.porcentajeBateria);
        
        Serial2.printf("%d,%.1f,%d,%.2f,%d\n",
                       datos.idSensor, datos.temperatura, datos.movimiento,
                       datos.voltajeBateria, datos.porcentajeBateria);
        
        // Evaluacion logica del termostato autonomo
        if (!modoManualVentiladorBridge) {
            
            // 1. Corte termico con histeresis superior
            if (datos.temperatura >= (tempActivacion + 0.5) && ventiladorActivo) {
                ventiladorActivo = false;
                ventiladorEncendidoHasta = 0; 
                Serial.printf("[TERMOSTATO] Umbral de corte alcanzado (%.1f C). Desactivando actuador.\n", datos.temperatura);
                Serial2.println("ACTUADOR:2:ESTADO:0");
                enviarEstadoVentilador(0);
            }
            // 2. Activacion por demanda termica y presencia
            else if (datos.movimiento == 1 && datos.temperatura < tempActivacion) {
                ventiladorEncendidoHasta = millis() + VENTILADOR_TIMEOUT_MS;
                
                if (!ventiladorActivo) {
                    ventiladorActivo = true;
                    Serial.printf("[TERMOSTATO] Activacion termica (%.1f C). Ciclo programado: %d min.\n", 
                                  datos.temperatura, VENTILADOR_TIMEOUT_MS / 60000);
                    Serial2.println("ACTUADOR:2:ESTADO:1");
                    enviarEstadoVentilador(1);
                } else {
                    Serial.printf("[TERMOSTATO] Extendiendo ciclo del actuador (+%d min).\n", VENTILADOR_TIMEOUT_MS / 60000);
                }
            }
        } else {
            Serial.println("[TERMOSTATO] Excepcion: Prioridad de modo manual activa.");
        }
    }
    else if (idSensor == 5) {
        // Procesamiento Nodo Electrico
        lastElectricoTime = millis();
        
        if (alertaElectricoEnviada) {
            Serial.println("[CONECTIVIDAD] Nodo Electrico (ID:5) - Estado: ONLINE");
            Serial2.println("ALERTA:SENSOR_ELECTRICO:ONLINE");
            alertaElectricoEnviada = false;
        }

        DatosElectricos datos;
        memcpy(&datos, incomingData, sizeof(datos));
        
        int ackID = datos.msgID;
        esp_now_send(sensorElectricoMAC, (uint8_t*)&ackID, sizeof(ackID));
        
        Serial.printf("[RX ESP-NOW] ID: %d | V: %.1fV | I: %.2fA | P: %.1fW | FP: %.2f\n",
                      totalMensajes,
                      datos.voltajeRMS,
                      datos.corrienteRMS,
                      datos.potencia,
                      datos.factorPotencia);
        
        Serial2.printf("E,%.1f,%.2f,%.1f,%.2f\n",
                       datos.voltajeRMS,
                       datos.corrienteRMS,
                       datos.potencia,
                       datos.factorPotencia);
    }
}

//  RUTINA DE LECTURA UART (DESDE GATEWAY) 
void leerGateway() {
    if (Serial2.available()) {
        String mensaje = Serial2.readStringUntil('\n');
        mensaje.trim();
        
        if (mensaje.length() == 0) return;
        
        // Periodo de gracia para estabilizacion del bus UART en arranque
        if (millis() - tiempoInicioBridge < TIEMPO_IGNORAR_GATEWAY_MS) {
            Serial.printf("[UART] Inicializando bus. Trama omitida: %s\n", mensaje.c_str());
            return;
        }
        
        Serial.printf("[UART RX] Trama recibida: %s\n", mensaje.c_str());
        
        // Actualizacion de parametros de control en memoria Flash
        if (mensaje.startsWith("CONFIG:UMBRAL:")) {
            tempActivacion = mensaje.substring(mensaje.lastIndexOf(":") + 1).toFloat();
            
            prefs.begin("termostato", false);
            prefs.putFloat("umbral", tempActivacion);
            prefs.end();
            
            Serial.printf("[SISTEMA] Nuevo umbral termico almacenado en NVRAM: %.1f C\n", tempActivacion);
            return;
        }
        
        // Gestion de prioridades y modos de operacion
        if (mensaje.startsWith("MODO:VENTILADOR:MANUAL")) {
            modoManualVentiladorBridge = true;
            lastManualVentilador = millis();
            Serial.println("[SISTEMA] Modificacion de prioridad: Control Manual activado.");
            return;
        }
        else if (mensaje.startsWith("MODO:VENTILADOR:AUTOMATICO")) {
            modoManualVentiladorBridge = false;
            Serial.println("[SISTEMA] Modificacion de prioridad: Control Automatico activado.");
            return;
        }
        
        if (mensaje.startsWith("CONFIRM:2")) {
            Serial.printf("[UART] Acuse de recibo (ACK) del controlador secundario.\n");
            return;
        }
        
        if (mensaje.startsWith("RECONECTADO:")) {
            int id = mensaje.substring(mensaje.indexOf(":") + 1).toInt();
            if (id == 2 && ventiladorActivo) {
                Serial.println("[UART] Dispositivo reconectado. Sincronizando estado vigente.");
                Serial2.println("ACTUADOR:2:ESTADO:1");
            }
            return;
        }
        
        // Retrocompatibilidad de tramas
        if (mensaje.startsWith("MANUAL:ACTUADOR:2:ESTADO:")) {
            int estado = mensaje.substring(mensaje.lastIndexOf(":") + 1).toInt();
            if (estado == 1) {
                modoManualVentiladorBridge = true;
                lastManualVentilador = millis();
                Serial.println("[SISTEMA] Modo manual forzado a activo.");
            } else {
                modoManualVentiladorBridge = false;
                Serial.println("[SISTEMA] Modo manual desactivado.");
            }
            return;
        }
        
        // Procesamiento de comandos remotos de actuacion
        if (mensaje.startsWith("ACTUADOR:2:ESTADO:")) {
            int estado = mensaje.substring(mensaje.lastIndexOf(":") + 1).toInt();
            
            if (modoManualVentiladorBridge) {
                Serial.printf("[SISTEMA] Comando omitido por conflicto de prioridad (Modo manual activo).\n");
                return;
            }
            
            if (millis() - tiempoUltimoApagadoManual < TIEMPO_IGNORAR_ENCENDIDO_MS) {
                Serial.printf("[SISTEMA] Comando omitido (Periodo de gracia de actuador).\n");
                return;
            }
            
            if (estado == 1) {
                unsigned long tiempoRestante = 0;
                if (ventiladorActivo && ventiladorEncendidoHasta > millis()) {
                    tiempoRestante = (ventiladorEncendidoHasta - millis()) / 1000;
                }
                
                if (!ventiladorActivo || tiempoRestante < 30) {
                    ventiladorActivo = true;
                    ventiladorEncendidoHasta = millis() + VENTILADOR_TIMEOUT_MS;
                    Serial.println("[CONTROL] Actuacion de hardware: Ciclo de encendido iniciado.");
                    Serial2.println("ACTUADOR:2:ESTADO:1");
                    enviarEstadoVentilador(1);
                }
            } else {
                ventiladorActivo = false;
                Serial.println("[CONTROL] Actuacion de hardware: Ciclo interrumpido.");
                Serial2.println("ACTUADOR:2:ESTADO:0");
                enviarEstadoVentilador(0);
            }
            return;
        }
        
        if (mensaje.startsWith("CONFIRM:")) {
            int id = mensaje.substring(mensaje.indexOf(":") + 1).toInt();
            Serial.printf("[UART] Confirmacion de pasarela para actuador %d registrada.\n", id);
            return;
        }
    }
}

// CONTROL Y AUDITORIA DE TEMPORIZADORES 
void verificarTemporizadores() {
    unsigned long ahora = millis();
    
    // Revocacion de prioridad manual por inactividad
    if (modoManualVentiladorBridge && (ahora - lastManualVentilador > MANUAL_VENTILADOR_TIMEOUT_MS)) {
        modoManualVentiladorBridge = false;
        Serial.println("[SISTEMA] Expiracion de prioridad manual. Retornando a control logico.");
    }
    
    static unsigned long lastPrintTime = 0;
    if (ventiladorActivo && (ahora - lastPrintTime > 5000)) {
        lastPrintTime = ahora;
        unsigned long tiempoRestante = (ventiladorEncendidoHasta > ahora) ? (ventiladorEncendidoHasta - ahora) / 1000 : 0;
        Serial.printf("[ESTADO ACTUADOR] Tiempo restante: %lu seg | Origen: %s\n", 
                      tiempoRestante, modoManualVentiladorBridge ? "MANUAL" : "LOGICO");
    }
    
    // Ejecucion de timeout de actuador
    if (ventiladorActivo && ahora >= ventiladorEncendidoHasta) {
        if (modoManualVentiladorBridge) {
            Serial.println("[SISTEMA] Suspension de timeout (Prioridad manual activa).");
            ventiladorEncendidoHasta = ahora + MANUAL_VENTILADOR_TIMEOUT_MS;
            return;
        }
        
        ventiladorActivo = false;
        Serial.println("[CONTROL] Timeout logico expirado. Desconectando actuador.");
        Serial2.println("ACTUADOR:2:ESTADO:0");
        enviarEstadoVentilador(0);
        ultimoEstadoVentiladorEnviado = -1;
    }
}

// AUDITORIA DE SALUD DE RED (ESP-NOW) 
void verificarEstadoSensores() {
    unsigned long ahora = millis();
    
    // Tolerancia en el periodo de inicializacion
    if (ahora - tiempoInicioBridge < 15000) return; 
    
    // 1. Verificacion de enlace Nodo Ambiental
    if (!alertaAmbientalEnviada && (ahora - lastAmbientalTime > TIMEOUT_SENSOR_MS)) {
        Serial.println("\n[ALERTA RED] Perdida de comunicacion con Nodo Ambiental (Timeout logico excedido).");
        Serial2.println("ALERTA:SENSOR_AMBIENTAL:OFFLINE"); 
        alertaAmbientalEnviada = true; 
    }
    
    // 2. Verificacion de enlace Nodo Electrico
    if (!alertaElectricoEnviada && (ahora - lastElectricoTime > TIMEOUT_SENSOR_MS)) {
        Serial.println("\n[ALERTA RED] Perdida de comunicacion con Nodo Electrico (Timeout logico excedido).");
        Serial2.println("ALERTA:SENSOR_ELECTRICO:OFFLINE"); 
        alertaElectricoEnviada = true; 
    }

    // 3. Fallo catastrofico de capa PHY o stack de red
    if (lastDataReceived > 0 && (ahora - lastDataReceived > (WATCHDOG_TIMEOUT_MS + 60000UL))) {
        Serial.println("\n[SISTEMA CRITICO] Inactividad total en interfaz ESP-NOW detectada.");
        Serial.println("[SISTEMA] Iniciando recuperacion forzada mediante reinicio del SoC...");
        Serial.flush();
        delay(1000);
        ESP.restart();
    }
}

// INTERFAZ DE DIAGNOSTICO (CLI) MOSTRMSO ESTADOS DEL ESP 32 BRIDGE
void mostrarEstado() {
    unsigned long tiempoRestante = 0;
    if (ventiladorActivo && ventiladorEncendidoHasta > millis()) {
        tiempoRestante = (ventiladorEncendidoHasta - millis()) / 1000;
    }
    
    Serial.println("\n[INFO] Resumen de Estado del Nodo Bridge:");
    Serial.printf(" - Mensajes procesados: %d\n", totalMensajes);
    Serial.printf(" - Inactividad Rx (ESP-NOW): %lu seg\n", (millis() - lastDataReceived) / 1000);
    Serial.printf(" - Conectividad Ambiental: %s\n", alertaAmbientalEnviada ? "OFFLINE" : "ONLINE");
    Serial.printf(" - Conectividad Electrico: %s\n", alertaElectricoEnviada ? "OFFLINE" : "ONLINE");
    Serial.printf(" - Actuador Termico: %s\n", ventiladorActivo ? "ACTIVO" : "INACTIVO");
    Serial.printf(" - Prioridad de Control: %s\n", modoManualVentiladorBridge ? "MANUAL" : "AUTOMATICO");
    if (ventiladorActivo) {
        Serial.printf(" - Tiempo Restante: %lu seg\n", tiempoRestante);
    }
    Serial.printf(" - Umbral Termico Base: %.1f C\n", tempActivacion);
    Serial.printf(" - Canal RF Asignado: %d\n", WiFi.channel());
}

// INICIALIZACION DEL SISTEMA 
void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17);
    
    // Inicializacion de Task Watchdog (perro guardian de tareas)
    esp_task_wdt_init(WDT_LOOP_TIMEOUT, true);
    esp_task_wdt_add(NULL);

    Serial.println("\n[SISTEMA] Inicializando Controlador Mesh Bridge (Nodo #2)");
    Serial.println("[SISTEMA] Interfaces configuradas: ESP-NOW (Rx) / UART (Tx)");
    Serial.println("[SISTEMA] Enlaces esperados: Ambiental (ID:1) / Electrico (ID:5)");
    
    // Recuperacion de variables en NVRAM (Preferences)
    prefs.begin("termostato", true);
    tempActivacion = prefs.getFloat("umbral", 18.0); 
    prefs.end();
    Serial.printf("[MEMORIA] Parametro de termostato restaurado: %.1f C\n", tempActivacion);

    // Deteccion de motivo de reinicio y restauracion desde memoria RTC (Warm Boot persisetncai de varaibles para su funcionamiento)
    esp_reset_reason_t reason = esp_reset_reason();
    bool reinicioPorFallo = (reason == ESP_RST_SW || reason == ESP_RST_PANIC || reason == ESP_RST_INT_WDT || reason == ESP_RST_TASK_WDT || reason == ESP_RST_WDT);

    if (reinicioPorFallo && rtc_bootCount > 0) {
        Serial.println("\n[SISTEMA] Recuperacion de contexto desde memoria de retencion RTC iniciada.");
        
        // Condicionales de sincronizacion temporal de variables
        if (rtc_tempActivacion != 18.0) {
            tempActivacion = rtc_tempActivacion;
        }
        ventiladorActivo = rtc_ventiladorActivo;
        modoManualVentiladorBridge = rtc_modoManual;
        alertaAmbientalEnviada = rtc_alertaAmbientalEnviada;
        alertaElectricoEnviada = rtc_alertaElectricoEnviada;
        
        if (ventiladorActivo && rtc_tiempoRestanteVentilador > 0) {
            ventiladorEncendidoHasta = millis() + rtc_tiempoRestanteVentilador;
        }
    } else {
        // Ejecucion Cold Boot
        alertaAmbientalEnviada = true;
        alertaElectricoEnviada = true;
    }
    rtc_bootCount++;

    // Configuracion de interfaz RF (radiofrecuencia)
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(FIXED_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[RED] Canal RF fijado en banda: %d\n", FIXED_CHANNEL);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR CRITICO] Fallo en la inicializacion del stack ESP-NOW.");
        return;
    }
    esp_now_register_recv_cb(OnDataRecv);
    
    // Registro de pares de red
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, sensorMAC, 6);
    peerInfo.channel = FIXED_CHANNEL;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    esp_now_peer_info_t peerInfoElectrico = {};
    memcpy(peerInfoElectrico.peer_addr, sensorElectricoMAC, 6);
    peerInfoElectrico.channel = FIXED_CHANNEL;
    peerInfoElectrico.encrypt = false;
    esp_now_add_peer(&peerInfoElectrico);
    
    Serial.printf("[SISTEMA] Direccion MAC asignada: %s\n", WiFi.macAddress().c_str());
    Serial.printf("[CONFIG] Temporizador de actuador termico: %d min\n", VENTILADOR_TIMEOUT_MS / 60000);
    Serial.printf("[CONFIG] Timeout de revocacion manual: %d min\n", MANUAL_VENTILADOR_TIMEOUT_MS / 60000);
    
    // Sincronizacion de los tiempos base
    lastSensorTime = millis();
    lastDataReceived = millis();
    lastAmbientalTime = millis();
    lastElectricoTime = millis();
    tiempoInicioBridge = millis();
    
    if (!reinicioPorFallo) { 
        ventiladorActivo = false;
        modoManualVentiladorBridge = false;
        ultimoEstadoVentiladorEnviado = -1;
        enviarEstadoVentilador(0);
    }
    
    Serial.println("\n[SISTEMA] Rutina de inicio completada. A la espera de telemetria de red.\n");
}

// CICLO DE EJECUCION PRINCIPAL
void loop() {
    esp_task_wdt_reset(); 
    
    auditarSaludHardware(); 
    
    verificarTemporizadores();
    leerGateway();
    verificarEstadoSensores(); 
    
    // Rutina de integridad y refuerzo de comandos UART (comunicacion por cable)
    if (millis() - lastReenvioEstado > INTERVALO_REENVIO_MS) {
        lastReenvioEstado = millis();
        
        if (ventiladorActivo && !modoManualVentiladorBridge) {
            unsigned long tiempoRestante = 0;
            if (ventiladorEncendidoHasta > millis()) {
                tiempoRestante = (ventiladorEncendidoHasta - millis()) / 1000;
            }
            
            if (tiempoRestante > 30) {
                Serial.printf("[MANTENIMIENTO RED] Retransmitiendo comando de actuacion (%lu seg restantes).\n", tiempoRestante);
                Serial2.println("ACTUADOR:2:ESTADO:1");
            }
        }
    }
    
    // Telemetria de diagnostico local (muestra el estado de la plca ESP32 bridge (puente))
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 30000) {
        lastStatus = millis();
        mostrarEstado();
    }
    
    // Respaldo continuo en memoria RTC para contingencias de hardware
    rtc_tempActivacion = tempActivacion;
    rtc_ventiladorActivo = ventiladorActivo;
    rtc_modoManual = modoManualVentiladorBridge;
    rtc_alertaAmbientalEnviada = alertaAmbientalEnviada;
    rtc_alertaElectricoEnviada = alertaElectricoEnviada;
    if (ventiladorActivo && ventiladorEncendidoHasta > millis()) {
        rtc_tiempoRestanteVentilador = ventiladorEncendidoHasta - millis();
    } else {
        rtc_tiempoRestanteVentilador = 0;
    }

    delay(10);
}