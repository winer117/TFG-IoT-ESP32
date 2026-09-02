/*
    
   ACTUADOR - VENTILADOR / CALEFACTOR (ID 2)
    
   
   DESCRIPCION GENERAL:
   Este nodo es el responsable físico de encender o apagar el ventilador (o calefactor). 
   Se conecta a la red inalámbrica Mesh del hogar y obedece las órdenes del Gateway.
   
   CARACTERISTICAS PRINCIPALES:
   - Tolerancia a Fallos: Implementa un Watchdog lógico. Si el nodo pierde comunicación 
   - con el Gateway por más de 120 segundos, se reinicia solo para intentar reconectarse.
   - Sincronización Inmediata: Cuando logra conectarse a la red, avisa de inmediato 
   - quién es ("IDENTIFY:2") y en qué estado se encuentra ("STATE:2").
   - Estrategia Híbrida de Memoria: Para evitar el desgaste del disco interno (NVS) 
   - por escribir datos a cada rato, los cambios rápidos se guardan en la memoria RTC 
   - (que es muy rápida y sobrevive a reinicios). Solo se hace un respaldo al disco NVS 
   - como medida de seguridad.
   - Sistema de Acuses (ACK): Al recibir y ejecutar una orden, siempre envía una 
   - confirmación de vuelta (CONFIRM:2) para que el Gateway sepa que la tarea se cumplió.
    
 */

#include <painlessMesh.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ========== PARAMETROS DE LA RED MESH ==========
#define MESH_PREFIX     "Mesh_Red_Usuaeio"
#define MESH_PASSWORD   "Mehs_clave"
#define MESH_PORT       5555
#define MESH_CHANNEL    6
#define HEARTBEAT_TIMEOUT_MS 120000 // 120 segundos de tolerancia ante pérdida de red

// ========== ASIGNACION DE PINES FÍSICOS ==========
#define PIN_LED          2 // LED integrado en la placa para indicar funcionamiento
#define PIN_RELE         4 // Pin de control de potencia para el relé

// ========== LOGICA DE ESTADOS DEL RELE ==========
// Dependiendo del módulo de relé físico, HIGH o LOW pueden significar encendido o apagado
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// ========== VARIABLES GLOBALES ==========
painlessMesh mesh;
Preferences prefs;

bool estadoActual = false;
bool gatewayConectado = false;
unsigned long ultimoHeartbeat = 0;
bool identificado = false;

// ========== ZONA DE MEMORIA SEGURA (RTC) ==========
// Estas variables se alojan en una sección especial de la RAM que no pierde energía 
// ni siquiera cuando el chip ejecuta un comando de reinicio (ESP.restart).
RTC_DATA_ATTR bool estadoRTC = false;
RTC_DATA_ATTR unsigned long timestampRTC = 0;
RTC_DATA_ATTR int contadorCambiosRTC = 0;

// Escribe permanentemente el estado en el almacenamiento no volátil de la placa (Flash)
void guardarEstadoEnNVS(bool estado) {
    prefs.begin("actuador", false);
    prefs.putBool("estado", estado);
    prefs.end();
    Serial.println("[MEMORIA] Respaldo de estado sincronizado en el disco interno (NVS).");
}

// ========== CONTROL DEL HARDWARE (RELE) ==========
// Esta función traduce la orden lógica de software en un pulso eléctrico real en los pines
void controlarRele(bool estado) {
    bool estadoAnterior = estadoActual;
    estadoActual = estado;
    
    // Acción de fuerza bruta incluso si el software cree que el ventilador ya está encendido
    // en el estado correcto, forzamos el pulso eléctrico. Esto previene desincronizaciones 
    // en caso de que el relé físico haya sufrido un microcorte de energía.
    digitalWrite(PIN_RELE, estadoActual ? RELAY_ON : RELAY_OFF);
    digitalWrite(PIN_LED, estadoActual ? HIGH : LOW);
    
    // Si el estado lógico realmente cambió frente a lo que teníamos antes...
    if (estadoAnterior != estadoActual) {
        Serial.println(estadoActual ? "[HARDWARE] Accionando rele: Ventilador ENCENDIDO" : "[HARDWARE] Accionando rele: Ventilador APAGADO");
        
        // Guardamos el cambio en la memoria ultrarrápida (RTC)
        estadoRTC = estadoActual;
        timestampRTC = millis();
        contadorCambiosRTC++;
        
        // Realizamos el respaldo formal en el disco interno (NVS)
        guardarEstadoEnNVS(estadoActual);
    } else {
        Serial.println("[SISTEMA] Refresco de pin fisico ejecutado (Estado sin cambios lógicos).");
    }
}

// ========== RECEPCIÓN DE MENSAJES DE LA RED ==========
// Aquí aterrizan todos los textos que el Gateway u otros nodos mandan por el aire
void receivedCallback(uint32_t from, String &msg) {
    ultimoHeartbeat = millis();
    gatewayConectado = true;
    
    Serial.printf("[RX MESH] Trama entrante desde nodo %u: %s\n", from, msg.c_str());

    // CASO 1: Si la red está haciendo un censo de dispositivos, respondemos con nuestro DNI
    if (msg == "IDENTIFY") {
        mesh.sendSingle(from, "IDENTIFY:2");
        identificado = true;
        Serial.println(" - Trama reconocida. Respondiendo identidad a la red: ACTUADOR:2.");
        return;
    }

    // CASO 2: Si el Gateway necesita saber cómo tenemos el relé en este preciso momento
    if (msg == "REQUEST_STATE:2") {
        mesh.sendSingle(from, "STATE:2:" + String(estadoActual ? 1 : 0));
        Serial.printf(" - Transmitiendo estado actual al Gateway: %s\n", estadoActual ? "ENCENDIDO" : "APAGADO");
        return;
    }

    // CASO 3: Es una orden directa para encender o apagar nuestro relé
    if (msg.startsWith("ACTUADOR:2:ESTADO:")) {
        int nuevoEstado = msg.substring(msg.lastIndexOf(":") + 1).toInt();
        
        Serial.printf(" - Instruccion de control recibida: Nivel %d\n", nuevoEstado);
        
        controlarRele(nuevoEstado == 1);
        
        // Acuse de recibo. Le decimos al Gateway que la tarea se cumplió.
        mesh.sendSingle(from, "CONFIRM:2");
        Serial.printf(" - Protocolo completado. Confirmacion de ejecucion (ACK) enviada al nodo %u.\n", from);
    }
    
    // CASO 4: Trama de supervivencia. Nos avisa que el Gateway sigue vivo y administrando la red.
    if (msg == "KEEPALIVE") {
        ultimoHeartbeat = millis();
        gatewayConectado = true;
        mesh.sendSingle(from, "ALIVE");
        Serial.println(" - Senal de mantenimiento validada. Enlace de radiofrecuencia estable.");
    }
}

// Se ejecuta cada vez que este nodo logra emparejarse con otro integrante de la red Mesh
void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("\n[RED MESH] Enlace topológico establecido con el nodo: %u\n", nodeId);
    gatewayConectado = true;
    ultimoHeartbeat = millis();
    
    // Sincronización post-enlace: Avisamos proactivamente quiénes somos y cómo estamos
    mesh.sendSingle(nodeId, "IDENTIFY:2");
    identificado = true;
    Serial.println(" - Transmitiendo identidad raiz...");
    
    mesh.sendSingle(nodeId, "REQUEST_STATE:2");
    Serial.println(" - Solicitando vectores de estado de recuperacion...");
    
    if (estadoActual) {
        mesh.sendSingle(nodeId, "ACTUADOR:2:ESTADO:1");
        Serial.println(" - Reporte de telemetria emitido: ENCENDIDO.");
    } else {
        mesh.sendSingle(nodeId, "ACTUADOR:2:ESTADO:0");
        Serial.println(" - Reporte de telemetria emitido: APAGADO.");
    }
}

// Función de protección (Watchdog lógico). Vigila que no quedemos aislados de la red.
void verificarConexion() {
    if (gatewayConectado && (millis() - ultimoHeartbeat > HEARTBEAT_TIMEOUT_MS)) {
        Serial.println("\n[SISTEMA CRÍTICO] Perdida total de enlace con el nodo raiz (Gateway).");
        Serial.println("[SISTEMA CRÍTICO] Ejecutando reinicio de red para intentar recuperacion de ruta...");
        Serial.flush();
        delay(500);
        ESP.restart();
    }
}

// Interfaz de diagnóstico para leer los parámetros internos del nodo desde un ordenador
void mostrarEstado() {
    Serial.println("           PANEL DE ESTADO - ACTUADOR TÉRMICO            ");
    Serial.println("---------------------------------------------------------");
    Serial.printf(" Estado Logico y Fisico: %s\n", estadoActual ? "ACTIVO" : "INACTIVO");
    Serial.printf(" Enlace de Red (Gateway): %s\n", gatewayConectado ? "ESTABLE" : "PERDIDO");
    Serial.printf(" Registro de Identidad: %s\n", identificado ? "COMPLETADO" : "PENDIENTE");
    Serial.printf(" Conteo de transacciones RTC: %d\n", contadorCambiosRTC);
    Serial.printf(" Limite de Tolerancia Red: %d segundos\n", HEARTBEAT_TIMEOUT_MS/1000);
    Serial.printf(" Frecuencia Operativa (Canal): %d\n", WiFi.channel());
    Serial.println("---------------------------------------------------------\n");
}

// Rutina de configuración al encender el microcontrolador
void setup() {
    Serial.begin(115200);
    delay(2000); // Pausa de cortesía para que la energía se estabilice en la placa
    
    Serial.println("\n=========================================================");
    Serial.println("      INICIALIZANDO SUBSISTEMA: ACTUADOR TÉRMICO (ID 2)  ");
    Serial.println("=========================================================\n");
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_RELE, OUTPUT);
    
    // Por precaución, siempre arrancamos apagados a nivel eléctrico
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_RELE, RELAY_OFF);
    estadoActual = false;
    
    // ========== ARRANQUE CON MEMORIA HÍBRIDA ==========
    // Primero, revisamos la memoria RAM ultrarrápida (RTC) para ver si fue un reinicio rápido
    if (estadoRTC) {
        estadoActual = estadoRTC;
        Serial.println("[SISTEMA] Parametros operativos restaurados exitosamente desde la memoria RTC.");
    } else {
        // Si no hay datos en la RAM, significa que hubo un apagón físico. 
        // Procedemos a extraer la copia de seguridad desde el disco duro interno (NVS).
        prefs.begin("actuador", true);
        estadoActual = prefs.getBool("estado", false);
        prefs.end();
        Serial.println("[SISTEMA] Respaldo de parámetros extraído desde el almacenamiento flash (NVS).");
        
        // Volvemos a llenar la memoria RAM rápida para futuras operaciones
        estadoRTC = estadoActual;
        timestampRTC = millis();
        contadorCambiosRTC = 0;
    }
    
    // Imponemos el estado lógico recuperado a los pines eléctricos reales
    digitalWrite(PIN_RELE, estadoActual ? RELAY_ON : RELAY_OFF);
    digitalWrite(PIN_LED, estadoActual ? HIGH : LOW);
    Serial.printf("[SISTEMA] Configuración eléctrica aplicada. Estado final: %s.\n", estadoActual ? "ACTIVO" : "INACTIVO");
    
    // Inicialización de la red local
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[RED] Transmisor configurado en el canal base: %d.\n", MESH_CHANNEL);
    
    Serial.println("[RED] Desplegando topología de red compartida (MESH)...");
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_STA, MESH_CHANNEL);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    
    Serial.printf("[RED] Asignación lógica completada. Nodo ID: %u\n", mesh.getNodeId());
    
    // Proceso de presentación oficial ante la red de la casa
    delay(1000);
    mesh.sendBroadcast("IDENTIFY:2");
    identificado = true;
    Serial.println("[RED] Protocolo de presentación concluido. Dispositivo registrado y operativo.");
    
    ultimoHeartbeat = millis();
}

// Bucle maestro del microcontrolador
void loop() {
    // La librería painlessMesh requiere que esta función corra continuamente 
    // para procesar el tráfico de aire y mantener las tablas de enrutamiento
    mesh.update();
    
    // Monitoreo constante de la salud del enlace
    verificarConexion();
    
    // Tarea de mantenimiento: Guardamos de forma segura el estado en el disco interno (NVS)
    // una vez cada hora para prevenir pérdidas graves de datos ante cortes prolongados de energía.
    static unsigned long lastNVSBackup = 0;
    if (millis() - lastNVSBackup > 3600000) {
        lastNVSBackup = millis();
        guardarEstadoEnNVS(estadoActual);
    }
    
    // Impresión del diagnóstico por consola cada 30 segundos
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 30000) {
        lastStatus = millis();
        mostrarEstado();
    }
    
    // Pequeño descanso de 10 milisegundos para oxigenar el procesador 
    // y evitar problemas con los temporizadores internos del ESP32
    delay(10);
}