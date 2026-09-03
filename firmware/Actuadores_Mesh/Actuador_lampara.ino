/*
    
   ACTUADOR - LÁMPARA (ID 1)
    
   
   DESCRIPCION GENERAL:
   Este nodo es el responsable físico de controlar el encendido de la iluminación. 
   Se enlaza a la red Mesh de la casa y ejecuta las órdenes enviadas por el Gateway.
   
   CARACTERISTICAS PRINCIPALES:
   - Tolerancia a Fallos: Cuenta con un Watchdog lógico que reinicia el chip 
   - si pierde contacto con el Gateway durante más de 120 segundos.
   - Recuperación Automática: Cuando logra conectarse a la red, avisa de inmediato 
   - quién es ("IDENTIFY:1") y en qué estado se encuentra ("STATE:1").
   - Estrategia Híbrida de Memoria: Para alargar la vida útil del disco interno (NVS) 
   - evitando miles de escrituras, los cambios cotidianos de encendido/apagado se 
   - guardan en la memoria RTC (ultra rápida). Solo se hace un respaldo al NVS 
   - cada cierto tiempo por seguridad.
   - Sistema de Acuses (ACK): Al recibir y ejecutar una orden, siempre envía una 
   - confirmación ("CONFIRM:1") para que el Gateway sepa que la luz realmente cambió.
    
 */

#include <painlessMesh.h>
#include <esp_wifi.h>
#include <Preferences.h>

//   PARAMETROS DE ACCESO A LA RED MESH    
#define MESH_PREFIX     "Usuario_mesh"
#define MESH_PASSWORD   "clave_mesh"
#define MESH_PORT       5555
#define MESH_CHANNEL    6
#define HEARTBEAT_TIMEOUT_MS 120000      // 120 segundos de tolerancia ante pérdida de red

//   ASIGNACION DE PINES FÍSICOS    
#define PIN_LED          2    // LED integrado en la placa para indicar funcionamiento
#define PIN_RELE         4    // Pin de control de potencia para el relé

//   LOGICA DE ESTADOS DEL RELE    
// Dependiendo del módulo físico de relé, HIGH o LOW pueden significar encendido o apagado
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

//   VARIABLES GLOBALES    
painlessMesh mesh;
Preferences prefs;

bool estadoActual = false;
bool gatewayConectado = false;
unsigned long ultimoHeartbeat = 0;

// Bandera que confirma si nos hemos presentado oficialmente ante la red
bool identificado = false;

//   ZONA DE MEMORIA SEGURA (RTC)    
// Estas variables se alojan en un área de RAM que sobrevive a los reinicios rápidos (ESP.restart)
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

//   CONTROL DEL HARDWARE (RELE)    
// Esta función traduce la orden de software en energía real para encender la luz
void controlarRele(bool estado) {
    // Si la placa ya tiene el relé en la posición que nos piden, ignoramos la orden 
    // para no someter a estrés innecesario a los componentes mecánicos del relé.
    if (estadoActual == estado) {
        Serial.println("[SISTEMA] Instruccion omitida. El circuito ya se encuentra en el estado solicitado.");
        return; 
    }
    
    estadoActual = estado;
    digitalWrite(PIN_RELE, estadoActual ? RELAY_ON : RELAY_OFF);
    Serial.println(estadoActual ? "[HARDWARE] Accionando rele: Lampara ENCENDIDA" : "[HARDWARE] Accionando rele: Lampara APAGADA");
    
    // Anotamos este cambio en nuestra memoria rápida RTC
    estadoRTC = estadoActual;
    timestampRTC = millis();
    contadorCambiosRTC++;
    
    // Hacemos una copia de seguridad profunda en la memoria Flash
    guardarEstadoEnNVS(estadoActual);
    
    // Encendemos o apagamos el foquito azul de la placa ESP32 para saber si funcionó
    digitalWrite(PIN_LED, estadoActual ? HIGH : LOW);
}

// RECEPCIÓN DE MENSAJES DE LA RED    
// Aquí entran todos los comandos que envía el Gateway por el aire
void receivedCallback(uint32_t from, String &msg) {
    ultimoHeartbeat = millis();
    gatewayConectado = true;
    
    Serial.printf("[RX MESH] Trama entrante desde nodo %u: %s\n", from, msg.c_str());

    // CASO 1: Censo de dispositivos. La red pregunta quiénes somos.
    if (msg == "IDENTIFY") {
        mesh.sendSingle(from, "IDENTIFY:1");
        identificado = true;
        Serial.println(" - Trama reconocida. Respondiendo identidad a la red: ACTUADOR:1 (LAMPARA).");
        return; 
    }

    // CASO 2: El Gateway nos exige que le informemos el estado real del relé en este momento
    if (msg == "REQUEST_STATE:1") {
        mesh.sendSingle(from, "STATE:1:" + String(estadoActual ? 1 : 0));
        Serial.printf(" - Transmitiendo estado actual al Gateway: %s\n", estadoActual ? "ENCENDIDA" : "APAGADA");
        return;
    }

    // CASO 3: Es una orden directa para prender o apagar la luz
    if (msg.startsWith("ACTUADOR:1:ESTADO:")) {
        int nuevoEstado = msg.substring(msg.lastIndexOf(":") + 1).toInt();
        
        controlarRele(nuevoEstado == 1);
        
        // Enviamos el acuse de recibo de regreso al Gateway para que deje de retransmitir la orden
        mesh.sendSingle(from, "OK: Instruccion procesada");
        mesh.sendSingle(from, "CONFIRM:1");
        Serial.printf(" - Protocolo completado. Confirmacion de ejecucion (ACK) enviada al nodo %u.\n", from);
    }
    
    // CASO 4: Trama de supervivencia del Gateway (Heartbeat)
    if (msg == "KEEPALIVE") {
        ultimoHeartbeat = millis();
        gatewayConectado = true;
        mesh.sendSingle(from, "ALIVE");
        Serial.println(" - Senal de mantenimiento validada. Enlace de radiofrecuencia estable.");
    }
}

// Se dispara cuando esta placa logra conectarse con el Gateway o la red Mesh
void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("\n[RED MESH] Enlace topologico establecido con el nodo: %u\n", nodeId);
    gatewayConectado = true;
    ultimoHeartbeat = millis();
    
    // Nos presentamos automáticamente para que el Gateway nos registre
    mesh.sendSingle(nodeId, "IDENTIFY:1");
    identificado = true;
    Serial.println(" - Transmitiendo identidad raiz: ACTUADOR:1 (LAMPARA).");
    
    // Solicitamos parámetros o informamos cómo está nuestro foco al conectarnos
    mesh.sendSingle(nodeId, "REQUEST_STATE:1");
    Serial.println(" - Solicitando sincronizacion con nodo raiz...");
    
    if (estadoActual) {
        mesh.sendSingle(nodeId, "ACTUADOR:1:ESTADO:1");
        Serial.println(" - Reporte de telemetria emitido: ENCENDIDA.");
    } else {
        mesh.sendSingle(nodeId, "ACTUADOR:1:ESTADO:0");
        Serial.println(" - Reporte de telemetria emitido: APAGADA.");
    }
}

// Función de protección (Watchdog lógico). Nos reinicia si perdemos al Gateway
void verificarConexion() {
    if (gatewayConectado && (millis() - ultimoHeartbeat > HEARTBEAT_TIMEOUT_MS)) {
        Serial.println("\n[SISTEMA CRITICO] Perdida total de enlace con el nodo raiz (Gateway).");
        Serial.println("[SISTEMA CRITICO] Ejecutando reinicio de red para intentar recuperacion...");
        Serial.flush();
        delay(500);
        ESP.restart();
    }
}

// Interfaz de diagnóstico para leer los parámetros por el monitor serie de la PC
void mostrarEstado() {
    Serial.println("           PANEL DE ESTADO - ACTUADOR ILUMINACION        ");
    Serial.println("---------------------------------------------------------");
    Serial.printf(" Estado Logico y Fisico: %s\n", estadoActual ? "ACTIVO" : "INACTIVO");
    Serial.printf(" Enlace de Red (Gateway): %s\n", gatewayConectado ? "ESTABLE" : "PERDIDO");
    Serial.printf(" Registro de Identidad: %s\n", identificado ? "COMPLETADO" : "PENDIENTE");
    Serial.printf(" Conteo de transacciones RTC: %d\n", contadorCambiosRTC);
    Serial.printf(" Limite de Tolerancia Red: %d segundos\n", HEARTBEAT_TIMEOUT_MS/1000);
    Serial.printf(" Frecuencia Operativa (Canal): %d\n", WiFi.channel());
    Serial.println("---------------------------------------------------------\n");
}

// Configuración inicial del nodo
void setup() {
    Serial.begin(115200);
    delay(2000); // Pausa de cortesía para la estabilización eléctrica
    
    Serial.println("\n ------------------------------------------------------");
    Serial.println("      INICIALIZANDO SUBSISTEMA: ACTUADOR LÁMPARA (ID 1)  ");
    Serial.println("                   \n");
    
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_RELE, OUTPUT);
    
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_RELE, RELAY_OFF);
    estadoActual = false;
    
    //   ARRANQUE CON MEMORIA HÍBRIDA    
    // Comprobamos la memoria RAM primero por si fue un reinicio lógico leve
    if (estadoRTC) {
        estadoActual = estadoRTC;
        Serial.println("[SISTEMA] Parametros operativos restaurados exitosamente desde la memoria RTC.");
    } else {
        // Si no hay datos, fue un apagón. Usamos el respaldo lento pero permanente (NVS).
        prefs.begin("actuador", true);
        estadoActual = prefs.getBool("estado", false);
        prefs.end();
        Serial.println("[SISTEMA] Respaldo de parametros extraido desde el almacenamiento flash (NVS).");
        
        // Emparejamos ambas memorias para el funcionamiento diario
        estadoRTC = estadoActual;
        timestampRTC = millis();
        contadorCambiosRTC = 0;
    }
    
    digitalWrite(PIN_RELE, estadoActual ? RELAY_ON : RELAY_OFF);
    digitalWrite(PIN_LED, estadoActual ? HIGH : LOW);
    Serial.printf("[SISTEMA] Configuracion electrica aplicada. Estado final: %s.\n", estadoActual ? "ACTIVA" : "INACTIVA");
    
    // Configuración de la red local
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(MESH_CHANNEL, WIFI_SECOND_CHAN_NONE);
    Serial.printf("[RED] Transmisor configurado en el canal base: %d.\n", MESH_CHANNEL);
    
    Serial.println("[RED] Desplegando topologia de red compartida (MESH)...");
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_STA, MESH_CHANNEL);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    
    Serial.printf("[RED] Asignacion logica completada. Nodo ID: %u\n", mesh.getNodeId());
    
    delay(1000); 
    mesh.sendBroadcast("IDENTIFY:1");
    identificado = true;
    Serial.println("[RED] Protocolo de presentacion concluido. Dispositivo registrado y operativo.");
    
    ultimoHeartbeat = millis();
}

// Ciclo maestro de trabajo
void loop() {
    // Mantener vivas las tablas de comunicación de la red Mesh
    mesh.update();
    
    // Cuidar que no nos hayamos quedado desconectados en la nada
    verificarConexion();
    
    // Cada hora realizamos una escritura de seguridad en el disco flash interno
    static unsigned long lastNVSBackup = 0;
    if (millis() - lastNVSBackup > 3600000) {  
        lastNVSBackup = millis();
        guardarEstadoEnNVS(estadoActual);
    }
    
    // Reporte periódico en pantalla
    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 30000) {
        lastStatus = millis();
        mostrarEstado();
    }
    
    delay(10); // Pequeña pausa para no sobresaturar el procesador
}