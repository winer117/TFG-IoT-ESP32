/*
 NODO GATEWAY - ESP32-S3
 ENRUTADOR MESH Y GESTOR DE COMUNICACIONES SERIALES


 
 COMPONENTES Y CARACTERISTICAS:
 - Topologia Central: Actua como nodo raiz MESH y puente bidireccional UART (CONEXION POR CABLE).
 - Sincronizacion de Estados: Restaura actuadores tras perdidas de conexion.
 - Persistencia NVRAM: Retencion de comandos y modos mediante libreria (Preferences).
 - Tolerancia a Fallos: Implementacion de Watchdog distribuido para nodos actuadores.
 - Mantenimiento Autonomo: Reinicio preventivo programado cada 4 horas (para evitar corrupcion de memoria).
 - Control Horario: Automatizacion de iluminacion basada en franjas nocturnas.
 - Gestion de Prioridades: El control manual suspende los comandos automaticos.
 - Acuses de Recibo (ACK): Protocolo de confirmacion y retransmision de comunicacion.
*/

#include <painlessMesh.h>
#include <esp_wifi.h>
#include <map>
#include <Preferences.h>

// CONFIGURACION MESH   
// Credenciales y usuario de la red mes que creara el sistema 
#define MESH_PREFIX     "Nombre_redMesh"
#define MESH_PASSWORD   "contrseña_mesh"
#define MESH_PORT       5555
#define MESH_CHANNEL    6 // Configuramos un canal predeterminado para evitar caidas de los nodos

// PARAMETROS DEL WATCHDOG DISTRIBUIDO   
#define HEARTBEAT_INTERVAL_MS 15000
#define NODE_TIMEOUT_MS 75000
#define MAX_CONSECUTIVE_FAILURES 3

// REINICIO PROGRAMADO PARA EVITAR QUE SE COROMPA LA MEMORIA 
#define TIEMPO_REINICIO_HORAS 4
#define TIEMPO_REINICIO_MS (TIEMPO_REINICIO_HORAS * 3600UL * 1000UL)

// CONFIRMACION DE TRAMAS   
#define TIEMPO_ESPERA_CONFIRMACION_MS 2000

// RESTAURACION PERIODICA   
#define STATE_RESTORE_INTERVAL_MS 30000

// CONFIGURACION HORARIO   
int horaInicioNoche = 17; // Horario predeterminado que el usuario podra variar segun confort
int horaFinNoche = 2;     

#define LAMPARA_TIMEOUT_MS (10UL * 60UL * 1000UL)  // Tiempo de duracion de encendido de la lampara despues de activarse por el sensor
#define MANUAL_TIMEOUT_MS 3600000 // Tiempo que durara el modo manual enviado por el usuario

// ID DEL ACTUADOR OBJETIVO   
#define ACTUADOR_ID 1017574301

// TASAS DE TRANSMISION SERIAL DE DATOS   
#define SERIAL1_BAUDRATE 921600
#define SERIAL2_BAUDRATE 115200

// VARIABLES GLOBALES   
painlessMesh mesh;
Preferences prefs;
int totalComandos = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastStatus = 0;
unsigned long lastAliveFromActuador = 0;
unsigned long lastProgrammedRestart = 0;
unsigned long lastStateRestore = 0;
bool reinicioPorFallo = false;
bool actuadorRegistrado = false;

std::map<uint32_t, unsigned long> mapaLatidos;
std::map<uint32_t, int> mapaFallos;
std::map<uint32_t, int> ultimoComando;

// VARIABLES PARA CONFIRMACION   
bool confirmacionRecibida = false;
unsigned long tiempoEnvio = 0;
String ultimoComandoEnviado = "";
int actuadorConfirmado = -1;
int intentosReenvio = 0;

// VARIABLES PARA CONTROL HORARIO   
bool horaInicializada = false;
unsigned long lamparaEncendidaHasta = 0;
bool lamparaActiva = false;
bool lamparaControlManual = false;       
unsigned long lastManualCommand = 0;
int horaActual = 0;

// VARIABLES PARA VENTILADOR   
bool ventiladorControlManualGateway = false;
unsigned long lastManualVentiladorGateway = 0;
#define MANUAL_VENTILADOR_TIMEOUT_MS 3600000 // Tiempo que estara activo por el modo manual el vetilador (calefactor)

// VARIABLES PARA MAPEO DE ACTUADORES   
std::map<uint32_t, int> mapaNodeToActuador;
uint32_t nodeIdVentilador = 0;

// VARIABLES PARA PREVENIR BUCLE   
int lastProcessedField1 = -1;
int lastProcessedField2 = -1;

// PROTOTIPOS   
void enviarComandoConPersistencia(int actuadorID, int estado, bool desdeMQTT = false);
void procesarDatosSensor(String comando);
void verificarTimeoutLampara();
bool esDeNoche();
void enviarModoAlEdge(int actuadorID, bool manual);
void enviarEstadoYModoAlEdge(int actuadorID, int estado, bool manual);
void enviarEstadoCompletoAlEdge();

// FUNCIONES DE PERSISTENCIA NVRAM   
void guardarComando(uint32_t nodeId, int estado) {
    if (nodeId == nodeIdVentilador || nodeId == 2) return;
    ultimoComando[nodeId] = estado;
    prefs.begin("gateway", false);
    prefs.putInt(("cmd_" + String(nodeId)).c_str(), estado);
    prefs.putBool("lampara_manual", lamparaControlManual);
    prefs.end();
}

void restaurarComandos() {
    prefs.begin("gateway", true);
    for (auto &nodo : ultimoComando) {
        uint32_t nodeId = nodo.first;
        int estado = prefs.getInt(("cmd_" + String(nodeId)).c_str(), -1);
        if (estado != -1) {
            if (nodeId == nodeIdVentilador || nodeId == 2) continue;
            mesh.sendBroadcast("ACTUADOR:1:ESTADO:" + String(estado));
            delay(100);
        }
    }
    prefs.end();
}

// LOGICA PARA EL HORARIO   
bool esDeNoche() {
    if (!horaInicializada) {
        Serial.println("[HORARIO] Sincronizacion pendiente. Asumiendo periodo nocturno por seguridad.");
        return true;
    }
    
    int hora = horaActual;
    bool esNoche = false;

    if (horaInicioNoche > horaFinNoche) { 
        esNoche = (hora >= horaInicioNoche || hora < horaFinNoche); 
    } else {
        esNoche = (hora >= horaInicioNoche && hora < horaFinNoche);
    }
    
    static unsigned long lastLog = 0;
    if (millis() - lastLog > 300000) {  
        lastLog = millis();
        Serial.printf("[HORARIO] Reloj local: %02d:00 - Periodo activo: %s (Ventana configurada: %d:00 a %d:00)\n", 
                      hora, esNoche ? "NOCTURNO" : "DIURNO", horaInicioNoche, horaFinNoche);
    }
    
    return esNoche;
}

// SINCRONIZACION DE MODOS (UART1 CONEXIÓN POR CABLE)
void enviarModoAlEdge(int actuadorID, bool manual) {
    if (actuadorID == 1) {
        if (manual) {
            Serial1.println("MODO:LAMPARA:MANUAL");
            Serial.printf("[TX UART1] Sincronizacion: MODO:LAMPARA:MANUAL\n");
        } else {
            Serial1.println("MODO:LAMPARA:AUTOMATICO");
            Serial.printf("[TX UART1] Sincronizacion: MODO:LAMPARA:AUTOMATICO\n");
        }
        Serial1.flush();
    } else if (actuadorID == 2) {
        if (manual) {
            Serial1.println("MODO:VENTILADOR:MANUAL");
        } else {
            Serial1.println("MODO:VENTILADOR:AUTOMATICO");
        }
        Serial1.flush();
    }
    //  Para agregar mas actuadores solo ponemos un
    // 'else if (actuadorID == 3)' aquí para sincronizar su modo manual/automático.
}
  //Comunicacion con el edge controller
void enviarEstadoYModoAlEdge(int actuadorID, int estado, bool manual) {
    enviarModoAlEdge(actuadorID, manual);
    delay(10);
    String cmd = "ACTUADOR:" + String(actuadorID) + ":ESTADO:" + String(estado);
    Serial1.println(cmd);
    Serial1.flush();
    Serial.printf("[TX UART1] Transmision de estado: %s\n", cmd.c_str());
}

void enviarEstadoCompletoAlEdge() {
    Serial1.println("GATEWAY:REBOOT");
    Serial1.flush();
    Serial.println("[SISTEMA] Notificacion de inicializacion enviada al Edge Controller.");
    delay(50);
    
    int estadoL = lamparaActiva ? 1 : 0;
    Serial1.println("ACTUADOR:1:ESTADO:" + String(estadoL));
    Serial1.flush();
    Serial.printf("[SISTEMA] Restaurando estado Lampara: %d\n", estadoL);
    delay(20);
    
    int estadoV = 0;
    if (nodeIdVentilador != 0 && ultimoComando.find(nodeIdVentilador) != ultimoComando.end()) {
        estadoV = ultimoComando[nodeIdVentilador];
    } else if (ultimoComando.find(2) != ultimoComando.end()) {
        estadoV = ultimoComando[2];
    }
    Serial1.println("ACTUADOR:2:ESTADO:" + String(estadoV));
    Serial1.flush();
    Serial.printf("[SISTEMA] Restaurando estado Ventilador: %d\n", estadoV);
    delay(20);
    
    // PARA AGREGAR MÁS ACTUADORES: 
    //Repite el bloque de arriba para leer 'ultimoComando[3]' y asi enviar el estodo "ACTUADOR:3:ESTADO:X"
    
    Serial1.println("GATEWAY:REBOOT_COMPLETO");
    Serial1.flush();
    Serial.println("[SISTEMA] Secuencia de sincronizacion de estado completada.");
}

// PROCESAMIENTO DE SENSORES Y COMPORTAMIENTO   
void procesarDatosSensor(String comando) {
    if (lamparaControlManual) {
        Serial.println("[CONTROL] Prioridad manual activa. Omitiendo estimulo ambiental.");
        return;
    }
    
    int primeraComa = comando.indexOf(',');
    int segundaComa = comando.indexOf(',', primeraComa + 1);
    int terceraComa = comando.indexOf(',', segundaComa + 1);
    
    if (primeraComa == -1 || segundaComa == -1 || terceraComa == -1) return;
    
    int movimiento = comando.substring(segundaComa + 1, terceraComa).toInt();
    
    if (movimiento == 1 && esDeNoche()) {
        if (!lamparaControlManual) {
            lamparaEncendidaHasta = millis() + LAMPARA_TIMEOUT_MS;
            if (!lamparaActiva) {
                lamparaActiva = true;
                lastProcessedField1 = 1;
                mesh.sendBroadcast("ACTUADOR:1:ESTADO:1");
                enviarEstadoYModoAlEdge(1, 1, false);
                Serial2.println("ACTUADOR:1:ESTADO:1");
                Serial2.flush();
                Serial.println("[CONTROL AUTOMATICO] Presencia detectada. Activando circuito de iluminacion.");
            } else {
                lamparaEncendidaHasta = millis() + LAMPARA_TIMEOUT_MS;
                Serial.println("[CONTROL AUTOMATICO] Presencia detectada. Extendiendo temporizador de iluminacion.");
            }
        }
    }
}

// GESTION DE TIMEOUTS (TIEMPOS FUERA) 
void verificarTimeoutLampara() {
    unsigned long ahora = millis();
    
    if (lamparaActiva) {
        unsigned long tiempoRestante = (lamparaEncendidaHasta > ahora) ? (lamparaEncendidaHasta - ahora) / 1000 : 0;
        static unsigned long lastPrint = 0;
        if (ahora - lastPrint > 5000) {
            lastPrint = ahora;
            if (lamparaControlManual) {
                Serial.printf("[ESTADO ACTUADOR] Circuito: Iluminacion | Modo: MANUAL | Timeout en: %lu seg\n", tiempoRestante);
            } else {
                Serial.printf("[ESTADO ACTUADOR] Circuito: Iluminacion | Modo: AUTOMATICO | Timeout en: %lu seg\n", tiempoRestante);
            }
        }
    }
    
    if (lamparaControlManual && !lamparaActiva) {
        lamparaControlManual = false;
        Serial.println("[CONTROL] Prioridad manual finalizada por desconexion del actuador.");
        enviarModoAlEdge(1, false);
    }
    
    if (lamparaControlManual && (ahora - lastManualCommand > MANUAL_TIMEOUT_MS)) {
        lamparaControlManual = false;
        Serial.println("[CONTROL] Expiracion de prioridad manual (Timeout de seguridad).");
        if (lamparaActiva) {
            lamparaEncendidaHasta = millis() + LAMPARA_TIMEOUT_MS;
            Serial.println("[CONTROL] Transicionando a modo automatico. Temporizador estandar aplicado.");
            enviarModoAlEdge(1, false);
        }
    }
    
    if (lamparaActiva && ahora >= lamparaEncendidaHasta && !lamparaControlManual) {
        lamparaActiva = false;
        mesh.sendBroadcast("ACTUADOR:1:ESTADO:0");
        enviarEstadoYModoAlEdge(1, 0, false);
        Serial2.println("ACTUADOR:1:ESTADO:0");
        Serial2.flush();
        Serial.println("[CONTROL AUTOMATICO] Temporizador expirado. Desconectando circuito de iluminacion.");
    }
}

// REVISIÓN DE REINICIOS   
void detectarCausaReinicio() {
    static bool primeraVez = true;
    if (primeraVez) {
        reinicioPorFallo = false;
        primeraVez = false;
    } else {
        reinicioPorFallo = true;
    }
}

// MANTENIMIENTO DE RED MESH   
void enviarHeartbeat() {
    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL_MS) {
        lastHeartbeat = millis();
        mesh.sendBroadcast("KEEPALIVE");
        Serial.println("[MESH] Trama de mantenimiento (Heartbeat) transmitida.");
    }
}

void verificarActuador() {
    unsigned long ahora = millis();
    if (actuadorRegistrado && (ahora - lastAliveFromActuador > NODE_TIMEOUT_MS)) {
        Serial.printf("\n[ALERTA MESH] Nodo actuador %u inalcanzable (Timeout logico).\n", ACTUADOR_ID);
        Serial.println("[SISTEMA] Ejecutando reinicio de modulo Gateway para recuperacion de red...");
        delay(5000);
        ESP.restart();
    }
}

void verificarReinicioProgramado() {
    if (millis() - lastProgrammedRestart > TIEMPO_REINICIO_MS) {
        Serial.println("\n[MANTENIMIENTO] Ejecutando reinicio preventivo programado.");
        delay(1000);
        ESP.restart();
    }
}

// CALLBACKS RED MESH   
void receivedCallback(uint32_t from, String &msg) {
    mapaLatidos[from] = millis();
    mapaFallos[from] = 0;
    lastAliveFromActuador = millis();
    actuadorRegistrado = true;
    
    Serial.printf("[RX MESH] Nodo origen: %u | Trama: %s\n", from, msg.c_str());
    
    Serial1.println(msg);
    Serial2.println(msg);
    
    if (msg.startsWith("IDENTIFY:")) {
        int actuadorID = msg.substring(msg.indexOf(":") + 1).toInt();
        mapaNodeToActuador[from] = actuadorID;
        if (actuadorID == 2) {
            nodeIdVentilador = from;
            Serial.printf("[MESH] Registro de actuador identificado: Ventilador (ID de Red: %u)\n", from);
            if (ultimoComando.find(from) != ultimoComando.end()) {
                int estado = ultimoComando[from];
                if (estado == 1) {
                    mesh.sendSingle(from, "ACTUADOR:2:ESTADO:1");
                }
            }
        }
        return;
    }
    
    if (msg.startsWith("ACTUADOR:1:ESTADO:")) {
        mapaNodeToActuador[from] = 1;
        int estado = msg.substring(msg.lastIndexOf(":") + 1).toInt();
        lamparaActiva = (estado == 1);
        guardarComando(from, estado);
        lastProcessedField1 = estado;
        Serial.printf("[ESTADO ACTUADOR] Circuito Iluminacion: %s\n", estado ? "ACTIVO" : "INACTIVO");
        enviarEstadoYModoAlEdge(1, estado, false);
    }
    else if (msg.startsWith("ACTUADOR:2:ESTADO:")) {
        mapaNodeToActuador[from] = 2;
        nodeIdVentilador = from;
        int estado = msg.substring(msg.lastIndexOf(":") + 1).toInt();
        ultimoComando[from] = estado;
        ultimoComando[2] = estado;
        lastProcessedField2 = estado;
        Serial.printf("[ESTADO ACTUADOR] Circuito Termico: %s\n", estado ? "ACTIVO" : "INACTIVO");
    }
    else if (msg.startsWith("ACTUADOR:3:ESTADO:")) {
        mapaNodeToActuador[from] = 3;
        int estado = msg.substring(msg.lastIndexOf(":") + 1).toInt();
        guardarComando(from, estado);
        Serial.printf("[ESTADO ACTUADOR] Circuito Auxiliar 3: %s\n", estado ? "ACTIVO" : "INACTIVO");
    }
    // PARA AGREGAR MÁS ACTUADORES: 
    //Añadimos un 'else if (msg.startsWith("ACTUADOR:4:ESTADO:"))' aquí para capturar su estado desde la red Mesh.
    
    if (msg.startsWith("CONFIRM:")) {
        int id = msg.substring(msg.lastIndexOf(":") + 1).toInt();
        confirmacionRecibida = true;
        actuadorConfirmado = id;
        Serial.printf("[MESH] Acuse de recibo validado (Actuador %d)\n", id);
        Serial1.println("CONFIRM:" + String(id));
        if (id == 1 || id == 2) {
            Serial2.println("CONFIRM:" + String(id));
        }
    }
}

void newConnectionCallback(uint32_t nodeId) {
    mapaLatidos[nodeId] = millis();
    mapaFallos[nodeId] = 0;
    lastAliveFromActuador = millis();
    actuadorRegistrado = true;
    Serial.printf("\n[MESH] Nuevo emparejamiento de red. ID de Nodo: %u\n", nodeId);
    
    mesh.sendSingle(nodeId, "IDENTIFY");
    Serial1.println("NODE:CONNECTED:" + String(nodeId));
    Serial2.println("NODE:CONNECTED:" + String(nodeId));
    
    if (ultimoComando.find(nodeId) != ultimoComando.end()) {
        int estado = ultimoComando[nodeId];
        if (estado == 1) {
            int actuadorID = 1;
            if (mapaNodeToActuador.find(nodeId) != mapaNodeToActuador.end()) {
                actuadorID = mapaNodeToActuador[nodeId];
            }
            
            if (actuadorID == 1) {
                if (esDeNoche()) {
                    mesh.sendBroadcast("ACTUADOR:1:ESTADO:1");
                    Serial.printf("[RECUPERACION] Condicion nocturna detectada. Restaurando circuito de iluminacion.\n");
                    enviarEstadoYModoAlEdge(1, 1, false);
                } else {
                    mesh.sendBroadcast("ACTUADOR:1:ESTADO:0");
                    Serial.printf("[RECUPERACION] Condicion diurna detectada. Suspendiendo restauracion de iluminacion.\n");
                }
            } else if (actuadorID == 2) {
                mesh.sendSingle(nodeId, "ACTUADOR:2:ESTADO:1");
                mesh.sendBroadcast("ACTUADOR:2:ESTADO:1");
                Serial.printf("[RECUPERACION] Restaurando actuador termico a estado ACTIVO.\n");
            } else {
                mesh.sendBroadcast("ACTUADOR:" + String(actuadorID) + ":ESTADO:1");
                Serial.printf("[RECUPERACION] Restaurando actuador auxiliar %d a estado ACTIVO.\n", actuadorID);
            }
        }
    }
}

void changedConnectionCallback() {
    int numNodos = mesh.getNodeList().size();
    Serial.printf("[MESH] Reconfiguracion topologica. Nodos activos actuales: %d\n", numNodos);
}

// INTERFAZ DE DIAGNOSTICO (CLI)   
void mostrarEstado() {
    Serial.println("\n[INFO] Resumen de Estado del Nodo Gateway:");
    Serial.printf(" - Comandos procesados: %d\n", totalComandos);
    Serial.printf(" - Nodos en subred MESH: %d\n", mesh.getNodeList().size());
    Serial.printf(" - Integridad de conexion actuador: %s\n", actuadorRegistrado ? "ESTABLE" : "PENDIENTE");
    Serial.printf(" - Identificador Actuador Termico: %s\n", nodeIdVentilador != 0 ? String(nodeIdVentilador).c_str() : "NO ASIGNADO");
    Serial.printf(" - Inactividad Actuador Principal: %lu seg\n", (millis() - lastAliveFromActuador) / 1000);
    Serial.printf(" - Circuito Iluminacion: %s\n", lamparaActiva ? "ACTIVO" : "INACTIVO");
    Serial.printf(" - Prioridad Iluminacion: %s\n", lamparaControlManual ? "MANUAL" : "AUTOMATICA");
    Serial.printf(" - Prioridad Termica: %s\n", ventiladorControlManualGateway ? "MANUAL" : "AUTOMATICA");
    Serial.printf(" - Enlace UART1 (Edge Controller): %d bps\n", SERIAL1_BAUDRATE);
    Serial.printf(" - Enlace UART2 (Bridge): %d bps\n", SERIAL2_BAUDRATE);
    Serial.printf(" - Intervalo KeepAlive (Heartbeat): %d seg\n", HEARTBEAT_INTERVAL_MS/1000);
    if (lamparaControlManual) {
        unsigned long restante = (lamparaEncendidaHasta > millis()) ? (lamparaEncendidaHasta - millis()) / 1000 : 0;
        Serial.printf(" - Tiempo Restante Prioridad Manual: %lu seg\n", restante);
    }
}

// Actualizacion peridodica de estados   
void restaurarEstadoPeriodico() {
    unsigned long ahora = millis();
    if (ahora - lastStateRestore < STATE_RESTORE_INTERVAL_MS) return;
    lastStateRestore = ahora;
    
    if (lamparaActiva && !esDeNoche() && !lamparaControlManual) {
        lamparaActiva = false;
        mesh.sendBroadcast("ACTUADOR:1:ESTADO:0");
        guardarComando(1, 0);
        Serial.println("[CONTROL AUTOMATICO] Periodo diurno validado. Suspendiendo circuito de iluminacion.");
        enviarEstadoYModoAlEdge(1, 0, false);
    }
    
    if (!ventiladorControlManualGateway && ultimoComando.find(2) != ultimoComando.end()) {
        int estado = ultimoComando[2];
        if (estado == 1) {
            mesh.sendBroadcast("ACTUADOR:2:ESTADO:1");
            if (nodeIdVentilador != 0) {
                mesh.sendSingle(nodeIdVentilador, "ACTUADOR:2:ESTADO:1");
            }
            Serial.println("[MANTENIMIENTO RED] Actualización periodica: Actuador Termico transmitido.");
        }
    }
    
    for (auto &nodo : ultimoComando) {
        uint32_t nodeId = nodo.first;
        int estado = nodo.second;
        if (estado == 1 && nodeId != nodeIdVentilador && nodeId != 2) {
            int actuadorID = 1;
            if (mapaNodeToActuador.find(nodeId) != mapaNodeToActuador.end()) {
                actuadorID = mapaNodeToActuador[nodeId];
            }
            
            if (actuadorID == 1) {
                if (lamparaControlManual) continue;
                if (esDeNoche() && lamparaActiva) {
                    mesh.sendBroadcast("ACTUADOR:1:ESTADO:1");
                    Serial.println("[MANTENIMIENTO RED] Actualización periodica: Circuito Iluminacion (Nocturno).");
                    enviarEstadoYModoAlEdge(1, 1, false);
                } else if (!esDeNoche()) {
                    if (lamparaActiva && !lamparaControlManual) {
                        lamparaActiva = false;
                        mesh.sendBroadcast("ACTUADOR:1:ESTADO:0");
                        Serial.println("[MANTENIMIENTO RED] Correccion de estado: Circuito Iluminacion (Diurno).");
                        enviarEstadoYModoAlEdge(1, 0, false);
                    }
                }
            } else {
                mesh.sendBroadcast("ACTUADOR:" + String(actuadorID) + ":ESTADO:1");
                Serial.printf("[MANTENIMIENTO RED] Actualización periodica: Actuador auxiliar %d.\n", actuadorID);
            }
        }
    }
}

// RUTINA DE TRANSMISION (CON PROTOCOLO ACK - COMUNICACION ENTRE PADRE E HIJOS)   
void enviarComandoConPersistencia(int actuadorID, int estado, bool desdeMQTT) {
    String cmd = "ACTUADOR:" + String(actuadorID) + ":ESTADO:" + String(estado);
    
    if (desdeMQTT) {
        Serial2.println(cmd);
        Serial2.flush();
        Serial.printf("[TX UART2] Direccionamiento de control: %s\n", cmd.c_str());
    }
    
    if (actuadorID == 2 && nodeIdVentilador != 0) {
        mesh.sendSingle(nodeIdVentilador, cmd);
        mesh.sendBroadcast(cmd);
        Serial.printf("[TX MESH] Transmision Unicast dirigida a nodo %u: %s\n", nodeIdVentilador, cmd.c_str());
    } else if (actuadorID != 2) {
        mesh.sendBroadcast(cmd);
        Serial.printf("[TX MESH] Transmision Broadcast: %s\n", cmd.c_str());
    }
    
    totalComandos++;
    
    confirmacionRecibida = false;
    tiempoEnvio = millis();
    ultimoComandoEnviado = cmd;
    actuadorConfirmado = -1;
    
    unsigned long startWait = millis();
    while (millis() - startWait < TIEMPO_ESPERA_CONFIRMACION_MS) {
        mesh.update();
        if (confirmacionRecibida) {
            Serial.printf("[MESH] Transaccion validada (ACK) en %lu ms. ID Dispositivo: %d\n", 
                          millis() - startWait, actuadorConfirmado);
            if (actuadorID != 2) {
                guardarComando(actuadorID, estado);
            }
            return;
        }
        delay(10);
    }
    
    if (!confirmacionRecibida) {
        Serial.println("[MESH] Timeout en recepcion de ACK. Inicializando rutina de retransmision.");
        if (actuadorID == 2 && nodeIdVentilador != 0) {
            mesh.sendSingle(nodeIdVentilador, cmd);
        } else if (actuadorID != 2) {
            mesh.sendBroadcast(cmd);
        }
        totalComandos++;
        delay(1000);
        if (actuadorID != 2) {
            guardarComando(actuadorID, estado);
        }
    }
}

// PROCESAMIENTO DE TRAMAS UART1 (comunicaicon por cable a l Edge controller)   
void leerEdgeController() {
    if (Serial1.available()) {
        String comando = Serial1.readStringUntil('\n');
        comando.trim();
        
        if (comando.length() == 0) return;
        
        if (comando.startsWith("ACK:")) {
            Serial1.println("ACK:" + comando);
            Serial1.flush();
            Serial.printf("[UART1 TX] Retransmision ACK: %s\n", comando.c_str());
            return;
        }

        if (comando.startsWith("CONFIG:UMBRAL:")) {
            Serial2.println(comando);
            Serial2.flush();
            Serial.printf("[RUTEO UART] Trama redireccionada hacia Bridge: %s\n", comando.c_str());
            return; 
        }

        if (comando.startsWith("CONFIG:HORA_INICIO:")) {
            horaInicioNoche = comando.substring(comando.lastIndexOf(":") + 1).toInt();
            prefs.begin("gateway", false);
            prefs.putInt("horaInicio", horaInicioNoche);
            prefs.end();
            Serial.printf("[CONFIGURACION] Modificacion de ventana nocturna: Hora de inicio %d:00\n", horaInicioNoche);
            return;
        }

        if (comando.startsWith("CONFIG:HORA_FIN:")) {
            horaFinNoche = comando.substring(comando.lastIndexOf(":") + 1).toInt();
            prefs.begin("gateway", false);
            prefs.putInt("horaFin", horaFinNoche);
            prefs.end();
            Serial.printf("[CONFIGURACION] Modificacion de ventana nocturna: Hora de fin %d:00\n", horaFinNoche);
            return;
        }
        
        Serial.printf("[UART1 RX] Trama desde Edge Controller: %s\n", comando.c_str());
        
        Serial1.println("ACK:" + comando);
        Serial1.flush();
        Serial.printf("[UART1 TX] Transmision de acuse: %s\n", comando.c_str());
        
        if (comando.startsWith("TIME:")) {
            String horaStr = comando.substring(5);
            horaStr.trim();
            
            int dosPuntos = horaStr.indexOf(':');
            if (dosPuntos != -1) {
                horaInicializada = true;
                horaActual = horaStr.substring(0, dosPuntos).toInt();
                Serial.printf("[SISTEMA] Sincronizacion de reloj en base completada: %s (Horas operativas: %d)\n", horaStr.c_str(), horaActual);
            } else {
                Serial.printf("[ERROR] Integridad de trama horaria comprometida: '%s'\n", comando.c_str());
            }
            return;
        }
                
        if (comando.startsWith("MODO:LAMPARA:")) {
            String modo = comando.substring(comando.indexOf(":") + 2);
            modo.trim();
            if (modo == "MANUAL") {
                lamparaControlManual = true;
                lastManualCommand = millis();
                lamparaActiva = true;
                lamparaEncendidaHasta = millis() + MANUAL_TIMEOUT_MS;
                Serial.println("[PRIORIDAD] Asignacion de modo MANUAL para circuito de iluminacion (Timeout 1H).");
            } else if (modo == "AUTOMATICO") {
                lamparaControlManual = false;
                Serial.println("[PRIORIDAD] Restauracion a modo AUTOMATICO para circuito de iluminacion.");
            }
            Serial2.println(comando);
            Serial2.flush();
            return;
        }
        
        if (comando.startsWith("MODO:VENTILADOR:")) {
            String modo = comando.substring(comando.indexOf(":") + 2);
            modo.trim();
            if (modo == "MANUAL") {
                ventiladorControlManualGateway = true;
                lastManualVentiladorGateway = millis();
                Serial.println("[PRIORIDAD] Asignacion de modo MANUAL para actuador termico.");
            } else if (modo == "AUTOMATICO") {
                ventiladorControlManualGateway = false;
                Serial.println("[PRIORIDAD] Restauracion a modo AUTOMATICO para actuador termico.");
            }
            Serial2.println(comando);
            Serial2.flush();
            return;
        }
        // PARA AGREGAR MÁS ACTUADORES: 
        // duplicamos el bloque de arriba cambiando "MODO:VENTILADOR:" por "MODO:ACTUADOR_3:"
        
        if (comando.startsWith("ACTUADOR:1:ESTADO:")) {
            int estado = comando.substring(comando.lastIndexOf(":") + 1).toInt();
            
            if (estado == 1) {
                lamparaControlManual = true;
                lastManualCommand = millis();
                lamparaActiva = true;
                lamparaEncendidaHasta = millis() + MANUAL_TIMEOUT_MS;
                Serial.println("[PRIORIDAD] Sobreescritura externa: Activacion manual forzada de iluminacion.");
                enviarModoAlEdge(1, true);
            } else {
                lamparaControlManual = false;
                lamparaActiva = false;
                lamparaEncendidaHasta = 0;
                Serial.println("[PRIORIDAD] Sobreescritura externa: Suspension manual de iluminacion.");
                enviarModoAlEdge(1, false);
            }
            
            guardarComando(1, estado);
            
            Serial2.println(comando);
            Serial2.flush();
            Serial.printf("[RUTEO UART] Trama redireccionada hacia Bridge: %s\n", comando.c_str());
            
            mesh.sendBroadcast(comando);
            Serial.printf("[TX MESH] Transmision Broadcast en subred: %s\n", comando.c_str());
            
            Serial1.println("CONFIRM:1:" + String(estado));
            Serial1.flush();
            Serial.printf("[UART1 TX] Ratificacion de operacion enviada al Edge: CONFIRM:1:%d\n", estado);
            return;
        }
        
        if (comando.startsWith("ACTUADOR:2:ESTADO:")) {
            int estado = comando.substring(comando.lastIndexOf(":") + 1).toInt();
            
            if (estado == 1) {
                ventiladorControlManualGateway = true;
                lastManualVentiladorGateway = millis();
                Serial.println("[PRIORIDAD] Sobreescritura externa: Activacion manual forzada de actuador termico.");
            } else {
                ventiladorControlManualGateway = false;
                Serial.println("[PRIORIDAD] Sobreescritura externa: Suspension manual de actuador termico.");
            }
            
            Serial2.println(comando);
            Serial2.flush();
            Serial.printf("[RUTEO UART] Trama redireccionada hacia Bridge: %s\n", comando.c_str());
            
            if (nodeIdVentilador != 0) {
                mesh.sendSingle(nodeIdVentilador, comando);
                Serial.printf("[TX MESH] Transmision Unicast al objetivo %u: %s\n", nodeIdVentilador, comando.c_str());
            }
            mesh.sendBroadcast(comando);
            Serial.printf("[TX MESH] Transmision de redundancia (Broadcast): %s\n", comando.c_str());
            
            ultimoComando[2] = estado;
            if (nodeIdVentilador != 0) {
                ultimoComando[nodeIdVentilador] = estado;
            }
            lastProcessedField2 = estado;
            
            Serial1.println("CONFIRM:2:" + String(estado));
            Serial1.flush();
            Serial.printf("[UART1 TX] Ratificacion de operacion enviada al Edge: CONFIRM:2:%d\n", estado);
            return;
        }
        //  PARA AGREGAR MÁS ACTUADORES: 
        // repetimos este bloque cambiando "ACTUADOR:2:ESTADO:" por "ACTUADOR:3:ESTADO:"
        
        else {
            Serial2.println(comando);
            Serial2.flush();
            Serial.printf("[RUTEO UART] Trama redireccionada hacia Bridge: %s\n", comando.c_str());
            mesh.sendBroadcast(comando);
            Serial.printf("[TX MESH] Transmision Broadcast en subred: %s\n", comando.c_str());
        }
    }
}

// PROCESAMIENTO DE TRAMAS UART2 (comunicacion por cable al BRIDGE)   
void leerBridge() {
    if (Serial2.available()) {
        String comando = Serial2.readStringUntil('\n');
        comando.trim();
        if (comando.length() == 0) return;
        
        Serial.printf("[UART2 RX] Trama desde Bridge: %s\n", comando.c_str());
        
        if (comando[0] >= '0' && comando[0] <= '9') {
            Serial1.println(comando);
            procesarDatosSensor(comando);
            return;
        }
        else if (comando.startsWith("E,")) {
            Serial1.println(comando);
            return;
        }
        else if (comando.startsWith("STATUS:VENTILADOR:")) {
            int estado = comando.substring(comando.lastIndexOf(":") + 1).toInt();
            if (nodeIdVentilador != 0) {
                ultimoComando[nodeIdVentilador] = estado;
            }
            ultimoComando[2] = estado;
            lastProcessedField2 = estado;
            Serial1.println(comando);
            Serial.printf("[SISTEMA] Sincronizacion de registro. Estado actuador termico actual: %s\n", estado ? "ACTIVO" : "INACTIVO");
            return;
        }
        else if (comando.startsWith("CONFIRM:")) {
            Serial1.println(comando);
            return;
        }
        else if (comando.startsWith("ACTUADOR:1:ESTADO:")) {
            int estado = comando.substring(comando.lastIndexOf(":") + 1).toInt();
            
            if (lamparaControlManual) {
                Serial.println("[SISTEMA] Bloqueo por prioridad: Instruccion logica del Bridge ignorada.");
                Serial2.println("MODO:LAMPARA:MANUAL");
                Serial2.flush();
                return;
            }
            
            lamparaActiva = (estado == 1);
            lastProcessedField1 = estado;
            
            if (estado == 1) {
                lamparaEncendidaHasta = millis() + LAMPARA_TIMEOUT_MS;
                Serial.println("[CONTROL AUTOMATICO] Peticion de activacion desde Bridge. Asignando temporizador (10 min).");
            } else {
                lamparaEncendidaHasta = 0;
                Serial.println("[CONTROL AUTOMATICO] Peticion de desactivacion desde Bridge procesada.");
            }
            
            enviarComandoConPersistencia(1, estado, false);
            enviarEstadoYModoAlEdge(1, estado, false);
            return;
        }
        else if (comando.startsWith("ACTUADOR:2:ESTADO:")) {
            int estado = comando.substring(comando.lastIndexOf(":") + 1).toInt();
            
            if (ventiladorControlManualGateway) {
                Serial.println("[SISTEMA] Bloqueo por prioridad termica: Instruccion logica del Bridge ignorada.");
                Serial2.println("MODO:VENTILADOR:MANUAL");
                Serial2.flush();
                return;
            }
            
            lastProcessedField2 = estado;
            
            if (nodeIdVentilador != 0) {
                mesh.sendSingle(nodeIdVentilador, comando);
                Serial.printf("[TX MESH] Reenvio Unicast hacia actuador termico: %s\n", comando.c_str());
            }
            mesh.sendBroadcast(comando);
            Serial.printf("[TX MESH] Reenvio Broadcast de redundancia: %s\n", comando.c_str());
            
            ultimoComando[2] = estado;
            if (nodeIdVentilador != 0) {
                ultimoComando[nodeIdVentilador] = estado;
            }
            
            Serial1.println(comando);
            Serial1.flush();
            return;
        }
        // PARA AGREGAR MÁS ACTUADORES: 
        // repetimos el bloque 'else if (comando.startsWith("ACTUADOR:2:ESTADO:"))' modificándolo para el actuador 3
        else {
            Serial1.println(comando);
        }
    }
}

// INICIALIZACION DEL SISTEMA   
void setup() {
    Serial.begin(115200);
    Serial1.begin(SERIAL1_BAUDRATE, SERIAL_8N1, 14, 15);
    Serial2.begin(SERIAL2_BAUDRATE, SERIAL_8N1, 16, 17);
    
    lamparaControlManual = false;
    lamparaActiva = false;
    lastManualCommand = 0;
    ventiladorControlManualGateway = false;
    lastManualVentiladorGateway = 0;
    
    Serial.println("\n [SISTEMA] INICIALIZANDO CONTROLADOR MESH GATEWAY (NODO RAIZ)");
    Serial.println(" - Rol        : Enrutador primario y puente comunicacional.");
    Serial.println(" - Red        : AP MESH Dedicado (WIFI_AP)");
    Serial.println(" - Interfaces : UART1 (Hacia Edge) / UART2 (Hacia Bridge)");
    Serial.println(" - Protocolos : Retransmision ACK y persistencia de memoria NVRAM\n");
    
    lastAliveFromActuador = millis();
    lastProgrammedRestart = millis();
    lastStateRestore = millis();
    
    detectarCausaReinicio();
    
    // Configuramos Gateway SOLO como AP (punto de acceso) para la MESH
    // NO usamos STA para evitar conflictos de canal con el Edge Controller
    WiFi.mode(WIFI_AP);
    
    Serial.println("[RED MESH] Inicializando interfaz de capa fisica (AP Mode)...");
    mesh.init(MESH_PREFIX, MESH_PASSWORD, MESH_PORT, WIFI_AP, MESH_CHANNEL);
    mesh.setRoot(true);
    mesh.setContainsRoot(true);
    Serial.println("[RED MESH] Configuracion de topologia establecida. Identidad: NODO RAIZ.");
    
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    
    Serial.printf("[SISTEMA] ID Logico asignado en subred MESH: %u\n", mesh.getNodeId());
    
    // RESTAURACION DE CONTEXTO DESDE MEMORIA NVRAM   
    prefs.begin("gateway", true);

    horaInicioNoche = prefs.getInt("horaInicio", 17); 
    horaFinNoche = prefs.getInt("horaFin", 2);        
    Serial.printf("[MEMORIA] Parametros de ventana operativa restaurados: %d:00 a %d:00\n", horaInicioNoche, horaFinNoche);

    int estadoLampara = prefs.getInt("cmd_1", -1);
    if (estadoLampara != -1) {
        lamparaActiva = (estadoLampara == 1);
        lamparaControlManual = prefs.getBool("lampara_manual", false);
        
        if (lamparaControlManual && lamparaActiva) {
            lamparaEncendidaHasta = millis() + MANUAL_TIMEOUT_MS;
            lastManualCommand = millis();
            Serial.println("[MEMORIA] Restitucion de estado: Circuito Iluminacion ACTIVO (Modo: MANUAL).");
        } else if (lamparaActiva) {
            lamparaEncendidaHasta = millis() + LAMPARA_TIMEOUT_MS;
            Serial.println("[MEMORIA] Restitucion de estado: Circuito Iluminacion ACTIVO (Modo: AUTOMATICO).");
        } else {
            lamparaEncendidaHasta = 0;
            lamparaControlManual = false;
            Serial.println("[MEMORIA] Restitucion de estado: Circuito Iluminacion INACTIVO.");
        }
        
        mesh.sendBroadcast("ACTUADOR:1:ESTADO:" + String(estadoLampara));
    } else {
        lamparaActiva = false;
        lamparaControlManual = false;
        lamparaEncendidaHasta = 0;
        Serial.println("[MEMORIA] Sin registro de estado previo. Asignando valores por defecto (INACTIVO).");
    }
    prefs.end();
    
    if (reinicioPorFallo) {
        Serial.println("[SISTEMA] Recuperacion de fallo electrico detectada. Reestableciendo matriz de estado...");
        restaurarComandos();
    }
    
    delay(500);
    enviarEstadoYModoAlEdge(1, lamparaActiva ? 1 : 0, lamparaControlManual);
    
    int estadoVentilador = 0;
    if (ultimoComando.find(2) != ultimoComando.end()) {
        estadoVentilador = ultimoComando[2];
    }
    enviarEstadoYModoAlEdge(2, estadoVentilador, ventiladorControlManualGateway);
    
    Serial.println("[SISTEMA] Proceso de sincronizacion de arranque con Edge Controller concluido.");
    Serial.println("\n[SISTEMA] RUTINA DE INICIO FINALIZADA. OPERACION NORMAL ESTABLECIDA.\n");
}

// CICLO DE EJECUCION PRINCIPAL   
void loop() {
    static bool reinicioNotificado = false;
    if (!reinicioNotificado) {
        reinicioNotificado = true;
        Serial.println("[SISTEMA] Trasmision de paquete flag (REBOOT) requerida en inicio.");
        enviarEstadoCompletoAlEdge();
    }
    
    mesh.update();
    enviarHeartbeat();
    verificarActuador();
    
    unsigned long currentMillis = millis();
    
    if (!confirmacionRecibida && ultimoComandoEnviado != "") {
        if (currentMillis - tiempoEnvio >= TIEMPO_ESPERA_CONFIRMACION_MS) {
            if (intentosReenvio < MAX_CONSECUTIVE_FAILURES) {
                Serial.printf("[MESH] Ausencia de ACK detectada. Ejecutando retransmision (Intento %d de %d)\n", 
                              intentosReenvio + 1, MAX_CONSECUTIVE_FAILURES);
                mesh.sendBroadcast(ultimoComandoEnviado);
                tiempoEnvio = currentMillis;
                intentosReenvio++;
            } else {
                Serial.println("[ERROR MESH] Aborto de transmision: Limite de reintentos excedido.");
                confirmacionRecibida = true;
                intentosReenvio = 0;
                ultimoComandoEnviado = "";
            }
        }
    } else {
        intentosReenvio = 0;
    }
    
    verificarReinicioProgramado();
    leerEdgeController();
    leerBridge();
    restaurarEstadoPeriodico();
    verificarTimeoutLampara();
    
    if (ventiladorControlManualGateway && (millis() - lastManualVentiladorGateway > MANUAL_VENTILADOR_TIMEOUT_MS)) {
        ventiladorControlManualGateway = false;
        Serial.println("[CONTROL] Prioridad manual sobre actuador termico finalizada (Timeout).");
        enviarModoAlEdge(2, false);
    }
    
    if (millis() - lastStatus > 30000) {
        lastStatus = millis();
        mostrarEstado();
    }
    
    delay(10);
}