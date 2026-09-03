import os
import sys
import datetime
import requests
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense, Dropout
from dotenv import load_dotenv

# Cargamos las variables de entorno desde el archivo oculto .env localmente
load_dotenv()

# 1. Credenciales de ThingSpeak
# Definimos los canales de donde descargaremos la telemetría del hogar 
# usando variables de entorno para proteger la seguridad del repositorio.

CH_AMBIENTAL = os.getenv("CH_AMBIENTAL", "000000")
API_AMBIENTAL = os.getenv("API_AMBIENTAL", "API_AMBIENTAL")

CH_ELECTRICO = os.getenv("CH_ELECTRICO", "000000")
API_ELECTRICO = os.getenv("API_ELECTRICO", "API_ELECTRICO")

CH_ACTUADORES = os.getenv("CH_ACTUADORES", "000000")
API_ACTUADORES = os.getenv("API_ACTUADORES", "API_ACTUADORES")

# Días de historial que usaremos para entrenar a la IA
DIAS_HISTORICO = 21

def descargar_canal(ch_id: str, api_key: str, prefijo: str) -> pd.DataFrame:
    """
    Se conecta a ThingSpeak, descarga el CSV y promedia los datos por hora.
    """
    print(f"   -> Descargando canal {ch_id} ({prefijo})...")
    url = f"https://api.thingspeak.com/channels/{ch_id}/feeds.csv?api_key={api_key}&days={DIAS_HISTORICO}"
    
    try:
        # El timeout de 15 segundos evita que el script se cuelgue si el internet falla
        resp = requests.get(url, timeout=15)
        resp.raise_for_status()
    except requests.exceptions.RequestException as e:
        print(f"[Error de red] No se pudo descargar el canal {ch_id}: {e}")
        return None

    # Guardamos un respaldo crudo por seguridad
    nombre_arch = f"raw_{prefijo}.csv"
    with open(nombre_arch, 'w', encoding='utf-8') as f:
        f.write(resp.text)
        
    df = pd.read_csv(nombre_arch)
    if df.empty:
        print(f"[Advertencia] El canal {ch_id} no tiene datos.")
        return None

    # Convertimos la fecha a un formato que Pandas entienda y la usamos como índice
    df['created_at'] = pd.to_datetime(df['created_at'])
    df.set_index('created_at', inplace=True)
    
    # Limpiamos errores y agrupamos (promediamos) los datos en bloques de 1 hora ('h' minúscula)
    df = df.apply(pd.to_numeric, errors='coerce').resample('h').mean()
    
    return df

def main():
    hora_actual = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    print(f"[{hora_actual}] Iniciando Pipeline ETL Automático...\n")
    
    
    # PASO 1: Descarga de datos
    
    print("Paso 1: Extrayendo datos de los 3 canales de ThingSpeak...")
    df_amb = descargar_canal(CH_AMBIENTAL, API_AMBIENTAL, "ambiental")
    df_elec = descargar_canal(CH_ELECTRICO, API_ELECTRICO, "electrico")
    df_act = descargar_canal(CH_ACTUADORES, API_ACTUADORES, "actuadores")

    # Si falta algún canal, abortamos para no entrenar a la IA con datos rotos
    if any(df is None for df in [df_amb, df_elec, df_act]):
        print("[Error crítico] Fallo en la descarga. Verifica las API Keys o la conexión.")
        sys.exit(1)

 
    # PASO 2: Limpieza y unificación (Dataset Maestro)
    
    print("Paso 2: Limpiando y creando el Dataset Maestro...")
    df_amb.rename(columns={'field1': 'Temperatura', 'field2': 'Movimiento'}, inplace=True)
    df_elec.rename(columns={'field1': 'Voltaje', 'field3': 'Potencia'}, inplace=True)
    df_act.rename(columns={'field1': 'Lampara_Estado', 'field2': 'Ventilador_Estado'}, inplace=True)

    # Unimos todas las columnas en una sola tabla usando la hora como guía
    df_master = pd.concat([
        df_amb['Temperatura'], df_amb['Movimiento'], 
        df_elec['Voltaje'], df_elec['Potencia'], 
        df_act['Lampara_Estado'], df_act['Ventilador_Estado']
    ], axis=1, sort=False)
    
    # Rellenamos los huecos vacíos (si el sensor perdió señal unos minutos) uniendo los puntos
    df_master = df_master.interpolate(method='linear').fillna(0)
    df_master.to_csv('dataset_hogar_completo.csv')
    print("Base de datos unificada guardada exitosamente.\n")

    
    # PASO 3: Construcción de matrices para la Red Neuronal
    
    print("Paso 3: Construyendo tensores para el entrenamiento...")
    
    potencia_vals = df_master['Potencia'].values
    # Buscamos el pico máximo para normalizar (convertir todo a valores entre 0 y 1)
    max_potencia = np.max(potencia_vals) if np.max(potencia_vals) > 0 else 1.0
    potencia_norm = potencia_vals / max_potencia
    
    X, y = [], []
    ventanas_totales = len(potencia_norm) - 168 - 24
    
    # Creamos bloques de datos: Usamos 168h del pasado para predecir las 24h futuras
    for i in range(ventanas_totales):
        historial = potencia_norm[i : i + 168]
        fecha_actual = df_master.index[i + 168]
        
        # Le decimos a la IA qué hora y día de la semana es (One-Hot Encoding)
        hora_onehot = np.zeros(24)
        hora_onehot[fecha_actual.hour] = 1.0
        
        dia_onehot = np.zeros(7)
        dia_onehot[fecha_actual.weekday()] = 1.0
        
        X.append(np.concatenate([historial, hora_onehot, dia_onehot]))
        y.append(potencia_norm[i + 168 : i + 168 + 24])

    if len(X) == 0:
        print("[Error] No hay suficientes datos. Se necesitan más de 8 días de historial continuo.")
        sys.exit(1)

    # Convertimos a float32 para asegurar compatibilidad con la memoria del ESP32
    X, y = np.array(X, dtype=np.float32), np.array(y, dtype=np.float32)
    
    # Separamos el 80% para entrenar y el 20% para examinar si la IA aprendió bien
    split = int(0.8 * len(X))
    X_train, y_train = X[:split], y[:split]
    X_val, y_val = X[split:], y[split:]

    
    # PASO 4: Entrenamiento del modelo (TinyML)
    
    print("Paso 4: Entrenando red neuronal TinyML (Arquitectura: 199 -> 64 -> 32 -> 24)...")
    
    # Creamos una red neuronal secuencial con capas densas
    model = Sequential([
        Dense(64, activation='relu', input_shape=(199,)),
        Dropout(0.2), # Apagamos neuronas al azar para evitar que memorice de memoria (Overfitting)
        Dense(32, activation='relu'),
        Dropout(0.2),
        Dense(24, activation='linear') # 24 salidas: una predicción por cada hora de mañana
    ])
    
    model.compile(optimizer='adam', loss='mse', metrics=['mae'])
    model.fit(X_train, y_train, epochs=50, batch_size=32, validation_data=(X_val, y_val), verbose=0)
    
    loss, mae = model.evaluate(X_val, y_val, verbose=0)
    print(f" -> Error Medio Absoluto (MAE) logrado en pruebas: {mae:.4f}\n")

    
    # PASO 5: Cuantización (Reducción de tamaño para Microcontroladores)
    
    print("Paso 5: Cuantizando a int8 y exportando a model.tflite...")
    
    converter = tf.lite.TFLiteConverter.from_keras_model(model)
    converter.optimizations = [tf.lite.Optimize.DEFAULT]
    
    # Forzamos a que las matemáticas internas pasen de 32 bits a 8 bits (INT8)
    converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
    converter.inference_input_type = tf.int8
    converter.inference_output_type = tf.int8
    
    # Le pasamos unos ejemplos a TensorFlow para que sepa cómo escalar los números a 8 bits
    def representative_data_gen():
        for input_value in tf.data.Dataset.from_tensor_slices(X_train).batch(1).take(100):
            yield [input_value]
            
    converter.representative_dataset = representative_data_gen
    tflite_model = converter.convert()
    
    # Guardamos el archivo final que descargará el ESP32
    with open('model.tflite', 'wb') as f:
        f.write(tflite_model)
        
    tamano_kb = len(tflite_model) / 1024
    print(f" Proceso finalizado! 'model.tflite' generado ({tamano_kb:.2f} KB).")
    print(f" IMPORTANTE PARA C++: La potencia máxima de desnormalización es {max_potencia:.2f} W")

if __name__ == "__main__":
    main()