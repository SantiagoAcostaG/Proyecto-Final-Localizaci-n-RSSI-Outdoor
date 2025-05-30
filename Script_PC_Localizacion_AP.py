# pc_data_receiver_ALL_ENHANCEMENTS_v2_documented.py
# ---------------------------------------------------------------------------------
# Script de Python para el Computador (PC) - Sistema de Localización en Tiempo Real
# Nombre original del archivo base: pc_data_receiver_ALL_ENHANCEMENTS_v2.py
#
# Funcionalidades Principales:
# 1.  Comunicación Serial: Establece una conexión serial para recibir datos RSSI
#     (ID de Nodo Fijo y valor RSSI) desde un microcontrolador Pico W que actúa
#     como concentrador de datos.
# 2.  Obtención de Datos IMU (HTTP): Inicia un hilo (thread) separado para solicitar
#     periódicamente datos de un sensor IMU (MPU6050: acelerómetro y giroscopio)
#     desde el Pico W Nodo Móvil (que actúa como Punto de Acceso y servidor HTTP).
#     Los datos se reciben en formato JSON.
# 3.  Conversión RSSI a Distancia: Utiliza un modelo de propagación en espacio libre
#     (FSPL) ajustado con parámetros de calibración (Potencia de Transmisión del AP,
#     Frecuencia del Canal WiFi, y un Factor K experimental) para convertir los
#     valores RSSI en estimaciones de distancia.
# 4.  Filtrado de Posición (EMA): Aplica un filtro de Media Móvil Exponencial (EMA)
#     a la posición cruda estimada (obtenida de la trilateración) para suavizar la
#     trayectoria visualizada del Nodo Móvil.
# 5.  Trilateración/Multilateración: Con al menos 3 distancias válidas y recientes,
#     calcula la posición (x,y) cruda del Nodo Móvil utilizando un algoritmo de
#     mínimos cuadrados.
# 6.  Visualización en Tiempo Real (Matplotlib):
#     - Muestra un mapa 2D con la posición configurada de los Nodos Fijos.
#     - Grafica la posición estimada (filtrada por EMA) del Nodo Móvil.
#     - (Funcionalidad de trayectoria histórica y filtrado EMA de distancias individuales
#       fueron discutidas como mejoras, pero esta versión se basa en el código que
#       proporcionaste que no las tenía explícitamente implementadas de la forma más reciente).
#     - Círculos indicando la distancia estimada desde cada Nodo Fijo al Móvil,
#       con indicación visual para datos obsoletos.
#     - Información textual sobre las distancias RSSI actuales.
#     - Gráficas separadas para los datos del Acelerómetro y Giroscopio del IMU.
# 7.  Registro de Datos (Logging): Guarda un log detallado de datos relevantes
#     (timestamps, posiciones, RSSI, distancias, datos IMU) en un archivo CSV
#     para análisis posterior.
#
# Autores: Daniel Suarez - Sergio Botia - Daniel Bueno - Santiago Acosta
# Fecha: 29 de Mayo de 2025

# Requisitos de Software (Python 3.x):
# - pyserial: Para la comunicación serial. (pip install pyserial)
# - numpy: Para cálculos numéricos. (pip install numpy)
# - matplotlib: Para la visualización y animación. (pip install matplotlib)
# - requests: Para las peticiones HTTP. (pip install requests)
#
# Requisitos de Hardware del Sistema Externo:
# - Pico W Concentrador: Conectado al PC vía USB en el puerto SERIAL_PORT,
#   ejecutando un script que envía datos "ID_Nodo,RSSI" por serial.
# - Pico W Nodo Móvil AP: Configurado como AP, con un MPU6050 conectado,
#   y ejecutando un script que sirve datos IMU en formato JSON en IMU_DATA_URL.
# - Nodos Fijos: Enviando sus mediciones RSSI al Concentrador.
#
# Uso:
# 1.  Asegurar que todos los componentes del sistema (Nodos Pico W) estén funcionando correctamente.
# 2.  Verificar y actualizar los parámetros de configuración en este script:
#     - SERIAL_PORT (puerto COM o /dev/tty* del Concentrador).
#     - NODOS_FIJOS_COORDS (coordenadas exactas en metros de los nodos fijos).
#     - P_TX_DBM, FREQ_MHZ, K_FACTOR_DB (valores obtenidos de la calibración RSSI-distancia).
#     - IMU_DATA_URL (IP del AP del Nodo Móvil).
# 3.  Conectar el PC a la red WiFi "NodoMovil_Proyecto" (creada por el Pico AP).
# 4.  Ejecutar este script desde una terminal: python <nombre_del_script>.py
# ---------------------------------------------------------------------------------

import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import requests
import threading
import json
import csv
from datetime import datetime
from collections import deque 

# --- Parámetros de Configuración y Calibración ---
SERIAL_PORT = 'COM7' 
BAUD_RATE = 115200    # Tasa de baudios para la comunicación serial

# Coordenadas de los nodos fijos (en metros) 
NODOS_FIJOS_COORDS = {
    0: np.array([0.0, 15.0]), 1: np.array([23.0, 15.0]),
    2: np.array([23.0, 0.0]), 3: np.array([0.0, 0.0])
}
ETIQUETAS_NODOS_FIJOS = [f"NF{i} ({NODOS_FIJOS_COORDS[i][0]:.1f},{NODOS_FIJOS_COORDS[i][1]:.1f})" for i in NODOS_FIJOS_COORDS]

# --- VALORES DE CALIBRACIÓN  ---
P_TX_DBM = 15.0       # Potencia de transmisión supuesta del AP (dBm)
FREQ_MHZ = 2437.0     # Frecuencia del canal WiFi (Canal 6 = 2437 MHz)
# K calculado con P_TX=15.0dBm, F=2437MHz, y tu RSSI_exp(1m)=-59.1dBm
K_FACTOR_DB = -33.92
# ------------------------------------------------------------------------------------

# --- Configuración para Datos IMU ---
IMU_DATA_URL = "http://192.168.4.1/" # URL del servidor HTTP en el Pico W Móvil
latest_imu_data = {"accel": {"x":0.0,"y":0.0,"z":0.0}, "gyro": {"x":0.0,"y":0.0,"z":0.0}} # Últimos datos IMU
imu_data_lock = threading.Lock() # Lock para acceso seguro a latest_imu_data
stop_imu_thread_flag = threading.Event() # Flag para detener el hilo IMU

# Historial para las gráficas del IMU
MAX_IMU_SAMPLES = 200 # Número máximo de muestras a mostrar
time_imu_hist, accel_x_hist, accel_y_hist, accel_z_hist, gyro_x_hist, gyro_y_hist, gyro_z_hist = [],[],[],[],[],[],[]

# --- Almacenamiento y Filtro de Localización ---
ultimos_rssi_por_nodo = {} # Diccionario: {node_id: {'rssi':, 'timestamp':, 'distancia':}}
RSSI_TIMEOUT_S = 10.0      # Tiempo (s) para considerar un dato RSSI como obsoleto
posicion_estimada_cruda = np.array([23.0/2, 15.0/2]) # Posición calculada directamente por trilateración
posicion_estimada_ema = np.array([23.0/2, 15.0/2])   # Posición después del filtro EMA, inicializada igual que la cruda
ALPHA_EMA_POS = 0.3        # Factor de suavizado para EMA de posición (0 < alpha <= 1)

# --- Configuración para Guardar CSV (Log Completo) ---
LOG_FULL_FILENAME = f"log_localizacion_completo_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
log_full_file_handle = None
csv_writer_full = None
LOG_FULL_ENABLED = True # Poner a False para deshabilitar el log

if LOG_FULL_ENABLED:
    try:
        log_full_file_handle = open(LOG_FULL_FILENAME, 'w', newline='', encoding='utf-8')
        csv_writer_full = csv.writer(log_full_file_handle)
        # Cabeceras del archivo CSV
        header_full = ['timestamp_pc', 'pos_cruda_x', 'pos_cruda_y', 'pos_ema_x', 'pos_ema_y']
        for nf_id_sorted in sorted(NODOS_FIJOS_COORDS.keys()): # Asegurar un orden consistente de nodos
            # En esta versión, 'dist_m' se refiere a la distancia cruda (no filtrada por EMA individualmente)
            header_full.extend([f'nf{nf_id_sorted}_rssi', f'nf{nf_id_sorted}_dist_m'])
        header_full.extend(['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'imu_error_msg'])
        csv_writer_full.writerow(header_full)
        print(f"Log completo se guardará en: {LOG_FULL_FILENAME}")
    except IOError as e:
        print(f"Error al abrir archivo de log completo {LOG_FULL_FILENAME}: {e}")
        LOG_FULL_ENABLED = False

# --- Funciones de Cálculo ---
def rssi_a_distancia_FSPL(rssi_medido_dbm, p_tx_dbm, freq_mhz, k_factor_db):
    """Convierte RSSI a distancia usando el modelo FSPL ajustado con K."""
    if rssi_medido_dbm is None or rssi_medido_dbm <= -110 or rssi_medido_dbm == -127: return float('inf')
    try:
        fspl_estimado_db = p_tx_dbm - rssi_medido_dbm + k_factor_db
        log_term_freq_const = (20 * np.log10(freq_mhz)) + 32.44
        log_d_km_numerator = fspl_estimado_db - log_term_freq_const
        exponent = log_d_km_numerator / 20.0
        if exponent > 3.5: return float('inf')
        if exponent < -5: return 0.05
        d_km = 10**exponent; distancia_m = d_km * 1000.0
        return max(0.05, distancia_m)
    except Exception: return float('inf')

def trilateracion_min_cuadrados(puntos_distancias):
    """Realiza trilateración 2D usando mínimos cuadrados."""
    if len(puntos_distancias) < 3: return None
    A,b = [],[]; x_ref,y_ref = puntos_distancias[0][0]; d_ref_sq = puntos_distancias[0][1]**2
    for i in range(1, len(puntos_distancias)):
        xi,yi = puntos_distancias[i][0]; di_sq = puntos_distancias[i][1]**2
        # CORRECCIÓN: Usar **2 para potencias
        A.append([2*(xi-x_ref), 2*(yi-y_ref)]); b.append(d_ref_sq - di_sq + xi**2 - x_ref**2 + yi**2 - y_ref**2)
    A_np,b_np = np.array(A),np.array(b)
    try:
        if A_np.shape[0] < A_np.shape[1]: return None
        return np.linalg.lstsq(A_np,b_np,rcond=None)[0]
    except Exception: return None

# --- Hilo para Obtener Datos IMU ---
def fetch_imu_data_thread_func():
    """Función para obtener datos IMU del Pico AP Móvil en un hilo separado."""
    global latest_imu_data, accel_x_hist, accel_y_hist, accel_z_hist, gyro_x_hist, gyro_y_hist, gyro_z_hist, time_imu_hist
    imu_sample_count = 0
    print("Hilo de IMU iniciado. Conectando a:", IMU_DATA_URL)
    while not stop_imu_thread_flag.is_set():
        response = None
        try:
            response = requests.get(IMU_DATA_URL, timeout=1.0)
            if response.status_code == 200:
                data = response.json()
                with imu_data_lock:
                    latest_imu_data = data
                    if "accel" in data and isinstance(data["accel"],dict) and all(k in data["accel"] for k in ["x","y","z"]) and \
                       "gyro" in data and isinstance(data["gyro"],dict) and all(k in data["gyro"] for k in ["x","y","z"]) and \
                       "error" not in data: # Solo procesar si no hay un campo "error" explícito en el JSON del Pico
                        imu_sample_count +=1; time_imu_hist.append(imu_sample_count)
                        accel_x_hist.append(float(data["accel"]["x"])); accel_y_hist.append(float(data["accel"]["y"])); accel_z_hist.append(float(data["accel"]["z"]))
                        gyro_x_hist.append(float(data["gyro"]["x"])); gyro_y_hist.append(float(data["gyro"]["y"])); gyro_z_hist.append(float(data["gyro"]["z"]))
                        if len(time_imu_hist) > MAX_IMU_SAMPLES: # Mantener tamaño del historial
                            lists_to_pop = [time_imu_hist,accel_x_hist,accel_y_hist,accel_z_hist,gyro_x_hist,gyro_y_hist,gyro_z_hist]
                            for lst_item in lists_to_pop: lst_item.pop(0)
        except requests.exceptions.Timeout: pass # Ignorar timeouts silenciosamente
        except requests.exceptions.ConnectionError: time.sleep(0.5); pass # Esperar un poco si la conexión es rechazada
        except requests.exceptions.RequestException: time.sleep(0.3); pass # Otros errores de requests
        except json.JSONDecodeError:
            err_text = response.text[:100] if response and hasattr(response,'text') else "No response data"
            # print(f"[Hilo IMU ERROR] JSONDecodeError. Rsp: {err_text}") # Comentado para reducir spam
        except Exception as e: print(f"[Hilo IMU ERROR] Error inesperado en hilo: {e}")
        time.sleep(0.4) # Intervalo entre peticiones IMU
    print("Hilo de IMU detenido.")

# --- Configuración de Matplotlib (Layout y Elementos Estáticos) ---
fig=plt.figure(figsize=(15,8))
gs=fig.add_gridspec(2,2,width_ratios=[2.5,1.5],height_ratios=[1,1])
ax_main=fig.add_subplot(gs[:,0])
ax_accel=fig.add_subplot(gs[0,1])
ax_gyro=fig.add_subplot(gs[1,1])

ax_main.set_xlabel("X (m)"); ax_main.set_ylabel("Y (m)")
ax_main.set_title("Localización por RSSI y Datos IMU"); ax_main.grid(True)
all_nf_x_init=[c[0] for c in NODOS_FIJOS_COORDS.values()]; all_nf_y_init=[c[1] for c in NODOS_FIJOS_COORDS.values()]
ax_main.set_xlim(min(all_nf_x_init)-2, max(all_nf_x_init)+2)
ax_main.set_ylim(min(all_nf_y_init)-2, max(all_nf_y_init)+2)
ax_main.set_aspect('equal',adjustable='box')

nodos_fijos_plots=[ax_main.plot(NODOS_FIJOS_COORDS[i][0],NODOS_FIJOS_COORDS[i][1],'s',color='red',ms=8,label=ETIQUETAS_NODOS_FIJOS[i])[0] for i in NODOS_FIJOS_COORDS]
nodo_movil_plot, = ax_main.plot([],[],'o',color='blue',ms=10,label="Nodo Móvil (EMA)")
# La funcionalidad de 'trayectoria_plot' y 'text_calidad_loc' no estaba en el script base proporcionado.
# Si se desea, se pueden añadir como se discutió en mejoras previas.
distancia_circulos=[plt.Circle((0,0),0.1,color='gray',fill=False,linestyle=':',alpha=0.7) for _ in NODOS_FIJOS_COORDS]
for circle_artist in distancia_circulos: ax_main.add_patch(circle_artist)
ax_main.legend(fontsize='x-small',loc='upper right')
text_info_distancias=ax_main.text(0.02,0.98,'',transform=ax_main.transAxes,fontsize=7,va='top',bbox=dict(boxstyle='round,pad=0.3',fc='aliceblue',alpha=0.8))

ax_accel.set_title("Acelerómetro",fontsize=10);ax_accel.set_xlabel("Muestras",fontsize=8);ax_accel.tick_params(axis='x',labelsize=7)
ax_accel.set_ylabel("g",fontsize=8);ax_accel.tick_params(axis='y',labelsize=7);ax_accel.grid(True)
accel_x_line,=ax_accel.plot([],[],lw=1.5,label="AccX",color='orangered');accel_y_line,=ax_accel.plot([],[],lw=1.5,label="AccY",color='limegreen');accel_z_line,=ax_accel.plot([],[],lw=1.5,label="AccZ",color='dodgerblue')
ax_accel.legend(fontsize='xx-small',loc='upper right');ax_accel.set_ylim(-3,3)

ax_gyro.set_title("Giroscopio",fontsize=10);ax_gyro.set_xlabel("Muestras",fontsize=8);ax_gyro.tick_params(axis='x',labelsize=7)
ax_gyro.set_ylabel("°/s",fontsize=8);ax_gyro.tick_params(axis='y',labelsize=7);ax_gyro.grid(True)
gyro_x_line,=ax_gyro.plot([],[],lw=1.5,label="GyroX",color='orangered');gyro_y_line,=ax_gyro.plot([],[],lw=1.5,label="GyroY",color='limegreen');gyro_z_line,=ax_gyro.plot([],[],lw=1.5,label="GyroZ",color='dodgerblue')
ax_gyro.legend(fontsize='xx-small',loc='upper right');ax_gyro.set_ylim(-350,350)
fig.tight_layout(pad=2.0) # Ajusta el padding para que no se solapen los títulos/etiquetas

# --- Conexión Serial ---
ser = None
def conectar_serial():
    """Intenta establecer la conexión serial con el Pico Concentrador."""
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.05) # Timeout corto para lectura no bloqueante
        print(f"Conectado a {SERIAL_PORT}"); return True
    except serial.SerialException as e:
        print(f"Error al abrir {SERIAL_PORT}: {e}"); ser = None; return False

# --- Función de Actualización para la Animación de Matplotlib ---
def actualizar_plot(frame):
    """Función llamada periódicamente por FuncAnimation para actualizar la gráfica."""
    global posicion_estimada_cruda, posicion_estimada_ema
    current_time_cycle = time.time() # Timestamp para este ciclo de actualización del plot

    # 1. LEER DATOS RSSI DEL PUERTO SERIE
    if ser and ser.is_open:
        while ser.in_waiting > 0: # Procesar todos los datos disponibles en el buffer serial
            try:
                line = ser.readline().decode('utf-8',errors='ignore').strip()
                if line:
                    parts = line.split(',')
                    if len(parts) == 2:
                        node_id_str, rssi_str = parts
                        try:
                            node_id, rssi = int(node_id_str), int(rssi_str)
                            # 'distancia' aquí es la distancia cruda calculada del RSSI actual
                            dist = rssi_a_distancia_FSPL(rssi,P_TX_DBM,FREQ_MHZ,K_FACTOR_DB)
                            ultimos_rssi_por_nodo[node_id] = {'rssi':rssi,'timestamp':current_time_cycle,'distancia':dist}
                        except ValueError: pass # Ignorar errores de conversión
            except Exception: pass # Ignorar otros errores de lectura serial
            
    # 2. PREPARAR DATOS PARA TRILATERACIÓN Y ACTUALIZAR CÍRCULOS DE DISTANCIA
    puntos_para_trilateracion_actuales = []
    dist_txt_display = "Dist. (RSSI):\n" # Texto para mostrar en el plot
    
    for i, node_id_key in enumerate(sorted(NODOS_FIJOS_COORDS.keys())):
        coord_fijo = NODOS_FIJOS_COORDS[node_id_key]
        dist_radio_para_circulo = 0.1 # Radio por defecto si no hay dato o es inválido
        visible_circulo_actual = False # Por defecto, no visible

        if node_id_key in ultimos_rssi_por_nodo and current_time_cycle - ultimos_rssi_por_nodo[node_id_key]['timestamp'] < RSSI_TIMEOUT_S:
            # Dato RSSI es reciente
            data_nodo_actual = ultimos_rssi_por_nodo[node_id_key]
            dist_valida_para_trilat = data_nodo_actual['distancia'] # Distancia cruda para trilateración
            
            distancia_circulos[i].set_linestyle(':'); distancia_circulos[i].set_edgecolor('dimgray'); distancia_circulos[i].set_alpha(0.7) # Estilo para dato activo
            
            if dist_valida_para_trilat != float('inf') and dist_valida_para_trilat < 100: # Umbral de distancia máx.
                puntos_para_trilateracion_actuales.append((coord_fijo, dist_valida_para_trilat))
                dist_radio_para_circulo = dist_valida_para_trilat
                visible_circulo_actual = True
                dist_txt_display += f"NF{node_id_key}({data_nodo_actual['rssi']}): {dist_valida_para_trilat:.1f}m\n"
            else: # Distancia inválida o demasiado lejos
                dist_txt_display += f"NF{node_id_key}({data_nodo_actual['rssi']}): Inv/Lejos\n"
        else: # Dato RSSI es obsoleto o no existe
            distancia_circulos[i].set_edgecolor('salmon'); distancia_circulos[i].set_linestyle((0,(5,5))); distancia_circulos[i].set_alpha(0.5) # Estilo para dato obsoleto
            visible_circulo_actual = True # Mantener visible el círculo obsoleto
            if node_id_key in ultimos_rssi_por_nodo: # Si existe pero es viejo
                 dist_obs = ultimos_rssi_por_nodo[node_id_key]['distancia']
                 dist_radio_para_circulo = dist_obs if dist_obs!=float('inf') and dist_obs < 100 else 0.2
                 dist_txt_display += f"NF{node_id_key}({ultimos_rssi_por_nodo[node_id_key]['rssi']}): {dist_radio_para_circulo:.1f}m (Viejo)\n"
            else: 
                dist_txt_display += f"NF{node_id_key}: -- (No Data)\n"
        
        if i < len(distancia_circulos): # Salvaguarda
            distancia_circulos[i].center=(coord_fijo[0],coord_fijo[1])
            distancia_circulos[i].set_radius(dist_radio_para_circulo)
            distancia_circulos[i].set_visible(visible_circulo_actual)
            
    text_info_distancias.set_text(dist_txt_display.strip())

    # 3. REALIZAR TRILATERACIÓN
    if len(puntos_para_trilateracion_actuales) >= 3:
        nueva_pos_cruda_calculada = trilateracion_min_cuadrados(puntos_para_trilateracion_actuales)
        if nueva_pos_cruda_calculada is not None:
            posicion_estimada_cruda = nueva_pos_cruda_calculada # Actualizar solo si la trilateración tuvo éxito
    
    # Aplicar filtro de media móvil exponencial (EMA) a la posición
    # La primera vez (si posicion_estimada_ema es su valor inicial), EMA será igual a la cruda.
    posicion_estimada_ema = (posicion_estimada_cruda * ALPHA_EMA_POS) + \
                            (posicion_estimada_ema * (1 - ALPHA_EMA_POS)) 
    
    nodo_movil_plot.set_data([posicion_estimada_ema[0]], [posicion_estimada_ema[1]])
    # Si quisieras mostrar también la posición cruda (sin filtrar) para comparar:
    # if 'nodo_movil_crudo_plot' in globals(): nodo_movil_crudo_plot.set_data([posicion_estimada_cruda[0]], [posicion_estimada_cruda[1]])
    
    # 4. ACTUALIZAR PLOTS DEL IMU
    with imu_data_lock:
        if time_imu_hist: 
            min_x_plot_imu, max_x_plot_imu = min(time_imu_hist), max(time_imu_hist) if len(time_imu_hist)>1 else min(time_imu_hist)+1
            ax_accel.set_xlim(min_x_plot_imu, max_x_plot_imu); ax_gyro.set_xlim(min_x_plot_imu, max_x_plot_imu)
            accel_x_line.set_data(time_imu_hist,accel_x_hist); accel_y_line.set_data(time_imu_hist,accel_y_hist); accel_z_line.set_data(time_imu_hist,accel_z_hist)
            gyro_x_line.set_data(time_imu_hist,gyro_x_hist); gyro_y_line.set_data(time_imu_hist,gyro_y_hist); gyro_z_line.set_data(time_imu_hist,gyro_z_hist)
    
    # 5. GUARDAR EN CSV DETALLADO
    if LOG_FULL_ENABLED and csv_writer_full:
        try:
            # Preparar datos para el log CSV
            pos_ema_x_log = posicion_estimada_ema[0] if posicion_estimada_ema is not None else "N/A"
            pos_ema_y_log = posicion_estimada_ema[1] if posicion_estimada_ema is not None else "N/A"
            
            log_row_data = [current_time_cycle, 
                            posicion_estimada_cruda[0], posicion_estimada_cruda[1],
                            pos_ema_x_log, pos_ema_y_log] # Posiciones
            
            for nf_id_s in sorted(NODOS_FIJOS_COORDS.keys()):
                if nf_id_s in ultimos_rssi_por_nodo and current_time_cycle - ultimos_rssi_por_nodo[nf_id_s]['timestamp'] < RSSI_TIMEOUT_S:
                    data_nf_log = ultimos_rssi_por_nodo[nf_id_s]
                    # Guardar la distancia cruda en este log ('distancia' en ultimos_rssi_por_nodo es la cruda)
                    log_row_data.extend([data_nf_log['rssi'], round(data_nf_log['distancia'],2) if data_nf_log['distancia']!=float('inf') else "inf"])
                else: 
                    log_row_data.extend(["N/A","N/A"]) 
            
            with imu_data_lock:
                imu_error_log = latest_imu_data.get("error")
                imu_accel_log = latest_imu_data.get("accel", {})
                imu_gyro_log = latest_imu_data.get("gyro", {})
                log_row_data.extend([imu_accel_log.get("x"), imu_accel_log.get("y"), imu_accel_log.get("z"),
                                     imu_gyro_log.get("x"), imu_gyro_log.get("y"), imu_gyro_log.get("z"),
                                     imu_error_log])
            csv_writer_full.writerow(log_row_data)
        except Exception as e_csv_write: print(f"Error escribiendo a CSV detallado: {e_csv_write}")

    # Devolver todos los artistas que se han modificado para FuncAnimation
    artistas_a_actualizar = [nodo_movil_plot, text_info_distancias]
    # Las siguientes líneas estaban comentadas en tu script original, las mantengo comentadas.
    # Si trayectoria_plot y text_calidad_loc se definen y usan, deben incluirse aquí para blit=True.
    # if 'trayectoria_plot' in globals(): artistas_a_actualizar.append(trayectoria_plot)
    # if 'text_calidad_loc' in globals(): artistas_a_actualizar.append(text_calidad_loc)
    artistas_a_actualizar.extend(distancia_circulos) 
    artistas_a_actualizar.extend([accel_x_line,accel_y_line,accel_z_line,gyro_x_line,gyro_y_line,gyro_z_line]) 
    return artistas_a_actualizar

# --- Bucle Principal del Programa en PC ---
if __name__ == "__main__":
    if not conectar_serial(): 
        print("ADVERTENCIA: No se pudo conectar al puerto serie. Los datos de localización no estarán disponibles.")
    
    imu_thread = threading.Thread(target=fetch_imu_data_thread_func, daemon=True)
    imu_thread.start()
    
    # Usar blit=False para mayor robustez inicial, especialmente si hay problemas con los artistas.
    # blit=True puede ser más eficiente pero a veces más difícil de depurar.
    ani = animation.FuncAnimation(fig, actualizar_plot, interval=200, blit=False, save_count=50) 

    plt.show() # Esta línea bloquea la ejecución hasta que se cierra la ventana de Matplotlib

    # Código de limpieza al cerrar la ventana de Matplotlib o al interrumpir el script
    print("Cerrando programa...")
    stop_imu_thread_flag.set() # Señalizar al hilo de IMU que termine
    if imu_thread.is_alive(): # Solo hacer join si el hilo realmente se inició y está vivo
        imu_thread.join(timeout=1.5) # Esperar un poco a que el hilo termine
    
    if LOG_FULL_ENABLED and log_full_file_handle: # Cerrar el archivo de log
        try:
            log_full_file_handle.close()
            print(f"Log guardado en: {LOG_FULL_FILENAME}")
        except Exception as e_close_log:
            print(f"Error cerrando archivo de log: {e_close_log}")

    if ser and ser.is_open: 
        ser.close()
        print("Puerto serie cerrado.")
    print("Programa finalizado.")
