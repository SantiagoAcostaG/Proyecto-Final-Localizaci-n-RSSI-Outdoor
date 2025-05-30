# nodo_ap_movil_con_imu_server.py
# -----------------------------------------------------------------------------
# Script para Raspberry Pi Pico W actuando como Nodo Móvil.
# Funcionalidades:
# 1. Crea un Punto de Acceso (AP) WiFi.
# 2. Lee datos de un sensor MPU6050 (acelerómetro y giroscopio) conectado vía I2C.
# 3. Sirve los últimos datos leídos del MPU6050 a través de un simple servidor HTTP en formato JSON.
#
# Autores: Daniel Suarez - Sergio Botia - Daniel Bueno - Santiago Acosta
# Fecha: 30 de Mayo de 2025
#
# Hardware Requerido:
# - Raspberry Pi Pico W
# - Sensor MPU6050 conectado a I2C (ver configuración de pines I2C_BUS_ID, SDA_PIN_NUM, SCL_PIN_NUM)
#
# Librerías Externas Requeridas en el Pico W:
# - mpu6050.py (una librería compatible con MicroPython para el MPU6050)
#
# Uso:
# - Cargar este script y la librería mpu6050.py al Pico W.
# - Al ejecutar, el Pico creará un AP WiFi.
# - Otros dispositivos (como un PC) pueden conectarse a este AP.
# - El PC puede solicitar los datos del IMU haciendo una petición HTTP GET a la IP del Pico AP (usualmente http://192.168.4.1/).
# -----------------------------------------------------------------------------

import network
import time
from machine import Pin, I2C
import socket
import json

# --- Configuración del Punto de Acceso (AP) ---
AP_SSID = "NodoMovil_Proyecto"    # Nombre de la red WiFi que creará el Pico
AP_PASSWORD = "password123"     # Contraseña para la red WiFi (mínimo 8 caracteres)
# --- Configuración I2C y MPU6050 ---
I2C_BUS_ID = 0                # Bus I2C a usar (0 o 1)
SDA_PIN_NUM = 4               # Pin GPIO para SDA ( GP4 para I2C0)
SCL_PIN_NUM = 5               # Pin GPIO para SCL ( GP5 para I2C0)
MPU6050_I2C_ADDR = 0x68       # Dirección I2C estándar del MPU6050

# Intenta importar la librería MPU6050 necesaria
try:
    from mpu6050 import MPU6050 # El archivo debe llamarse mpu6050.py
except ImportError:
    print("ERROR: Librería MPU6050 no encontrada. El sensor IMU no funcionará.")
    MPU6050 = None # Permite que el script continúe sin funcionalidad MPU

# --- Variables Globales ---
led = None                     # Objeto para el LED integrado (opcional)
access_point = None            # Objeto para la interfaz WLAN del AP
mpu_sensor = None              # Objeto para el sensor MPU6050
imu_server_socket = None       # Socket para el servidor HTTP de datos IMU
latest_imu_data = {            # Diccionario para almacenar los últimos datos IMU o mensajes de error
    "error": "MPU/AP no inicializado", # Estado inicial
    "accel": {"x": 0.0, "y": 0.0, "z": 0.0},
    "gyro": {"x": 0.0, "y": 0.0, "z": 0.0}
}
last_imu_read_time = time.ticks_ms() # Timestamp de la última lectura IMU
IMU_READ_INTERVAL_MS = 100           # Intervalo para leer el IMU (en milisegundos)
DEBUG_PRINTS = True                # Habilitar/deshabilitar impresiones de depuración

# --- Funciones de Configuración ---
def setup_led():
    """Inicializa el LED integrado del Pico W si está disponible."""
    global led
    try:
        led = Pin("LED", Pin.OUT)
    except TypeError:
        if DEBUG_PRINTS: print("Advertencia: LED integrado no disponible.")
        led = None

def setup_ap_interface():
    """Configura y activa la interfaz WLAN del Pico W en modo Punto de Acceso (AP)."""
    global access_point, latest_imu_data
    ap = network.WLAN(network.AP_IF)
    ap.active(False) # Desactivar primero para una configuración limpia
    time.sleep_ms(200) # Pequeña pausa
    ap.active(True)

    try:
        # Configuración básica del AP. El firmware suele inferir WPA2-PSK si se da contraseña.
        # El canal también será uno por defecto.
        ap.config(essid=AP_SSID, password=AP_PASSWORD)
        if DEBUG_PRINTS: print(f"AP configurado (básico): SSID='{AP_SSID}'")
    except Exception as e_config: 
        if DEBUG_PRINTS: print(f"Fallo total al configurar AP: {e_config}")
        latest_imu_data["error"] = f"Error AP Config: {str(e_config)[:30]}" # Limitar longitud del error
        access_point = None; return False
    
    # Esperar a que el AP esté activo y tenga una IP asignada
    max_wait = 10; wait_count = 0
    while wait_count < max_wait:
        if ap.active() and ap.ifconfig()[0] != '0.0.0.0':
            break
        wait_count +=1; time.sleep(1)

    if ap.active() and ap.ifconfig()[0] != '0.0.0.0':
        final_ip_config = ap.ifconfig()
        print(f"Nodo Móvil AP: SSID '{ap.config('essid')}', IP: {final_ip_config[0]}, Canal: (por defecto, verificar con analizador)")
        access_point = ap
        # No se borra "error" de latest_imu_data aquí, se hará si MPU y server inician OK
        return True
    
    err_msg_ap = "Error: No se pudo activar el AP o no obtuvo IP válida."
    print(err_msg_ap); latest_imu_data["error"] = err_msg_ap
    access_point = None; return False

def setup_mpu6050():
    """Inicializa la comunicación I2C y el sensor MPU6050."""
    global mpu_sensor, latest_imu_data
    if not MPU6050: # Si la librería no se cargó
        latest_imu_data["error"] = "Librería MPU6050 no encontrada"; return False
    try:
        i2c_bus = I2C(I2C_BUS_ID, scl=Pin(SCL_PIN_NUM), sda=Pin(SDA_PIN_NUM), freq=400000)
        devices_found = i2c_bus.scan()
        if MPU6050_I2C_ADDR not in devices_found:
            err_msg = f"MPU6050 no en 0x{MPU6050_I2C_ADDR:02x}. Encontrados: {[hex(d) for d in devices_found]}"
            if DEBUG_PRINTS: print(f"Error: {err_msg}"); latest_imu_data["error"] = err_msg; return False

        mpu_sensor = MPU6050(i2c_bus) # Asume que la librería usa 0x68 por defecto si no se pasa `address`
        mpu_sensor.wake() # Activar el MPU6050

        who_am_i_val = mpu_sensor.who_am_i() # Leer el registro WHO_AM_I (debería ser 0x68)
        if who_am_i_val == 0x68:
            print(f"MPU6050 detectado (who_am_i: {hex(who_am_i_val)}) e inicializado.")
        else:
            err_msg = f"MPU6050 who_am_i incorrecto: esperado 0x68, obtenido {hex(who_am_i_val)}"
            if DEBUG_PRINTS: print(f"Error: {err_msg}"); latest_imu_data["error"] = err_msg
            mpu_sensor = None; return False # Fallo si who_am_i no es el esperado
        
        if DEBUG_PRINTS: print(f"MPU6050 Rangos - Accel: {mpu_sensor.read_accel_range()}, Gyro: {mpu_sensor.read_gyro_range()}")
        latest_imu_data.pop("error", None); return True # Quitar campo "error" si todo es exitoso
    except Exception as e:
        err_msg = f"Excepción inicializando MPU6050: {e}"
        if DEBUG_PRINTS: print(f"Error: {err_msg}"); latest_imu_data["error"] = err_msg
        mpu_sensor = None; return False

def setup_imu_data_server():
    """Configura y arranca un servidor HTTP simple para los datos del IMU."""
    global imu_server_socket, latest_imu_data
    if not (access_point and access_point.active() and access_point.ifconfig()[0] != '0.0.0.0'):
        err_msg = "Servidor IMU no iniciado: AP inactivo o sin IP."
        if DEBUG_PRINTS: print(err_msg)
        # No sobrescribir un error MPU_LIBRARY_NOT_FOUND o MPU_INIT_FAIL con este error de AP
        if "error" not in latest_imu_data or latest_imu_data.get("error") is None or "MPU" not in latest_imu_data.get("error", "") : 
            latest_imu_data["error"] = err_msg
        return False
    try:
        addr = socket.getaddrinfo('0.0.0.0', 80)[0][-1] # Escuchar en todas las interfaces, puerto 80 (HTTP)
        imu_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # Socket TCP
        imu_server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # Permitir reusar dirección
        imu_server_socket.bind(addr)
        imu_server_socket.listen(3) # Aumentado backlog a 3
        imu_server_socket.setblocking(False) # Modo no bloqueante
        print(f"Servidor IMU escuchando en http://{access_point.ifconfig()[0]}/")
        return True
    except Exception as e:
        err_msg = f"Error al iniciar servidor IMU: {e}"
        if DEBUG_PRINTS: print(err_msg)
        if "error" not in latest_imu_data or latest_imu_data.get("error") is None or "MPU" not in latest_imu_data.get("error", "") :
            latest_imu_data["error"] = err_msg
        if imu_server_socket: imu_server_socket.close() # Intentar cerrar si se creó parcialmente
        imu_server_socket = None; return False

def read_and_update_imu_data():
    """Lee los datos del MPU6050 y actualiza la variable global latest_imu_data."""
    global latest_imu_data, last_imu_read_time
    if not mpu_sensor: # Si el MPU no se inicializó
        return

    if time.ticks_diff(time.ticks_ms(), last_imu_read_time) >= IMU_READ_INTERVAL_MS:
        try:
            accel = mpu_sensor.read_accel_data() # Espera una tupla (ax, ay, az)
            gyro = mpu_sensor.read_gyro_data()   # Espera una tupla (gx, gy, gz)
            
            if DEBUG_PRINTS: print(f"Pico IMU Raw: Accel={accel}, Gyro={gyro}")

            # Crear un nuevo diccionario para asegurar que no se sirven datos parciales
            current_sensor_data = {
                "accel": {"x": round(accel[0], 3), "y": round(accel[1], 3), "z": round(accel[2], 3)},
                "gyro": {"x": round(gyro[0], 2), "y": round(gyro[1], 2), "z": round(gyro[2], 2)}
            }
            # Actualizar la variable global. Si hubo un error previo, se sobrescribe con datos buenos.
            latest_imu_data = current_sensor_data
            
        except Exception as e:
            error_msg = f"Lectura MPU fallida: {e}"
            if DEBUG_PRINTS: print(error_msg)
            latest_imu_data["error"] = error_msg # Añadir/actualizar el campo de error
        
        last_imu_read_time = time.ticks_ms()

def handle_imu_data_request():
    """Maneja una petición HTTP entrante para los datos del IMU."""
    if not imu_server_socket:
        return
    
    client_sock = None # Definir fuera para poder usarlo en finally
    try:
        client_sock, client_addr = imu_server_socket.accept() # No bloqueante
        if client_sock: # Si se aceptó una conexión
            if DEBUG_PRINTS: print(f"Cliente IMU conectado desde: {client_addr}")
            
            # Leer y descartar la petición del cliente (para un GET simple)
            try:
                # Leer la primera línea (ej. GET / HTTP/1.1)
                client_sock.settimeout(0.2) # Timeout corto para la lectura de la petición
                req_line_bytes = client_sock.readline()
                # Leer y descartar el resto de las cabeceras HTTP
                while True:
                    header_line_bytes = client_sock.readline()
                    if not header_line_bytes or header_line_bytes == b'\r\n': # Fin de cabeceras
                        break
            except Exception: # Timeout u otro error leyendo petición
                pass # Continuar para enviar respuesta de todas formas o cerrar

            response_str = json.dumps(latest_imu_data)
            response_bytes = response_str.encode('utf-8')
            
            # Enviar respuesta HTTP
            client_sock.sendall(b"HTTP/1.1 200 OK\r\n")
            client_sock.sendall(b"Content-Type: application/json\r\n")
            client_sock.sendall(b"Content-Length: " + str(len(response_bytes)).encode('utf-8') + b"\r\n")
            client_sock.sendall(b"Connection: close\r\n") # Indicar al cliente que cierre la conexión
            client_sock.sendall(b"\r\n") # Línea vacía crucial entre cabeceras y cuerpo
            client_sock.sendall(response_bytes)
            
            if DEBUG_PRINTS: print(f"Datos IMU enviados a {client_addr}")
            
    except OSError as e_accept:
        # errno 11 (EAGAIN) o -11 en algunos firmwares significa "no hay conexiones pendientes"
        # Esto es normal y esperado para un socket no bloqueante.
        if e_accept.args[0] != 11 and e_accept.args[0] != -11: 
            if DEBUG_PRINTS: print(f"Error de socket al aceptar conexión IMU: {e_accept}")
    except Exception as e_handle: # Capturar otros errores inesperados
        if DEBUG_PRINTS: print(f"Error general manejando petición IMU: {e_handle}")
    finally:
        if client_sock: # Asegurarse de que el socket del cliente se cierre
            client_sock.close()

# --- Bucle Principal ---
if __name__ == "__main__":
    setup_led()
    ap_ok = setup_ap_interface() # Intenta configurar el AP
    
    mpu_ok = False # Asumir que no está OK hasta que se verifique
    if MPU6050: # Solo intentar inicializar si la librería se cargó
        mpu_ok = setup_mpu6050()
    
    server_ok = False
    # Iniciar servidor si el AP está OK, incluso si MPU falló (para poder servir el JSON con el error del MPU)
    if ap_ok: 
        server_ok = setup_imu_data_server()
    else: # Si el AP no está OK, el servidor no puede iniciarse
        if DEBUG_PRINTS: print("AP no OK, servidor IMU no se iniciará.")

    if DEBUG_PRINTS: print("Iniciando bucle principal...")
    led_state = False
    last_blink_time = time.ticks_ms()

    try:
        while True:
            current_main_loop_time = time.ticks_ms() # Renombrar para claridad

            # Parpadeo del LED integrado para indicar actividad
            if led:
                if time.ticks_diff(current_main_loop_time, last_blink_time) >= 1000: # Parpadea cada segundo
                    led_state = not led_state
                    led.value(led_state)
                    last_blink_time = current_main_loop_time
            
            if mpu_ok: # Solo leer datos del IMU si la inicialización fue exitosa
                read_and_update_imu_data()
            
            if server_ok: # Solo manejar peticiones si el servidor se inició correctamente
                handle_imu_data_request()
            
            time.sleep_ms(10) # Pequeña pausa para ceder CPU y permitir otras tareas
            
    except KeyboardInterrupt:
        print("\nDetenido por usuario.")
    finally:
        # Limpieza de recursos al finalizar
        if imu_server_socket:
            imu_server_socket.close()
            if DEBUG_PRINTS: print("Servidor IMU detenido.")
        if access_point and access_point.active():
            access_point.active(False)
            if DEBUG_PRINTS: print("AP detenido.")
        if led:
            led.off()
        print("Nodo móvil apagado.")