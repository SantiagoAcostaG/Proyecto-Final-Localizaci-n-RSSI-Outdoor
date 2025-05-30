# Codigo para Nodos Fijos con NRF y pantalla OLED.py
# ---------------------------------------------------------------------------------
# Script para Raspberry Pi Pico W actuando como Nodo Fijo del sistema de localización.
# Funcionalidades:
# 1. Se conecta a una red WiFi predefinida (el AP del Nodo Móvil).
# 2. Mide la Intensidad de Señal Recibida (RSSI) de la conexión WiFi establecida.
# 3. Transmite su ID de nodo y el valor RSSI medido a un Nodo Concentrador
#    utilizando un transceptor NRF24L01.
# 4. Muestra su ID de nodo y el último RSSI medido en una pantalla OLED SSD1306.
#
# Autores: Daniel Suarez - Sergio Botia - Daniel Bueno - Santiago Acosta
# Fecha: 30 de Mayo de 2025
#
# Hardware Requerido:
# - Raspberry Pi Pico W
# - Módulo NRF24L01 conectado vía SPI (ver configuración de pines)
# - Pantalla OLED SSD1306 (128x32 o 128x64) conectada vía I2C (ver config. pines)
#
# Librerías Externas Requeridas en el Pico W:
# - nrf24l01.py (driver para el transceptor NRF24L01)
# - ssd1306.py (driver para la pantalla OLED SSD1306)
#
# Uso:
# - Cargar este script y las librerías nrf24l01.py y ssd1306.py al Pico W.
# - ¡IMPORTANTE! Modificar la variable NODE_ID para que sea única para cada nodo fijo.
# - Al ejecutar, el nodo intentará conectarse al WiFi, luego medirá RSSI y lo transmitirá.
# ---------------------------------------------------------------------------------

import network
import time
from machine import Pin, SPI, I2C # Asegurar que I2C está importado
from nrf24l01 import NRF24L01  # Asumiendo que el archivo es nrf24l01.py
import struct 
try:
    from ssd1306 import SSD1306_I2C # Importar librería OLED
except ImportError:
    print("ADVERTENCIA: Librería ssd1306.py no encontrada. OLED no funcionará.")
    SSD1306_I2C = None

# --- Definición de Constantes NRF24L01 (para la librería usada) ---
# Estos valores deben coincidir con los que espera la función set_power_speed() de la librería.
POWER_0 = const(0x00); POWER_1 = const(0x02); POWER_2 = const(0x04); POWER_3 = const(0x06) # Niveles de potencia
SPEED_1M = const(0x00); SPEED_2M = const(0x08); SPEED_250K = const(0x20) # Velocidades de datos

# --- Configuración WiFi ---
TARGET_SSID = "NodoMovil_Proyecto"     # SSID del Punto de Acceso del Nodo Móvil
TARGET_PASSWORD = "password123"    # Contraseña del AP Móvil

# --- Identificador de este Nodo Fijo ---
NODE_ID = 2 # ¡¡¡CAMBIAR ESTO PARA CADA NODO FIJO (0, 1, 2, 3)!!!

# --- Configuración de Medición RSSI ---
# Límites para considerar una lectura RSSI como válida.
RSSI_MIN_VALID = -120 # Valor RSSI mínimo aceptable (dBm)
RSSI_MAX_VALID = 0    # Valor RSSI máximo aceptable (dBm)

# --- Configuración Pines NRF24L01 ---
SPI_ID = 0               # ID del bus SPI a usar (0 o 1)
SCK_PIN_NRF_NUM = 2      # Pin GPIO para SCK del NRF
MOSI_PIN_NRF_NUM = 3     # Pin GPIO para MOSI del NRF
MISO_PIN_NRF_NUM = 4     # Pin GPIO para MISO del NRF
CSN_PIN_NRF_NUM = 5      # Pin GPIO para CSN (Chip Select Not) del NRF
CE_PIN_NRF_NUM = 6       # Pin GPIO para CE (Chip Enable) del NRF

# --- Configuración Parámetros NRF24L01 ---
# Estos valores deben ser idénticos en todos los Nodos Fijos y el Concentrador.
PIPE_ADDR_NRF = b"\xe1\xf0\xf0\xf0\xf0" # Dirección de la tubería de comunicación
PAYLOAD_SIZE_NRF = 5                    # Tamaño del payload: 1 byte (NODE_ID) + 4 bytes (RSSI int)
NRF_CHANNEL_CONFIG = 76                 # Canal RF (0-125)
NRF_DATARATE_CONFIG = SPEED_1M          # Velocidad de datos (usar constantes: SPEED_250K, SPEED_1M, SPEED_2M)
NRF_PA_LEVEL_CONFIG = POWER_3           # Nivel de potencia de transmisión (POWER_0 a POWER_3 [max])

# --- Configuración OLED ---
# ¡AJUSTA ESTOS PINES SEGÚN TU CONEXIÓN OLED!
OLED_I2C_BUS_ID = 1   # Bus I2C1 para la OLED
OLED_SDA_PIN_NUM = 14 # GP14 para SDA de la OLED
OLED_SCL_PIN_NUM = 15 # GP15 para SCL de la OLED
OLED_WIDTH = 128      # Ancho de la pantalla OLED en píxeles
OLED_HEIGHT = 32      # Alto de la pantalla OLED en píxeles (común: 32 o 64)
OLED_I2C_ADDR = None  # La dirección I2C se intentará detectar con i2c.scan()

# --- Variables Globales ---
led = None                  # Objeto para el LED integrado (opcional)
wlan_sta = None             # Objeto para la interfaz WLAN en modo Station
oled = None                 # Objeto para la pantalla OLED
last_displayed_rssi = "N/A" # Último valor RSSI mostrado en la OLED

def setup_oled():
    """Inicializa la pantalla OLED SSD1306 si está disponible y conectada."""
    global oled, OLED_I2C_ADDR, last_displayed_rssi
    if not SSD1306_I2C: # Si la librería no se importó
        print(f"NF {NODE_ID}: Librería SSD1306 no disponible.")
        return
    try:
        i2c_oled_bus = I2C(OLED_I2C_BUS_ID, scl=Pin(OLED_SCL_PIN_NUM), sda=Pin(OLED_SDA_PIN_NUM), freq=400000)
        print(f"NF {NODE_ID}: Bus I2C {OLED_I2C_BUS_ID} para OLED inicializado (SDA:GP{OLED_SDA_PIN_NUM}, SCL:GP{OLED_SCL_PIN_NUM}).")
        devices = i2c_oled_bus.scan() # Escanear dispositivos en el bus
        print(f"NF {NODE_ID}: Dispositivos I2C encontrados: {[hex(d) for d in devices]}")

        if not devices:
            print(f"NF {NODE_ID}: NINGÚN dispositivo I2C detectado en bus {OLED_I2C_BUS_ID}.")
            last_displayed_rssi = "No OLED" # Actualizar estado para display principal
            return
        
        # Intentar con direcciones I2C comunes para SSD1306
        addr_found = None
        for addr_candidate in [0x3c, 0x3d]:
            if addr_candidate in devices:
                addr_found = addr_candidate
                break
        
        if addr_found:
            OLED_I2C_ADDR = addr_found
            print(f"NF {NODE_ID}: OLED detectada en dirección {hex(OLED_I2C_ADDR)}.")
            oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_oled_bus, addr=OLED_I2C_ADDR)
        else:
            # Si no se encontró en 0x3C/0x3D pero hay otros dispositivos, probar con el primero
            # o dejar que SSD1306_I2C intente su dirección por defecto (usualmente 0x3C)
            print(f"NF {NODE_ID}: OLED no encontrada en 0x3C/0x3D. Probando default o primer dispositivo ({hex(devices[0]) if devices else 'ninguno'}).")
            oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_oled_bus, addr=devices[0] if devices else 0x3c)

        oled.fill(0) # Limpiar pantalla
        oled.text(f"NF:{NODE_ID} Init...", 0, 0) # Mensaje inicial corto
        oled.show()
        print(f"NF {NODE_ID}: OLED inicializada.")
        last_displayed_rssi = "Booting.." 
        update_oled_display() # Mostrar estado inicial
    except Exception as e:
        print(f"NF {NODE_ID}: Error severo inicializando OLED: {e}")
        oled = None # Asegurar que oled es None si la inicialización falla

def update_oled_display(status_line=""):
    """Actualiza la pantalla OLED con la información del nodo y el RSSI."""
    if oled: # Solo si la OLED fue inicializada correctamente
        try:
            oled.fill(0)
            oled.text(f"NodoFijo: {NODE_ID}", 0, 0) # Primera línea: ID del Nodo
            oled.text(f"RSSI: {last_displayed_rssi}dBm", 0, 10) # Segunda línea: Último RSSI
            if OLED_HEIGHT == 64 and status_line: # Tercera línea opcional para estado (pantallas 128x64)
                 oled.text(str(status_line)[:16], 0, 20) # Truncar para que quepa
            elif OLED_HEIGHT == 32 and status_line: # Para 32px, la tercera línea es apretada
                 oled.text(str(status_line)[:16], 0, 20)
            oled.show()
        except Exception as e:
            print(f"NF {NODE_ID}: Error actualizando OLED: {e}")

def connect_to_wifi():
    """Maneja la conexión a la red WiFi especificada."""
    global wlan_sta, last_displayed_rssi
    if wlan_sta and wlan_sta.isconnected():
        return True # Ya está conectado

    print(f"NF {NODE_ID}: Conectando a WiFi '{TARGET_SSID}'...")
    update_oled_display("WiFi Conn...") # Informar en OLED

    if wlan_sta is None: # Primera vez que se llama o si se perdió la interfaz
        wlan_sta = network.WLAN(network.STA_IF)
    
    if not wlan_sta.active():
        wlan_sta.active(True)
        time.sleep(1) # Dar tiempo para que la interfaz se active

    if not wlan_sta.isconnected():
        wlan_sta.connect(TARGET_SSID, TARGET_PASSWORD)
        max_wait_connect_s = 15 
        connect_start_time = time.ticks_ms()
        while not wlan_sta.isconnected():
            if time.ticks_diff(time.ticks_ms(), connect_start_time) > (max_wait_connect_s * 1000):
                print(f"NF {NODE_ID}: Timeout conectando a WiFi.")
                last_displayed_rssi = "No WiFi"; update_oled_display("Timeout WiFi"); return False
            # print(f"NF {NODE_ID}: Esperando conexión WiFi... Estado: {wlan_sta.status()}") # Puede ser muy verboso
            update_oled_display(f"WiFi St:{wlan_sta.status()}") # Mostrar estado de conexión
            time.sleep(1)
    
    if wlan_sta.isconnected():
        print(f"NF {NODE_ID}: Conectado a WiFi! IP: {wlan_sta.ifconfig()[0]}")
        last_displayed_rssi = "WiFi OK"; update_oled_display("WiFi OK"); return True # Indicar éxito y actualizar OLED
    else: # Si después del bucle no conectó
        print(f"NF {NODE_ID}: Fallo final al conectar a WiFi. Estado: {wlan_sta.status()}")
        last_displayed_rssi = "No WiFi"; update_oled_display("Fail WiFi"); return False

def setup_nrf24l01_tx(): 
    """Configura el transceptor NRF24L01 para transmisión."""
    global last_displayed_rssi
    # Inicialización de pines SPI y NRF
    sck_pin  = Pin(SCK_PIN_NRF_NUM); mosi_pin = Pin(MOSI_PIN_NRF_NUM); miso_pin = Pin(MISO_PIN_NRF_NUM)
    csn_pin  = Pin(CSN_PIN_NRF_NUM, Pin.OUT, value=1); ce_pin = Pin(CE_PIN_NRF_NUM, Pin.OUT, value=0)
    spi_nrf = SPI(SPI_ID, sck=sck_pin, mosi=mosi_pin, miso=miso_pin) # baudrate por defecto de SPI suele ser suficiente
    
    # Creación del objeto NRF24L01
    # El constructor de la librería ya establece algunos defaults (Canal 46, 250kbps, POWER_3)
    nrf = NRF24L01(spi_nrf, csn_pin, ce_pin, payload_size=PAYLOAD_SIZE_NRF)
    
    # Aplicar explícitamente las configuraciones deseadas para sobrescribir defaults
    print(f"NF {NODE_ID}: Configurando NRF...")
    nrf.set_channel(NRF_CHANNEL_CONFIG)
    nrf.set_power_speed(NRF_PA_LEVEL_CONFIG, NRF_DATARATE_CONFIG) # PA Level, luego Data Rate
    print(f"  NF {NODE_ID}: Config NRF - Ch:{NRF_CHANNEL_CONFIG}, DR:{NRF_DATARATE_CONFIG}, PA:{NRF_PA_LEVEL_CONFIG}")
        
    nrf.open_tx_pipe(PIPE_ADDR_NRF) # Configurar la dirección para transmitir
    nrf.stop_listening()            # Poner en modo transmisor (PTX)
    print(f"NF {NODE_ID}: NRF24L01 configurado como transmisor.")
    last_displayed_rssi = "NRF OK"; update_oled_display("NRF TX OK"); return nrf

# --- Bucle Principal ---
if __name__ == "__main__":
    # Inicializar periféricos (LED y OLED)
    setup_oled() # Cambiado de setup_peripherals a setup_oled
    try: 
        led = Pin("LED", Pin.OUT) # Inicializar LED aquí también si setup_oled no lo hace
    except TypeError:
        led = None
    
    update_oled_display("Config...") # Mensaje inicial

    # Inicializar NRF24L01 para transmisión
    nrf_tx = setup_nrf24l01_tx()

    if not nrf_tx:
        print(f"NF {NODE_ID}: Fallo inicialización NRF. Deteniendo.")
        last_displayed_rssi = "NRF ERR"; update_oled_display("NRF InitFail")
    else:
        print(f"NF {NODE_ID}: Listo para medir RSSI de conexión y transmitir...")
        last_displayed_rssi = "Listo..."; update_oled_display("Listo")
        
        measure_transmit_interval_s = 1 # Intervalo para cada ciclo de medición y transmisión

        try:
            while True:
                if led: led.value(1) # Indicar inicio de ciclo de trabajo
                current_rssi = -127  # Valor por defecto si la medición falla

                # Asegurar conexión WiFi y obtener RSSI
                if connect_to_wifi(): # Intenta conectar/asegurar conexión
                    try:
                        rssi_value = wlan_sta.status('rssi') # Leer RSSI de la conexión activa
                        if RSSI_MIN_VALID <= rssi_value <= RSSI_MAX_VALID:
                            current_rssi = rssi_value
                            print(f"NF {NODE_ID}: RSSI Conexión: {current_rssi}dBm")
                            last_displayed_rssi = str(current_rssi) 
                        else:
                            print(f"NF {NODE_ID}: RSSI Conexión: {rssi_value}dBm (descartado, fuera de rango)")
                            last_displayed_rssi = f"{rssi_value}X" # Indicar descarte
                        update_oled_display() # Mostrar el RSSI medido (o descarte)
                    except Exception as e:
                        print(f"NF {NODE_ID}: Error obteniendo RSSI de conexión: {e}")
                        last_displayed_rssi = "Err RSSI"; update_oled_display("Err RSSI")
                else:
                    # connect_to_wifi() ya actualiza la OLED si falla la conexión
                    # last_displayed_rssi = "No WiFi"; update_oled_display("No WiFi Con") # Redundante
                    pass 
                
                if led: led.value(0) # Apagar LED después de la fase de medición

                # Transmisión NRF del último RSSI medido (o -127 si falló)
                nrf_tx.stop_listening() # Asegurar modo TX
                payload = struct.pack("<Bi", NODE_ID, current_rssi) 
                
                print(f"NF {NODE_ID}: Enviando (ID:{NODE_ID}, RSSI:{current_rssi})")
                tx_status_oled = "TX NRF..." # Estado para OLED
                update_oled_display(tx_status_oled) # Mostrar que se está intentando enviar

                try:
                    nrf_tx.send(payload) # Intenta enviar, la librería maneja el timeout y OSError
                    print(f"NF {NODE_ID}: Payload enviado exitosamente (ACK recibido).")
                    tx_status_oled = "TX OK (ACK)"
                    if led: led.on(); time.sleep_ms(50); led.off() # Blink corto para ACK
                except OSError as e: 
                    print(f"NF {NODE_ID}: Error NRF al enviar (no ACK?): {e}")
                    tx_status_oled = "TX Fail NRF"
                
                update_oled_display(tx_status_oled) # Actualizar OLED con el resultado del TX
                
                time.sleep(measure_transmit_interval_s) # Esperar para el próximo ciclo

        except KeyboardInterrupt:
            print(f"\nDeteniendo NF {NODE_ID}...")
            update_oled_display("Detenido")
        finally:
            # Limpieza de recursos al finalizar
            if wlan_sta and wlan_sta.isconnected(): 
                wlan_sta.disconnect()
                print(f"NF {NODE_ID}: Desconectado de WiFi.")
            if wlan_sta and wlan_sta.active():
                wlan_sta.active(False) 
                print(f"NF {NODE_ID}: Interfaz WLAN desactivada.")
            if led: 
                led.off()
            print(f"NF {NODE_ID} detenido.")
            if oled: # Mensaje final en OLED
                oled.fill(0)
                oled.text(f"NF:{NODE_ID} Off",0,0)
                oled.show()