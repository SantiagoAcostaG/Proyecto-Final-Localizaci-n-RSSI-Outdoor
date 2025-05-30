# concentrator_nodo.py
# -----------------------------------------------------------------------------
# Script para Raspberry Pi Pico W actuando como Nodo Concentrador.
# Funcionalidades:
# 1. Configura un transceptor NRF24L01 en modo receptor.
# 2. Escucha continuamente por paquetes de datos enviados por los Nodos Fijos.
# 3. Al recibir un paquete, lo desempaqueta para obtener el ID del Nodo Fijo
#    y el valor RSSI medido por ese nodo.
# 4. Imprime estos datos (ID, RSSI) por la conexión serial USB, en un formato
#    simple (ej. "0,-55"), para que un script en un PC pueda leerlos y procesarlos.
# 5. Utiliza un LED integrado (opcional) para indicar la recepción de paquetes.
#
# Autores: Daniel Suarez - Sergio Botia - Daniel Bueno - Santiago Acosta
# Fecha: 29 de Mayo de 2025
#
#
# Hardware Requerido:
# - Raspberry Pi Pico W
# - Módulo NRF24L01 conectado vía SPI (ver configuración de pines)
#
# Librerías Externas Requeridas en el Pico W:
# - nrf24l01.py (driver para el transceptor NRF24L01)
#
# Uso:
# - Cargar este script y la librería nrf24l01.py al Pico W.
# - Conectar el Pico W al PC mediante USB.
# - Ejecutar el script. El Pico comenzará a escuchar por datos NRF.
# - Abrir un monitor serie en el PC (o el script de procesamiento de datos)
#   para ver los datos (ID,RSSI) que reenvía el concentrador.
# -----------------------------------------------------------------------------

from machine import Pin, SPI
from nrf24l01 import NRF24L01 # Asegúrate de que el nombre del archivo de librería sea nrf24l01.py
from micropython import const # Para definir constantes de forma eficiente
import time
import struct # Para empaquetar/desempaquetar datos binarios

# --- Definición de constantes NRF24L01 (para la librería usada) ---
# Estos valores deben coincidir con los que espera la función set_power_speed()
POWER_0 = const(0x00)  # -18 dBm
POWER_1 = const(0x02)  # -12 dBm
POWER_2 = const(0x04)  # -6 dBm
POWER_3 = const(0x06)  # 0 dBm (Máxima potencia)

SPEED_1M = const(0x00)   # 1 Mbps
SPEED_2M = const(0x08)   # 2 Mbps
SPEED_250K = const(0x20) # 250 kbps

# --- Configuración de Pines del SPI y NRF24L01 ---
SPI_ID = 0          # ID del bus SPI del Pico W a utilizar (0 o 1)
SCK_PIN = 2         # Pin GPIO para SCK (Serial Clock)
MOSI_PIN = 3        # Pin GPIO para MOSI (Master Out Slave In)
MISO_PIN = 4        # Pin GPIO para MISO (Master In Slave Out)
CSN_PIN = 5         # Pin GPIO para CSN (Chip Select Not)
CE_PIN = 6          # Pin GPIO para CE (Chip Enable)

# --- Parámetros de Comunicación NRF24L01 ---
# Estos parámetros DEBEN COINCIDIR EXACTAMENTE con los configurados en los Nodos Fijos
PIPE_ADDR_NRF = b"\xe1\xf0\xf0\xf0\xf0" # Dirección de la tubería de recepción (debe ser la misma que la TX de los nodos fijos)
PAYLOAD_SIZE = 5                      # Tamaño del payload esperado: 1 byte (NODE_ID) + 4 bytes (RSSI int)
NRF_CHANNEL = 76                      # Canal RF (0-125)
NRF_DATARATE = SPEED_1M               # Velocidad de datos (debe coincidir con los nodos fijos)
NRF_POWER = POWER_3                   # Nivel de potencia (más relevante para TX, pero se pasa a set_power_speed)

# LED integrado (opcional para feedback visual de recepción)
try:
    led = Pin("LED", Pin.OUT)
except TypeError: # Manejo por si el firmware no define "LED" así
    print("Advertencia: LED integrado no disponible o no se pudo inicializar.")
    led = None

# --- Configuración del receptor NRF ---
def setup_nrf_rx():
    """Inicializa y configura el módulo NRF24L01 en modo receptor."""
    # Inicializar pines SPI
    spi_bus = SPI(SPI_ID, sck=Pin(SCK_PIN), mosi=Pin(MOSI_PIN), miso=Pin(MISO_PIN))
    # Inicializar pines CE y CSN
    csn_pin_obj = Pin(CSN_PIN, Pin.OUT, value=1) # CSN alto por defecto
    ce_pin_obj = Pin(CE_PIN, Pin.OUT, value=0)   # CE bajo por defecto

    # Crear instancia del driver NRF24L01
    # El constructor de la librería establece algunos defaults (ej. canal 46, 250kbps, POWER_3)
    nrf = NRF24L01(spi_bus, csn_pin_obj, ce_pin_obj, payload_size=PAYLOAD_SIZE)

    # Aplicar explícitamente las configuraciones deseadas para sobrescribir defaults
    print(f"Concentrador: Configurando NRF...")
    nrf.set_channel(NRF_CHANNEL)
    nrf.set_power_speed(NRF_POWER, NRF_DATARATE) # PA Level, luego Data Rate
    
    # Imprimir configuración para verificación (opcional)
    # print(f"  Concentrador: Config NRF - Canal Leído: {nrf.reg_read(0x05)}, RF_SETUP Leído: {hex(nrf.reg_read(0x06))}")
    print(f"  Concentrador: Config NRF - Canal Objetivo: {NRF_CHANNEL}, DataRate Objetivo: {NRF_DATARATE}, PA Objetivo: {NRF_POWER}")

    # Configurar el NRF para recibir
    nrf.open_rx_pipe(1, PIPE_ADDR_NRF) # Abrir Pipe 1 para la recepción con la dirección especificada
    nrf.start_listening()              # Poner el NRF en modo escucha (PRX)

    print("Concentrador: NRF configurado y escuchando en canal", NRF_CHANNEL)
    return nrf

# --- Bucle principal ---
def main():
    """Función principal: inicializa NRF y entra en bucle de recepción de datos."""
    nrf_rx_instance = setup_nrf_rx() # Obtener la instancia configurada del NRF

    if not nrf_rx_instance:
        print("Error crítico: Fallo en la inicialización del NRF. El concentrador no puede funcionar.")
        return # Salir si el NRF no se pudo configurar

    print("Esperando datos de los Nodos Fijos...")

    while True:
        try:
            # Verificar si hay algún paquete disponible en el NRF
            if nrf_rx_instance.any():
                if led: led.on() # Encender LED para indicar recepción
                
                try:
                    payload = nrf_rx_instance.recv() # Leer el payload recibido
                    
                    # Verificar que el payload tenga el tamaño esperado
                    if len(payload) == PAYLOAD_SIZE:
                        # Desempaquetar el payload: 
                        # "<" indica little-endian
                        # "B" indica un byte sin signo (para NODE_ID)
                        # "i" indica un entero de 4 bytes con signo (para RSSI)
                        node_id, rssi_value = struct.unpack("<Bi", payload)
                        
                        # Imprimir los datos por la conexión serial USB en formato CSV
                        # Este es el formato que el script de PC esperará.
                        print(f"{node_id},{rssi_value}")
                    else:
                        # Si el payload no tiene el tamaño esperado, es un error o dato corrupto.
                        print(f"⚠ Payload con tamaño inesperado. Longitud: {len(payload)}, Datos: {payload}")
                
                except OSError as e_nrf_recv: # Errores específicos de la comunicación NRF durante recv
                    print("Error NRF durante la recepción:", e_nrf_recv)
                except Exception as e_proc: # Otros errores al procesar el paquete
                    print("Error procesando paquete NRF:", e_proc)
                
                if led: # Pequeño blink para el LED
                    time.sleep_ms(50) # Mantener LED encendido brevemente
                    led.off()
            
            # Pequeña pausa para no acaparar la CPU si no hay datos NRF
            time.sleep_ms(20) 

        except KeyboardInterrupt: # Permitir detener el script con Ctrl+C
            print("\n🛑 Detenido por el usuario.")
            break # Salir del bucle while
        except Exception as e_loop: # Capturar otros errores inesperados en el bucle principal
            print("Error inesperado en el bucle principal:", e_loop)
            time.sleep(1) # Esperar un poco antes de reintentar

    # Limpieza final al salir del bucle (por KeyboardInterrupt o error grave)
    if nrf_rx_instance: # Solo si nrf_rx_instance se creó
        nrf_rx_instance.stop_listening()
    if led:
        led.off()
    print("Concentrador detenido.")

# --- Ejecutar el programa principal ---
if __name__ == "__main__":
    main()