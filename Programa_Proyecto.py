import uasyncio as asyncio
import time
from machine import Pin, ADC, I2C
from ssd1306 import SSD1306_I2C

# Pines Luz Nocturna
LDRPin = ADC(Pin(15))
LEDPin = Pin(2, Pin.OUT)
LDRPin.atten(ADC.ATTN_11DB)  # Configura el rango de 0-3.6V
LDRPin.width(ADC.WIDTH_12BIT)  # Resolución de 12 bits (0-4095)
umbral = 500  # Umbral de oscuridad

# Pines Velocímetro/odómetro
HALL_Sensor = Pin(25, Pin.IN)
Boton_Reset = Pin(14, Pin.IN, Pin.PULL_UP)
Boton_Convertir = Pin(12, Pin.IN, Pin.PULL_UP)

# Pines Direccionales
LED_Der = Pin(5, Pin.OUT)
Boton_Der = Pin(13, Pin.IN, Pin.PULL_UP)
Buzz = Pin(18, Pin.OUT)
LED_Izq = Pin(19, Pin.OUT)
Boton_Izq = Pin(27, Pin.IN, Pin.PULL_UP)

# Configuración del I2C
i2c = I2C(0, scl=Pin(22), sda=Pin(21))  # Cambia los pines si es necesario
oled_width = 128
oled_height = 64
oled = SSD1306_I2C(oled_width, oled_height, i2c)

# Variables para cálculos Vel/Dis
perimetro_rueda = 2 * 3.1416 * 0.025  # Radio en metros (0.35 m, ajusta según tu rueda)
ultima_lectura = 0
tiempo_entre_pulsos = 0
vueltas = 0
distancia_total = 0
velocidad = 0

# Estado de las unidades: False = m/s y m, True = km/h y km
usar_kmh = False
usar_km = False

# Estado Pila
Voltaje = ADC(Pin(32))  # Pin ADC
Voltaje.atten(ADC.ATTN_11DB)  # Configura el rango de 0-3.6V
Voltaje.width(ADC.WIDTH_12BIT)  # Resolución de 12 bits (0-4095)
voltaje_bateria = 0
R1 = 19500 # Resistencia 1
R2 = 19500  # Resistencia 2

# Estados iniciales de direccionales
estado_der = False
estado_izq = False

# Tiempo de parpadeo para direccionales
TIEMPO_PARPADEO = 300

# Función para manejar el sensor de Hall
def sensor_Hall(pin):
    global ultima_lectura, tiempo_entre_pulsos, vueltas, distancia_total, velocidad
    
    tiempo_actual = time.ticks_ms()
    if ultima_lectura != 0:
        tiempo_entre_pulsos = time.ticks_diff(tiempo_actual, ultima_lectura) / 1000  # En segundos
        if tiempo_entre_pulsos > 0:
            velocidad = perimetro_rueda / tiempo_entre_pulsos  # m/s
    ultima_lectura = tiempo_actual
    vueltas += 1
    distancia_total = vueltas * perimetro_rueda

# Función para cambiar de unidades
def boton_convertir(pin):
    global usar_kmh, usar_km
    usar_kmh = not usar_kmh
    usar_km = not usar_km

# Función para reiniciar distancia
def boton_reset(pin):
    global distancia_total, vueltas
    distancia_total = 0
    vueltas = 0

# Configurar interrupciones
HALL_Sensor.irq(trigger=Pin.IRQ_FALLING, handler=sensor_Hall)
Boton_Convertir.irq(trigger=Pin.IRQ_FALLING, handler=boton_convertir)
Boton_Reset.irq(trigger=Pin.IRQ_FALLING, handler=boton_reset)

# Función para calcular el voltaje de la batería
def calcular_voltaje(valor_adc, R1, R2):
    vout = (valor_adc / 4095) * 3.3  # Convertir lectura ADC a voltaje
    vin = vout * (R1 + R2) / R2      # Reconstruir el voltaje de la batería
    return vin

# Manejar direccionales
async def manejar_direccionales():
    global estado_der, estado_izq
    while True:
        if not Boton_Der.value():  # Botón derecho presionado
            estado_der = not estado_der
            estado_izq = False
            await asyncio.sleep_ms(300)  # Tiempo de rebote

        if not Boton_Izq.value():  # Botón izquierdo presionado
            estado_izq = not estado_izq
            estado_der = False
            await asyncio.sleep_ms(300)  # Tiempo de rebote

        # Control de parpadeo
        if estado_der:
            LED_Der.value(1)
            Buzz.value(1)
            await asyncio.sleep_ms(TIEMPO_PARPADEO)
            LED_Der.value(0)
            Buzz.value(0)
            await asyncio.sleep_ms(TIEMPO_PARPADEO)
        elif estado_izq:
            LED_Izq.value(1)
            Buzz.value(1)
            await asyncio.sleep_ms(TIEMPO_PARPADEO)
            LED_Izq.value(0)
            Buzz.value(0)
            await asyncio.sleep_ms(TIEMPO_PARPADEO)
        else:
            LED_Der.value(0)
            LED_Izq.value(0)
            Buzz.value(0)
        await asyncio.sleep_ms(50)

# Tarea para la luz nocturna
async def luz_nocturna():
    while True:
        ldrValor = LDRPin.read()
        LEDPin.value(1 if ldrValor < umbral else 0)
        await asyncio.sleep_ms(500)

# Tarea para mostrar datos en la pantalla OLED
async def mostrar_datos():
    global velocidad, distancia_total, usar_kmh, usar_km, voltaje_bateria
    while True:
        valor_adc = Voltaje.read()
        voltaje_bateria = calcular_voltaje(valor_adc, R1, R2)

        oled.fill(0)  # Limpia el contenido del display
        if usar_kmh:
            velocidad_convertida = velocidad * 3.6  # Convertir de m/s a km/h
            oled.text(f"Vel:", 0, 0)
            oled.text(f"{velocidad_convertida:.2f} km/h", 0, 16)
        else:
            oled.text(f"Vel:", 0, 0)
            oled.text(f"{velocidad:.2f} m/s", 0, 16)
        if usar_km:
            distancia_convertida = distancia_total / 1000
            oled.text(f"Distancia:", 0, 32)
            oled.text(f"{distancia_convertida:.2f} km", 0, 48)
        else:
            oled.text(f"Distancia:", 0, 32)
            oled.text(f"{distancia_total:.2f} m", 0, 48)
        
        Porcentaje_pila = ((voltaje_bateria - 3.3) / (4.2 - 3.3) * 100)
        oled.text(f"{voltaje_bateria:.2f}V", 72, 0)

        oled.show()
        await asyncio.sleep(1)

# Tarea principal
async def main():
    await asyncio.gather(
        luz_nocturna(),
        manejar_direccionales(),
        mostrar_datos()
    )

# Ejecuta el bucle principal
asyncio.run(main())