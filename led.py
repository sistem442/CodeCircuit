# Bibliotheken laden
from machine import Pin
from time import sleep

# Initialisierung von GPIO 13, 14 und 15
yellow_turn1 = Pin(10, Pin.OUT, value=0)
red1 = Pin(13, Pin.OUT, value=0)
yellow1 = Pin(14, Pin.OUT, value=0)
green1 = Pin(15, Pin.OUT, value=0)
red2 = Pin(18, Pin.OUT, value=0)
yellow2 = Pin(17, Pin.OUT, value=0)
green2 = Pin(16, Pin.OUT, value=0)
p_red1 = Pin(11, Pin.OUT, value=0)
p_green1 = Pin(12, Pin.OUT, value=0)
p_red2 = Pin(19, Pin.OUT, value=0)
p_green2 = Pin(20 , Pin.OUT, value=0)
t_green2 = Pin(21 , Pin.OUT, value=0)
btn1 = Pin(9, Pin.IN, Pin.PULL_DOWN)
btn2 = Pin(22, Pin.IN, Pin.PULL_DOWN)


# Wiederholung (Endlos-Schleife)
while True:
    #Richtung 1 offnen
    # Ampelphase:     ROT Richtung 1, GRÜN Richtung 2
    red1.value(1)
    green2.value(1)
    p_green1.value(1)
    p_red2.value(1)
    t_green2.value(0)
    sleep(2)
    # Ampelphase:     Fußgägner 1 ROT
    p_green1.value(0)
    p_red1.value(1)
    sleep(1)
    # Ampelphase:     ROT Richtung 1, GELB Richtung 2
    green2.value(0)
    yellow2.value(1)
    sleep(1)
    # Ampelphase:     ROT Richtung 1, ROT Richtung 2
    yellow2.value(0)
    red2.value(1)
    sleep(1)
    # Ampelphase:     GELB Richtung 1, ROT Richtung 2 
    yellow1.value(1)
    sleep(1)
    # Ampelphase:     GRÜN Richtung 1, ROT Richtung 2
    red1.value(0)
    yellow1.value(0)
    green1.value(1)
    sleep(1)
    # Ampelphase:     Fußgägner 2 GRÜN
    p_red2.value(0)
    p_green2.value(1)
    sleep(5)        
    
    #Richtung 2 offnen
    # Ampelphase:     Fußgägner 2 ROT
    p_red2.value(1)
    p_green2.value(0)
    sleep(1)
    # Ampelphase:     ROT Richtung 2, GELB Richtung 1
    green1.value(0)
    yellow1.value(1)
    sleep(1)
    # Ampelphase:     ROT Richtung 2, ROT Richtung 1
    yellow1.value(0)
    red1.value(1)
    sleep(1)
    # Ampelphase:     GELB Richtung 2, ROT Richtung 1
    yellow2.value(1)
    sleep(1)
    # Ampelphase:     GRÜN Richtung 2, ROT Richtung 1
    red2.value(0)
    yellow2.value(0)
    green2.value(1)
    t_green2.value(1)
    sleep(1)
    # Ampelphase:     Fußgägner 1 GRÜN
    p_green1.value(1)
    p_red1.value(0)
    # Ampelphase:     Richtung 2 Warnung Blink 1 GELB
    for y in range(0, 8):
        yellow_turn1.value(1)
        sleep(0.5)
        yellow_turn1.value(0)
        sleep(0.5)
        y += 1
    

