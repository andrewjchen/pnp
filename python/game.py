import pygame
import time
import serial
WIDTH=640
HEIGHT=480

class EventHandler:
    def __init__(self):
        self.click_listeners = []
        self.key_listeners = []

    def subscribe_clicks(self, l):
        self.click_listeners.append(l)

    def subscribe_keys(self, l):
        self.key_listeners.append(l)

    def poll(self):
        for event in pygame.event.get():
            # print("event=" + str(event))
            if event.type == pygame.MOUSEBUTTONUP:
                # print("mouse clicked! position="
                #     + str(event.pos))
                for l in self.click_listeners:
                    l(event.button, event.pos)
            elif event.type == pygame.KEYDOWN:
                for l in self.key_listeners:
                    l(event.key, event.mod)
class Driver:
    def __init__(self):
        hi = 2
        print('opening device...')
        self.port =  serial.Serial(port='/dev/ttyACM0',
                     baudrate=115200,
                     bytesize=serial.EIGHTBITS,
                     stopbits=serial.STOPBITS_ONE,
                     timeout=0.1)
        self.port.open()
        time.sleep(3)
        self.send('g91')
        print('device initialized!')

    def send(self, str):
        self.port.write(str + '\r\n')
        self.port.flush()

    def key_callback(self, key, mod):
        #print("key_callback: key=" + str(key) + ",\t mod=" + str(mod))
        if key == pygame.K_DOWN:
            print("v")
            self.send('g0y5')
        elif key == pygame.K_UP:
            print("^")
            self.send('g0y-5')
        elif key == pygame.K_RIGHT:
            print(">")
            self.send('g0x5')
        elif key == pygame.K_LEFT:
            print("<")
            self.send('g0x-5')
 

pygame.init()
window = pygame.display.set_mode((WIDTH, HEIGHT))
eh = EventHandler()
d = Driver()
eh.subscribe_keys(d.key_callback)
while True:
    window.fill((0,0,0))
    eh.poll()
    pygame.display.flip()
    time.sleep(.01)

