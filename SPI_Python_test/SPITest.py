import time
import spidev

bus = 0
device = 1

spi = spidev.SpiDev()
spi.open(bus, 1)

spi.max_speed_hz = 1000000
spi.mode = 0
message_tx = [0xFF,0,0xFF,2,0xFF,0]

a = input("appuyez sur une touche pour transmettre : ");

try:
	b = int(a)
except:
	b = 0

while True:
	b+=1
	if b > 100:
		b = 0
	message_rx = spi.xfer([0,b,0,8,0,16])
	print(message_rx)
	time.sleep(0.100)
