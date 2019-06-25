# Heart_Monitor
Heart Monitor using ATmega328p, Rapbery Pi 3 and AD8232 shield.

This project uses an Arduino Uno for reading heart rate from AD8232 shield and send it for the raspberry Pi via UART. The raspbery send it using websocket to a server.

## Schematic:

![Schematic](https://github.com/daniellycosta/Heart_Monitor/blob/master/Heart_monitor_Schematic.png)

## Instructions:

- Build the circuit and download the codes
- Upload `heartMonitorAtmega328p.c` to the Arduino board using *Arduino IDE*, *PlatformIO* or other IDE.
- Upload `heartMonitor.py` to the Raspberry and run

## Attention:
 - I used Python 3
 - Maybe you should change the server configuration for sending to your one
 
~~~~
server_url = "YOUR SERVER ADDR"
hub_connection = HubConnectionBuilder().with_url(server_url).build()
hub_connection.on("TOPIC", onMessage)
 ~~~~
