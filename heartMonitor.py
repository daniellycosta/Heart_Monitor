# -*- coding: utf-8 -*-
import serial
import time
from socket import *
from struct import *
from threading import Thread
from signalrcore.hub_connection_builder import HubConnectionBuilder

class thread (Thread):
    def __init__(self, threadID, threadName, threadCounter, function,params):
        Thread.__init__(self)
        self.id = threadID
        self.name = threadName
        self.counter = threadCounter
        self.function = function
        self.params = params
    def run(self):
        return executa_tarefa(self.counter,self.function, self.params)

value = 0

def executa_tarefa(counter,function,params):
    while counter:
        counter -= 1
        return function(params)

def onMessage(ws,msg):
    print(msg)

def readUSART(ser):
    global value
    while True:
        cont = 0
        received_data = b''
        while cont < 2:
            received_data += ser.read(1)
            cont += 1

        if received_data != '':
            read_value = unpack('H',received_data)
        value = read_value[0]

def sendData(hub_connection):
    while True:
        time.sleep(0.02)
        hub_connection.send("BroadcastFrequency", [value])

ser = serial.Serial('/dev/ttyS0', 9600)
server_url = "wss://heart-monitor-galileo.azurewebsites.net/heartMonitorHub"
hub_connection = HubConnectionBuilder().with_url(server_url).build()
hub_connection.on("ReceiveFrequency", onMessage)
hub_connection.start()
