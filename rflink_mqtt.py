#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
import signal
import time
import serial
import _thread
import traceback
from queue import Queue
import json
import paho.mqtt.publish as mqtt_publish
import paho.mqtt.client as mqtt_client

# external files/classes
import logger
import serviceReport
import settings

# Temp-Humi Sensoren THGR810
humStatusTable = ["Dry", "Comfort", "Normal", "Wet"]

sendQueue = Queue(maxsize=0)
current_sec_time = lambda: int(round(time.time()))

exit = False
serialPort = None


def signal_handler(_signal, frame):
    global exit

    print('You pressed Ctrl+C!')
    exit = True


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Client connected successfully")
        client.subscribe([(settings.MQTT_TOPIC_OUT, 1), (settings.MQTT_TOPIC_CHECK, 1)])
    else:
        print(("ERROR: MQTT Client connected with result code %s " % str(rc)))


def setKaKu(unit, val):
    #10;NewKaku;00baea06;3;ON;
    if int(val) != 0:
        sendQueue.put(("10;NewKaku;00baea06;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;NewKaku;00baea06;%d;OFF;\r\n" % unit).encode())


def setEcolite(unit, val):
    #10;TriState;85562a;2;OFF;
    if int(val) != 0:
        sendQueue.put(("10;TriState;85562a;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;TriState;85562a;%d;OFF;\r\n" % unit).encode())


def setSilverCrest(unit, val):
    #10;Unitec;60bf;3;ON;
    if int(val) != 0:
        sendQueue.put(("10;Unitec;60bf;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;Unitec;60bf;%d;OFF;\r\n" % unit).encode())


def setFlamingo(unit, val):
    #10;AB400D;48;3;ON;
    if int(val) != 0:
        sendQueue.put(("10;AB400D;48;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;AB400D;48;%d;OFF;\r\n" % unit).encode())


# The callback for when a PUBLISH message is received from the server
def on_message(client, userdata, msg):
    print(('ERROR: Received ' + msg.topic + ' in on_message function' + str(msg.payload)))


def on_message_homelogic(client, userdata, msg):
    #print(msg.topic + " " + str(msg.payload))
    topics = msg.topic.split("/")

    deviceName = topics[2] #huis/RFXtrx/KaKu-12/out
    cmnd = deviceName.split("-") #KaKu-12

    # KaKu-12
    if cmnd[0] == "KaKu":
        #print("Activate KaKu WCD: %s" % cmnd[1])
        setKaKu(int(cmnd[1]), msg.payload)

    # Ecolite-2
    elif cmnd[0] == "Ecolite":
        #print("Activate Ecolite WCD %s: %s" % (cmnd[1], msg.payload))
        setEcolite(int(cmnd[1]), msg.payload)

    # SilverCrest-4
    elif cmnd[0] == "SilverCrest":
        #print("Activate SilverCrest WCD: %s" % cmnd[1])
        setSilverCrest(int(cmnd[1]), msg.payload)

    # Flamingo-3
    elif cmnd[0] == "Flamingo":
        #print("Activate Flamingo WCD: %s" % cmnd[1])
        setFlamingo(int(cmnd[1]), msg.payload)


def openSerialPort():
    global exit
    try:
        ser = serial.Serial(port=settings.serialPortDevice,  # port='/dev/ttyACM0',
                            baudrate=settings.serialPortBaudrate,
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)  # 1=1sec 0=non-blocking None=Blocked

        if ser.isOpen():
            print(("rflink_mqtt: Successfully connected to serial port %s" % (settings.serialPortDevice)))

        return ser

    # Handle other exceptions and print the error
    except Exception as arg:
        print("%s" % str(arg))
        traceback.print_exc()

        #Report failure to Home Logic system check
        serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_NOTHING, 'Serial port open failure on port %s, wrong port or USB cable missing' % (settings.serialPortDevice))

        # Suppress restart loops
        time.sleep(900) # 15 min
        exit = True


def closeSerialPort(ser):
    ser.close()


def getId(msgStr):
    id = msgStr.split('=')
    return id[1]


def getTemp(msgStr):
    val = msgStr.split('=')
    t = int(val[1], 16)
    temp = (t & 0x7FFF) * 0.1
    if (t & 0x8000) != 0:
        temp = -temp
    return temp


def getHum(msgStr):
    val = msgStr.split('=')
    hum = int(val[1], 10)
    return hum


def getHumStatus(msgStr):
    val = msgStr.split('=')
    humStatus = int(val[1], 10)
    return humStatusTable[humStatus]


def getBattStatus(msgStr):
    val = msgStr.split('=')
    if val[1] == "OK":
        return 99
    else:
        return 0


def getWindDirection(msgStr):
    val = msgStr.split('=')
    wd = int(val[1], 10)
    windDir = float(wd) * 22.5
    return windDir


def getWind(msgStr):
    val = msgStr.split('=')
    w = int(val[1], 16)
    wind = float(w) * 0.1
    return wind


def getRain(msgStr):
    val = msgStr.split('=')
    r = int(val[1], 16)
    rain = float(r) * 0.1
    return rain


def serialPortThread():
    global exit
    global sensorData
    global serialPort

    msgCounter = 0
    serialPort = {}

    serialPort = openSerialPort()

    while not exit:
        try:
            ####################################################################
            #RFLink Protocol Reference: http://www.rflink.nl/blog2/protref
            ####################################################################
            serInLine = serialPort.readline().decode()

            if serInLine != "":
                serInLine = serInLine.rstrip("\r\n")
                #print("RFLink serial in: %s" % (serInLine))
                msg = serInLine.split(';')
                del msg[-1] # Delete last (empty) item. There is a last empty item, because there is a trailing ';' and split does add an empty item

                # Only handle messages starting with 'OK' from RFLink
                if msg[0] == '20':
                    # Reset the Rx timeout timer
                    serviceReport.systemWatchTimer = current_sec_time()

                    # print 'OK found!'
                    del msg[0]  # remove '20' from list

                    # msgCount 0..FF
                    tCnt = int(msg[0], 16)
                    if msgCounter != tCnt:
                        print(("WARNING: Msg counter not OK, missing messages! msgCounter: %d, msg[1]: %d" % (msgCounter, tCnt)))
                        msgCounter = tCnt
                    #else:
                    #    print("msgCount OK")
                    msgCounter = msgCounter + 1
                    if msgCounter >= 256:
                        msgCounter = 0
                    del msg[0]  # remove msgCount from list

                    deviceName = msg[0]
                    del msg[0]  # remove deviceName from list

                    #print("RFLink: %s %s" % (deviceName, msg))

                    #Oregon TempHygro ['ID=52833', 'TEMP=00d4', 'HUM=71', 'HSTATUS=3', 'BAT=OK']
                    if deviceName == "Oregon TempHygro":
                        location = ""
                        id = getId(msg[0])
                        sensorId = (int(id, 16) & 0xF0000) >> 16
                        temp = getTemp(msg[1])
                        hum = getHum(msg[2])
                        #humStatus = getHumStatus(msg[3])

                        # Temp-Humi Sensoren THGR810
                        #if sensorId == 1:
                        #    location = "Temp-Buiten"
                        #elif sensorId == 2:
                        #    location = "Temp-Wasruimte"
                        #elif sensorId == 3:
                        #    location = "Temp-Badkamer"
                        #elif sensorId == 4:
                        #    location = "Temp-Woonkamer"
                        if sensorId == 5:
                            location = "Temp-Werkplaats"
                        elif sensorId == 6:
                            location = "Temp-Kelder"
                        elif sensorId == 7:
                            location = "Temp-Kasje"
                        # elif sensorId == 8:
                        #     location = "Temp-Gang-boven"
                        #else:
                        #    location = "Temp-Unknown"
                        #    mqttTopic = "huis/RFLink/Temp-Unknown/temp"

                        if location != "":
                            sensorData = {}
                            sensorData['Temperature'] = "%1.1f" % temp
                            sensorData['Humidity'] = hum
                            #sensorData['Humidity status'] = humStatus
                            #sensorData['Battery level'] = batteryLevel
                            #sensorData['Signal level'] = 1

                            mqttTopic = "huis/RFLink/%s/temp" % location
                            mqtt_publish.single(mqttTopic, json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

                            #print("Oregon %s(id=%d) temp=%2.1fC hum=%d(%s)" % (location, sensorId, temp, hum, getHumStatus(msg[3])))

                    # Oregon Wind ['ID=1AC4', 'WINDIR=0015', 'WINGS=0015', 'WINSP=0020', 'BAT=OK']
                    #WINDIR=0..15 (1step=22,5 degrees)
                    #WINGS=Wind gust in km/h [0,1km/h]
                    #WINSP=Wind speed in km/h [0,1km/h]
                    elif deviceName == "Oregon Wind":
                        #ID OK?
                        if getId(msg[0]) == '1AC4':
                            windDir   = getWindDirection(msg[1])
                            windGust  = getWind(msg[2])
                            windSpeed = getWind(msg[3])

                            sensorData = {}
                            sensorData['WindDir'] = "%1.0f" % windDir
                            sensorData['WindGust'] = "%1.1f" % windGust
                            sensorData['WindSpeed'] = "%1.1f" % windSpeed
                            #sensorData['Battery level'] = batteryLevel
                            #sensorData['Signal level'] = 1

                            mqtt_publish.single("huis/RFLink/Wind/weer", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

                            #print("Oregon Wind windDir=%1.1f° windGust=%1.1f km/h windSpeed=%1.1 km/hf" % (windDir, windGust, windSpeed))

                    #Oregon Rain2 ['ID=2A6E', 'RAINRATE=00b9', 'RAIN=004a', 'BAT=OK']
                    #RAINRATE=Rain rate in mm [0,1mm]
                    #RAIN=Total rain in mm [0,1mm]
                    elif deviceName == "Oregon Rain2":
                        #ID OK?
                        if getId(msg[0]) == '2A6E':
                            rainRate  = getRain(msg[1])
                            rainTotal = getRain(msg[2])

                            sensorData = {}
                            sensorData['RainRate'] = "%1.1f" % rainRate
                            sensorData['RainTotal'] = "%1.1f" % rainTotal
                            #sensorData['Battery level'] = batteryLevel
                            #sensorData['Signal level'] = 1

                            mqtt_publish.single("huis/RFLink/Rain/weer", json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

                            #print("Oregon Rain rainRate=%1.1fmm rainTotal=%1.1fmm" % (rainRate, rainTotal))

                    # Msg: RM174RF ['ID=5bab23', 'SWITCH=01', 'CMD=ON', 'SMOKEALERT=ON', '']
                    elif deviceName == "RM174RF":
                        id = getId(msg[0])
                        if id == "5bab23": #Wasruimte
                            mqtt_publish.single("huis/RFLink/Rook-Wasruimte/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        # elif id == "1d0615": #Technische ruimte
                        #     mqtt_publish.single("huis/RFLink/Rook-Technische-ruimte/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        elif id == "cbcc23": #Overloop
                            mqtt_publish.single("huis/RFLink/Rook-Overloop/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        # elif id == "5bab23": #Wasruimte
                        #     mqtt_publish.single("huis/RFLink/Rook-Wasruimte/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        # else:
                        #     mqtt_publish.single("huis/RFLink/Rook-Unknown/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        #print("RFLink: %s %s" % (deviceName, msg))

                    # Msg: FA20RF ['ID=9f118c', 'SWITCH=01', 'CMD=ON', 'SMOKEALERT=ON', '']
                    elif deviceName == "FA20RF":
                        id = getId(msg[0])
                        if id == "668fd5": #Werkplaats
                            mqtt_publish.single("huis/RFLink/Rook-Werkplaats/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        elif id == "1d0615": #Technische ruimte
                            mqtt_publish.single("huis/RFLink/Rook-Technische-ruimte/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        # elif id == "e605df": #Overloop
                        #     mqtt_publish.single("huis/RFLink/Rook-Overloop/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        # elif id == "9f118c": #Wasruimte
                        #     mqtt_publish.single("huis/RFLink/Rook-Wasruimte/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        # else:
                        #     mqtt_publish.single("huis/RFLink/Rook-Unknown/rook", 1, qos=1, hostname=settings.MQTT_ServerIP)
                        #print("RFLink: %s %s" % (deviceName, msg))

                    elif deviceName == "NewKaku":
                        id = getId(msg[0])
                        #print("RFLink: %s %s" % (deviceName, msg))

                    # Auriol ['ID=004B', 'TEMP=8028', 'BAT=LOW']
                    elif deviceName == "Auriol":
                        id = getId(msg[0])
                        temp = getTemp(msg[1])
                        if id == "004B": #Binnenplaats
                            sensorData = {}
                            sensorData['Temperature'] = temp
                            #sensorData['Humidity'] = -1
                            #sensorData['Humidity status'] = -1
                            #sensorData['Battery level'] = getBattStatus(msg[2])
                            #sensorData['Signal level'] = 1

                            mqttTopic = "huis/RFLink/Temp-Binnenplaats/temp"
                            mqtt_publish.single(mqttTopic, json.dumps(sensorData, separators=(', ', ':')), hostname=settings.MQTT_ServerIP, retain=True)

                            print(("Binnenplaats temp: %2.1f" % temp))

                    elif deviceName.startswith("Nodo RadioFrequencyLink"):
                        # RFLink is started
                        print("RFLink detected!!!")
                    elif deviceName.startswith("OK"):
                        # RFLink OK
                        print()
                        #print("OK!")
                    #else:
                    #    print("RFLink: Device: '%s' not implemented yet" % deviceName);
                    #    print("RFLink: %s %s" % (deviceName, msg))

            # Check if there is any message to send via JeeLink
            if not sendQueue.empty():
                sendMsg = sendQueue.get_nowait()
                #print "SendMsg:", sendMsg
                if sendMsg != "":
                    serialPort.write(sendMsg)

        # In case the message contains unusual data
        except ValueError as arg:
            print(arg)
            traceback.print_exc()
            time.sleep(1)

        # Quit the program by Ctrl-C
        except KeyboardInterrupt:
            print("Program aborted by Ctrl-C")
            exit()

        # Handle other exceptions and print the error
        except Exception as arg:
            print("%s" % str(arg))
            traceback.print_exc()
            time.sleep(10)


def print_time(delay):
    count = 0
    while count < 5:
        time.sleep(delay)
        count += 1
        print("%s" % (time.ctime(time.time())))


###
# Initalisation ####
###
logger.initLogger(settings.LOG_FILENAME)

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

# Make the following devices accessable for user
os.system("sudo chmod 666 %s" % settings.serialPortDevice)

# Give Home Assistant and Mosquitto the time to startup
time.sleep(2)

# First start the MQTT client
client = mqtt_client.Client()
client.message_callback_add(settings.MQTT_TOPIC_OUT,       on_message_homelogic)
client.message_callback_add(settings.MQTT_TOPIC_CHECK,     serviceReport.on_message_check)
client.on_connect = on_connect
client.on_message = on_message
client.connect(settings.MQTT_ServerIP, settings.MQTT_ServerPort, 60)
client.loop_start()

# Create the serialPortThread
try:
    # thread.start_new_thread( print_time, (60, ) )
    _thread.start_new_thread(serialPortThread, ())
except Exception:
    print("Error: unable to start the serialPortThread")


# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.


while not exit:
    time.sleep(60)  # 60s

if serialPort is not None:
    closeSerialPort(serialPort)
    print('Closed serial port')

print("Clean exit!")
