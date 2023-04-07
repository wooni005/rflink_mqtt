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
import configparser

# external files/classes
import logger
import serviceReport

# Temp-Humi Sensoren THGR810
humStatusTable = ["Dry", "Comfort", "Normal", "Wet"]

RFLINK_ZOLDER = 1
RFLINK_WOONKAMER = 2

settings = {}
rfLinkNr = 0

rfLinkDevicesNameTable = {
    "Oregon TempHygro": {
        1: [ RFLINK_WOONKAMER, "huis/RFLink/Temp-Buiten/temp" ],
        2: [ RFLINK_WOONKAMER, "huis/RFLink/Temp-Wasruimte/temp" ],
        3: [ RFLINK_WOONKAMER, "huis/RFLink/Temp-Badkamer/temp" ],
        4: [ RFLINK_WOONKAMER, "huis/RFLink/Temp-Woonkamer/temp" ],
        5: [ RFLINK_ZOLDER,    "huis/RFLink/Temp-Werkplaats/temp" ],
        6: [ RFLINK_ZOLDER,    "huis/RFLink/Temp-Kelder/temp" ],
        7: [ RFLINK_ZOLDER,    "huis/RFLink/Temp-Kasje/temp" ],
        8: [ RFLINK_WOONKAMER, "huis/RFLink/Temp-Gang-boven/temp" ]
    },
    "RM174RF": {
        "5bab23": [ RFLINK_ZOLDER,    "huis/RFLink/Rook-Wasruimte/rook" ],
        "52b453": [ RFLINK_ZOLDER,    "huis/RFLink/Rook-Technische-ruimte/rook" ],
        "cbcc23": [ RFLINK_WOONKAMER, "huis/RFLink/Rook-Overloop/rook" ],
        "ea1e53": [ RFLINK_WOONKAMER, "huis/RFLink/Rook-Werkkamer/rook" ],
        "63aec3": [ RFLINK_ZOLDER,    "huis/RFLink/Rook-Werkplaats/rook" ]
    },
    "Auriol": {
        "00B1": [ RFLINK_ZOLDER,      "huis/RFLink/Temp-Binnenplaats/temp" ]
    },
    #Firstline ['ID=0080', 'TEMP=017d']
    "Firstline": {
        "0080": [ RFLINK_ZOLDER,      "huis/RFLink/Temp-Vriezer-Werkplaats/temp" ]
    }
}


sendQueue = Queue(maxsize=0)
exitThread = False
serialPort = None


def current_sec_time():
    return int(round(time.time()))


def signal_handler(_signal, frame):
    global exitThread

    print('You pressed Ctrl+C!')
    exitThread = True


# The callback for when the client receives a CONNACK response from the server.
def on_connect(_client, userdata, flags, rc):
    if rc == 0:
        print("MQTT Client connected successfully")
        _client.subscribe([(settings["MQTTtopicOut"], 1), (settings["MQTTtopicCheck"], 1)])
    else:
        print(("ERROR: MQTT Client connected with result code %s " % str(rc)))


def setKaKu(unit, val):
    # 10;NewKaku;00baea06;3;ON;
    if int(val) != 0:
        sendQueue.put(("10;NewKaku;00baea06;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;NewKaku;00baea06;%d;OFF;\r\n" % unit).encode())


def setEcolite(unit, val):
    # 10;TriState;85562a;2;OFF;
    if int(val) != 0:
        sendQueue.put(("10;TriState;85562a;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;TriState;85562a;%d;OFF;\r\n" % unit).encode())


def setSilverCrest(unit, val):
    # 10;Unitec;60bf;3;ON;
    if int(val) != 0:
        sendQueue.put(("10;Unitec;60bf;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;Unitec;60bf;%d;OFF;\r\n" % unit).encode())


def setFlamingo(unit, val):
    # 10;AB400D;48;3;ON;
    if int(val) != 0:
        sendQueue.put(("10;AB400D;48;%d;ON;\r\n" % unit).encode())
    else:
        sendQueue.put(("10;AB400D;48;%d;OFF;\r\n" % unit).encode())


# The callback for when a published message is received from the server
def on_message(_client, userdata, msg):
    print(('ERROR: Received ' + msg.topic + ' in on_message function' + str(msg.payload)))


def on_message_homelogic(_client, userdata, msg):
    # print(msg.topic + " " + str(msg.payload))
    topics = msg.topic.split("/")

    deviceName = topics[2]  # huis/RFXtrx/KaKu-12/out
    cmnd = deviceName.split("-")  # KaKu-12

    # KaKu-12
    if cmnd[0] == "KaKu":
        # print("Activate KaKu WCD: %s" % cmnd[1])
        setKaKu(int(cmnd[1]), msg.payload)

    # Ecolite-2
    elif cmnd[0] == "Ecolite":
        # print("Activate Ecolite WCD %s: %s" % (cmnd[1], msg.payload))
        setEcolite(int(cmnd[1]), msg.payload)

    # SilverCrest-4
    elif cmnd[0] == "SilverCrest":
        # print("Activate SilverCrest WCD: %s" % cmnd[1])
        setSilverCrest(int(cmnd[1]), msg.payload)

    # Flamingo-3
    elif cmnd[0] == "Flamingo":
        # print("Activate Flamingo WCD: %s" % cmnd[1])
        setFlamingo(int(cmnd[1]), msg.payload)


def openSerialPort():
    global exitThread
    try:
        ser = serial.Serial(port=settings["serialPortDevice"],  # port='/dev/ttyACM0',
                            baudrate=int(settings["serialPortBaudrate"]),
                            parity=serial.PARITY_NONE,
                            stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS,
                            timeout=1)  # 1=1sec 0=non-blocking None=Blocked

        if ser.isOpen():
            print(("rflink_mqtt: Successfully connected to serial port %s" % settings["serialPortDevice"]))

        return ser

    # Handle other exceptions and print the error
    except Exception as arg:
        print("%s" % str(arg))
        traceback.print_exc()

        # Report failure to Home Logic system check
        serviceReport.sendFailureToHomeLogic(serviceReport.ACTION_NOTHING, 'Serial port open failure on port %s, wrong port or USB cable missing' % settings["serialPortDevice"])

        # Suppress restart loops
        time.sleep(900)  # 15 min
        exitThread = True


def closeSerialPort(ser):
    ser.close()


def getId(msgStr):
    _id = msgStr.split('=')
    return _id[1]


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
    return val[1]


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
    global exitThread
    global serialPort

    msgCounter = 0
    serialPort = {}

    serialPort = openSerialPort()

    while not exitThread:
        try:
            ####################################################################
            # RFLink Protocol Reference: http://www.rflink.nl/blog2/protref
            ####################################################################
            serInLine = serialPort.readline().decode()

            if serInLine != "":
                serInLine = serInLine.rstrip("\r\n")
                # print("RFLink serial in: %s" % (serInLine))
                msg = serInLine.split(';')
                del msg[-1]  # Delete last (empty) item. There is a last empty item, because there is a trailing ';' and split does add an empty item

                # Only handle messages starting with 'OK' from RFLink
                if msg[0] == '20':
                    # Reset the Rx timeout timer
                    serviceReport.systemWatchTimer = current_sec_time()

                    # print 'OK found!'
                    del msg[0]  # remove "20" from list

                    # msgCount 0..FF
                    tCnt = int(msg[0], 16)
                    if msgCounter != tCnt:
                        print(("WARNING: Msg counter not OK, missing messages! msgCounter: %d, msg[1]: %d" % (msgCounter, tCnt)))
                        msgCounter = tCnt
                    # else:
                    #    print("msgCount OK")
                    msgCounter = msgCounter + 1
                    if msgCounter >= 256:
                        msgCounter = 0
                    del msg[0]  # remove msgCount from list

                    deviceName = msg[0]
                    del msg[0]  # remove deviceName from list
                    
                    # print("%s %s" % (deviceName, msg))

                    # Temp-Humi Sensoren THGR810
                    # Oregon TempHygro ['ID=52833', 'TEMP=00d4', 'HUM=71', 'HSTATUS=3', 'BAT=OK']
                    if deviceName == "Oregon TempHygro":
                        _id = getId(msg[0])
                        sensorId = (int(_id, 16) & 0xF0000) >> 16
                        if rfLinkDevicesNameTable[deviceName][sensorId][0] == rfLinkNr:
                            temp = getTemp(msg[1])
                            hum = getHum(msg[2])
                            
                            sensorData = {'Temperature': "%1.1f" % temp, 'Humidity': hum}
                            sensorData['Hstat'] = getHumStatus(msg[3])
                            sensorData['Batt'] = getBattStatus(msg[4])

                            mqttTopic = rfLinkDevicesNameTable[deviceName][sensorId][1]
                            mqtt_publish.single(mqttTopic, json.dumps(sensorData, separators=(', ', ':')), hostname=settings["MQTT_ServerIP"], retain=True)
                            # print(" -> Oregon %s(id=%d) temp=%2.1fC hum=%d(%s) bat=%s" % (mqttTopic.split("/")[2], sensorId, temp, hum, getHumStatus(msg[3]), getBattStatus(msg[4])))

                    # Msg: RM174RF ['ID=5bab23', 'SWITCH=01', 'CMD=ON', 'SMOKEALERT=ON', '']
                    elif deviceName == "RM174RF":
                        _id = getId(msg[0])

                        if rfLinkDevicesNameTable[deviceName][_id][0] == rfLinkNr:
                            mqttTopic = rfLinkDevicesNameTable[deviceName][_id][1]
                            mqtt_publish.single(mqttTopic, 1, qos=1, hostname=settings["MQTT_ServerIP"])
                            # print(" -> RM174RF %s (id:%s)" % (mqttTopic.split("/")[2], id))

                    elif deviceName == "NewKaku":
                        _id = getId(msg[0])
                        # print("RFLink: %s %s" % (deviceName, msg))

                    # Auriol ['ID=004B', 'TEMP=8028', 'BAT=LOW']
                    # 20;38;Auriol;ID=00B1;TEMP=000c;BAT=OK;
                    elif deviceName == "Auriol":
                        _id = getId(msg[0])
                        if rfLinkDevicesNameTable[deviceName][_id][0] == rfLinkNr:
                            temp = getTemp(msg[1])
                            sensorData = {'Temperature': round(temp, 1)}
                            sensorData['Batt'] = getBattStatus(msg[2])

                            mqttTopic = rfLinkDevicesNameTable[deviceName][_id][1]
                            mqtt_publish.single(mqttTopic, json.dumps(sensorData, separators=(', ', ':')), hostname=settings["MQTT_ServerIP"], retain=True)
                            # print((" -> Auriol Binnenplaats temp: %2.1f" % temp))

                    elif deviceName == "Firstline":
                        _id = getId(msg[0])
                        if rfLinkDevicesNameTable[deviceName][_id][0] == rfLinkNr:
                            temp = getTemp(msg[1])
                            sensorData = {'Temperature': round(temp, 1)}

                            mqttTopic = rfLinkDevicesNameTable[deviceName][_id][1]
                            mqtt_publish.single(mqttTopic, json.dumps(sensorData, separators=(', ', ':')), hostname=settings["MQTT_ServerIP"], retain=True)
                            # print((" -> Firstline Vriezer Werkplaats temp: %2.1f" % temp))

                    elif deviceName.startswith("Nodo RadioFrequencyLink"):
                        # RFLink is started
                        print("RFLink detected!!!")
                    elif deviceName.startswith("OK"):
                        # RFLink OK
                        print()
                        # print("OK!")
                    # else:
                    #    print("RFLink: Device: '%s' not implemented yet" % deviceName);
                    #    print("RFLink: %s %s" % (deviceName, msg))

            # Check if there is any message to send via JeeLink
            if not sendQueue.empty():
                sendMsg = sendQueue.get_nowait()
                # print "SendMsg:", sendMsg
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
config = configparser.ConfigParser()
config.read('settings.ini')
settings = config["DEFAULT"]

rfLinkNr = int(settings["RFLinkNr"])

serviceReport.setingsMQTT(settings["MQTT_ServerIP"], settings["MQTTtopicReport"])

logger.initLogger(settings["logFileName"])

# Init signal handler, because otherwise Ctrl-C does not work
signal.signal(signal.SIGINT, signal_handler)

# Make the following devices accessable for user
os.system("sudo chmod 666 %s" % settings["serialPortDevice"])

# Give Home Assistant and Mosquitto the time to startup
time.sleep(2)

# First start the MQTT client
client = mqtt_client.Client()
client.message_callback_add(settings["MQTTtopicOut"],       on_message_homelogic)
client.message_callback_add(settings["MQTTtopicCheck"],     serviceReport.on_message_check)
client.on_connect = on_connect
client.on_message = on_message
client.connect(settings["MQTT_ServerIP"], int(settings["MQTT_ServerPort"]), 60)
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


while not exitThread:
    time.sleep(1)  # 60s

if serialPort is not None:
    closeSerialPort(serialPort)
    print('Closed serial port')

print("Clean exit!")
