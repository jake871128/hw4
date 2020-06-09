import serial
import time
import paho.mqtt.client as paho
import matplotlib.pyplot as plt
import numpy as np
mqttc = paho.Client()


# Settings for connection

host = '192.168.43.232'

topic= "tilt"

port = 1883


# Callbacks

def on_connect(self, mosq, obj, rc):

    print("Connected rc: " + str(rc))


def on_message(mosq, obj, msg):

    print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n");


def on_subscribe(mosq, obj, mid, granted_qos):

    print("Subscribed OK")


def on_unsubscribe(mosq, obj, mid, granted_qos):

    print("Unsubscribed OK")


# Set callbacks

mqttc.on_message = on_message

mqttc.on_connect = on_connect

mqttc.on_subscribe = on_subscribe

mqttc.on_unsubscribe = on_unsubscribe


#Connect and subscribe

print("Connecting to " + host + "/" + topic)

mqttc.connect(host, port=1883, keepalive=60)

mqttc.subscribe(topic, 0)

#XBee setting
serdev = '/dev/ttyUSB0'
s = serial.Serial(serdev, 9600)
s.write("+++".encode())
char = s.read(2)
print("Enter AT mode.")
print(char.decode())
s.write("ATMY 0x233\r\n".encode())
char = s.read(3)
print("Set MY <BASE_MY>.")
print(char.decode())
s.write("ATDL 0x232\r\n".encode())
char = s.read(3)
print("Set DL <BASE_DL>.")
print(char.decode())
s.write("ATID 0x0\r\n".encode())
char = s.read(3)
print("Set PAN ID <PAN_ID>.")
print(char.decode())
s.write("ATWR\r\n".encode())
char = s.read(3)
print("Write config.")
print(char.decode())
s.write("ATMY\r\n".encode())
char = s.read(4)
print("MY :")
print(char.decode())
s.write("ATDL\r\n".encode())
char = s.read(4)
print("DL : ")
print(char.decode())
s.write("ATCN\r\n".encode())
char = s.read(3)
print("Exit AT mode.")
print(char.decode())
print("start sending RPC")
N=10
x = np.zeros(3*N)
y = np.zeros(3*N) # signal vector; create Fs samples
z = np.zeros(3*N)
num = np.zeros(N)
i=0
j=0
k=0
while i<=N:
    # send RPC to remote
    s.write("/status/run\r".encode())
    line=s.readline()
    line=line.decode()
    line=line.split()
    print(line)
    if len(line)==1:
        tmp=float(line[0])
        if tmp==100:
            while 1:
                line=s.readline()
                line=line.decode()
                line=line.split()
                tmp=float(line[0])
                if tmp==500:
                    break
             #   print(line)
                if len(line)==1:
                    num[j]=int(line[0])
                #    print(num[j])
                    j=j+1
                else:
                    x[k]=float(line[0])
                    y[k]=float(line[1])
                    z[k]=float(line[2])
               #     print(x[k],k)
                    k=k+1
    time.sleep(1)
    i=i+1

x=x[0:k]
y=y[0:k]
z=z[0:k]
#print(x)

tilt = np.zeros(k)
import math
for i in range(0,k):
    a1 = math.atan(x[i] / math.sqrt(y[i]*y[i]+z[i]*z[i]))
    b1= math.atan(y[i] / math.sqrt(x[i]*x[i]+z[i]*z[i]))
    c1= math.atan(z[i] / math.sqrt(x[i]*x[i]+y[i]*y[i]))
    thex = a1*180/math.pi
    they =b1*180/math.pi
    thez =c1*180/math.pi
    if abs(thex) >=45 or abs(they) >=45:
        tilt[i]=1
    else:
        tilt[i]=0

t= np.arange(0,j,j/k)
fig, ax = plt.subplots(2, 1)
ax[0].plot(t,x)
ax[0].plot(t,y)
ax[0].plot(t,z)
ax[0].set_xlabel('Timestamp')
ax[0].set_ylabel('acc value')
ax[0].legend(['x','y','z'])
ax[1].stem(t,tilt)
ax[1].set_xlabel('Timestamp')
ax[1].set_ylabel('tilt')
plt.show()

s.close()