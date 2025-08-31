import serial
import time
import numpy
import csv
import matplotlib.pyplot as plt

inst=serial.Serial("COM43",9600)
Start_Freq=100000     ##Hz unit
Stop_Freq=10000000     ##Hz unit
Step_Freq=100000       ##Hz unit
Wait=0.01             ##sec
Ampl=[2000,2000]  ##mV unit. 50ohm loaded.

step=int((Stop_Freq-Start_Freq)/Step_Freq)
data=numpy.zeros((step,3))
for i in range(step):
    freq=Start_Freq+i*Step_Freq
    buf='S1,'+f'{freq}'+',0,'+f'{Ampl[0]}'+','
    inst.write(buf.encode())
    time.sleep(Wait)
    buf='?'
    inst.write(buf.encode())
    inst.read(4)
    buf=inst.read(4)
    data1=int(buf)
    inst.read(1)
    buf=inst.read(4)
    data2=int(buf)
    inst.read(3)    ##remove \n\r
    print('Freq=',int(freq),'Hz, Ch1=',data1,'mV, Ch2=',data2,'mV')
    data[i][0]=int(freq)
    data[i][1]=data1
    data[i][2]=data2
    
with open('data.csv','w',newline="") as f:
    writer=csv.writer(f)
    writer.writerows(data)
    
plt.subplot(1,2,1)
plt.plot(data[:,0],data[:,1])
plt.subplot(1,2,2)
plt.plot(data[:,0],data[:,2])
plt.show()
