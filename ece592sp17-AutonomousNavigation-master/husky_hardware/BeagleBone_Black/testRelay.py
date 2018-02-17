import serial
import time
USB_Jetson = serial.Serial(
    port='/dev/jetson',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
USB_Husky = serial.Serial(
    port='/dev/husky',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
)
USB_Jetson.close()
USB_Jetson.open()
USB_Husky.close()
USB_Husky.open()
buffer1 =''
buffer2 =''
if 1:
    print "Initialising Pipe..."
    while 1:
        bytesToRead_From_Husky= USB_Husky.inWaiting()
        if bytesToRead_From_Husky>0:
            val = USB_Husky.read(bytesToRead_From_Husky)
            buffer1 = buffer1 + val
        elif (bytesToRead_From_Husky == 0) and (buffer1 != ''):
            #print "I have:%s" %buffer1
            USB_Jetson.write(buffer1)
            buffer1 = ''
        bytesToRead_From_Jetson= USB_Jetson.inWaiting()
        if bytesToRead_From_Jetson>0:
            Val = USB_Jetson.read(bytesToRead_From_Jetson)
            buffer2 = buffer2 + Val
        elif (bytesToRead_From_Jetson ==0) and (buffer2!=''):
            #print "I wrote: %s" %buffer2
            USB_Husky.write(buffer2)
            buffer2= ''
