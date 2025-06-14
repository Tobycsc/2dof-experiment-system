import os
import sys
from pathlib import Path
os.chdir(Path(__file__).parent.absolute())

import time
import serial
import xlwt
import xlrd
import struct



s = serial.Serial('COM8', 115200, timeout=0.5,write_timeout=0.1)

head = '72 7d'
tail = '7e 7d'

sendcount = 5000

            
def replay():
    global sendcount
    while(sendcount < 15001):
        #data = head + ' ' + hex(sendcount)[2:]
        data = head + ' ' + hex(sendcount)[4:] + ' ' + hex(sendcount)[2:4] + ' ' + hex(sendcount)[4:]+ ' ' + hex(sendcount)[2:4]  + ' ' + tail
        print(data)
        try:
            s.write(bytes.fromhex(data))
        except serial.SerialTimeoutException:
            print("bruh")
        sendcount = sendcount + 10
        if(sendcount == 15000):
            sendcount = 5000
        time.sleep(0.005)
        




def main():

    
    #freq = record()
    #print(freq)
    replay()




if __name__ == "__main__":  # 当程序执行时
    # 调用函数
     main()

     print("RUA")