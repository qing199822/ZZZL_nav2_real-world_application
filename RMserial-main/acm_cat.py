import serial
ser = serial.Serial("/dev/ttyACM0", 9600, timeout=0.1)

while True:
    data = ser.readline()
    try:
        print(data.decode())
    except:
        print(data.hex())  # 解不出来就打印16进制，不会乱码

