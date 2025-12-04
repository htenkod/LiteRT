import serial
import argparse
import csv


def main():
    parser = argparse.ArgumentParser(description="Read data from a serial port.")
    parser.add_argument(
        "--port",
        type=str,
        required=True,
        help="Serial port to read from (e.g., COM3 or /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=230400,
        help="Baud rate for the serial communication (default: 230400)",
    )
    args = parser.parse_args()
    fH = None
    fileNum = 0
    try:
        with serial.Serial(args.port, args.baudrate, timeout=1) as ser:
            # print(f"Connected to {args.port} at {args.baudrate} baud")
            while True:
                line = ser.readline().decode('utf-8').rstrip()                    
                if line != '':
                    if("######################################" in line):
                        if(fH):
                            fH.close()  
                        fileNum += 1
                        fH = open('data%s.csv' % fileNum, mode='w+')
                        fH.write('ax\tay\taz\tgx\tgy\tgz\n')      
                    else:
                        print (line)
                        fH.write(line + '\n')                    
    except serial.SerialException as e:
        print(e)


if __name__ == "__main__":
    main()
