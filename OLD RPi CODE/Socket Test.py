import socket
import sys

TCP_IP = '192.168.168.1'
TCP_PORT = 10000
BUFFER_SIZE = 1024
MESSAGE = "HELLO"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print('Socket Created')

s.connect((TCP_IP, TCP_PORT))

while True:
    MESSAGE = input(">> ") #gets user input 10 times
    s.send(bytes(MESSAGE, 'ASCII')); #sends bytes
    msg = s.recv(BUFFER_SIZE) #gets bytes grom server and convers to text
    msgString = msg.decode('ASCII')
    print(msgString)
    if(msgString == 'STOP'):
       print('Stopping');
    if (msgString == 'START'):
       print('Started')
    if (msgString == ' '):
       print('egh')
       
    

s.close()

                  
                  
