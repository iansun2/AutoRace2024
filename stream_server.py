import cv2
import threading
import socket
import numpy as np

send_frame = None

# Create a socket for server-side communication
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('0.0.0.0', 6969))
server_socket.listen(10)

# Accept a client connection

# Function to send video frames to the client
def send():
    while True:
        #frame = cv2.resize(frame, (640, 480))
        if send_frame is not None:
            data = cv2.imencode('.jpg', send_frame)[1].tobytes()
            try:
                print('send')
                #print(send_frame.shape)
                connection.write(data)
            except:
                client_socket, client_address = server_socket.accept()
                connection = client_socket.makefile('wb')


send_thread = threading.Thread(target=send)
send_thread.start()



def setFrame(frame):
    global send_frame
    send_frame = frame.copy()
    #print(send_frame.shape)