import cv2
import threading
import socket
import numpy as np

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('0.0.0.0', 6969))
server_socket.listen(10)

client_socket, client_address = server_socket.accept()
connection = client_socket.makefile('wb')


def receive():
    while True:
        data = connection.read()
        if not data:
            break
        nparr = np.fromstring(data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        cv2.imshow('Server', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


receive_thread = threading.Thread(target=receive)
receive_thread.start()