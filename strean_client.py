import cv2
import threading
import socket
import numpy as np

# Create a socket for client-side communication
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.130', 6969))  # Replace 'server_ip_here' with the server's IP or domain name


# Function to receive video frames from the server
def receive():
    while True:
        data = client_socket.recv(1024)
        if not data:
            break
        nparr = np.fromstring(data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        cv2.imshow('Client', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Create and start threads for sending and receiving
receive_thread = threading.Thread(target=receive)
receive_thread.start()
# Main loop
receive_thread.join()
# Release resources
cv2.destroyAllWindows()