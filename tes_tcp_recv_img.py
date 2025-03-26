#!/usr/local/bin/python3.6

import socket
import sys

# Create socket object
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print('Socket successfully created')
except OSError as err:
    print('Socket creation failed with error: %s' % (err))
    sys.exit()

# Connecting
ip_addr = '192.168.8.2'
port = 7

try:
    s.connect((ip_addr,port))
except OSError as err:
    print('Connection failed due to %s' % (err))
    print('Socket closed')
    s.close()
    sys.exit()

# Send data
print('Try sending data...',end='')

# Create byterray object with length 1, to hold the value of our intended row number (n) later
n = 0
row = bytearray(1)

import numpy as np
# Create a 2D array with dimensions 240x640 filled with zeros
pix_array = np.zeros((240, 640), dtype=np.uint8)

while n < 240:
    # Requesting which row to the server
    row[0] = n
    s.send(row)
    
    # Receive back data
    s.settimeout(5)
    try:
        rx_buf = s.recv(640)
    except OSError as err:
        print('Receive failed due to %s' % (err))
    else:
        # Convert received bytes to numpy array, then fill up the row
        text = rx_buf.decode('latin-1')
        pix_array[n, :] = np.frombuffer(text.encode('latin-1'), dtype=np.uint8)
    
    # Next row
    n += 1

# Converting a string of character to numpy byte array
print(pix_array)
print('n =',n)

print('Socket closed')
s.close()

# Processing the image
import cv2 as cv

# Create a 3D array filled with zeros to hold the image pixel in BGR format
img_qvga = np.zeros((240, 320, 3), dtype=np.uint8)

for x in range(0, 240, 1):
    for y in range(0, 320, 1):
        # Blue (Note: Take a look at OV7670 datasheet Fig.11 for position of bit in RGB565 format)
        img_qvga[x, y, 0] = pix_array[x, y*2 + 1] << 3
        # Green
        img_qvga[x, y, 1] = (pix_array[x, y*2] << 5) | (pix_array[x, y*2 + 1] >> 3) & 0xFC
        # Red
        img_qvga[x, y, 2] = pix_array[x, y*2] & 0xF8

# Save the image
cv.imwrite('OV7670_Captured_Image(7).jpg', img_qvga)

# Display the image directly
cv.imshow('OV7670 Captured Image', img_qvga)
cv.waitKey(0)
cv.destroyAllWindows()

sys.exit()
