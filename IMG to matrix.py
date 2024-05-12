from PIL import Image
import serial
from time import sleep
import os

def png_to_matrix(image_path, width, height):
    # Open the image file
    image = Image.open(image_path)

    # Ensure the image has at least 3 channels (R, G, B)
    if image.mode != 'RGBA' and image.mode != 'RGB':
        image = image.convert('RGB')

    # Convert the image to a matrix of RGB tuples
    matrix = list((image.getdata()))

    # Initialize the preset matrix with zeros
    preset = [[0 for _ in range(height)] for _ in range(width)]

    # Populate the preset matrix with the corresponding pixels
    for i in range(width * height):
        row = i // width
        col = i % width
        preset[col][row] = matrix[i]  # Assign RGB tuple to the matrix
    
    # Convert the matrix to a binary matrix
    binary_matrix = [[pixel_to_str(p) for p in row] for row in preset]

    # Return the binary matrix
    return binary_matrix

def pixel_to_str(pixel):
    r, g, b = pixel  # Unpack RGB tuple
    brightness = (r + g + b) // 3  # Calculate brightness
    return brightness

    
def print_matrix(matrix):
        # Get binary matrix from function
    electrode=[
        [1, 2, 3, 4, 5, 6],
        [7, 8, 9, 10, 11, 12],
        [13, 14, 15, 16, 17, 18],
        [19, 20, 21, 22, 23, 24],
        [25, 26, 27, 28, 29, 30],
        [31, 32, 33, 34, 35, 36]
        ]

    pin=[14,15,16,17,18,19,20,21,22,
         23,24,25,26,27,28,29,30,31,0,
         33,34,35,36,37,38,39,40,41,0,
         43,44,45,46,47,48,49,50,51,0]

    pin2=[31,15,17,19,21,22,
         30,14,16,18,33,34,
         28,29,27,18,35,36,
         26,25,47,37,39,38,
         24,23,49,45,43,40,
         51,50,48,46,44,41,
         ]
    # Flatten the matrix into a 36-element list
    binary_list = [pixel for row in matrix for pixel in row]
    sorted_list = [str(x) for (y, x) in sorted(zip(pin2, binary_list))]
    result = ' '.join(sorted_list)
    print(result)
    sleep(.5)
    return result


# Set up Serial connection to Arduino
for i in range(1,10):#max is 256
    port=i
    try:
        ser = serial.Serial("COM"+str(port), 9600, timeout=1)
    except:
        print("COM"+str(port), 'not available')
    
    
while 1:
    msg=ser.readline().decode() 
    if msg=='':
        msg=input("Ready to receive!")
        if msg=="print":
            ser.write(bytes("matrix66", 'utf-8'))
            sleep(.5)
            msg=ser.readline().decode()
            print(msg)
            #implement delay wait msg
            if msg!="ok":
                # Get binary matrix from function
                binary_matrix = png_to_matrix('C:/Users/mon ordi Fujitsu/Desktop/_Test_print/77.png', 6, 6)
                # Send binary list to Arduino
                msg= print_matrix(binary_matrix)
                ser.write(bytes(msg, 'utf-8'))
                print('Sent')
                sleep(.5)
                if ser.in_waiting > 0:
                    msg = ser.read(ser.in_waiting).decode()
                    print(msg)
                else:
                    print("No data available")

                    """#Modified code from main loop: 
s = serial.Serial(5)

#Modified code from thread reading the serial port
while 1:
  tdata = s.read()           # Wait forever for anything
  time.sleep(1)              # Sleep (or inWaiting() doesn't give the correct value)
  data_left = s.inWaiting()  # Get the number of characters ready to be read
  tdata += s.read(data_left) # Do the read and combine it with the first character
"""
        elif msg=='quit':
            break
            
        else:
            ser.write(bytes(msg+'\n', 'utf-8'))
        os.system('cls')
    sleep(.5)

# Close Serial connection
ser.close()
os.system('cls')
print("Merci de votre utilisation, à la prochaine!")
sleep(1)
quit()
