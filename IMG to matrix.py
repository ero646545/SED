import tkinter as tk
import serial
import time
import tkinter.filedialog as filedialog

# Set up Serial connection to Arduino
ser = serial.Serial("COM8", 9600, timeout=1)

# Create a tkinter canvas and bind mouse events to it
canvas_width = 6
canvas_height = 6
root = tk.Tk()
root.wm_title("Impression 3D électrochimique - LISE")
root.rowconfigure(0, weight=1)
root.columnconfigure(0, weight=1)
drawing_canvas = tk.Canvas(root, width=canvas_width*40, height=canvas_height*40, bg="white", highlightthickness=1, highlightbackground="black")
drawing_canvas.grid(row=0, column=0)

drawing = [[0 for _ in range(canvas_height)] for _ in range(canvas_width)]
def draw_pixel(event):
    x = event.x // 40
    y = event.y // 40
    if event.num == 1:
        # Left click to toggle
        if drawing[x][y] == 0:
            drawing[x][y] = 1
            pin_number = electrode[y][x]
            
            drawing_canvas.create_rectangle(x*40, y*40, (x+1)*40, (y+1)*40, fill="black", outline="white")
            drawing_canvas.create_text((x+0.5)*40, (y+0.5)*40, text=str(pin2[electrode[y][x]-1]), fill="red")
        else:
            drawing[x][y] = 0
            drawing_canvas.create_rectangle(x*40, y*40, (x+1)*40, (y+1)*40, fill="white", outline="white")
    elif event.num == 3:
        # Right click to erase
        drawing[x][y] = 0
        drawing_canvas.create_rectangle(x*40, y*40, (x+1)*40, (y+1)*40, fill="white", outline="white")
        
drawing_canvas.bind("<Button-1>", draw_pixel)
drawing_canvas.bind("<Button-3>", draw_pixel)

# Define electrode matrix with disabled ports as 0
electrode=[
        [1, 2, 3, 4, 5, 6],
        [7, 8, 9, 10, 11, 12],
        [13, 14, 15, 16, 17, 18],
        [19, 20, 21, 22, 23, 24],
        [25, 26, 27, 28, 29, 30],
        [31, 32, 33, 34, 35, 36]
        ]
pin=    [14,15,16,17,18,19,0,0,22,23,
         24,25,26,27,28,29,30,31,32,33,
         0,35,36,37,38,39,40,41,42,43,
         0,45,46,47,48,49,50,51,52,53]#36 pin

pin2=   [53,15,17,19,23,24,
         52,14,16,22,25,26,
         50,51,18,29,27,28,
         48,47,49,39,31,30,
         46,45,41,37,35,32,
         43,42,40,38,36,33,
         ]
"""pin2=[31,15,17,19,21,22,
         30,14,16,18,33,34,
         28,29,27,18,35,36,
         26,25,47,37,39,38,
         24,23,49,45,43,40,
         51,50,48,46,44,41,
         ]#old electrode"""


# Initialize the drawing list with the mapped electrode matrix
def print_matrix(matrix):
    matrix=[list(row) for row in zip(*matrix)]
    print(matrix)
    # Flatten the matrix into a 36-element list
    binary_list = [pixel for row in matrix for pixel in row]
    
    # Prepare a mapping dictionary from pin2 to corresponding binary values
    mapping = dict(zip(pin2, binary_list))
    
    # Initialize the sorted list with the same format as pin
    sorted_list = []
    for p in pin:
        if p == 0:
            sorted_list.append('0')  # Keep disabled ports as 0
        elif p in mapping:
            sorted_list.append(str(mapping[p]))
        else:
            sorted_list.append('0')  # Handle missing mappings
    
    # Join the sorted list into a string and return
    result = ''.join(sorted_list)
    print(result)
    serial_monitor.insert(tk.END, "Sent: " + result + "\n")
    serial_monitor.see(tk.END)  # Scroll to the end
    ser.write(bytes(result, 'utf-8'))
    print('Sent')
    time.sleep(0.5)

    # Wait for the Arduino to finish printing and send an "ok" message
    if ser.in_waiting > 0:
        msg = ser.readline().decode('utf-8').strip()
        serial_monitor.insert(tk.END, "Received: " + msg + "\n")
        serial_monitor.see(tk.END)  # Scroll to the end

def send_matrix():
    print(drawing)
    print_matrix(drawing)
    serial_monitor.insert(tk.END, "Matrix Sent\n")
    serial_monitor.see(tk.END)  # Scroll to the end

def check_for_messages():
    if ser.in_waiting > 0:
        msg = ser.readline().decode('utf-8').strip()
        serial_monitor.insert(tk.END, "Received: " + msg + "\n")
        serial_monitor.see(tk.END)  # Scroll to the end
    root.after(100, check_for_messages)  # Check for new messages every 100 milliseconds

def send_request(event=None):
    request = request_entry.get()
    serial_monitor.insert(tk.END, "Sent: " + request + "\n")
    serial_monitor.see(tk.END)  # Scroll to the end
    ser.write(bytes(request, 'utf-8'))
    print('Sent:'+ request)
    time.sleep(0.5)
    if ser.in_waiting > 0:
        msg = ser.readline().decode('utf-8').strip()
        serial_monitor.insert(tk.END, "Received: " + msg + "\n")
        serial_monitor.see(tk.END)  # Scroll to the end
    request_entry.delete(0, tk.END)  # Clear the input field
def stop(event=None):
    request = "stop"
    serial_monitor.insert(tk.END, "Sent: " + request + "\n")
    serial_monitor.see(tk.END)  # Scroll to the end
    ser.write(bytes(request, 'utf-8'))
    print('Sent:'+ request)
    time.sleep(0.5)
    if ser.in_waiting > 0:
        msg = ser.readline().decode('utf-8').strip()
        serial_monitor.insert(tk.END, "Received: " + msg + "\n")
        serial_monitor.see(tk.END)  # Scroll to the end

def export_data():
    data = serial_monitor.get('1.0', 'end-1c')  # Get the text from the Text widget
    file = filedialog.asksaveasfile(mode='w', defaultextension='.txt')  # Open a file dialog
    if file is None:  # If the user cancels the file dialog
        return
    file.write(data)  # Write the data to the file
    file.close()  # Close the file
    print("Data exported to", file.name)
    
request_entry = tk.Entry(root, width=40)
request_entry.grid(row=1, column=0, sticky="ew")
request_entry.bind("<Return>", send_request)

button_frame = tk.Frame(root)
button_frame.grid(row=2, column=0, sticky="ew")
send_request_button = tk.Button(button_frame, text="Send Request", command=send_request)
send_request_button.pack(side=tk.LEFT)
stop_button = tk.Button(button_frame, text="Stop", command=stop)
stop_button.pack(side=tk.LEFT)
send_matrix_button = tk.Button(button_frame, text="Send Matrix", command=send_matrix)
send_matrix_button.pack(side=tk.LEFT)
export_button = tk.Button(button_frame, text="Export Data", command=export_data)
export_button.pack(side=tk.RIGHT)

serial_monitor = tk.Text(root, width=80, height=10)
serial_monitor.grid(row=3, column=0, sticky="nsew")
check_for_messages()
root.mainloop()
