import tkinter as tk
from tkinter import messagebox
import serial
import threading

# Configurare port serial
port = "COM5"  # Modifică în funcție de portul folosit
baud_rate = 115200  # Baud rate
serial_connection = None

# Funcție pentru conectare la serial
def connect_serial():
    global serial_connection
    try:
        serial_connection = serial.Serial(port, baud_rate, timeout=1)
        messagebox.showinfo("Conectare Serial", f"Conectat la {port} cu baud rate {baud_rate}.")
    except serial.SerialException as e:
        messagebox.showerror("Eroare Serial", f"Nu s-a putut conecta la {port}: {e}")

# Funcție pentru trimitere și recepție date
def send_and_receive():
    global serial_connection
    if serial_connection is None or not serial_connection.is_open:
        messagebox.showerror("Eroare", "Portul serial nu este conectat.")
        return

    value = input_field.get()
    if not value.strip():
        messagebox.showwarning("Avertisment", "Câmpul de text este gol.")
        return

    try:
        # Trimite valoarea
        serial_connection.write(f"{value}\n".encode())
        
        # Așteaptă răspuns
        response = serial_connection.readline().decode().strip()
        
        # Afișează răspunsul
        output_label.config(text=f"Răspuns primit: {response}")
    except Exception as e:
        messagebox.showerror("Eroare", f"A apărut o problemă: {e}")

# Funcție pentru închidere aplicație
def close_app():
    global serial_connection
    if serial_connection and serial_connection.is_open:
        serial_connection.close()
    root.destroy()

# Funcție pentru asocierea Enter cu Submit
def enter_pressed(event):
    send_and_receive()

# Interfață Tkinter
root = tk.Tk()
root.title("Interfață Serială")
root.geometry("400x200")

# Etichetă și câmp de intrare
input_label = tk.Label(root, text="Introduceți o valoare:")
input_label.pack(pady=10)

input_field = tk.Entry(root, width=30)
input_field.pack(pady=5)
input_field.bind("<Return>", enter_pressed)

# Buton de Submit
submit_button = tk.Button(root, text="Submit", command=send_and_receive)
submit_button.pack(pady=10)

# Etichetă pentru afișarea răspunsului
output_label = tk.Label(root, text="Răspuns primit: ", fg="blue")
output_label.pack(pady=10)

# Conectare automată la serial la pornirea aplicației
threading.Thread(target=connect_serial, daemon=True).start()

# Închidere aplicație
root.protocol("WM_DELETE_WINDOW", close_app)
root.mainloop()
