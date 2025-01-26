import tkinter as tk
from tkinter import ttk
import time
from pypid import PIDController

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Controle PID com Slider")

        # Progress bar
        self.progress = ttk.Progressbar(root, length=300, mode="determinate")
        self.progress.pack(pady=20)

        # Label para mostrar o valor da barra
        self.progress_label = tk.Label(root, text="Valor: 0")
        self.progress_label.pack(pady=10)

        # Slider
        self.slider = tk.Scale(root, from_=0, to=100, orient="horizontal", label="Setpoint")
        self.slider.pack(pady=10)

        # Instancia o controlador PID
        self.pid = PIDController(mode="manual", actuator_callback=self.update_progress)

        # Atualiza o PID a cada 100 ms
        self.update_pid()

    def update_pid(self):
        # Obtém o valor do slider e atualiza o setpoint
        setpoint = self.slider.get()
        self.pid.update(setpoint)

        # Chama a função novamente após 100ms
        self.root.after(100, self.update_pid)

    def update_progress(self, output):
        # Pega o valor atual da progress bar
        current_value = self.progress['value']
        
        # Ajuste a saída do PID para o intervalo de 0 a 100
        new_value = current_value + output / 2  # Ajusta a escala para o intervalo de 0 a 100
        new_value = max(0, min(100, new_value))  # Garante que o valor fique entre 0 e 100

        # Atualiza o valor da progress bar
        self.progress['value'] = new_value

        # Atualiza o texto da label com o valor atual
        self.progress_label.config(text=f"Valor: {int(new_value)}")

        # Atualiza a posição do PID com a nova posição da barra
        self.pid.pot(new_value)  # Atualiza a posição no PID diretamente

        self.root.update_idletasks()

# Criação da janela Tkinter
root = tk.Tk()
app = App(root)
root.mainloop()
