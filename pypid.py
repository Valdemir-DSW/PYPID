class PIDController:
    def __init__(self, mode="automatic", actuator_callback=None):
        """
        Inicializa o controlador PID.

        :param mode: Modo de operação, "manual" ou "automatic" (padrão: "automatic").
        :param actuator_callback: Função callback que recebe o valor de saída (-100 a 100).
        """
        if mode not in ["manual", "automatic"]:
            raise ValueError("O modo deve ser 'manual' ou 'automatic'.")

        self.mode = mode
        self.actuator_callback = actuator_callback

        # Parâmetros PID
        self.Kp = 1.0
        self.Ki = 0.1
        self.Kd = 0.05
        self.setpoint = 0.0
        self.current_position = 0.0
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        self.oscillation_history = []
        self.max_time = 1.0  # Tempo máximo para considerar ajuste
        self.tolerance = 2.0  # Tolerância para estabilidade

    def config(self, Kp=None, Ki=None, Kd=None):
        """
        Configura os parâmetros do PID (somente no modo manual).

        :param Kp: Valor proporcional.
        :param Ki: Valor integral.
        :param Kd: Valor derivativo.
        """
        if self.mode != "manual":
            raise RuntimeError("A configuração de parâmetros só é permitida no modo manual.")

        if Kp is not None:
            self.Kp = Kp
        if Ki is not None:
            self.Ki = Ki
        if Kd is not None:
            self.Kd = Kd

    def pot(self, position):
        """
        Atualiza a posição atual do sistema.

        :param position: Valor atual da posição.
        """
        self.current_position = position

    def _autotune(self, error, dt):
        """
        Ajusta automaticamente os parâmetros PID com base na oscilação.

        :param error: Erro atual.
        :param dt: Intervalo de tempo desde a última atualização.
        """
        self.oscillation_history.append((error, dt))
        if len(self.oscillation_history) > 10:
            self.oscillation_history.pop(0)
        elapsed_time = sum([entry[1] for entry in self.oscillation_history])
        errors = [entry[0] for entry in self.oscillation_history]

        if elapsed_time > self.max_time:
            if max(errors) - min(errors) > self.tolerance:
                # Ajuste básico (exemplo simplificado de Ziegler-Nichols)
                Ku = 4.0 / (sum(errors) / len(errors) if errors else 1)
                Tu = elapsed_time / len(self.oscillation_history)

                self.Kp = 0.6 * Ku
                self.Ki = 2 * self.Kp / Tu
                self.Kd = self.Kp * Tu / 8

    def update(self, setpoint):
        """
        Atualiza o setpoint desejado e calcula a saída do controlador.

        :param setpoint: Novo valor desejado.
        """
        self.setpoint = setpoint
        error = self.setpoint - self.current_position
        import time
        current_time = time.time()
        dt = 0.1 if self.previous_time is None else current_time - self.previous_time
        self.previous_time = current_time
        if self.mode == "automatic":
            self._autotune(error, dt)
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, 100), -100)
        if self.actuator_callback:
            self.actuator_callback(output)

    def log(self):
        """
        Retorna os parâmetros atuais e o estado interno do controlador.

        :return: Dicionário com os parâmetros e estado.
        """
        return {
            "mode": self.mode,
            "Kp": self.Kp,
            "Ki": self.Ki,
            "Kd": self.Kd,
            "setpoint": self.setpoint,
            "current_position": self.current_position,
            "integral": self.integral,
            "previous_error": self.previous_error
        }

# Exemplo de uso
def atuador_callback(valor):
    print(f"Valor enviado ao atuador: {valor}")

pid = PIDController(mode="automatic", actuator_callback=atuador_callback)

# Atualização da posição atual
pid.pot(20)

# Definir o setpoint desejado
pid.update(50)

# Obter os logs
print(pid.log())
