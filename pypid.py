import time

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
        self.Kp = 6.7
        self.Ki = 2.1
        self.Kd = 0.1
        self.setpoint = 0.0
        self.current_position = 0.0
        self.min_time_to_adjust = 0
        self.integral = 0.0
        self.stability_threshold = 0
        self.previous_error = 0.0
        self.previous_time = None
        self.oscillation_history = []
        self.max_time = 1.0  # Tempo máximo para considerar ajuste
        self.tolerance = 2.0  # Tolerância para estabilidade
        self.start_time = None  # Marca o tempo do início do movimento

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
        Ajusta automaticamente os parâmetros PID com base na oscilação de forma suave e controlada.

        :param error: Erro atual.
        :param dt: Intervalo de tempo desde a última atualização.
        """
        # Armazena o erro e o intervalo de tempo
        self.oscillation_history.append((error, dt))
        
        # Limita o tamanho do histórico para evitar o uso de muita memória
        if len(self.oscillation_history) > 5:  # Histórico pequeno para maior agilidade
            self.oscillation_history.pop(0)
        
        # Calcula o tempo total e os erros armazenados
        elapsed_time = sum([entry[1] for entry in self.oscillation_history])
        errors = [entry[0] for entry in self.oscillation_history]

        # Se o tempo de observação já for suficientemente longo para uma decisão
        if elapsed_time > self.min_time_to_adjust:
            # Verifica a amplitude das oscilações
            amplitude = max(errors) - min(errors)
            
            # Se a amplitude for suficientemente grande, ajusta os parâmetros
            if amplitude > self.tolerance:
                # Calcula o ganho proporcional (Ku) de forma controlada
                Ku = 4.0 / amplitude if amplitude != 0 else 1  # Para evitar divisão por zero
                
                # Tempo de oscilação (Tu) é a média do tempo entre os picos de erro
                Tu = elapsed_time / len(self.oscillation_history)

                # Ajuste mais suave dos parâmetros PID
                # Usamos multiplicadores mais baixos para evitar ajustes excessivos
                self.Kp = min(1.2 * Ku, 10.0)  # Kp nunca ultrapassa 10, ajustando de forma mais suave
                self.Ki = min(1.5 * self.Kp / Tu, 2.0)  # Ki limitado para evitar sobrecarga
                self.Kd = self.Kp * Tu / 10  # Ajuste de Kd mais suave para evitar mudanças abruptas

                # Verifica se o erro está diminuindo com o tempo para garantir que o sistema esteja estabilizando
                if abs(self.oscillation_history[-1][0]) < self.stability_threshold:
                    self.Kp *= 0.95  # Reduz levemente o Kp quando o sistema começa a estabilizar
                    self.Ki *= 0.95  # Reduz o Ki de forma gradual
                    self.Kd *= 0.95  # Reduz o Kd de forma gradual

                # Reset a história após o ajuste para iniciar um novo ciclo de ajuste
                self.oscillation_history = []


    def update(self, setpoint):
        """
        Atualiza o setpoint desejado e calcula a saída do controlador.

        :param setpoint: Novo valor desejado.
        """
        self.setpoint = setpoint
        error = self.setpoint - self.current_position
        current_time = time.time()
        dt = current_time - self.previous_time if self.previous_time is not None else 0.1  # Ajuste no cálculo de dt
        self.previous_time = current_time

        if self.start_time is None:
            self.start_time = current_time  # Marca o tempo de início do movimento

        # Verifica se o tempo excedeu 1 segundo sem atingir o setpoint
        if current_time - self.start_time > 1 and abs(error) > self.tolerance:
            # Ajusta os parâmetros PID para correção mais agressiva
            self.Kp *= 1.5  # Aumenta a proporcionalidade
            self.Ki *= 1.2  # Aumenta a integral para corrigir mais rapidamente
            self.Kd *= 1.1  # Aumenta a derivada para evitar overshoot

        if self.mode == "automatic":
            self._autotune(error, dt)
        
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        output = max(min(output, 100), -100)
        
        # Se o erro for muito pequeno, ou se o tempo passou muito, pare de ajustar agressivamente
        if abs(error) < self.tolerance and current_time - self.start_time > 1:
            self.Kp /= 1.5
            self.Ki /= 1.2
            self.Kd /= 1.1
            self.start_time = None  # Reseta o tempo de início quando o setpoint é alcançado

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
