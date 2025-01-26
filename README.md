# PYPID
PID control system for PYTHON

![image](https://github.com/user-attachments/assets/1adf0287-5165-4d9f-b869-0261ccad03e8)
this was the EXE I left, this link>https://drive.google.com/file/d/14ss6niEMOBahxKG7pD3g7_qy0O1qeM9T/view?usp=sharing



## Descrição
O `PIDController` é uma implementação de um controlador PID (Proporcional, Integral, Derivativo) para sistemas de controle automatizado. Ele permite ajustar automaticamente ou manualmente os parâmetros do PID, visando melhorar a precisão e estabilidade do sistema.

Este controlador pode ser utilizado em sistemas que requerem controle de posição ou de algum valor desejado, como motores, dispositivos de aquecimento ou sistemas de estabilidade. Ele possui um modo automático de ajuste de parâmetros, o que facilita a adaptação a mudanças no comportamento do sistema.

## Funcionalidades
- **Modo Automático:** Ajuste dinâmico dos parâmetros PID para otimizar a performance do controlador.
- **Modo Manual:** Permite ao usuário configurar manualmente os parâmetros PID (`Kp`, `Ki`, `Kd`).
- **Ajuste Automático:** Detecta oscilações e ajusta os parâmetros PID automaticamente para melhorar a resposta do sistema.
- **Controle Suave:** O controlador ajusta os parâmetros com base na performance observada, evitando mudanças abruptas e instabilidade.

## Instalação

Para utilizar o controlador PID, basta copiar a classe `PIDController` para o seu código Python.

## Uso

### 1. Inicialização
Crie uma instância da classe `PIDController` passando o modo de operação desejado ("automatic" ou "manual") e uma função de callback para o atuador (se necessário).

```python
def actuator_callback(output):
    # Envia o valor de saída para o atuador
    print(f"Saída do controlador: {output}")

pid = PIDController(mode="automatic", actuator_callback=actuator_callback)
```

### 2. Configuração dos Parâmetros (Somente no Modo Manual)
No modo manual, é possível ajustar os parâmetros `Kp`, `Ki` e `Kd` para controlar a performance do controlador.

```python
pid.config(Kp=10.0, Ki=1.0, Kd=0.5)
```

### 3. Atualização da Posição
Atualize a posição atual do sistema utilizando o método `pot(position)`, passando o valor da posição. Isso é importante para que o controlador possa calcular o erro.

```python
pid.pot(position=50.0)
```

### 4. Definindo o Setpoint
Para determinar o valor desejado (setpoint) que o sistema deve alcançar, use o método `update(setpoint)`. O controlador calculará a saída com base no erro entre a posição atual e o setpoint.

```python
pid.update(setpoint=100.0)
```

### 5. Log dos Parâmetros
Para visualizar os parâmetros atuais do controlador, use o método `log()`.

```python
print(pid.log())
```

## Ajuste Automático
No modo automático, o controlador irá ajustar seus parâmetros de forma dinâmica com base nas oscilações observadas e no tempo de resposta. Isso permite que o controlador se adapte a diferentes condições e mantenha o sistema estável.

### 6. Exemplo Completo

```python
import time

def actuator_callback(output):
    print(f"Saída do controlador: {output}")

pid = PIDController(mode="automatic", actuator_callback=actuator_callback)

# Atualizando a posição do sistema
pid.pot(position=50.0)

# Atualizando o setpoint
pid.update(setpoint=100.0)

# Verificando os parâmetros do controlador
print(pid.log())
```

## Funções e Parâmetros

### `__init__(self, mode="automatic", actuator_callback=None)`
Construtor que inicializa o controlador PID.
- `mode`: Modo de operação do controlador ("manual" ou "automatic").
- `actuator_callback`: Função de callback que recebe a saída do controlador (valor entre -100 e 100).

### `config(Kp=None, Ki=None, Kd=None)`
Configura os parâmetros `Kp`, `Ki`, `Kd` no modo manual.
- `Kp`: Valor proporcional.
- `Ki`: Valor integral.
- `Kd`: Valor derivativo.

### `pot(position)`
Atualiza a posição atual do sistema.
- `position`: Valor da posição atual do sistema.

### `update(setpoint)`
Atualiza o setpoint desejado e calcula a saída do controlador.
- `setpoint`: Valor desejado (setpoint).

### `log()`
Retorna os parâmetros atuais e o estado interno do controlador.

## Ajuste e Tuning

- **Modo Automático:** O controlador ajusta automaticamente os parâmetros PID com base nas oscilações do sistema.
  - O controlador observa o erro e o intervalo de tempo entre os ajustes para calcular os parâmetros PID.
  - Ajustes suaves são feitos para evitar oscilações excessivas ou sobreajuste.

- **Modo Manual:** Você pode configurar manualmente os parâmetros `Kp`, `Ki` e `Kd` para maior controle sobre o desempenho do controlador.

### Como ajustar os parâmetros PID:
- **Kp (Proporcional):** Aumentar `Kp` torna o sistema mais responsivo, mas pode causar oscilações se for muito alto.
- **Ki (Integral):** Aumentar `Ki` ajuda a eliminar o erro persistente, mas muito alto pode causar overshoot.
- **Kd (Derivativo):** Aumentar `Kd` ajuda a reduzir oscilações e melhora a resposta dinâmica.

