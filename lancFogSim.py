import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Constantes
g = 9.81  # Aceleração gravitacional (m/s^2)
Cd = 0.5  # Coeficiente de arrasto típico
rho = 1.225  # Densidade do ar ao nível do mar (kg/m^3)

# Parâmetros do foguete
altura = 12.6  # metros
diametro = 0.57  # metros
area_frontal = np.pi * (diametro / 2)**2  # Área frontal
empuxo = 240000  # Newtons
massa_inicial = 2570  # kg (estimativa com combustível)
massa_final = 625  # kg (estimativa sem combustível)
tempo_queima = 31 # segundos

# Modelo de impulso específico
Isp = empuxo / (massa_inicial * g)  # impulso específico

# Controle do ângulo da trajetória (manobras)
manobras = {0: 90, 10: 85, 15: 85, 20: 85, 25: 85, 30: 85}   # Exemplo: Graviturn

def calcular_angulo(t):
    """Interpolar o ângulo da trajetória baseado nas manobras."""
    tempos = sorted(manobras.keys())
    angulos = [manobras[tempo] for tempo in tempos]
    return np.interp(t, tempos, angulos)

def aceleracao(t, massa, velocidade_atual):
    """Calcula a aceleração considerando empuxo, gravidade e arrasto."""
    if t <= tempo_queima:
        thrust = empuxo
        massa = massa_inicial - (massa_inicial - massa_final) * (t / tempo_queima)
    else:
        thrust = 0  # Após o combustível acabar, sem empuxo

    # Calcula o ângulo da trajetória
    theta = np.radians(calcular_angulo(t))  # Ângulo em radianos

    # Arrasto aerodinâmico
    drag = 0.5 * Cd * rho * area_frontal * velocidade_atual**2

    # Aceleração total nas direções vertical (z) e horizontal (x)
    ax = (thrust * np.cos(theta) - drag * np.cos(theta)) / massa
    az = (thrust * np.sin(theta) - drag * np.sin(theta) - massa * g) / massa
    return ax, az

# Simulação
dt = 0.1  # passo de tempo
n_steps = int(31 / dt)  # simulação de 200 segundos
posicoes = [(0, 0, 0)]  # x, y, z
velocidade = [(0, 0)]  # vx, vz
massa = massa_inicial

def simulacao_3d():
    """Simula o lançamento e gera uma trajetória 3D."""
    global massa
    for step in range(n_steps):
        t = step * dt

        # Velocidade atual
        vx, vz = velocidade[-1]

        # Atualiza aceleração
        ax, az = aceleracao(t, massa, np.sqrt(vx**2 + vz**2))

        # Atualiza velocidades
        vx += ax * dt
        vz += az * dt
        velocidade.append((vx, vz))

        # Atualiza posição
        x, z = posicoes[-1][0], posicoes[-1][2]
        x += vx * dt
        z += vz * dt
        posicoes.append((x, 0, z))  # y permanece 0 (2D em 3D)

        # Atualiza massa
        if t <= tempo_queima:
            massa = massa_inicial - (massa_inicial - massa_final) * (t / tempo_queima)
'''
        # Parar se atingir a altitude alvo (110 km)
        if z >= 110000:
            break
'''

# Executa a simulação
simulacao_3d()

# Conversão para arrays
posicoes = np.array(posicoes)
x, y, z = posicoes[:, 0], posicoes[:, 1], posicoes[:, 2]

# Visualização 3D com círculo do foguete
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

ax.plot(x, y, z, label="Trajetória do Foguete")
ax.scatter(17500, 0, 110000, color="red", label="Altitude Alvo (110 km)")

'''
# Adicionando o círculo azul para representar o foguete
for i in range(0, len(x), 10):  # Intervalo para melhorar performance
    ax.scatter(x[i], y[i], z[i], color='blue', s=50, label="Foguete" if i == 0 else "")
'''

ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Altitude (m)")
ax.set_title("Lançamento de Foguete com Manobras - Trajetória 3D")
ax.legend()

plt.show()
