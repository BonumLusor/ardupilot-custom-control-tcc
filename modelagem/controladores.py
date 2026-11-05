"""Espelhos em Python dos controladores de atitude que voam no SITL.

Cada classe calcula, a cada laco de controle, o esforco normalizado u de UM
eixo, a partir das mesmas grandezas que o firmware tem em maos:

    e_th   erro de angulo, alvo - medido (rad)            thrust_heading_rotation_angles
    w_t    velocidade angular alvo no corpo (rad/s)       get_attitude_target_ang_vel
    a_t    aceleracao angular alvo (rad/s2), por diferenca entre lacos
    giro   velocidade angular medida e filtrada (rad/s)   get_gyro_latest
    limite True se o laco anterior saturou o motor        _motors->limit

e todas tem a mesma interface, ``passo(e_th, w_t, a_t, giro, limite) -> u``,
para que a malha simulada (malha.py) troque de controlador sem mudar nada.

    CascataPID  o nativo (raiz=True: AC_AttitudeControl + AC_PID) e o backend
                de exemplo cc_pid (raiz=False: AC_CustomControl_PID)
    LQI         realimentacao de [x_I, e_th, e_w], com x_I a integral de e_th
    LQRESO      realimentacao de [e_th, e_w] e cancelamento da perturbacao
                estimada por um observador de estado estendido
    LQG         a mesma lei do LQRESO sobre os estados estimados por um filtro
                de Kalman que modela o passa-baixas do giroscopio, o ruido que
                o angulo do AHRS integra e o laco de atraso dele

Nas duas ultimas, o erro de velocidade angular compara o giroscopio filtrado
com o alvo passado pelo MESMO passa-baixas (fc_alvo, em geral INS_GYRO_FILTER):
sem isso, o atraso do filtro do giroscopio vira, na subida de um degrau, um
erro de taxa que nao existe, e a malha empurra alem do alvo. Na regulacao (alvo
parado) o filtro nao age, e margens e rejeicao de perturbacao nao mudam.

As duas ultimas sao a referencia do backend AC_CustomControl_LQR
(firmware/), que repete estas contas na mesma ordem.
"""

from __future__ import annotations

import math

import numpy as np

from blocos import ACC_CTRL, PID, PassaBaixas2p, raiz_quadrada


def modelo_discreto(a, b, dt):
    """Planta de um eixo, dw/dt = -a*w + b*u + d, com segurador de ordem zero.

    Devolve (fi, beta): w[k+1] = fi*w[k] + beta*(b*u[k] + d[k]), com
    fi = e^(-a*dt) e beta = (1 - fi)/a. E a Eq. (modelo-eixo-discreto) do Cap. 4.
    """
    fi = math.exp(-a * dt)
    beta = (1.0 - fi) / a if a > 0 else dt
    return fi, beta


def _limitar(x, lim):
    return x if lim is None or lim <= 0 else min(max(x, -lim), lim)


class CascataPID:
    """Controlador de angulo (raiz quadrada ou P) e PID de velocidade angular.

    Com ``ff`` = (a, b), soma a mesma alimentacao direta pura do LQI,
    u_ff = (a_t + a*w_t)/b, calculada so da trajetoria do alvo. E o que o
    backend cc_pid faz com CC2_FF = 1 (firmware/ardupilot.patch). Nao e o FF/D_FF
    do AC_PID, que multiplicam o alvo de taxa inteiro, inclusive a correcao do
    erro de angulo, e por isso mudam tambem a realimentacao.
    """

    def __init__(self, g, eixo, dt, raiz=True, ff=None):
        self.pid = PID(g["kp"], g["ki"], g["kd"], g["imax"], g["fltt"], g["flte"],
                       g["fltd"], dt, g.get("kff", 0.0), g.get("kdff", 0.0))
        self.p = g["ang_p"]
        self.raiz = raiz
        self.dt = dt
        lo, hi = ACC_CTRL[eixo]
        self.lim = (min(max(g["acc_max"] / 2.0, lo), hi)
                    if (raiz and g["acc_max"] > 0) else 0.0)
        self.ff = ff
        self.sinais = {}

    def passo(self, e_th, w_t, a_t, giro, limite=False):
        alvo_taxa = (raiz_quadrada(e_th, self.p, self.lim, self.dt) if self.raiz
                     else self.p * e_th) + w_t
        u = self.pid.atualizar(alvo_taxa, giro, limite)
        u_ff = (a_t + self.ff[0] * w_t) / self.ff[1] if self.ff else 0.0
        self.sinais = {"x_i": self.pid.integ, "u_ff": u_ff}
        return u + u_ff

    def integrador(self):
        return self.pid.integ

    def reset(self, i_nativo=0.0, giro=0.0):
        self.pid.integ = i_nativo

    def estado(self):
        return [self.pid.alvo, self.pid.erro, self.pid.deriv, self.pid.integ]

    def definir_estado(self, x):
        self.pid.alvo, self.pid.erro, self.pid.deriv, self.pid.integ = (float(v) for v in x)


class LQI:
    """u = u_ff + k_i*x_I + k_th*e_th + k_w*e_w, com x_I[k+1] = x_I[k] + dt*e_th[k].

    O ganho vem do LQR sobre a planta aumentada (projeto.py): u = -K z com
    z = [z_I, theta, w]. Na regulacao, e = -z, e por isso aqui os sinais sao
    todos positivos. O integrador para de crescer quando o motor satura no
    sentido do erro (integracao condicional).
    """

    def __init__(self, k_i, k_th, k_w, a, b, dt, ff=True, e_max=None, fc_alvo=0.0):
        self.k_i, self.k_th, self.k_w = k_i, k_th, k_w
        self.a, self.b, self.dt = a, b, dt
        self.ff, self.e_max = ff, e_max
        self.filtro_alvo = PassaBaixas2p(1.0 / dt, fc_alvo)
        self.x_i = 0.0
        self.u_ant = 0.0
        self.sinais = {}

    def passo(self, e_th, w_t, a_t, giro, limite=False):
        e_th = _limitar(e_th, self.e_max)
        e_w = self.filtro_alvo.aplicar(w_t) - giro
        u_ff = (a_t + self.a * w_t) / self.b if self.ff else 0.0
        u = u_ff + self.k_i * self.x_i + self.k_th * e_th + self.k_w * e_w
        if not (limite and e_th * self.u_ant > 0):
            self.x_i += self.dt * e_th
        self.u_ant = u
        self.sinais = {"x_i": self.x_i, "u_ff": u_ff}
        return u

    def reset(self, i_nativo=0.0, giro=0.0):
        # transferencia sem salto: o integrador assume a parcela que o
        # integrador do nativo segurava
        self.x_i = i_nativo / self.k_i if self.k_i else 0.0
        self.u_ant = i_nativo

    def estado(self):
        return [self.x_i]

    def definir_estado(self, x):
        self.x_i = float(x[0])


class LQRESO:
    """u = u_ff + k_th*e_th + k_w*e_w - d_hat/b, com d_hat de um ESO de [w, d].

    O observador usa o modelo identificado (nao o duplo integrador generico do
    ADRC), entao d_hat estima so a perturbacao, na mesma escala em rad/s2 em
    que o injetor a aplica:

        [w; d][k+1] = [[fi, beta], [0, 1]] [w; d][k] + [beta*b; 0] u[k]
                      + [l1; l2] (giro[k] - w_hat[k])

    E a forma preditora: o d_hat usado no laco k vem dos dados ate k-1. O u
    que entra no observador e o u aplicado, ja saturado, para que a estimativa
    nao acumule o que o motor nao entregou.
    """

    def __init__(self, k_th, k_w, l1, l2, a, b, dt, ff=True, e_max=None, u_max=1.0,
                 fc_alvo=0.0):
        self.k_th, self.k_w = k_th, k_w
        self.l1, self.l2 = l1, l2
        self.a, self.b, self.dt = a, b, dt
        self.ff, self.e_max, self.u_max = ff, e_max, u_max
        self.filtro_alvo = PassaBaixas2p(1.0 / dt, fc_alvo)
        self.fi, self.beta = modelo_discreto(a, b, dt)
        self.w_hat = 0.0
        self.d_hat = 0.0
        self.sinais = {}

    def passo(self, e_th, w_t, a_t, giro, limite=False):
        e_th = _limitar(e_th, self.e_max)
        e_w = self.filtro_alvo.aplicar(w_t) - giro
        u_ff = (a_t + self.a * w_t) / self.b if self.ff else 0.0
        u = u_ff + self.k_th * e_th + self.k_w * e_w - self.d_hat / self.b
        u_apl = _limitar(u, self.u_max)
        inov = giro - self.w_hat
        w_hat = self.fi * self.w_hat + self.beta * (self.b * u_apl + self.d_hat) + self.l1 * inov
        # eixo saturado no laco anterior: d_hat parado (como o backend)
        if not limite:
            self.d_hat = self.d_hat + self.l2 * inov
        self.w_hat = w_hat
        self.sinais = {"d_hat": self.d_hat, "w_hat": self.w_hat, "u_ff": u_ff}
        return u

    def reset(self, i_nativo=0.0, giro=0.0):
        # o integrador do nativo, em regime, segura -d/b: herda-se a estimativa
        self.d_hat = -self.b * i_nativo
        self.w_hat = giro

    def estado(self):
        return [self.w_hat, self.d_hat]

    def definir_estado(self, x):
        self.w_hat, self.d_hat = float(x[0]), float(x[1])


# --------------------------------------------------------------------------
# LQG
# --------------------------------------------------------------------------

# estados do estimador do LQG, em coordenadas de erro (alvo - medido)
ESTADOS_LQG = ("e_th", "e_w", "f1", "f2", "d", "eta", "bg", "z")
N_LQG = len(ESTADOS_LQG)
N_CONT = N_LQG - 1           # os estados continuos; o ultimo (z) e um atraso discreto


def matrizes_lqg(a, b, fc, tau_c, tau_d=None):
    """Modelo continuo do estimador do LQG: (Ac, Bu, G), sete estados.

        e_th' = e_w
        e_w'  = -a e_w - b u_fb - d                   (u_ff cancela o alvo)
        f1'   = f2
        f2'   = wc^2 (e_w - n) - wc^2 f1 - raiz(2) wc f2   Butterworth do giroscopio
        d'    = w_d - d/tau_d                          passeio aleatorio (sem tau_d)
                                                       ou Gauss-Markov
        eta'  = n - eta/tau_c                          ruido do angulo do AHRS
        bg'   = w_b                                    vies residual do giroscopio

    O giroscopio medido e o alvo de taxa passam pelo mesmo passa-baixas, entao a
    medida e_w = filtro(w_t) - giro e f1 - bg, o filtro aplicado a e_w - n menos
    o vies que a correcao do EKF deixa no giroscopio que o backend le (medido nos
    voos: -0,02 a -0,07 graus/s). O angulo do AHRS integra o MESMO ruido n sem
    filtro (validacao.conferir_ruido), mas nao esse vies: em baixa frequencia o
    EKF o mantem alinhado a gravidade. O erro de angulo medido e e_th - eta.
    Sem bg, um vies constante na taxa so se explicaria por um angulo que deriva,
    e o estimador o trocaria por um erro de angulo estatico (bg*tau_c, visto no
    SITL antes desta correcao). G tem as colunas de n, w_d e w_b.
    """
    wc = 2.0 * math.pi * fc
    Ac = np.zeros((N_CONT, N_CONT))
    Ac[0, 1] = 1.0
    Ac[1, 1], Ac[1, 4] = -a, -1.0
    Ac[2, 3] = 1.0
    Ac[3, 1], Ac[3, 2], Ac[3, 3] = wc * wc, -wc * wc, -math.sqrt(2.0) * wc
    Ac[4, 4] = -1.0 / tau_d if tau_d else 0.0
    Ac[5, 5] = -1.0 / tau_c if tau_c else 0.0
    Bu = np.zeros(N_CONT)
    Bu[1] = -b
    G = np.zeros((N_CONT, 3))
    G[3, 0], G[5, 0] = -wc * wc, 1.0
    G[4, 1] = 1.0
    G[6, 2] = 1.0
    return Ac, Bu, G


def expm_taylor(M, ordem=12):
    """exp(M) por serie de Taylor com escala e quadratura.

    E a conta que o backend em C++ repete a bordo: escala M por 2^s ate a
    norma ficar abaixo de 0,5, soma a serie e eleva ao quadrado s vezes.
    """
    norma = float(np.max(np.sum(np.abs(M), axis=1)))
    s = max(0, int(math.ceil(math.log2(norma / 0.5)))) if norma > 0.5 else 0
    X = M / (2.0 ** s)
    n = M.shape[0]
    E = np.eye(n)
    termo = np.eye(n)
    for k in range(1, ordem + 1):
        termo = termo @ X / k
        E = E + termo
    for _ in range(s):
        E = E @ E
    return E


def modelo_lqg(a, b, fc, dt, tau_c, tau_d=None, abl=0):
    """(Phi, Gam, C) do estimador do LQG, discretizado com segurador de ordem zero.

    Os sete estados de matrizes_lqg ganham um oitavo, z, o erro de angulo medido
    no laco anterior: o backend roda antes de read_AHRS (Copter.cpp) e le a
    atitude e o alvo de um laco atras. Medidas y = [e_th lido, e_w] = [z, f1 - bg].
    """
    Ac, Bu, _ = matrizes_lqg(a, b, fc, tau_c, tau_d)
    n = N_CONT
    M = np.zeros((n + 1, n + 1))
    M[:n, :n] = Ac * dt
    M[:n, n] = Bu * dt
    E = expm_taylor(M)
    Phi = np.zeros((N_LQG, N_LQG))
    Phi[:n, :n] = E[:n, :n]
    Phi[n, 0], Phi[n, 5] = 1.0, -1.0          # z[k+1] = e_th[k] - eta[k]
    Gam = np.zeros(N_LQG)
    Gam[:n] = E[:n, n]
    C = np.zeros((2, N_LQG))
    # abl (CC4_ABL): bit 2 tira o atraso do angulo (y_th = e_th - eta, e nao z);
    # bit 1 tira o filtro da taxa (y_w = e_w - bg, e nao f1 - bg). Para a ablacao.
    if abl & 2:
        C[0, 0], C[0, 5] = 1.0, -1.0
    else:
        C[0, n] = 1.0
    if abl & 1:
        C[1, 1], C[1, 6] = 1.0, -1.0
    else:
        C[1, 2], C[1, 6] = 1.0, -1.0
    return Phi, Gam, C


class LQG:
    """u = u_ff + k_th*e_th_hat + k_w*e_w_hat - d_hat/b, com os estados de um Kalman.

    O estimador, em coordenadas de erro e na forma corrente (o x_hat do laco k
    ja usa a medida do laco k):

        x_pred = Phi x_hat[k-1] + Gam u_fb_apl[k-1]
        x_hat  = x_pred + L (y[k] - C x_pred),   y = [e_th lido, e_w]

    com Phi, Gam e C de modelo_lqg, calculados a partir de a, b, fc, dt e tau_c,
    como o backend faz a bordo, e o ganho L (8 x 2) do projeto (projeto.py). A
    lei e a do LQRESO, sobre os estados estimados em vez das medidas: o LQG
    filtra o ruido com o modelo e recupera a fase que o passa-baixas do
    giroscopio tira, pelo angulo do AHRS, que nao passa por ele. O u que entra
    no estimador e o aplicado (saturado), menos a alimentacao direta.

    Com ``L_r`` (e ``tau_r``), um SEGUNDO estimador, rapido, roda junto, com as
    mesmas medidas e o mesmo u, e a lei usa a mistura alfa*x_rapido +
    (1 - alfa)*x_lento. alfa vai a 1 quando o alvo manobra (|w_t| >= w_ref) e
    volta a 0 com constante de tempo tau_alfa depois que ele para. E a correcao
    da guinada: o estimador lento (tau_c longo) desconfia do angulo do AHRS e
    rejeita torque com o ruido do nativo, mas rastreia mal um alvo que se move;
    o rapido (tau_c curto) rastreia, mas leva o ruido do AHRS aos motores. Os
    dois erros de estimacao convergem sozinhos, independentes de alfa, entao a
    mistura nao desestabiliza a malha (principio da separacao). Parado, alfa =
    0 e o controlador e o de um so estimador.
    """

    def __init__(self, k_th, k_w, L, a, b, dt, fc, tau_c, ff=True, e_max=None, u_max=1.0,
                 fc_alvo=0.0, tau_d=None, L_r=None, tau_r=None, w_ref=None, tau_alfa=1.0,
                 abl=0):
        self.k_th, self.k_w = k_th, k_w
        self.L = np.asarray(L, dtype=float).reshape(N_LQG, 2)
        self.a, self.b, self.dt = a, b, dt
        self.ff, self.e_max, self.u_max = ff, e_max, u_max
        self.Phi, self.Gam, self.C = modelo_lqg(a, b, fc, dt, tau_c, tau_d, abl)
        self.filtro_alvo = PassaBaixas2p(1.0 / dt, fc_alvo)
        self.x = np.zeros(N_LQG)
        self.duplo = L_r is not None and bool(w_ref)
        if self.duplo:
            self.L_r = np.asarray(L_r, dtype=float).reshape(N_LQG, 2)
            self.Phi_r, self.Gam_r, _ = modelo_lqg(a, b, fc, dt, tau_r, tau_d)
            self.w_ref = w_ref
            self.decai = math.exp(-dt / tau_alfa)
        self.x_r = np.zeros(N_LQG)
        self.m = 0.0
        self.u_fb_ant = 0.0
        self.sinais = {}

    def passo(self, e_th, w_t, a_t, giro, limite=False):
        e_th = _limitar(e_th, self.e_max)
        e_w = self.filtro_alvo.aplicar(w_t) - giro
        u_ff = (a_t + self.a * w_t) / self.b if self.ff else 0.0
        y = np.array([e_th, e_w])
        x_pred = self.Phi @ self.x + self.Gam * self.u_fb_ant
        self.x = x_pred + self._correcao(self.L, y - self.C @ x_pred, limite)
        alfa = 0.0
        if self.duplo:
            x_pred = self.Phi_r @ self.x_r + self.Gam_r * self.u_fb_ant
            self.x_r = x_pred + self._correcao(self.L_r, y - self.C @ x_pred, limite)
            self.m = max(abs(w_t) / self.w_ref, self.decai * self.m)
            alfa = min(self.m, 1.0)
            x = alfa * self.x_r + (1.0 - alfa) * self.x
        else:
            x = self.x
        u_fb = self.k_th * x[0] + self.k_w * x[1] - x[4] / self.b
        u = u_ff + u_fb
        self.u_fb_ant = _limitar(u, self.u_max) - u_ff
        self.sinais = {"d_hat": x[4], "w_hat": w_t - x[1], "th_hat": x[0],
                       "eta_hat": x[5], "bg_hat": x[6], "u_ff": u_ff, "alfa": alfa}
        return u

    @staticmethod
    def _correcao(L, inovacao, limite):
        """L (y - C x_pred), sem a linha da perturbacao (estado 4) com o eixo saturado."""
        c = L @ inovacao
        if limite:
            c[4] = 0.0
        return c

    def reset(self, i_nativo=0.0, giro=0.0):
        # o integrador do nativo, em regime, segura -d/b: herda-se a estimativa;
        # o resto (o vies inclusive) parte de zero
        self.x = np.zeros(N_LQG)
        self.x[4] = -self.b * i_nativo
        self.x_r = self.x.copy()
        self.m = 0.0
        self.u_fb_ant = i_nativo

    def estado(self):
        if not self.duplo:
            return list(self.x) + [self.u_fb_ant]
        return list(self.x) + list(self.x_r) + [self.u_fb_ant, self.m]

    def definir_estado(self, x):
        n = N_LQG
        self.x = np.array(x[:n], dtype=float)
        if not self.duplo:
            self.u_fb_ant = float(x[n])
            return
        self.x_r = np.array(x[n:2 * n], dtype=float)
        self.u_fb_ant = float(x[2 * n])
        self.m = float(x[2 * n + 1])


def linearizar(ctl, eps=1e-7):
    """Realizacao em espaco de estados do espelho, por perturbacao.

    Cada espelho e afim nas entradas e no estado interno enquanto nada satura
    (a raiz quadrada fica na faixa linear para erros pequenos), entao uma
    perturbacao de cada coordenada da as colunas exatas de

        x[k+1] = F x[k] + G y[k],   u[k] = H x[k] + J y[k],

    com y = [theta, w_med] na regulacao (alvo zero: e_th = -theta, w_t = 0).
    Assim a analise linear usa o mesmo codigo que a simulacao, e nao uma
    transcricao das equacoes. O estado do controlador e restaurado no fim.
    """
    salvo = ctl.estado()
    n = len(salvo)

    def passo(x, theta, w_med):
        ctl.definir_estado(x)
        u = ctl.passo(-theta, 0.0, 0.0, w_med, False)
        return np.array(ctl.estado(), dtype=float), u

    F, H = np.zeros((n, n)), np.zeros((1, n))
    G, J = np.zeros((n, 2)), np.zeros((1, 2))
    for i in range(n):
        x = np.zeros(n)
        x[i] = eps
        x1, u = passo(x, 0.0, 0.0)
        F[:, i], H[0, i] = x1 / eps, u / eps
    for j, (theta, w_med) in enumerate(((eps, 0.0), (0.0, eps))):
        x1, u = passo(np.zeros(n), theta, w_med)
        G[:, j], J[0, j] = x1 / eps, u / eps
    ctl.definir_estado(salvo)
    return F, G, H, J


def ganhos_eso(a, b, dt, w_o):
    """[l1, l2] que poem os dois polos do erro de estimacao em e^(-w_o*dt).

    Ackermann na forma fechada (o place_poles do scipy nao aceita polo duplo
    com uma unica saida). Com A_o = [[fi, beta], [0, 1]] e C = [1, 0], o erro
    evolui por A_o - L C, cujo polinomio caracteristico e
    z^2 - (fi + 1 - l1) z + (fi - l1 + beta*l2); igualando a (z - z_o)^2:
    """
    fi, beta = modelo_discreto(a, b, dt)
    z_o = math.exp(-w_o * dt)
    l1 = fi + 1.0 - 2.0 * z_o
    l2 = (z_o * z_o - fi + l1) / beta
    return l1, l2


def polos_eso(a, b, dt, l1, l2):
    fi, beta = modelo_discreto(a, b, dt)
    return np.linalg.eigvals(np.array([[fi - l1, beta], [-l2, 1.0]]))
