#!/usr/bin/env python3
"""Projeto do controlador de atitude proposto sobre o modelo do Capitulo 4.

Duas variantes, as mesmas do backend AC_CustomControl_LQR (firmware/):

    LQI       u = u_ff + k_i*x_I + k_th*e_th + k_w*e_w, x_I a integral de e_th;
              ganho do LQR discreto sobre a planta aumentada [z_I, theta, w]
    LQR+ESO   u = u_ff + k_th*e_th + k_w*e_w - d_hat/b; ganho do LQR sobre
              [theta, w] e d_hat de um observador de estado estendido de [w, d]

com u_ff = (a_t + a*w_t)/b, a inversao do modelo identificado.

A analise linear nao transcreve as equacoes dos controladores: cada espelho de
controladores.py (o mesmo codigo que a simulacao usa) e linearizado por
perturbacao, e a malha e fechada com a planta do Cap. 4, o passa-baixas do
giroscopio e o segurador de ordem zero do laco. Dela saem polos, margens
(malha aberta na entrada da planta), sensibilidade a perturbacao e ganho de
ruido. As respostas no tempo vem de malha.py, com os perfis de torque dos
cenarios de ensaio.

    ./projeto.py                                   # ganhos e analise no terminal
    ./projeto.py --figuras ../monografia/figuras --tabelas ../monografia/tabelas
    ./projeto.py --parm caminho/params.parm --tensao-repouso 12.44
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml
from scipy import linalg, signal

RAIZ = Path(__file__).resolve().parent
PROJETO = RAIZ.parent
CENARIOS = PROJETO / "ensaios" / "cenarios"
CONTROLADORES = PROJETO / "ensaios" / "controladores"
sys.path.insert(0, str(PROJETO / "ensaios"))
sys.path.insert(0, str(RAIZ))

from analise import metricas as M          # noqa: E402
import controladores as C                  # noqa: E402
import malha                               # noqa: E402
import modelo_sitl as ms                   # noqa: E402
import validacao as V                      # noqa: E402

EIXOS = ("roll", "pitch", "yaw")
NOMES = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}
SUFIXO = {"roll": "X", "pitch": "Y", "yaw": "Z"}


# --------------------------------------------------------------------------
# modelo
# --------------------------------------------------------------------------

@dataclass
class Modelo:
    """Planta linear de atitude de um voo: a, b por eixo e as taxas do laco."""
    a: float
    b: dict
    fs_fisica: float
    fs_laco: float
    fc_giro: float
    p: dict = field(repr=False)
    origem: Path = None
    v_repouso: float = None

    @property
    def ts(self):
        # o mesmo periodo de laco da simulacao: passos inteiros da fisica
        return max(int(round(self.fs_fisica / self.fs_laco)), 1) / self.fs_fisica


# O voo que define o modelo do projeto. Fixo, e nao "o mais recente": cada voo
# tem sua tensao de bateria, e com ela b muda na quarta casa — o bastante para
# os YAMLs regerados deixarem de ser os que voaram.
VOO_DO_PROJETO = PROJETO / "ensaios" / "resultados" / "vento" / "rajada" / "nativo" / \
    "2026-10-27_214349" / "params.parm"


def carregar_modelo(parm=None, v_repouso=None):
    """Modelo do Cap. 4 a partir do params.parm de um ensaio (VOO_DO_PROJETO).

    Sem ``v_repouso``, usa a mediana do BAT.VoltR do mesmo voo, que e a tensao
    com que o firmware compensa o empuxo (Secao modelo-linear). Sem o voo do
    projeto, cai no ensaio mais recente do nativo.
    """
    if parm is None and VOO_DO_PROJETO.exists():
        parm = VOO_DO_PROJETO
    if parm is None:
        candidatos = [c for c in (PROJETO / "ensaios" / "resultados").rglob("params.parm")
                      if c.parent.parent.name == "nativo"
                      and not c.parent.name.endswith("_falhou")]
        if not candidatos:
            raise SystemExit("nenhum ensaio do nativo em ensaios/resultados; use --parm")
        parm = max(candidatos, key=lambda c: c.parent.name)
    parm = Path(parm)
    p = ms.ler_parm(parm)
    if v_repouso is None and any(parm.parent.glob("*.BIN")):
        v_repouso = V.ler_bat_wnd(parm.parent)[0]
    alt = 15.0
    cen = parm.parent / "cenario.yaml"
    if cen.exists():
        alt = float((yaml.safe_load(cen.read_text(encoding="utf-8")).get("voo") or {})
                    .get("altitude_m", 15))
    av = ms.Aeronave(p, ms.ALTITUDE_CASA + alt, v_repouso)
    A, B = av.linearizar(av.pairado())
    return Modelo(a=float(-A[0, 0]), b={e: float(B[i, i]) for i, e in enumerate(EIXOS)},
                  fs_fisica=float(p.get("SIM_RATE_HZ", 1200)),
                  fs_laco=float(p.get("SCHED_LOOP_RATE", 400)),
                  fc_giro=float(p.get("INS_GYRO_FILTER", 20)), p=p, origem=parm,
                  v_repouso=v_repouso)


# --------------------------------------------------------------------------
# plantas discretas
# --------------------------------------------------------------------------

def planta_eixo(a, b, ts):
    """[theta, w] com segurador de ordem zero: Eq. (modelo-eixo-discreto) do Cap. 4."""
    fi, beta = C.modelo_discreto(a, b, ts)
    Ad = np.array([[1.0, beta], [0.0, fi]])
    Ed = np.array([[(ts - beta) / a], [beta]])
    return Ad, b * Ed, Ed


@dataclass
class Planta:
    """Planta de um eixo vista pelo controlador, discretizada a ts.

    Estados [theta, w, f1, f2] (+ u atrasado, se houver atraso): a planta
    contínua em serie com o Butterworth de 2a ordem do giroscopio, que filtra
    so a velocidade angular; o angulo do AHRS sai sem filtro. Entradas: u, d
    (perturbacao, rad/s2) e n (ruido somado a w antes do filtro).
    Saidas y = [theta, w_med].
    """
    A: np.ndarray
    Bu: np.ndarray
    Bd: np.ndarray
    Bn: np.ndarray
    Cy: np.ndarray
    # covariancia, por laco, do ruido BRANCO do giroscopio de intensidade ts
    # (equivalente, em baixa frequencia, a uma amostra unitaria segurada por um
    # laco, a convencao de Bn): metodo de Van Loan
    Qn: np.ndarray = None


def planta_com_filtro(a, b, ts, fc_giro, atraso_lacos=0, ruido_angulo=False, tau_c=None,
                      atraso_angulo=0, angulo_indep=0.0):
    """A Planta de um eixo; o primeiro estado e sempre o angulo verdadeiro.

    Com ``ruido_angulo``, um quinto estado eta: o angulo do AHRS integra o mesmo
    ruido n do giroscopio, sem o passa-baixas, e medido = theta + eta, com
    eta' = n - eta/tau_c (malha.py, validacao.conferir_ruido). Com
    ``atraso_angulo``, a medida de angulo chega esse numero de lacos atrasada,
    como o backend a le (run_custom_controller antes de read_AHRS). Com
    ``angulo_indep`` (fracao da potencia do ruido do giroscopio), eta integra tambem
    um ruido independente do giroscopio (ANGULO_INDEP; malha.simular, mesmo nome).
    """
    wc = 2.0 * math.pi * fc_giro
    Ac = np.array([[0.0, 1.0, 0.0, 0.0],
                   [0.0, -a, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 1.0],
                   [0.0, wc * wc, -wc * wc, -math.sqrt(2.0) * wc]])
    Bc = np.array([[0.0, 0.0, 0.0],
                   [b, 1.0, 0.0],
                   [0.0, 0.0, 0.0],
                   [0.0, 0.0, wc * wc]])
    Cc = np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0]])
    if ruido_angulo:
        Ac = np.pad(Ac, ((0, 1), (0, 1)))
        Ac[4, 4] = -1.0 / tau_c if tau_c else 0.0
        Bc = np.vstack([Bc, [[0.0, 0.0, 1.0]]])
        Cc = np.hstack([Cc, [[1.0], [0.0]]])
    Ad, Bd, Cd, _, _ = signal.cont2discrete((Ac, Bc, Cc, np.zeros((2, 3))), ts, method="zoh")
    Bu, Be, Bn = Bd[:, [0]], Bd[:, [1]], Bd[:, [2]]
    n0 = Ac.shape[0]
    G = Bc[:, [2]]
    M = np.zeros((2 * n0, 2 * n0))
    M[:n0, :n0] = -Ac
    W = G @ G.T
    if ruido_angulo and angulo_indep > 0:
        W[4, 4] += angulo_indep
    M[:n0, n0:] = W * ts
    M[n0:, n0:] = Ac.T
    E = linalg.expm(M * ts)
    Qn = E[n0:, n0:].T @ E[:n0, n0:]
    Qn = 0.5 * (Qn + Qn.T)
    for _ in range(atraso_lacos):
        # u atrasado de um laco: novo estado que guarda o u do laco anterior
        n = Ad.shape[0]
        Ad = np.block([[Ad, Bu], [np.zeros((1, n)), np.zeros((1, 1))]])
        Bu = np.vstack([np.zeros((n, 1)), [[1.0]]])
        Be = np.vstack([Be, [[0.0]]])
        Bn = np.vstack([Bn, [[0.0]]])
        Cd = np.hstack([Cd, np.zeros((2, 1))])
        Qn = np.pad(Qn, ((0, 1), (0, 1)))
    for _ in range(atraso_angulo):
        # a medida de angulo do laco anterior: z[k+1] = y_angulo[k]
        n = Ad.shape[0]
        Ad = np.block([[Ad, np.zeros((n, 1))], [Cd[[0]], np.zeros((1, 1))]])
        Bu, Be, Bn = (np.vstack([m, [[0.0]]]) for m in (Bu, Be, Bn))
        Cd = np.vstack([np.hstack([np.zeros((1, n)), [[1.0]]]),
                        np.hstack([Cd[[1]], [[0.0]]])])
        Qn = np.pad(Qn, ((0, 1), (0, 1)))
    return Planta(Ad, Bu, Be, Bn, Cd, Qn)


def dlqr(A, B, Q, R):
    """Ganho do LQR discreto, Eq. (ganho-lqr), com P da Eq. (riccati)."""
    P = linalg.solve_discrete_are(A, B, Q, R)
    return np.linalg.solve(R + B.T @ P @ B, B.T @ P @ A)


def ganhos_lqi(a, b, ts, q, r):
    """(k_i, k_th, k_w) do LQR sobre z = [z_I, theta, w], z_I[k+1] = z_I[k] + ts*theta[k]."""
    Ad, Bd, _ = planta_eixo(a, b, ts)
    A = np.block([[np.ones((1, 1)), np.array([[ts, 0.0]])], [np.zeros((2, 1)), Ad]])
    B = np.vstack([[[0.0]], Bd])
    K = dlqr(A, B, np.diag(q), np.array([[r]]))
    return tuple(float(k) for k in K[0])


def ganhos_lqr(a, b, ts, q, r):
    """(k_th, k_w) do LQR sobre [theta, w]."""
    Ad, Bd, _ = planta_eixo(a, b, ts)
    K = dlqr(Ad, Bd, np.diag(q), np.array([[r]]))
    return tuple(float(k) for k in K[0])


# --------------------------------------------------------------------------
# analise linear da malha fechada
# --------------------------------------------------------------------------

@dataclass
class Analise:
    polos_z: np.ndarray
    w: np.ndarray                 # rad/s
    L: np.ndarray                 # malha aberta na entrada da planta
    S_d: np.ndarray               # theta/d, rad por rad/s2
    margens: dict
    ruido: float                  # RMS de u por unidade de ruido do giroscopio
    # o mesmo, com o ruido branco (Planta.Qn): e o que vale para o LQG, que le o
    # angulo sem filtro; ruido fica como era, para as tabelas e YAMLs de antes
    ruido_branco: float = None
    # malha fechada, para as metricas lineares rapidas das varreduras:
    # x+ = Acl x + Bd d + Bn n, theta = C_th x, u = C_u x
    Acl: np.ndarray = None
    Bd: np.ndarray = None
    Bn: np.ndarray = None
    C_th: np.ndarray = None
    C_u: np.ndarray = None

    def estavel(self):
        return bool(np.max(np.abs(self.polos_z)) < 1.0)


def margens(w, L):
    """Cruzamento de ganho, margens de fase, de ganho e de atraso de L(e^jwT).

    Com o integrador e a planta tipo 1, a fase pode cruzar -180 graus tambem
    abaixo do cruzamento de ganho; ali a margem e de REDUCAO de ganho, e e
    reportada a parte (mg_reducao_db).
    """
    mod = np.abs(L)
    fase = np.unwrap(np.angle(L))
    res = {"wc": None, "mf_graus": None, "mg_db": None, "mg_reducao_db": None,
           "margem_atraso_s": None}
    desce = np.nonzero((mod[:-1] >= 1.0) & (mod[1:] < 1.0))[0]
    if len(desce):
        i = desce[-1]
        f = (0.0 - math.log(mod[i])) / (math.log(mod[i + 1]) - math.log(mod[i]))
        wc = math.exp(math.log(w[i]) + f * (math.log(w[i + 1]) - math.log(w[i])))
        fc = fase[i] + f * (fase[i + 1] - fase[i])
        mf = math.degrees(fc) + 180.0
        mf = (mf + 180.0) % 360.0 - 180.0
        res.update(wc=wc, mf_graus=mf, margem_atraso_s=math.radians(mf) / wc if mf > 0 else 0.0)
    # cruzamentos de fase com -180 (mod 360)
    k = np.floor((fase + math.pi) / (2 * math.pi))
    cruza = np.nonzero(k[:-1] != k[1:])[0]
    acima, abaixo = [], []
    for i in cruza:
        g = mod[i]
        (acima if g < 1.0 else abaixo).append(-20.0 * math.log10(g))
    if acima:
        res["mg_db"] = min(acima)
    if abaixo:
        res["mg_reducao_db"] = min(-x for x in abaixo)
    return res


def W_PADRAO(ts, n=2000):
    """Grade de frequencias (rad/s) de 0,003 rad/s ate quase Nyquist."""
    return np.logspace(-2.5, math.log10(0.999 * math.pi / ts), n)


def analisar(ctl, a, b, ts, fc_giro, atraso_lacos=0, w=None, planta=None):
    """Malha fechada de um eixo com o espelho ``ctl`` e a planta (a, b) dada.

    O controlador pode ter sido projetado com outro b: e assim que a robustez
    a erro de modelo e medida. ``planta`` e um dict com as opcoes de
    planta_com_filtro (ruido_angulo, tau_c, atraso_angulo); PLANTA_REAL e a do
    SITL conferida na Etapa 0.
    """
    pl = planta_com_filtro(a, b, ts, fc_giro, atraso_lacos, **(planta or {}))
    F, G, H, J = C.linearizar(ctl)
    n_p, n_c = pl.A.shape[0], F.shape[0]
    # u = H xc + J y,  y = Cy xp,  xp+ = A xp + Bu u,  xc+ = F xc + G y
    Acl = np.block([[pl.A + pl.Bu @ J @ pl.Cy, pl.Bu @ H],
                    [G @ pl.Cy, F]])
    Bd = np.vstack([pl.Bd, np.zeros((n_c, 1))])
    Bn = np.vstack([pl.Bn, np.zeros((n_c, 1))])
    # o angulo VERDADEIRO (primeiro estado da planta), e nao o medido
    C_th = np.zeros((1, n_p + n_c))
    C_th[0, 0] = 1.0
    C_u = np.hstack([J @ pl.Cy, H])
    polos = np.linalg.eigvals(Acl)

    if w is None:
        w = W_PADRAO(ts)
    z = np.exp(1j * w * ts)[:, None, None]
    # respostas em frequencia, resolvidas de uma vez para toda a grade
    P = pl.Cy @ np.linalg.solve(z * np.eye(n_p) - pl.A, np.broadcast_to(pl.Bu, (len(w), n_p, 1)))
    K = H @ np.linalg.solve(z * np.eye(n_c) - F, np.broadcast_to(G, (len(w), n_c, 2))) + J
    L = -(K @ P)[:, 0, 0]
    S = (C_th @ np.linalg.solve(z * np.eye(n_p + n_c) - Acl,
                                np.broadcast_to(Bd, (len(w), n_p + n_c, 1))))[:, 0, 0]

    ruido = ruido_branco = None
    if np.max(np.abs(polos)) < 1.0:
        W = linalg.solve_discrete_lyapunov(Acl, Bn @ Bn.T)
        ruido = float(np.sqrt((C_u @ W @ C_u.T)[0, 0]))
        Qcl = np.zeros_like(Acl)
        Qcl[:n_p, :n_p] = pl.Qn
        W = linalg.solve_discrete_lyapunov(Acl, Qcl)
        ruido_branco = float(np.sqrt((C_u @ W @ C_u.T)[0, 0]))
    return Analise(polos_z=polos, w=w, L=L, S_d=S, margens=margens(w, L), ruido=ruido,
                   ruido_branco=ruido_branco, Acl=Acl, Bd=Bd, Bn=Bn, C_th=C_th, C_u=C_u)


def polos_continuos(polos_z, ts):
    """Polos de tempo discreto levados ao plano s por s = ln(z)/ts.

    Os polos em z = 0 (estados de filtro desligados, atrasos puros) nao tem
    correspondente no plano s e ficam de fora.
    """
    z = np.asarray(polos_z, complex)
    z = z[np.abs(z) > 1e-9]
    return np.log(z) / ts


# --------------------------------------------------------------------------
# controladores do projeto
# --------------------------------------------------------------------------

def controlador_nativo(p, eixo, ts, prefixo="ATC", raiz=True):
    g = V.ganhos(p, prefixo, eixo)
    return C.CascataPID(g, eixo, ts, raiz=raiz), g


def equivalente_nativo(g):
    """(k_i, k_th, k_w) da cascata P de angulo + PI de taxa, sem filtros nem D.

    Com e_r = K_a*e_th + e_w e a integral de e_w igual a e_th:
    u = (I_r K_a) x_I + (P_r K_a + I_r) e_th + P_r e_w.
    """
    return g["ki"] * g["ang_p"], g["kp"] * g["ang_p"] + g["ki"], g["kp"]


def cascata_do_lqi(k_i, k_th, k_w, k_a_ref):
    """O inverso de equivalente_nativo: (P_r, I_r, K_a) que dao o mesmo LQI.

    K_a e raiz de k_w K_a^2 - k_th K_a + k_i = 0; das duas, a mais proxima do
    ganho de angulo atual. Sem raiz real, a cascata nao reproduz esse LQI.
    """
    disc = k_th * k_th - 4.0 * k_w * k_i
    if disc < 0:
        return None
    raizes = [(k_th + s * math.sqrt(disc)) / (2.0 * k_w) for s in (1.0, -1.0)]
    k_a = min(raizes, key=lambda r: abs(r - k_a_ref))
    return {"kp": k_w, "ki": k_i / k_a, "ang_p": k_a}


# --------------------------------------------------------------------------
# sintonia
# --------------------------------------------------------------------------

# Regra de Bryson para os estados (Secao tecnicas-de-controle): o desvio que se
# aceita de cada um. O peso do esforco nao e fixado a mao: sai da exigencia de
# que a malha tenha a mesma frequencia de cruzamento do nativo, para que a
# comparacao seja feita com a mesma banda passante e a diferenca venha da
# estrutura, e nao de mais ganho.
PESOS = {"roll": (math.radians(1.0), math.radians(30.0)),
         "pitch": (math.radians(1.0), math.radians(30.0)),
         "yaw": (math.radians(5.0), math.radians(30.0))}
ZETA_MIN = 0.7          # amortecimento minimo dos polos dentro da banda
GRADE_T_I = np.logspace(math.log10(3.0), math.log10(0.02), 36)      # s
GRADE_W_O = np.logspace(math.log10(0.5), math.log10(80.0), 36)      # rad/s
W_BUSCA = 600           # pontos de frequencia nas buscas (a analise final usa mais)


def fabrica_lqi(mod, eixo, t_i):
    th_m, w_m = PESOS[eixo]
    q = [1.0 / (th_m * t_i) ** 2, 1.0 / th_m ** 2, 1.0 / w_m ** 2]

    def fazer(r):
        k = ganhos_lqi(mod.a, mod.b[eixo], mod.ts, q, r)
        return C.LQI(*k, mod.a, mod.b[eixo], mod.ts), {"k_i": k[0], "k_th": k[1], "k_w": k[2]}
    return fazer


def fabrica_eso(mod, eixo, w_o):
    th_m, w_m = PESOS[eixo]
    q = [1.0 / th_m ** 2, 1.0 / w_m ** 2]
    l1, l2 = C.ganhos_eso(mod.a, mod.b[eixo], mod.ts, w_o)

    def fazer(r):
        k = ganhos_lqr(mod.a, mod.b[eixo], mod.ts, q, r)
        return (C.LQRESO(*k, l1, l2, mod.a, mod.b[eixo], mod.ts),
                {"k_th": k[0], "k_w": k[1], "l1": l1, "l2": l2, "w_o": w_o})
    return fazer


def casar(mod, eixo, fazer, alvo, medida="wc", lo=-2.0, hi=7.0, iters=32, planta=None):
    """Peso r do esforco que leva a ``medida`` (cruzamento "wc" ou "ruido") ao ``alvo``.

    Mais r, menos ganho: cruzamento mais baixo e menos ruido no esforco.
    Devolve None se nem o maior r da faixa baixa a medida ate o alvo (no ESO, o
    observador sozinho ja passa do cruzamento).
    """
    w = W_PADRAO(mod.ts, W_BUSCA)

    def valor(log_r):
        ctl, g = fazer(10.0 ** log_r)
        an = analisar(ctl, mod.a, mod.b[eixo], mod.ts, mod.fc_giro, w=w, planta=planta)
        if not an.estavel():
            return float("inf")
        return (an.margens["wc"] or 0.0) if medida == "wc" else an.ruido

    if valor(hi) > alvo or valor(lo) < alvo:
        return None
    for _ in range(iters):
        meio = 0.5 * (lo + hi)
        if valor(meio) > alvo:
            lo = meio
        else:
            hi = meio
    return 10.0 ** hi


def zeta_dominante(an, ts):
    """Menor amortecimento entre os polos de malha fechada dentro da banda (|s| < wc)."""
    s = polos_continuos(an.polos_z, ts)
    wc = an.margens["wc"] or np.inf
    dentro = [x for x in s if abs(x) < wc and abs(x) > 1e-9]
    return min((-x.real / abs(x) for x in dentro), default=1.0)


def polo_lento(an, ts):
    s = polos_continuos(an.polos_z, ts)
    return float(max(x.real for x in s))


def avaliar(mod, eixo, fazer, alvo, parametro, medida="wc", planta=None):
    r = casar(mod, eixo, fazer, alvo, medida, planta=planta)
    if r is None:
        return {"parametro": parametro, "viavel": False}
    ctl, g = fazer(r)
    an = analisar(ctl, mod.a, mod.b[eixo], mod.ts, mod.fc_giro, planta=planta)
    return {"parametro": parametro, "viavel": an.estavel(), "r": r, "ganhos": g,
            "analise": an, "mf": an.margens["mf_graus"], "zeta": zeta_dominante(an, mod.ts),
            "lento": polo_lento(an, mod.ts), "ruido": an.ruido}


def sintonizar(mod, eixo, variante, an_nativo, medida="wc", planta=None, relaxar=False):
    """Varre o parametro da variante e escolhe o ponto pelo criterio do projeto.

    Mesma frequencia de cruzamento do nativo (ou, com medida="ruido", o mesmo
    ganho de ruido), margem de fase nao menor que a dele e amortecimento
    dominante de ao menos ZETA_MIN; entre os pontos que passam, o de modo mais
    lento mais rapido (menor max Re s), que e o que governa a recuperacao de uma
    perturbacao sustentada. Devolve (escolhido, varredura).

    Com ``relaxar`` (as variantes de veiculo), um eixo em que a MF do nativo e
    inalcancavel nao aborta: fica o ponto de MF maxima (com ZETA_MIN), marcado
    com "mf_inalcancavel", so para as analises do modelo; o que voa sai de
    projetar_fixas. Com o giroscopio a 10 Hz, o LQI e o ESO (sem o termo D do
    nativo) ficam ~2 graus abaixo da MF dele em rolagem e arfagem.
    """
    mf_min = an_nativo.margens["mf_graus"]
    alvo = an_nativo.margens["wc"] if medida == "wc" else an_nativo.ruido
    if variante == "lqi":
        varredura = [avaliar(mod, eixo, fabrica_lqi(mod, eixo, t), alvo, t, medida, planta)
                     for t in GRADE_T_I]
    elif variante == "lqg":
        varredura = [avaliar(mod, eixo, fabrica_lqg(mod, eixo, q), alvo, q, medida, planta)
                     for q in GRADE_Q_D]
    else:
        varredura = [avaliar(mod, eixo, fabrica_eso(mod, eixo, w), alvo, w, medida, planta)
                     for w in GRADE_W_O]
    for v in varredura:
        v["passa"] = (v["viavel"] and v["mf"] is not None and v["mf"] >= mf_min - 1e-6
                      and v["zeta"] >= ZETA_MIN)
    candidatos = [v for v in varredura if v["passa"]]
    if not candidatos and relaxar:
        amortecidos = [v for v in varredura if v["viavel"] and v["mf"] is not None
                       and v["zeta"] >= ZETA_MIN]
        if amortecidos:
            v = max(amortecidos, key=lambda v: v["mf"])
            print(f"{eixo}/{variante}: MF do nativo ({mf_min:.1f}) inalcancavel; maxima "
                  f"{v['mf']:.1f} (parametro {v['parametro']:.3g}); o que voa e o do portao")
            return {**v, "mf_inalcancavel": True}, varredura
    if not candidatos:
        raise SystemExit(f"{eixo}/{variante}: nenhum ponto da varredura passa no criterio")
    return min(candidatos, key=lambda v: v["lento"]), varredura


# --------------------------------------------------------------------------
# LQG: Kalman em regime sobre o modelo do caminho de medida (Etapa 0)
# --------------------------------------------------------------------------

# A planta como o SITL a entrega ao backend, conferida nos logs
# (validacao.py --ruido): o angulo do AHRS integra o mesmo ruido do giroscopio,
# sem o passa-baixas, e chega um laco atrasado. As linhas antigas das tabelas
# projeto_* continuam na planta de antes; o LQG, as fronteiras e o portao usam
# esta, para todos os controladores.
PLANTA_REAL = {"ruido_angulo": True, "tau_c": V.TAU_AHRS, "atraso_angulo": 1}

# Fracao da potencia do ruido do giroscopio que o angulo medido integra de forma
# independente dele. No SITL, a derivada do erro do angulo e o erro da taxa tem
# coerencia de 0,93 a 0,98 de 1 a 10 Hz, e nao 1 (revisao de 3/11, Fase 3d); com o
# Kalman projetado para medidas perfeitamente consistentes (R_KALMAN quase nulo), o
# LQG jogava essa inconsistencia em d^, e o seu ruido no esforco saiu 1,4x (rolagem)
# e 6,4x (guinada) o previsto. Calibrada pelo esforco do cc_lqg medido com RND 20
# (monografia/verificacao/calibrar_angulo_indep.txt); a arfagem repete a rolagem.
ANGULO_INDEP = {"roll": 0.1766, "pitch": 0.1766, "yaw": 0.0411}


def planta_real(eixo):
    """A planta do portao (PLANTA_REAL) com o ruido independente do angulo do eixo."""
    return {**PLANTA_REAL, "angulo_indep": ANGULO_INDEP[eixo]}

# Ruido do giroscopio para o qual o Kalman e projetado: o dos voos de ruido/
# (SIM_GYRn_RND = 20), com o acelerador medio de 0,605 que o ruido_do_voo le
# deles (2,47 graus/s a 1 kHz, media de 8 sub-amostras).
RND_PROJETO = 20.0
ACEL_PAIRADO = 0.605

# Regularizacao das medidas. As duas medidas nao tem ruido proprio: o ruido do
# giroscopio entra como ruido de processo no filtro e em eta. A malha fechada
# converge para R abaixo de ~1e-12 (banda, margem e ganho de ruido mudam menos
# de 0,1 % por decada); daqui para baixo, R so evita a Riccati singular.
R_KALMAN = (1e-14, 1e-12)
# Constante de tempo do erro do angulo do AHRS NO ESTIMADOR. A fisica (Etapa 0)
# da ~3 s (V.TAU_AHRS, que a planta de analise usa), mas com ela o estimador
# atribui ao AHRS qualquer erro de angulo que o giroscopio nao confirme — um
# salto do alvo sem velocidade alvo, o vies que o EKF deixa no giroscopio — e o
# corrige em segundos: no SITL, 73 mdeg de erro estatico em rolagem e uma cauda
# de 0,1 grau em guinada. Em rolagem e arfagem, 0,3 s: o estimador confia no AHRS
# abaixo de ~0,5 Hz, que e o que se quer regular, e as metricas de torque ate
# melhoram — uma escolha de robustez, como a LTR, e nao uma descricao da fisica.
# Na guinada nao ha essa folga: o ganho sobre o nativo existe SO se o estimador
# descontar o angulo do AHRS por ~1,5 s ou mais (varredura de 31/10/2026: com
# 0,6 e 1,0 s nenhum LQG cumpre o portao; com 1,5 / 2,0 / 3,0 s, -22,6 / -27,4 /
# -23,0 % no pico do degrau de torque). O preco e o erro que um vies do
# giroscopio deixa no angulo medido, que decai em ~8 s (40 mdeg a 2 s para
# 0,05 graus/s, da ordem do erro estatico do nativo em guinada). 2,0 s: o maior
# ganho, com o mesmo preco de 1,5 s.
TAU_ESTIMADOR = {"roll": 0.3, "pitch": 0.3, "yaw": 2.0}
# A correcao da guinada: um segundo estimador, de tau_c curto, que assume quando o
# alvo manobra (|w_t| >= MANOBRA_W_REF) e devolve o controle ao lento com
# constante de tempo MANOBRA_TAU depois que ele para (controladores.LQG). Parado,
# so o lento age: o portao, feito com o alvo parado, nao muda. No modelo, o
# rastreamento do degrau de guinada passa de 0,305/0,102 graus (erro maximo/
# eficaz) para 0,141/0,029, contra 0,233/0,064 do PID com a mesma alimentacao
# direta, com a mesma rejeicao de torque e o mesmo ruido. Em rolagem e arfagem os
# dois estimadores seriam iguais, e o segundo fica desligado.
TAU_RAPIDO = {"roll": 0.3, "pitch": 0.3, "yaw": 0.3}
MANOBRA_W_REF = math.radians(5.0)      # rad/s
MANOBRA_TAU = 1.0                      # s
# Intensidade do passeio aleatorio do vies residual do giroscopio, (rad/s)^2/s:
# ~0,01 graus/s por raiz de segundo, o bastante para o estimador seguir o vies
# que o EKF deixa (-0,02 a -0,07 graus/s, variando em minutos) em segundos, e
# lento demais para disputar a banda do controle.
Q_VIES = math.radians(0.01) ** 2
GRADE_Q_D = np.logspace(-4.0, 5.0, 37)           # (rad/s2)^2 por s


def intensidade_ruido(rnd=RND_PROJETO, acel=ACEL_PAIRADO, sub=8):
    """Intensidade, em (rad/s)^2 s, do ruido branco do giroscopio: sigma^2 a 1 kHz x 1 ms."""
    sigma = math.radians(rnd * acel) / math.sqrt(3.0 * sub)
    return sigma * sigma * 1e-3


def ganhos_kalman(a, b, ts, fc, tau_c, q_n, q_d, r=R_KALMAN, tau_d=None, q_b=Q_VIES, q_a=0.0,
                  abl=0, sem_eta=False):
    """Ganho L (8 x 2) do Kalman em regime, na forma corrente, pela Riccati dual.

    Q discreta pelo metodo de Van Loan sobre o modelo continuo de
    controladores.matrizes_lqg, com ruidos n (giroscopio, intensidade q_n) e w_d
    (perturbacao, q_d, o botao de projeto). Como d entra no mesmo canal que
    b*u, subir q_d e a recuperacao da malha (LTR) de Doyle e Stein na entrada da
    planta. Phi e C sao os de controladores.modelo_lqg, os mesmos do espelho.
    """
    Ac, _, G = C.matrizes_lqg(a, b, fc, tau_c, tau_d)
    n = C.N_CONT
    # q_a: ruido independente do giroscopio que o angulo medido integra (em eta),
    # ANGULO_INDEP x q_n; com 0, as duas medidas sao supostas consistentes
    g_a = np.zeros((n, 1))
    g_a[5, 0] = 1.0
    G = np.hstack([G, g_a])
    if sem_eta:
        # ablacao: o estimador supoe o angulo do AHRS sem erro (eta sem ruido)
        G[5, 0] = 0.0
        q_a = 0.0
    M = np.zeros((2 * n, 2 * n))
    M[:n, :n] = -Ac
    M[:n, n:] = G @ np.diag([q_n, q_d, q_b, q_a]) @ G.T
    M[n:, n:] = Ac.T
    E = linalg.expm(M * ts)
    Phi_c = E[n:, n:].T
    Q = np.zeros((C.N_LQG, C.N_LQG))
    Q[:n, :n] = Phi_c @ E[:n, n:]
    Q = 0.5 * (Q + Q.T)
    Phi, _, Cm = C.modelo_lqg(a, b, fc, ts, tau_c, tau_d, abl)
    R = np.diag(r)
    P = linalg.solve_discrete_are(Phi.T, Cm.T, Q, R)
    return P @ Cm.T @ np.linalg.inv(Cm @ P @ Cm.T + R)


def fabrica_lqg(mod, eixo, q_d, q_n=None, tau_d=None):
    """LQG de um eixo: o LQR de [theta, w] do ESO (mesmos PESOS) e o Kalman de q_d."""
    th_m, w_m = PESOS[eixo]
    q = [1.0 / th_m ** 2, 1.0 / w_m ** 2]
    q_n = q_n or intensidade_ruido()
    L = ganhos_kalman(mod.a, mod.b[eixo], mod.ts, mod.fc_giro, TAU_ESTIMADOR[eixo],
                      q_n, q_d, tau_d=tau_d, q_a=ANGULO_INDEP[eixo] * q_n)

    def fazer(r):
        k = ganhos_lqr(mod.a, mod.b[eixo], mod.ts, q, r)
        return (C.LQG(k[0], k[1], L, mod.a, mod.b[eixo], mod.ts, mod.fc_giro, TAU_ESTIMADOR[eixo],
                      fc_alvo=mod.fc_giro, tau_d=tau_d),
                {"k_th": k[0], "k_w": k[1], "L": L, "q_d": q_d, "tau_c": TAU_ESTIMADOR[eixo],
                 "tau_d": tau_d})
    return fazer


# --------------------------------------------------------------------------
# metricas lineares rapidas e fronteiras rejeicao x ruido
# --------------------------------------------------------------------------

def resposta_linear(an, escada, t):
    """Angulo verdadeiro (graus) da malha fechada linear ao torque ``escada``, na grade t.

    O torque e segurado entre as escritas do injetor (50 Hz, alinhadas com o
    laco), entao a malha discreta com segurador de ordem zero e exata; difere
    da malha.py so na integracao de Euler a 1200 Hz.
    """
    d = malha.escada_em(t, escada)
    A, Bd, Ct = an.Acl, an.Bd[:, 0], an.C_th[0]
    x = np.zeros(A.shape[0])
    th = np.empty(len(t))
    for k in range(len(t)):
        th[k] = Ct @ x
        x = A @ x + Bd * d[k]
    return np.degrees(th)


_PERFIS = {}


def metricas_lineares(an, eixo, ts):
    """Pulso e degrau de torque do eixo (perfis dos cenarios), pela malha linear."""
    res = {}
    for ensaio in ("torque_pulso", "torque_degrau"):
        cen = f"{GRUPO[eixo]}/{ensaio}"
        if cen not in _PERFIS:
            _PERFIS[cen] = perfil_torque(cen, eixo)
        escada, jan, _amp = _PERFIS[cen]
        t = np.arange(jan[0] - 1.0, jan[1], ts)
        erro = -resposta_linear(an, escada, t)
        res[ensaio] = M.metricas_disturbio(t, erro, jan)
    return res


def h2_torque(an, sigma, tau, ts):
    """(RMS de theta em graus, RMS de u) sob torque Gauss-Markov de desvio sigma e tempo tau."""
    fi = math.exp(-ts / tau)
    n = an.Acl.shape[0]
    A = np.block([[an.Acl, an.Bd], [np.zeros((1, n)), np.array([[fi]])]])
    Bw = np.zeros((n + 1, 1))
    Bw[-1, 0] = sigma * math.sqrt(1.0 - fi * fi)
    W = linalg.solve_discrete_lyapunov(A, Bw @ Bw.T)
    ct = np.hstack([an.C_th, [[0.0]]])
    cu = np.hstack([an.C_u, [[0.0]]])
    return math.degrees(math.sqrt((ct @ W @ ct.T)[0, 0])), math.sqrt((cu @ W @ cu.T)[0, 0])


FAMILIAS = ("pid", "lqi", "eso", "lqg")
ROTULO_FAMILIA = {"pid": "PID (cascata, ganhos livres)", "lqi": "LQI", "eso": "LQR+ESO",
                  "lqg": "LQG"}
GRADE_R = np.logspace(-2.0, 7.0, 19)
# multiplicador do desvio aceito em w na regra de Bryson (PESOS): o terceiro
# botao do LQI, do ESO e do LQG, o equivalente ao ang_p livre do PID
GRADE_PW = (0.25, 0.5, 1.0, 2.0, 4.0)
GRADE_T_I_V = GRADE_T_I[::2]
GRADE_W_O_V = GRADE_W_O[::2]
GRADE_Q_D_V = np.logspace(-4.0, 5.0, 25)
# PID: multiplicadores dos ganhos do AutoTune, razao I/P (1/s) e FLTE (Hz)
GRADE_PID = {"kp": np.logspace(math.log10(0.4), math.log10(2.5), 9),
             "ki_kp": (0.1, 0.3, 1.0, 3.0, 10.0),
             "ang_p": np.logspace(math.log10(0.5), math.log10(2.0), 7),
             "flte": (0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0)}
# o mesmo torque estocastico do cenario torque_estocastico (Etapa 2): desvio
# por eixo e tempo de correlacao
TORQUE_ESTOCASTICO = {"sigma": {"roll": 10.0, "pitch": 10.0, "yaw": 1.0}, "tau": 0.1}

_CTX = {}


def ganhos_fixa(mod, eixo, familia, par):
    """Ganhos do LQI ou do LQR+ESO de um ponto da varredura, nos nomes do params_cc3."""
    a, b, ts = mod.a, mod.b[eixo], mod.ts
    th_m, w_m = PESOS[eixo]
    w_m *= par.get("pw", 1.0)
    if familia == "lqi":
        k_i, k_th, k_w = ganhos_lqi(a, b, ts, [1.0 / (th_m * par["t_i"]) ** 2, 1.0 / th_m ** 2,
                                               1.0 / w_m ** 2], par["r"])
        return {"k_i": k_i, "k_th": k_th, "k_w": k_w}
    l1, l2 = C.ganhos_eso(a, b, ts, par["w_o"])
    k_th, k_w = ganhos_lqr(a, b, ts, [1.0 / th_m ** 2, 1.0 / w_m ** 2], par["r"])
    return {"k_th": k_th, "k_w": k_w, "l1": l1, "l2": l2}


def _controlador_da_familia(mod, eixo, familia, par):
    a, b, ts = mod.a, mod.b[eixo], mod.ts
    th_m, w_m = PESOS[eixo]
    w_m *= par.get("pw", 1.0)
    if familia == "nativo":
        return novo_controlador(mod, eixo, "nativo")
    if familia == "pid":
        g = dict(V.ganhos(mod.p, "ATC", eixo))
        g.update(kp=g["kp"] * par["kp"], ang_p=g["ang_p"] * par["ang_p"],
                 flte=par["flte"])
        g["ki"] = g["kp"] * par["ki_kp"]
        return C.CascataPID(g, eixo, ts, raiz=True)
    if familia == "lqi":
        g = ganhos_fixa(mod, eixo, "lqi", par)
        return C.LQI(g["k_i"], g["k_th"], g["k_w"], a, b, ts)
    if familia == "eso":
        g = ganhos_fixa(mod, eixo, "eso", par)
        return C.LQRESO(g["k_th"], g["k_w"], g["l1"], g["l2"], a, b, ts)
    if familia.startswith("lqg_abl:"):
        opc = ABLACOES[familia.split(":", 1)[1]]
        chave = (familia, eixo, par["q_d"])
        cache = _CTX.setdefault("kalman", {})
        if chave not in cache:
            cache[chave] = kalman_ablacao(mod, eixo, par["q_d"], opc)
        k = ganhos_lqr(a, b, ts, [1.0 / th_m ** 2, 1.0 / w_m ** 2], par["r"])
        return C.LQG(k[0], k[1], cache[chave], a, b, ts, mod.fc_giro, TAU_ESTIMADOR[eixo],
                     fc_alvo=mod.fc_giro, abl=opc.get("abl", 0))
    if familia == "lqg":
        chave = (eixo, par["q_d"], ANGULO_INDEP[eixo])
        cache = _CTX.setdefault("kalman", {})
        if chave not in cache:
            q_n = intensidade_ruido()
            cache[chave] = ganhos_kalman(a, b, ts, mod.fc_giro, TAU_ESTIMADOR[eixo],
                                         q_n, par["q_d"], q_a=ANGULO_INDEP[eixo] * q_n)
        k = ganhos_lqr(a, b, ts, [1.0 / th_m ** 2, 1.0 / w_m ** 2], par["r"])
        return C.LQG(k[0], k[1], cache[chave], a, b, ts, mod.fc_giro, TAU_ESTIMADOR[eixo],
                     fc_alvo=mod.fc_giro)
    raise ValueError(familia)


def avaliar_ponto(tarefa):
    """Um controlador na planta real: margens, robustez, ruido e metricas de torque.

    Roda num processo do Pool; o modelo vem de _CTX (herdado no fork).
    """
    familia, eixo, par = tarefa
    mod = _CTX["mod"]
    a, b, ts = mod.a, mod.b[eixo], mod.ts
    w = W_PADRAO(ts, W_BUSCA)
    ctl = _controlador_da_familia(mod, eixo, familia, par)
    an = analisar(ctl, a, b, ts, mod.fc_giro, w=w, planta=planta_real(eixo))
    res = {"familia": familia, "eixo": eixo, "par": par, "estavel": an.estavel()}
    if not res["estavel"] or an.margens["mf_graus"] is None:
        res["estavel"] = False
        return res
    rob = []
    for _nome, fator, atraso in CASOS_ROBUSTEZ:
        r = analisar(ctl, a, b * fator, ts, mod.fc_giro, atraso_lacos=atraso, w=w,
                     planta=planta_real(eixo))
        rob.append(r.margens["mf_graus"] if r.estavel() and r.margens["mf_graus"] else None)
    te = TORQUE_ESTOCASTICO
    rms_th, rms_u = h2_torque(an, te["sigma"][eixo], te["tau"], ts)
    res.update(wc=an.margens["wc"], mf=an.margens["mf_graus"], mg=an.margens["mg_db"],
               ruido=an.ruido_branco, ruido_segurado=an.ruido, mf_rob=rob, zeta=zeta_dominante(an, ts),
               lento=polo_lento(an, ts), metricas=metricas_lineares(an, eixo, ts),
               estocastico={"rms_th": rms_th, "rms_u": rms_u})
    return res


def tarefas_familia(familia, eixo):
    if familia == "pid":
        gp = GRADE_PID
        return [("pid", eixo, {"kp": float(kp), "ki_kp": float(ki), "ang_p": float(ap),
                               "flte": float(fe)})
                for kp in gp["kp"] for ki in gp["ki_kp"] for ap in gp["ang_p"]
                for fe in gp["flte"]]
    if familia == "lqi":
        return [("lqi", eixo, {"t_i": float(t), "r": float(r), "pw": pw}) for t in GRADE_T_I_V
                for r in GRADE_R for pw in GRADE_PW]
    if familia == "eso":
        return [("eso", eixo, {"w_o": float(w), "r": float(r), "pw": pw}) for w in GRADE_W_O_V
                for r in GRADE_R for pw in GRADE_PW]
    # o LQG e as suas variantes de ablacao (lqg_abl:<nome>) varrem a mesma grade
    return [(familia, eixo, {"q_d": float(q), "r": float(r), "pw": pw}) for q in GRADE_Q_D_V
            for r in GRADE_R for pw in GRADE_PW]


def varrer(mod, eixos=("roll", "yaw"), familias=FAMILIAS, processos=None):
    """Avalia, em paralelo, o nativo e todos os pontos das familias nos eixos dados."""
    import multiprocessing as mp
    _CTX["mod"] = mod
    tarefas = [("nativo", e, {}) for e in eixos]
    for e in eixos:
        for f in familias:
            tarefas += tarefas_familia(f, e)
    with mp.get_context("fork").Pool(processos or max(mp.cpu_count() - 2, 1)) as pool:
        pontos = pool.map(avaliar_ponto, tarefas, chunksize=8)
    return pontos


# o que cada eixo quer melhorar (metrica primaria) e o que nao pode piorar
CRITERIO = {
    "roll": {"primaria": ("torque_pulso", "desvio_max_deg"),
             "nao_pior": (("torque_pulso", "desvio_max_deg"), ("torque_pulso", "rmse_deg"),
                          ("torque_degrau", "desvio_max_deg")),
             "ganho_sobre": "lqi", "teto_recuperacao": ("torque_pulso", "lqi", 1.1)},
    "pitch": {"primaria": ("torque_pulso", "desvio_max_deg"),
              "nao_pior": (("torque_pulso", "desvio_max_deg"), ("torque_pulso", "rmse_deg"),
                           ("torque_degrau", "desvio_max_deg")),
              "ganho_sobre": "lqi", "teto_recuperacao": ("torque_pulso", "lqi", 1.1)},
    "yaw": {"primaria": ("torque_degrau", "desvio_max_deg"),
            "nao_pior": (("torque_degrau", "desvio_max_deg"), ("torque_pulso", "desvio_max_deg"),
                         ("torque_pulso", "rmse_deg")),
            "ganho_sobre": "nativo", "teto_recuperacao": ("torque_degrau", None, 3.0)},
}
TOL_NAO_PIOR = 0.02      # "nao pior": ate 2 % acima do nativo, dentro do erro do modelo
GANHO_MIN = 0.10         # "ganho": 10 %, 3x o erro modelo x SITL e C++ x espelho
GANHO_ATRIBUICAO = 0.05  # P2: o LQG contra a melhor familia de estrutura fixa


def _m(p, ensaio, chave):
    v = (p.get("metricas") or {}).get(ensaio, {}).get(chave)
    return float("inf") if v is None else v


def viavel(p, nat, zeta_min):
    """Estavel, MF e MF robusta (cada caso) nao menores que as do nativo, zeta >= zeta_min."""
    if not p.get("estavel"):
        return False
    if p["mf"] < nat["mf"] - 1e-6 or p["zeta"] < zeta_min - 1e-9:
        return False
    for mf, mf_n in zip(p["mf_rob"], nat["mf_rob"]):
        if mf_n is not None and (mf is None or mf < mf_n - 1e-6):
            return False
    return True


def cumpre(p, nat, eixo, teto_rec, ruido_max):
    """viavel + ruido <= ruido_max + nada pior que o nativo (2 %) + teto de recuperacao."""
    cr = CRITERIO[eixo]
    if p["ruido"] > ruido_max * (1 + 1e-9):
        return False
    for ensaio, chave in cr["nao_pior"]:
        if _m(p, ensaio, chave) > _m(nat, ensaio, chave) * (1 + TOL_NAO_PIOR):
            return False
    ensaio = cr["teto_recuperacao"][0]
    return _m(p, ensaio, "tempo_recuperacao_s") <= teto_rec


def pareto(pontos, x="ruido", y=None):
    """Pontos nao dominados em (x, y), ordenados por x."""
    y = y or (lambda p: p["primaria"])
    ordem = sorted(pontos, key=lambda p: (p[x], y(p)))
    frente, melhor = [], float("inf")
    for p in ordem:
        if y(p) < melhor:
            frente.append(p)
            melhor = y(p)
    return frente


def fronteiras(pontos, eixo, familias=FAMILIAS):
    """Por familia, os pontos viaveis e a frente de Pareto (ruido x metrica primaria)."""
    nat = next(p for p in pontos if p["familia"] == "nativo" and p["eixo"] == eixo)
    zeta_min = min(ZETA_MIN, nat["zeta"])
    ens, chave = CRITERIO[eixo]["primaria"]
    res = {"nativo": nat, "zeta_min": zeta_min}
    for f in familias:
        vi = [p for p in pontos if p["familia"] == f and p["eixo"] == eixo
              and viavel(p, nat, zeta_min)]
        for p in vi:
            p["primaria"] = _m(p, ens, chave)
        res[f] = {"viaveis": vi, "frente": pareto(vi)}
    nat["primaria"] = _m(nat, ens, chave)
    return res


def candidatos_sob(frs, eixo, familia, ruido_max, teto_rec):
    """Os pontos da familia que cumprem tudo (cumpre), da melhor metrica primaria a pior."""
    nat = frs["nativo"]
    ok = [p for p in frs[familia]["viaveis"] if cumpre(p, nat, eixo, teto_rec, ruido_max)]
    return sorted(ok, key=lambda p: p["primaria"])


SEGUNDOS_RUIDO = 600.0
SEMENTES_RUIDO = (1, 2)


def ruido_no_tempo(mod, eixo, familia, par):
    """u eficaz (media de SEMENTES_RUIDO) numa malha.simular de SEGUNDOS_RUIDO sem perturbacao.

    Com o ruido do giroscopio de RND_PROJETO, o angulo do AHRS com o mesmo ruido
    e o laco de atraso: a simulacao que bateu com o SITL a +-9 % na Etapa 0.
    A analise com ruido branco ordena os pontos; e esta que confirma o escolhido,
    porque o modelo continuo do filtro nao e o biquad que roda no voo.
    """
    sig = math.sqrt(intensidade_ruido() * mod.fs_fisica)
    t = np.arange(0.0, SEGUNDOS_RUIDO, 1.0 / mod.fs_laco)
    passo = max(int(round(mod.fs_fisica / mod.fs_laco)), 1)
    us = []
    for s in SEMENTES_RUIDO:
        ctl = _controlador_da_familia(mod, eixo, familia, par)
        r = malha.simular(ctl, mod.a, mod.b[eixo], t, fs_fisica=mod.fs_fisica,
                          fs_laco=mod.fs_laco, fc_giro=mod.fc_giro, ruido_giro=sig, semente=s,
                          **opcoes_malha(planta_real(eixo)))
        us.append(float(np.std(r["u"][::passo][int(5 * mod.fs_laco):])))
    return float(np.mean(us))


def melhor_confirmado(mod, eixo, familia, candidatos, u_nativo, tentativas=12):
    """O primeiro candidato cujo ruido no tempo nao passa do nativo (ou None)."""
    for p in candidatos[:tentativas]:
        u = ruido_no_tempo(mod, eixo, familia, p["par"])
        p["u_tempo"] = u
        if u <= u_nativo:
            return p
    return None


def referencias(mod, proj, eixo, nat):
    """(referencia do ganho na metrica primaria, teto da recuperacao) de um eixo.

    Na rolagem e na arfagem a referencia e o LQI projetado (o que voou), na
    mesma planta real; na guinada, o nativo.
    """
    cr = CRITERIO[eixo]
    ens_rec, ref_rec, fator = cr["teto_recuperacao"]
    if ref_rec != "lqi":
        return nat["primaria"], fator
    g = proj[eixo]["lqi"]["ganhos"]
    ctl = C.LQI(g["k_i"], g["k_th"], g["k_w"], mod.a, mod.b[eixo], mod.ts)
    an = analisar(ctl, mod.a, mod.b[eixo], mod.ts, mod.fc_giro, planta=planta_real(eixo))
    met = metricas_lineares(an, eixo, mod.ts)
    return met[cr["primaria"][0]][cr["primaria"][1]], met[ens_rec]["tempo_recuperacao_s"] * fator


def escolher(mod, frs, eixo, familia, ref_teto):
    """O melhor ponto da familia que cumpre tudo, com o ruido confirmado no tempo."""
    nat = frs["nativo"]
    if "u_tempo" not in nat:
        nat["u_tempo"] = ruido_no_tempo(mod, eixo, "nativo", {})
    return melhor_confirmado(mod, eixo, familia,
                             candidatos_sob(frs, eixo, familia, nat["ruido"], ref_teto[1]),
                             nat["u_tempo"])


def portao(mod, pontos, proj, eixos=("roll", "yaw")):
    """P1-P3 do plano, com os numeros: devolve {eixo: veredito}.

    P1 (ganho real): o melhor LQG com ruido <= nativo (na analise e no tempo),
       MF e MF robusta >=, nada pior que o nativo (2 %) e teto de recuperacao,
       e >= 10 % melhor na primaria que a referencia (LQI na rolagem, nativo na
       guinada).
    P2 (atribuicao): esse LQG >= 5 % melhor na primaria que o melhor ponto de
       cada familia de estrutura fixa sob as mesmas restricoes; e, relaxado, que
       o melhor ponto de cada uma so com MF, MF robusta e ruido <= nativo.
    P3 (margens): algum ponto do LQG cumpre P1 (se nao, o LQG falha no eixo).
    """
    veredito = {}
    for eixo in eixos:
        frs = fronteiras(pontos, eixo)
        nat = frs["nativo"]
        ref, teto = referencias(mod, proj, eixo, nat)
        melhores = {f: escolher(mod, frs, eixo, f, (ref, teto)) for f in FAMILIAS}
        lqg = melhores["lqg"]
        relaxados = {}
        for f in FAMILIAS:
            ok = [p for p in frs[f]["viaveis"] if p["ruido"] <= nat["ruido"]]
            relaxados[f] = min(ok, key=lambda p: p["primaria"]) if ok else None
        p1 = lqg is not None and lqg["primaria"] <= ref * (1 - GANHO_MIN)
        fixas = [melhores[f] for f in ("pid", "lqi", "eso") if melhores[f] is not None]
        melhor_fixa = min(fixas, key=lambda p: p["primaria"]) if fixas else None
        p2 = (lqg is not None and (melhor_fixa is None or
                                   lqg["primaria"] <= melhor_fixa["primaria"] * (1 - GANHO_ATRIBUICAO)))
        fixas_r = [relaxados[f] for f in ("pid", "lqi", "eso") if relaxados[f] is not None]
        melhor_fixa_r = min(fixas_r, key=lambda p: p["primaria"]) if fixas_r else None
        p2r = (lqg is not None and (melhor_fixa_r is None or lqg["primaria"] <=
                                    melhor_fixa_r["primaria"] * (1 - GANHO_ATRIBUICAO)))
        # torque estocastico: o LQG escolhido, o nativo e o oraculo (o mesmo LQR,
        # com o Kalman projetado para o espectro do torque injetado)
        oraculo = None
        if lqg is not None:
            te = TORQUE_ESTOCASTICO
            sig, tau = te["sigma"][eixo], te["tau"]
            g = ganhos_lqg(mod, eixo, lqg["par"], tau_d=tau, q_d=2.0 * sig * sig / tau)
            ctl = C.LQG(g["k_th"], g["k_w"], g["L"], mod.a, mod.b[eixo], mod.ts, mod.fc_giro,
                        TAU_ESTIMADOR[eixo], fc_alvo=mod.fc_giro, tau_d=tau)
            an = analisar(ctl, mod.a, mod.b[eixo], mod.ts, mod.fc_giro, planta=planta_real(eixo))
            if an.estavel():
                rms_th, rms_u = h2_torque(an, sig, tau, mod.ts)
                oraculo = {"rms_th": rms_th, "rms_u": rms_u, "ruido": an.ruido_branco,
                           "mf": an.margens["mf_graus"]}
        veredito[eixo] = {"frs": frs, "melhores": melhores, "relaxados": relaxados, "ref": ref,
                          "teto_rec": teto, "P1": p1, "P2": p2, "P2r": p2r,
                          "P3": lqg is not None, "melhor_fixa": melhor_fixa,
                          "melhor_fixa_r": melhor_fixa_r, "oraculo": oraculo}
    return veredito


def ganhos_lqg(mod, eixo, par, tau_d=None, q_d=None):
    """Os ganhos do LQG de um ponto da varredura ({q_d, r, pw})."""
    th_m, w_m = PESOS[eixo]
    w_m *= par.get("pw", 1.0)
    k = ganhos_lqr(mod.a, mod.b[eixo], mod.ts, [1.0 / th_m ** 2, 1.0 / w_m ** 2], par["r"])
    q = q_d if q_d is not None else par["q_d"]
    q_n = intensidade_ruido()
    q_a = ANGULO_INDEP[eixo] * q_n
    L = ganhos_kalman(mod.a, mod.b[eixo], mod.ts, mod.fc_giro, TAU_ESTIMADOR[eixo],
                      q_n, q, tau_d=tau_d, q_a=q_a)
    g = {"k_th": float(k[0]), "k_w": float(k[1]), "L": L, "q_d": float(q),
         "r": float(par["r"]), "pw": float(par.get("pw", 1.0)), "tau_c": TAU_ESTIMADOR[eixo],
         "L_r": None, "tau_r": TAU_RAPIDO[eixo], "w_ref": 0.0, "tau_alfa": MANOBRA_TAU}
    if TAU_RAPIDO[eixo] != TAU_ESTIMADOR[eixo]:
        g["L_r"] = ganhos_kalman(mod.a, mod.b[eixo], mod.ts, mod.fc_giro, TAU_RAPIDO[eixo],
                                 q_n, q, tau_d=tau_d, q_a=q_a)
        g["w_ref"] = MANOBRA_W_REF
    return g


def lqg_de_ganhos(mod, eixo, g, b_ctl=None):
    """O espelho LQG (com o estimador rapido, se houver) de um dict de ganhos_lqg."""
    return C.LQG(g["k_th"], g["k_w"], g["L"], mod.a, b_ctl or mod.b[eixo], mod.ts, mod.fc_giro,
                 g["tau_c"], fc_alvo=mod.fc_giro, L_r=g.get("L_r"), tau_r=g.get("tau_r"),
                 w_ref=g.get("w_ref"), tau_alfa=g.get("tau_alfa", MANOBRA_TAU))


def _linha(p, nat):
    mt = p["metricas"]
    rec = mt["torque_pulso"]["tempo_recuperacao_s"]
    rec_d = mt["torque_degrau"]["tempo_recuperacao_s"]
    tempo = f" (tempo {p['u_tempo'] / nat['u_tempo']:.2f}x)" if "u_tempo" in p else ""
    return (f"primaria {p['primaria']:.3f} ({100 * (p['primaria'] / nat['primaria'] - 1):+.1f} % x "
            f"nativo)  ruido {p['ruido']:.4f}{tempo}  MF {p['mf']:.1f}  pulso "
            f"{mt['torque_pulso']['desvio_max_deg']:.3f}/{mt['torque_pulso']['rmse_deg']:.3f}/"
            f"{rec if rec is not None else float('nan'):.2f}  degrau "
            f"{mt['torque_degrau']['desvio_max_deg']:.3f}/"
            f"{rec_d if rec_d is not None else float('nan'):.2f}  {p['par']}")


def imprimir_portao(veredito):
    nome = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}
    for eixo, v in veredito.items():
        nat = v["frs"]["nativo"]
        ens, chave = CRITERIO[eixo]["primaria"]
        print(f"\n== portao: {nome[eixo]} (primaria: {ens} {chave}) ==")
        print(f"  nativo (planta real): ruido {nat['ruido']:.4f} (no tempo, u "
              f"{nat['u_tempo']:.5f})  MF {nat['mf']:.1f}  MF robusta "
              f"{['—' if x is None else f'{x:.1f}' for x in nat['mf_rob']]}  primaria "
              f"{nat['primaria']:.3f}  | {_linha(nat, nat)}")
        print(f"  referencia do ganho: {v['ref']:.3f}  (P1 exige <= "
              f"{v['ref'] * (1 - GANHO_MIN):.3f}); teto de recuperacao {v['teto_rec']:.2f} s")
        print("  -- sob todas as restricoes (ruido confirmado no tempo):")
        for f in FAMILIAS:
            p = v["melhores"][f]
            n_vi = len(v["frs"][f]["viaveis"])
            txt = _linha(p, nat) if p is not None else "nenhum ponto cumpre"
            print(f"  {ROTULO_FAMILIA[f]:30s} ({n_vi:4d} viaveis): {txt}")
        print("  -- relaxado (so MF, MF robusta e ruido <= nativo, na analise):")
        for f in FAMILIAS:
            p = v["relaxados"][f]
            print(f"  {ROTULO_FAMILIA[f]:30s}: {_linha(p, nat) if p is not None else '—'}")
        lqg = v["melhores"]["lqg"]
        if lqg is not None:
            e_n, e_l = nat["estocastico"], lqg["estocastico"]
            txt = (f"  torque estocastico (sigma {TORQUE_ESTOCASTICO['sigma'][eixo]:g}, tau "
                   f"{TORQUE_ESTOCASTICO['tau']:g} s): RMS theta nativo {e_n['rms_th']:.4f}, LQG "
                   f"{e_l['rms_th']:.4f} ({100 * (e_l['rms_th'] / e_n['rms_th'] - 1):+.1f} %)")
            o = v["oraculo"]
            if o:
                txt += (f", oraculo {o['rms_th']:.4f} ({100 * (o['rms_th'] / e_n['rms_th'] - 1):+.1f}"
                        f" %; ruido {o['ruido']:.4f}, MF {o['mf']:.1f})")
            print(txt)
        print(f"  P1 (ganho real) {'PASSA' if v['P1'] else 'falha'}   "
              f"P2 (atribuicao) {'PASSA' if v['P2'] else 'falha'} (relaxado "
              f"{'PASSA' if v['P2r'] else 'falha'})   P3 (margens) {'PASSA' if v['P3'] else 'falha'}")


def projetar_lqg(mod, proj, pontos, relaxar=False):
    """Os dois LQG de cada eixo, a partir das varreduras (pontos de varrer).

    lqg        mesma regra do LQI e do ESO (mesma frequencia de cruzamento do
               nativo, MF >= a dele, zeta >= 0,7, modo lento mais rapido),
               varrendo q_d, na planta real
    lqg_ruido  o escolhido pelo portao: ruido <= nativo (analise e tempo), MF e
               MF robusta >=, nada pior que o nativo, a melhor metrica primaria

    Um eixo em que nenhum ponto cumpre tudo fica com o melhor so com ruido, MF e MF
    robusta nao piores que as do nativo (o P2 relaxado), em vez de ficar sem LQG,
    marcado com "relaxado". Antes so as variantes de veiculo (``relaxar``) faziam
    isso; desde o reprojeto com ANGULO_INDEP (5/11/2026), a guinada de 20 Hz cai
    nesse caso, e o autor escolheu voar o melhor relaxado.
    """
    res = {}
    for eixo in EIXOS:
        frs = fronteiras(pontos, eixo)
        ref_teto = referencias(mod, proj, eixo, frs["nativo"])
        p = escolher(mod, frs, eixo, "lqg", ref_teto)
        relaxado = False
        if p is None:
            ok = [q for q in frs["lqg"]["viaveis"] if q["ruido"] <= frs["nativo"]["ruido"]]
            p = min(ok, key=lambda q: q["primaria"]) if ok else None
            relaxado = p is not None
            if relaxado:
                print(f"  {NOMES[eixo]}: LQG pelo criterio relaxado (ruido, MF e MF robusta)")
        an_n = analisar(novo_controlador(mod, eixo, "nativo"), mod.a, mod.b[eixo], mod.ts,
                        mod.fc_giro, planta=planta_real(eixo))
        esc, varr = sintonizar(mod, eixo, "lqg", an_n, planta=planta_real(eixo))
        par_b = {"q_d": esc["ganhos"]["q_d"], "r": esc["r"], "pw": 1.0}
        res[eixo] = {"lqg": {**esc, "varredura": varr, "ganhos": ganhos_lqg(mod, eixo, par_b)},
                     "lqg_ruido": None if p is None else
                     {"ponto": p, "ganhos": ganhos_lqg(mod, eixo, p["par"]), "relaxado": relaxado}}
    return res


# Ablacao do LQG (revisao de 3/11, item 2.9): o LQG de mesmo ruido sem uma parte do
# modelo do caminho de medida, cada variante reprojetada pela mesma regra (o portao).
# abl e o CC4_ABL do backend (bit 1 sem filtro, bit 2 sem atraso do angulo); sem_eta
# tira o ruido de eta; q_b ~ 0 tira o vies do giroscopio. So em rolagem.
# sem_bg: q_b = 1e-8 do nominal, e nao 0 — com 0, b_g vira um integrador sem
# excitacao e a Riccati discreta nao tem solucao estabilizante; com 1e-8, o ganho
# sobre b_g e desprezivel, e o estado sai na pratica.
ABLACOES = {"sem_filtro": {"abl": 1}, "sem_z": {"abl": 2}, "sem_eta": {"sem_eta": True},
            "sem_bg": {"q_b": Q_VIES * 1e-8}}


def kalman_ablacao(mod, eixo, q_d, opc):
    """O ganho de Kalman de uma variante de ablacao (ABLACOES)."""
    q_n = intensidade_ruido()
    return ganhos_kalman(mod.a, mod.b[eixo], mod.ts, mod.fc_giro, TAU_ESTIMADOR[eixo], q_n, q_d,
                         q_b=opc.get("q_b", Q_VIES), q_a=ANGULO_INDEP[eixo] * q_n,
                         abl=opc.get("abl", 0), sem_eta=opc.get("sem_eta", False))


def projetar_ablacao(mod, proj, eixo="roll", cache=None):
    """Cada variante de ablacao escolhida pela regra do LQG de mesmo ruido (o portao).

    Varre o LQG de cada variante (familias lqg_abl:<nome>) na planta real e escolhe
    como projetar_lqg: o melhor que cumpre tudo, ou o melhor relaxado (ruido, MF e MF
    robusta nao piores que as do nativo), marcado.
    """
    import pickle
    familias = tuple(f"lqg_abl:{n}" for n in ABLACOES)
    if cache is not None and cache.exists():
        pontos = pickle.loads(cache.read_bytes())
    else:
        pontos = varrer(mod, (eixo,), familias)
        if cache is not None:
            cache.write_bytes(pickle.dumps(pontos))
    _CTX["mod"] = mod
    frs = fronteiras(pontos, eixo, familias)
    ref_teto = referencias(mod, proj, eixo, frs["nativo"])
    res = {}
    for nome in ABLACOES:
        f = f"lqg_abl:{nome}"
        p = escolher(mod, frs, eixo, f, ref_teto)
        relaxado = False
        if p is None:
            ok = [q for q in frs[f]["viaveis"] if q["ruido"] <= frs["nativo"]["ruido"]]
            p = min(ok, key=lambda q: q["primaria"]) if ok else None
            relaxado = p is not None
        if p is None:
            res[nome] = None
            print(f"  ablacao {nome}: nenhum ponto viavel")
            continue
        opc = ABLACOES[nome]
        th_m, w_m = PESOS[eixo]
        w_m *= p["par"].get("pw", 1.0)
        k = ganhos_lqr(mod.a, mod.b[eixo], mod.ts, [1.0 / th_m ** 2, 1.0 / w_m ** 2], p["par"]["r"])
        g = {"k_th": float(k[0]), "k_w": float(k[1]),
             "L": kalman_ablacao(mod, eixo, p["par"]["q_d"], opc), "q_d": p["par"]["q_d"],
             "r": p["par"]["r"], "pw": p["par"].get("pw", 1.0), "tau_c": TAU_ESTIMADOR[eixo],
             "L_r": None, "tau_r": TAU_RAPIDO[eixo], "w_ref": 0.0, "tau_alfa": MANOBRA_TAU}
        res[nome] = {"ponto": p, "ganhos": g, "relaxado": relaxado, "abl": opc.get("abl", 0)}
        print(f"  ablacao {nome:10s}: primaria {p['primaria']:.3f} "
              f"({100 * (p['primaria'] / frs['nativo']['primaria'] - 1):+.1f} % x nativo)  "
              f"ruido {p['ruido']:.4f}  MF {p['mf']:.1f}{'  [relaxado]' if relaxado else ''}  {p['par']}")
    return res


def escrever_yamls_ablacao(mod, lqg, abl, eixo="roll", destino=CONTROLADORES):
    """cc_lqg_sem_<nome>.yaml: o LQG de mesmo ruido com o eixo da ablacao trocado."""
    s = SUFIXO[eixo]
    mascara = {"roll": 1, "pitch": 2, "yaw": 4}[eixo]
    base_cc = {"CC_TYPE": 4, "CC_AXIS_MASK": mascara, "RC6_OPTION": 109}
    com = ("\n# Ablacao: so o eixo da ablacao roda o backend (CC_AXIS_MASK); os outros ficam\n"
           "# com o nativo. Os CC4_* sao aplicados depois do reboot.\n")
    feitos = []
    for nome, d in abl.items():
        if d is None:
            continue
        lqg_v = {e: dict(lqg[e]) for e in EIXOS}
        lqg_v[eixo] = {**lqg[eixo], "lqg_ruido": {"ganhos": d["ganhos"]}}
        par = {**params_cc4(mod, lqg_v, "lqg_ruido"), **acc_max(), "CC4_ABL": d["abl"]}
        slug = f"cc_lqg_{nome}"
        texto = {"nome": f"AC_CustomControl — LQG de mesmo ruido, ablacao {nome.replace('_', ' ')}",
                 "descricao": (f"O LQG de cc_lqg_mesmo_ruido.yaml sem uma parte do modelo do caminho "
                               f"de medida ({nome.replace('_', ' ')}), reprojetado pela mesma regra "
                               f"(o portao) so em {NOMES[eixo]}"
                               + (", pelo criterio relaxado" if d["relaxado"] else "")
                               + ". Revisao de 3/11, item 2.9 (ablacao)."),
                 "interesse": [f"CC4_KTH_{s}", f"CC4_KW_{s}", "CC4_ABL"]}
        cen = [f"{GRUPO[eixo]}/torque_impulso", f"{GRUPO[eixo]}/torque_degrau"]
        t = yaml_controlador(slug, mod, par, base_cc, 4, com, cenarios=cen, texto=texto)
        (destino / f"{slug}.yaml").write_text(t, encoding="utf-8")
        feitos.append(destino / f"{slug}.yaml")
    return feitos


def projetar_fixas(mod, proj, pontos):
    """O LQI e o LQR+ESO das variantes de veiculo, pelo mesmo criterio do LQG delas.

    O que cumpre tudo (escolher, como o lqg_ruido); se nada cumpre, o melhor so
    com ruido, MF e MF robusta nao piores que as do nativo (o P2 relaxado), como
    o projetar_lqg com relaxar. Com o giroscopio a 10 Hz a regra da mesma banda
    do nativo nao tem solucao em rolagem e arfagem (sintonizar), e esta e a que
    voa. Devolve {eixo: {"lqi"|"eso": {"ponto", "ganhos", "relaxado"} ou None}}.
    """
    res = {}
    for eixo in EIXOS:
        frs = fronteiras(pontos, eixo)
        ref_teto = referencias(mod, proj, eixo, frs["nativo"])
        res[eixo] = {}
        for f in ("lqi", "eso"):
            p = escolher(mod, frs, eixo, f, ref_teto)
            relaxado = p is None
            if relaxado:
                ok = [q for q in frs[f]["viaveis"] if q["ruido"] <= frs["nativo"]["ruido"]]
                p = min(ok, key=lambda q: q["primaria"]) if ok else None
            res[eixo][f] = None if p is None else {
                "ponto": p, "relaxado": relaxado, "ganhos": ganhos_fixa(mod, eixo, f, p["par"])}
            if p is None:
                print(f"  {NOMES[eixo]}: {ROTULO_FAMILIA[f]} sem ponto com ruido, MF e MF "
                      f"robusta nao piores que as do nativo")
            else:
                print(f"  {NOMES[eixo]}: {ROTULO_FAMILIA[f]} "
                      f"{'pelo criterio relaxado' if relaxado else 'cumpre tudo'}: primaria "
                      f"{p['primaria']:.3f} ({100 * (p['primaria'] / frs['nativo']['primaria'] - 1):+.1f} "
                      f"% x nativo), ruido {p['ruido']:.4f}, MF {p['mf']:.1f}  {p['par']}")
    return res


# --------------------------------------------------------------------------
# projeto completo de um modelo
# --------------------------------------------------------------------------

VARIANTES = ("nativo", "lqi", "eso")
ROTULO = {"nativo": "Nativo", "lqi": "LQI", "eso": "LQR+ESO",
          "lqi_ruido": "LQI (mesmo ruído)"}


def variantes(proj, eixo):
    """As variantes projetadas num eixo: as tres e, na guinada, o LQI de mesmo ruido."""
    return VARIANTES + tuple(v for v in ("lqi_ruido",) if v in proj[eixo])


def novo_controlador(mod, eixo, variante, ganhos=None, b_ctl=None):
    """Um espelho novo (estado zerado) do controlador de um eixo."""
    if variante == "nativo":
        return controlador_nativo(mod.p, eixo, mod.ts)[0]
    g = ganhos
    b = b_ctl or mod.b[eixo]
    if variante in ("lqi", "lqi_ruido"):
        return C.LQI(g["k_i"], g["k_th"], g["k_w"], mod.a, b, mod.ts, ff=g.get("ff", True),
                     e_max=g.get("e_max"), fc_alvo=mod.fc_giro)
    if variante == "eso":
        return C.LQRESO(g["k_th"], g["k_w"], g["l1"], g["l2"], mod.a, b, mod.ts,
                        ff=g.get("ff", True), e_max=g.get("e_max"), fc_alvo=mod.fc_giro)
    if variante == "nativo_lqi":
        return C.CascataPID(g, eixo, mod.ts, raiz=True)
    raise ValueError(variante)


def projetar(mod, eixos=EIXOS, relaxar=False):
    """Sintonia das duas variantes em cada eixo, contra o nativo do mesmo voo."""
    proj = {}
    for eixo in eixos:
        nat = novo_controlador(mod, eixo, "nativo")
        an_n = analisar(nat, mod.a, mod.b[eixo], mod.ts, mod.fc_giro)
        lqi, var_lqi = sintonizar(mod, eixo, "lqi", an_n, relaxar=relaxar)
        eso, var_eso = sintonizar(mod, eixo, "eso", an_n, relaxar=relaxar)
        proj[eixo] = {
            "nativo": {"analise": an_n, "ganhos": V.ganhos(mod.p, "ATC", eixo)},
            "lqi": {**lqi, "varredura": var_lqi},
            "eso": {**eso, "varredura": var_eso},
        }
        if eixo == "yaw":
            # a outra regra de justica da guinada: o ganho de ruido do nativo, que
            # filtra o erro de taxa a 1 Hz (FLTE), no lugar da mesma banda
            lqi_r, var_r = sintonizar(mod, eixo, "lqi", an_n, medida="ruido", relaxar=relaxar)
            proj[eixo]["lqi_ruido"] = {**lqi_r, "varredura": var_r}
    return proj


# --------------------------------------------------------------------------
# perfis dos cenarios e simulacoes no tempo
# --------------------------------------------------------------------------

COLUNA_TQ = {"roll": "WND_TQX", "pitch": "WND_TQY", "yaw": "WND_TQZ"}
GRUPO = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}


def carregar_cenario(cen_id):
    return yaml.safe_load((CENARIOS / f"{cen_id}.yaml").read_text(encoding="utf-8"))


def forma_torque(t, vento):
    """forma_torque do vento.lua: pulso (bit 8), meio cosseno (16), degrau (32) e pulso
    de duracao exata (512, SIM_TWIST_TIME: o SITL aplica o torque por exatamente WND_TQDUR
    e os outros bits de torque sao ignorados)."""
    modo = int(vento["WND_MODE"])
    t0 = float(vento["WND_TQT0"])
    dur = max(float(vento.get("WND_TQDUR", 0.4)), 0.01)
    if modo & 512:
        return ((t >= t0) & (t < t0 + dur)).astype(float)
    dentro = (t >= t0) & (t <= t0 + dur)
    f = np.zeros_like(t)
    if modo & 8:
        f += dentro
    if modo & 16:
        f += np.where(dentro, 0.5 * (1.0 - np.cos(2.0 * np.pi * (t - t0) / dur)), 0.0)
    if modo & 32:
        f += t >= t0
    return f


def perfil_torque(cen_id, eixo):
    """(escada de torque a 50 Hz, janela das metricas, amplitude) de um cenario.

    A janela e a de metricas.janela_disturbio: do inicio do torque ate 8 duracoes
    (no minimo 4 s) depois, ou ate o fim do voo no degrau sustentado.
    """
    cen = carregar_cenario(cen_id)
    v = cen["vento"]
    amp = float(v[COLUNA_TQ[eixo]])
    duracao = float(cen["voo"]["duracao_s"])
    t0 = float(v["WND_TQT0"])
    modo = int(v["WND_MODE"])
    if modo & 128:
        # torque estocastico: a mesma sequencia do injetor (semente e passo fixo)
        from analise import dryden_ref
        t_esc, _wn, _we, *tq = dryden_ref.gerar(dryden_ref.Perfil.de_dict(v), duracao)
        valor = tq[("roll", "pitch", "yaw").index(eixo)]
        return (t_esc, valor), (t0, t0 + float(v["WND_TQDUR"])), amp
    t_esc = np.arange(0.0, duracao, 0.02)       # o injetor escreve a 50 Hz
    fim = duracao if modo & 32 else t0 + max(float(v["WND_TQDUR"]) * 8, 4.0)
    return (t_esc, amp * forma_torque(t_esc, v)), (t0, fim), amp


def opcoes_malha(planta):
    """As opcoes de malha.simular equivalentes a um dict de planta (PLANTA_REAL)."""
    planta = planta or {}
    return {"ruido_angulo": planta.get("ruido_angulo", False),
            "tau_ahrs": planta.get("tau_c"), "atraso_angulo": planta.get("atraso_angulo", 0),
            "angulo_indep": planta.get("angulo_indep", 0.0)}


def simular_torque(mod, eixo, ctl, cen_id, b_planta=None, atraso_lacos=0, ruido_giro=0.0,
                   planta=None):
    """Resposta a perturbacao de torque de um cenario, com as metricas da analise."""
    escada, jan, amp = perfil_torque(cen_id, eixo)
    t = np.arange(jan[0] - 1.0, jan[1], 1.0 / 400.0)
    r = malha.simular(ctl, mod.a, b_planta or mod.b[eixo], t, escada=escada,
                      fs_fisica=mod.fs_fisica, fs_laco=mod.fs_laco, fc_giro=mod.fc_giro,
                      atraso_lacos=atraso_lacos, ruido_giro=ruido_giro, u_max=1.0,
                      **opcoes_malha(planta))
    erro = -r["angulo_deg"]
    met = M.metricas_disturbio(t, erro, jan)
    met["u_max_abs"] = float(np.max(np.abs(r["u"])))
    return {"t": t - jan[0], "t0": jan[0], "erro": erro, "sim": r, "metricas": met, "amp": amp}


COLUNA_ALVO = {"roll": "roll_des", "pitch": "pitch_des", "yaw": "yaw_des_rel"}


def alvo_nominal(eixo):
    """Alvo de atitude registrado no voo nominal mais recente do nativo nesse eixo.

    Devolve (t, alvo em graus, t do degrau, ate) ou None. A velocidade e a
    aceleracao alvo saem por derivada numerica: o alvo e a saida do modelador de
    referencia do AC_AttitudeControl, suave por construcao.
    """
    pasta = PROJETO / "ensaios" / "resultados" / GRUPO[eixo] / "nominal" / "nativo"
    voos = sorted(p for p in pasta.glob("*") if (p / "series.csv").exists()) if pasta.exists() else []
    if not voos:
        return None
    col = V.ler_series(voos[-1] / "series.csv")
    t, ref = col["t"], col[COLUNA_ALVO[eixo]]
    deg = M.detectar_degrau(t, ref)
    if deg is None:
        return None
    t_deg, y0, y1, t_fim = deg
    ate = min(t_deg + 6.0, t_fim - M.GUARDA_ANTES_DISTURBIO_S) if t_fim else t_deg + 6.0
    sel = (t >= t_deg - 1.0) & (t <= ate) & np.isfinite(ref)
    t, ref = t[sel], ref[sel] - y0
    # origem do tempo no inicio da subida do alvo (2 % da amplitude): o instante
    # do comando e o do alvo modelado nao coincidem
    t_ini = float(t[np.nonzero(np.abs(ref) > 0.02 * abs(y1 - y0))[0][0]])
    return t, ref, t_ini, ate


def simular_degrau(mod, eixo, ctl, alvo, planta=None):
    t, ref, t_deg, ate = alvo
    th = np.radians(ref)
    w = np.gradient(th, t)
    acel = np.gradient(w, t)
    r = malha.simular(ctl, mod.a, mod.b[eixo], t, ref=(t, th, w, acel),
                      fs_fisica=mod.fs_fisica, fs_laco=mod.fs_laco, fc_giro=mod.fc_giro,
                      u_max=1.0, **opcoes_malha(planta))
    y = r["angulo_deg"]
    met = M.metricas_degrau(t, ref, y, ate=ate)
    return {"t": t - t_deg, "ref": ref, "y": y, "sim": r, "metricas": met}


def simulacoes(mod, proj, eixos=("roll", "yaw")):
    """Pulso e degrau de torque e degrau de atitude, para os tres controladores."""
    sims = {}
    for eixo in eixos:
        alvo = alvo_nominal(eixo)
        for var in variantes(proj, eixo):
            g = proj[eixo][var]["ganhos"] if var != "nativo" else None
            s = {}
            for ensaio in ("torque_pulso", "torque_degrau"):
                s[ensaio] = simular_torque(mod, eixo, novo_controlador(mod, eixo, var, g),
                                           f"{GRUPO[eixo]}/{ensaio}")
            if alvo is not None:
                s["nominal"] = simular_degrau(mod, eixo, novo_controlador(mod, eixo, var, g), alvo)
            sims[(eixo, var)] = s
    return sims


# --------------------------------------------------------------------------
# robustez e equivalencia
# --------------------------------------------------------------------------

CASOS_ROBUSTEZ = (("b -20 %", 0.8, 0), ("b +20 %", 1.2, 0), ("+1 laço", 1.0, 1))


def robustez(mod, proj, eixos=EIXOS):
    """Margens com a planta fora do nominal: b x 0,8 e x 1,2 e um laco a mais de atraso.

    O controlador continua o projetado (com o b nominal); muda so a planta.
    """
    res = {}
    for eixo in eixos:
        for var in variantes(proj, eixo):
            g = proj[eixo][var]["ganhos"] if var != "nativo" else None
            res[(eixo, var)] = {}
            for nome, fator, atraso in CASOS_ROBUSTEZ:
                an = analisar(novo_controlador(mod, eixo, var, g), mod.a, mod.b[eixo] * fator,
                              mod.ts, mod.fc_giro, atraso_lacos=atraso)
                res[(eixo, var)][nome] = an
    return res


def equivalencia(mod, proj, eixos=EIXOS):
    """O LQI escrito como cascata do nativo, com filtros e D desligados.

    Devolve, por eixo, os ganhos da cascata e a maior distancia entre os polos
    de malha fechada dela e os do LQI (no plano s).
    """
    res = {}
    for eixo in eixos:
        g_lqi = proj[eixo]["lqi"]["ganhos"]
        g_nat = proj[eixo]["nativo"]["ganhos"]
        cas = cascata_do_lqi(g_lqi["k_i"], g_lqi["k_th"], g_lqi["k_w"], g_nat["ang_p"])
        if cas is None:
            res[eixo] = None
            continue
        g = {**g_nat, **cas, "kd": 0.0, "fltt": 0.0, "flte": 0.0}
        an_c = analisar(C.CascataPID(g, eixo, mod.ts, raiz=True), mod.a, mod.b[eixo],
                        mod.ts, mod.fc_giro)
        an_l = proj[eixo]["lqi"]["analise"]
        pc = np.sort_complex(polos_continuos(an_c.polos_z, mod.ts))
        pl = np.sort_complex(polos_continuos(an_l.polos_z, mod.ts))
        # o espelho do nativo tem estados de filtro a mais (alvo, erro, derivada),
        # que com os filtros desligados ficam em z = 0 e nao entram em pc
        dist = max(min(abs(x - y) for y in pc) for x in pl)
        res[eixo] = {"ganhos": g, "analise": an_c, "dist_polos": dist}
    return res


# --------------------------------------------------------------------------
# saidas: ganhos, YAMLs e relatorio
# --------------------------------------------------------------------------

def _num(v, casas=6):
    return float(f"{float(v):.{casas}g}")


def params_cc3(mod, proj, variante):
    """Parametros CC3_* do backend AC_CustomControl_LQR para uma variante.

    "lqi_ruido" e o LQI com a guinada de mesmo ganho de ruido do nativo; nos
    outros eixos, o LQI de mesma banda.
    """
    lqi = variante in ("lqi", "lqi_ruido")
    par = {"CC3_MODO": 0 if lqi else 1, "CC3_FF": 1, "CC3_A": _num(mod.a),
           "CC3_FLTW": _num(mod.fc_giro)}
    nomes = (("k_i", "KI"), ("k_th", "KTH"), ("k_w", "KW")) if lqi else \
        (("k_th", "KTH"), ("k_w", "KW"), ("l1", "ESO_L1"), ("l2", "ESO_L2"))
    for eixo in EIXOS:
        s = SUFIXO[eixo]
        var = variante if variante in proj[eixo] else "lqi"
        par[f"CC3_B_{s}"] = _num(mod.b[eixo])
        for chave, nome in nomes:
            par[f"CC3_{nome}_{s}"] = _num(proj[eixo][var]["ganhos"][chave])
    return par


def params_cc4(mod, lqg, variante):
    """Parametros CC4_* do backend AC_CustomControl_LQG para "lqg" ou "lqg_ruido".

    Phi, Gam e C o backend calcula a bordo, de A, B, FLTW, TAUC e do periodo do
    laco (controladores.modelo_lqg); daqui vao os ganhos: K (KTH, KW) e as 16
    colunas de L, L<i>T e L<i>W, o ganho do estado i (1 a 8, na ordem de
    controladores.ESTADOS_LQG) sobre a inovacao do angulo (T) e da taxa (W).
    """
    par = {"CC4_FF": 1, "CC4_A": _num(mod.a), "CC4_FLTW": _num(mod.fc_giro)}
    for eixo in EIXOS:
        s = SUFIXO[eixo]
        g = lqg[eixo][variante]["ganhos"]
        par[f"CC4_TAUC_{s}"] = _num(g["tau_c"])
        # o estimador rapido (manobra); WMAN = 0 o desliga
        par[f"CC4_TAUR_{s}"] = _num(g["tau_r"])
        par[f"CC4_WMAN_{s}"] = _num(math.degrees(g["w_ref"])) if g.get("L_r") is not None else 0.0
        par[f"CC4_B_{s}"] = _num(mod.b[eixo])
        par[f"CC4_KTH_{s}"] = _num(g["k_th"])
        par[f"CC4_KW_{s}"] = _num(g["k_w"])
        L = np.asarray(g["L"])
        L_r = np.asarray(g["L_r"]) if g.get("L_r") is not None else L
        for i in range(C.N_LQG):
            par[f"CC4_L{i + 1}T_{s}"] = _num(L[i, 0], 9)
            par[f"CC4_L{i + 1}W_{s}"] = _num(L[i, 1], 9)
        for i in range(C.N_LQG):
            par[f"CC4_LR{i + 1}T_{s}"] = _num(L_r[i, 0], 9)
            par[f"CC4_LR{i + 1}W_{s}"] = _num(L_r[i, 1], 9)
    par["CC4_TMAN"] = _num(MANOBRA_TAU)
    return par


def acc_max(nativo="nativo"):
    """ATC_ACC_*_MAX do nativo: o alvo continua modelado pelo AC_AttitudeControl,
    e o limite tem de ser o mesmo nos dois para a comparacao valer. Numa variante
    de veiculo, o do nativo dela (o AutoTune o refaz)."""
    nat = yaml.safe_load((CONTROLADORES / f"{nativo}.yaml").read_text(encoding="utf-8"))["ganhos"]
    return {f"ATC_ACC_{x}_MAX": nat[f"ATC_ACC_{x}_MAX"] for x in ("R", "P", "Y")}


def params_nativo_lqi(mod, eq):
    """Ganhos ATC_* da cascata equivalente ao LQI, nos eixos em que ela existe."""
    par = {}
    for eixo in EIXOS:
        r = eq.get(eixo)
        if r is None:
            continue
        g = r["ganhos"]
        rat = {"roll": "RLL", "pitch": "PIT", "yaw": "YAW"}[eixo]
        par.update({f"ATC_RAT_{rat}_P": _num(g["kp"]), f"ATC_RAT_{rat}_I": _num(g["ki"]),
                    f"ATC_RAT_{rat}_D": 0.0, f"ATC_RAT_{rat}_FLTT": 0.0,
                    f"ATC_RAT_{rat}_FLTE": 0.0, f"ATC_ANG_{rat}_P": _num(g["ang_p"])})
    return par


CABECALHO_YAML = """\
# GERADO por modelagem/projeto.py --escrever-yaml a partir do modelo de
# {origem} (a = {a:.4g} 1/s, b = {bx:.4g} / {by:.4g} / {bz:.4g} rad/s2,
# BAT.VoltR = {vr} V). Para mudar os ganhos, mude o projeto e gere de novo.
"""

TEXTO_YAML = {
    "cc_lqi": dict(
        nome="AC_CustomControl — LQI",
        descricao=(
            "Realimentacao de estados com acao integral (LQR discreto sobre [x_I, theta, w]) "
            "e alimentacao direta pela inversao do modelo, no backend AC_CustomControl_LQR "
            "com CC3_MODO = 0. Mesma frequencia de cruzamento do nativo; o peso do integrador "
            "e o mais alto que mantem a margem de fase do nativo e amortecimento de 0,7."),
        interesse=["CC3_KI_X", "CC3_KTH_X", "CC3_KW_X", "CC3_KI_Z"]),
    "cc_eso": dict(
        nome="AC_CustomControl — LQR + ESO",
        descricao=(
            "Realimentacao de [theta, w] (LQR discreto), sem integrador, com a perturbacao "
            "estimada por um observador de estado estendido de [w, d] e cancelada (-d_hat/b), "
            "no backend AC_CustomControl_LQR com CC3_MODO = 1. A banda do observador e a que "
            "da a recuperacao mais rapida mantendo a margem de fase do nativo."),
        interesse=["CC3_KTH_X", "CC3_KW_X", "CC3_ESO_L1_X", "CC3_ESO_L2_X"]),
    "cc_lqi_mesmo_ruido": dict(
        nome="AC_CustomControl — LQI, guinada com o ruido do nativo",
        descricao=(
            "O LQI de cc_lqi.yaml em rolagem e arfagem; na guinada, sintonizado para o mesmo "
            "ganho de ruido do giroscopio no esforco que o nativo (que filtra o erro de taxa "
            "de guinada a 1 Hz), no lugar da mesma frequencia de cruzamento. Banda menor e "
            "desvio maior na guinada, em troca de sensibilidade a ruido igual a do nativo."),
        interesse=["CC3_KI_Z", "CC3_KTH_Z", "CC3_KW_Z"]),
    "cc_lqi_sem_ff": dict(
        nome="AC_CustomControl — LQI sem alimentacao direta",
        descricao=(
            "O LQI de cc_lqi.yaml com a alimentacao direta desligada: sem u_ff e sem o filtro "
            "do alvo de taxa (CC3_FF = 0, CC3_FLTW = 0), que so faz sentido junto com ela. "
            "Com o cc_pid_ff, fecha a comparacao 2x2 que separa o efeito da realimentacao do "
            "efeito da alimentacao direta. Nos ensaios de torque o alvo fica parado e ele e "
            "identico ao cc_lqi."),
        interesse=["CC3_FF", "CC3_FLTW", "CC3_KI_X", "CC3_KTH_X"]),
    "cc_pid_ff": dict(
        nome="AC_CustomControl — PID com a alimentacao direta do LQI",
        descricao=(
            "O cc_pid (cascata do nativo, ganhos do AutoTune) com a MESMA alimentacao direta pura "
            "do LQI, u_ff = (a_t + a*w_t)/b, so da trajetoria do alvo (CC2_FF, "
            "firmware/ardupilot.patch). Nao se usou o FF/D_FF do AC_PID porque eles "
            "multiplicam o alvo de taxa inteiro, inclusive a correcao do erro de angulo, e "
            "mudariam tambem a realimentacao. Nos ensaios de torque e identico ao cc_pid."),
        interesse=["CC2_FF", "CC2_FF_A", "CC2_FF_B_X", "CC2_FF_B_Z"]),
    "cc_lqg": dict(
        nome="AC_CustomControl — LQG",
        descricao=(
            "A lei do LQR+ESO (u = u_ff + k_th*e_th + k_w*e_w - d_hat/b) sobre os estados de um "
            "filtro de Kalman em regime, no backend AC_CustomControl_LQG (CC_TYPE 4). O "
            "estimador modela o passa-baixas do giroscopio, o ruido que o angulo do AHRS integra "
            "sem esse filtro e o laco de atraso com que o backend le a atitude. Mesma regra do "
            "LQI e do ESO: mesma frequencia de cruzamento do nativo, MF >= a dele, zeta >= 0,7."),
        interesse=["CC4_KTH_X", "CC4_KW_X", "CC4_L1T_X", "CC4_L5W_X"]),
    "cc_lqg_mesmo_ruido": dict(
        nome="AC_CustomControl — LQG, ganho de ruido do nativo",
        descricao=(
            "O LQG de cc_lqg.yaml projetado para o ganho real sobre o nativo (o portao do "
            "plano): ruido no esforco nao maior que o do nativo, margem de fase e margem com a "
            "planta alterada nao menores, nada pior que o nativo nas metricas de torque, e a "
            "menor metrica primaria (pico do pulso em rolagem e arfagem, pico do degrau de "
            "torque em guinada). Difere do cc_lqg nos tres eixos. Em guinada, com o ruido "
            "independente do angulo do SITL no modelo (ANGULO_INDEP), nenhum ponto cumpre tudo, "
            "e fica o melhor so com ruido, margem de fase e margem robusta nao piores que as "
            "do nativo (criterio relaxado, revisao de 3/11)."),
        interesse=["CC4_KTH_Z", "CC4_KW_Z", "CC4_L5T_Z", "CC4_L5W_Z"]),
    "nativo_lqi": dict(
        nome="PID nativo com os ganhos do LQI",
        descricao=(
            "Prova de equivalencia: a cascata P de angulo + PI de taxa do nativo com os ganhos "
            "que reproduzem o LQI (filtros FLTT/FLTE e D desligados). So ha cascata "
            "equivalente quando k_th^2 >= 4 k_w k_i; nos eixos em que nao ha, ficam os ganhos "
            "do AutoTune."),
        interesse=["ATC_RAT_RLL_P", "ATC_RAT_RLL_I", "ATC_ANG_RLL_P"]),
}


# Os 18 cenarios do lote original. Os controladores de verificacao (a
# equivalencia nativo_lqi e a decomposicao da alimentacao direta) so existem para
# eles, e a chave cenarios: do YAML os mantem fora das familias novas.
CENARIOS_ORIGINAIS = (
    [f"{g}/{e}" for g in ("rolagem", "arfagem", "guinada")
     for e in ("nominal", "torque_pulso", "torque_degrau", "torque_chirp")]
    + [f"rolagem_arfagem_guinada/{e}" for e in ("nominal", "torque_pulso", "torque_degrau")]
    + ["ruido/rnd20/torque_pulso", "ruido/rnd20/torque_degrau", "vento/rajada"])


def yaml_controlador(slug, mod, ganhos, params, espera_cc_tipo, extra_com="", cenarios=None,
                     texto=None):
    t = texto or TEXTO_YAML[slug]
    cab = CABECALHO_YAML.format(origem=mod.origem.parent.relative_to(PROJETO), a=mod.a,
                                bx=mod.b["roll"], by=mod.b["pitch"], bz=mod.b["yaw"],
                                vr=f"{mod.v_repouso:.2f}" if mod.v_repouso else "—")
    doc = {"nome": t["nome"], "slug": slug, "descricao": t["descricao"], "params": params,
           "reboot_apos_params": True}
    corpo = yaml.safe_dump(doc, allow_unicode=True, sort_keys=False, width=88)
    if espera_cc_tipo:
        corpo += ("\n# O backend so assume com a chave auxiliar em HIGH (RC6_OPTION 109);\n"
                  "# o run_ensaio.py forca o canal, e espera_cc_tipo confere no log.\n"
                  + yaml.safe_dump({"rc_override": {6: 2000}}, sort_keys=False))
    corpo += extra_com + yaml.safe_dump({"ganhos": ganhos}, sort_keys=False)
    corpo += yaml.safe_dump({"ganhos_de_interesse": t["interesse"], "msg_pid": "PIDR",
                             "espera_cc_tipo": espera_cc_tipo}, sort_keys=False)
    if cenarios:
        corpo += ("\n# So os cenarios do lote original (run_ensaio.voa_no_cenario): este\n"
                  "# controlador e uma verificacao feita para eles.\n"
                  + yaml.safe_dump({"cenarios": list(cenarios)}, sort_keys=False))
    return cab + corpo


def escrever_yamls_lqg(mod, lqg, destino=CONTROLADORES):
    """cc_lqg.yaml e cc_lqg_mesmo_ruido.yaml (backend AC_CustomControl_LQG, CC_TYPE 4)."""
    base_cc = {"CC_TYPE": 4, "CC_AXIS_MASK": 7, "RC6_OPTION": 109}
    com = ("\n# Os CC4_* so existem com o backend instanciado (CC_TYPE 4 + reboot): o\n"
           "# run_ensaio.py os aplica depois do reboot. O ATC_ACC_*_MAX e o do nativo.\n")
    feitos = []
    for slug, var in (("cc_lqg", "lqg"), ("cc_lqg_mesmo_ruido", "lqg_ruido")):
        if any(lqg[e][var] is None for e in EIXOS):
            continue
        texto = yaml_controlador(slug, mod, {**params_cc4(mod, lqg, var), **acc_max()},
                                 base_cc, 4, com)
        (destino / f"{slug}.yaml").write_text(texto, encoding="utf-8")
        feitos.append(destino / f"{slug}.yaml")
    return feitos


# Os cenarios das variantes de veiculo (o giroscopio de 10 Hz, a Etapa 2d do
# plano): o degrau e os torques de rolagem e guinada, o estocastico e o pulso
# com ruido.
CENARIOS_VARIANTE = ([f"{g}/{e}" for g in ("rolagem", "guinada")
                      for e in ("nominal", "torque_pulso", "torque_degrau")]
                     + ["rolagem_arfagem_guinada/torque_estocastico", "ruido/rnd20/torque_pulso"])


def escrever_yamls_variante(mod, lqg, fixas, sufixo, params_veiculo, destino=CONTROLADORES):
    """cc_lqi_<sufixo>, cc_eso_<sufixo> e cc_lqg_<sufixo>: os controladores projetados
    para um veiculo diferente (outro filtro do giroscopio), com os parametros desse
    veiculo nos params: e a chave cenarios: restrita a CENARIOS_VARIANTE. Nao
    toca nos YAMLs do lote principal. O nativo da variante (nativo_<sufixo>.yaml)
    sai do AutoTune feito com o mesmo veiculo. Os tres seguem o mesmo criterio
    (portao: ruido, MF e MF robusta nao piores que as do nativo, a melhor metrica
    primaria): o LQG de projetar_lqg e o LQI e o ESO de projetar_fixas.
    """
    feitos = []
    nat = f"nativo_{sufixo}" if (destino / f"nativo_{sufixo}.yaml").exists() else "nativo"
    nota = (f" Variante '{sufixo}': projetado com o modelo de {mod.fc_giro:g} Hz no filtro do "
            f"giroscopio, contra o nativo sintonizado pelo AutoTune nesse mesmo veiculo.")
    criterio = (" Na variante, o criterio e o do LQG dela (portao): ruido, MF e MF robusta nao "
                "piores que as do nativo e o menor pico; a regra da mesma banda do nativo nao "
                "tem solucao com o filtro mais lento.")
    for slug, var, tipo in (("cc_lqi", "lqi", 3), ("cc_eso", "eso", 3), ("cc_lqg", "lqg_ruido", 4)):
        base = {"CC_TYPE": tipo, "CC_AXIS_MASK": 7, "RC6_OPTION": 109, **params_veiculo}
        if tipo == 3:
            if any(fixas[e][var] is None for e in EIXOS):
                print(f"{slug}_{sufixo}: nao gravado (sem ponto pelo criterio do portao)")
                continue
            proj_v = {e: {var: fixas[e][var]} for e in EIXOS}
            ganhos = {**params_cc3(mod, proj_v, var), **acc_max(nat)}
        else:
            if any(lqg[e][var] is None for e in EIXOS):
                continue
            ganhos = {**params_cc4(mod, lqg, var), **acc_max(nat)}
        texto = dict(TEXTO_YAML[slug if slug != "cc_lqg" else "cc_lqg_mesmo_ruido"])
        texto["nome"] += f" ({sufixo})"
        texto["descricao"] += nota + (criterio if tipo == 3 else "")
        com = (f"\n# Os CC{tipo}_* so existem com o backend instanciado (CC_TYPE {tipo} + reboot).\n")
        corpo = yaml_controlador(slug, mod, ganhos, base, tipo, com, cenarios=CENARIOS_VARIANTE,
                                 texto=texto).replace(f"slug: {slug}\n", f"slug: {slug}_{sufixo}\n")
        caminho = destino / f"{slug}_{sufixo}.yaml"
        caminho.write_text(corpo, encoding="utf-8")
        feitos.append(caminho)
    return feitos


def escrever_yamls(mod, proj, eq, destino=CONTROLADORES):
    base_cc = {"CC_TYPE": 3, "CC_AXIS_MASK": 7, "RC6_OPTION": 109}
    feitos = []
    for slug, var in (("cc_lqi", "lqi"), ("cc_eso", "eso"), ("cc_lqi_mesmo_ruido", "lqi_ruido")):
        com = ("\n# Os CC3_* so existem com o backend instanciado (CC_TYPE 3 + reboot): o\n"
               "# run_ensaio.py os aplica depois do reboot. O ATC_ACC_*_MAX e o do nativo.\n")
        texto = yaml_controlador(slug, mod, {**params_cc3(mod, proj, var), **acc_max()},
                                 base_cc, 3, com)
        (destino / f"{slug}.yaml").write_text(texto, encoding="utf-8")
        feitos.append(destino / f"{slug}.yaml")
    # a decomposicao da alimentacao direta: LQI sem ela e PID com ela
    sem_ff = {**params_cc3(mod, proj, "lqi"), "CC3_FF": 0, "CC3_FLTW": 0.0}
    texto = yaml_controlador("cc_lqi_sem_ff", mod, {**sem_ff, **acc_max()}, base_cc, 3, com,
                             cenarios=CENARIOS_ORIGINAIS)
    (destino / "cc_lqi_sem_ff.yaml").write_text(texto, encoding="utf-8")
    feitos.append(destino / "cc_lqi_sem_ff.yaml")
    cc_pid = yaml.safe_load((destino / "cc_pid.yaml").read_text(encoding="utf-8"))
    ff = {"CC2_FF": 1, "CC2_FF_A": _num(mod.a),
          **{f"CC2_FF_B_{SUFIXO[e]}": _num(mod.b[e]) for e in EIXOS}}
    com_pid = ("\n# Os ganhos do cc_pid.yaml e a alimentacao direta (CC2_FF*), aplicados depois\n"
               "# do reboot: os CC2_* so existem com o backend instanciado.\n")
    texto = yaml_controlador("cc_pid_ff", mod, {**cc_pid["ganhos"], **ff}, cc_pid["params"], 2,
                             com_pid, cenarios=CENARIOS_ORIGINAIS)
    (destino / "cc_pid_ff.yaml").write_text(texto, encoding="utf-8")
    feitos.append(destino / "cc_pid_ff.yaml")
    nat = yaml.safe_load((destino / "nativo.yaml").read_text(encoding="utf-8"))["ganhos"]
    texto = yaml_controlador("nativo_lqi", mod, {**nat, **params_nativo_lqi(mod, eq)},
                             {"CC_TYPE": 0}, 0, cenarios=CENARIOS_ORIGINAIS)
    (destino / "nativo_lqi.yaml").write_text(texto, encoding="utf-8")
    feitos.append(destino / "nativo_lqi.yaml")
    return feitos


def _f(v, casas=2):
    return "—" if v is None else f"{v:.{casas}f}"


def relatorio(mod, proj, rob, eq, sims):
    print(f"modelo: {mod.origem.parent.relative_to(PROJETO)}  a = {mod.a:.4f} 1/s  "
          f"b = {mod.b['roll']:.2f} / {mod.b['pitch']:.2f} / {mod.b['yaw']:.3f} rad/s2  "
          f"BAT.VoltR = {_f(mod.v_repouso)} V  Ts = {mod.ts * 1e3:.2f} ms\n")
    print(f"{'eixo':8s} {'ctl':9s} {'param':>10s} {'wc':>7s} {'MF':>6s} {'MG':>6s} "
          f"{'lento':>7s} {'zeta':>5s} {'ruido':>7s}   ganhos")
    for eixo in EIXOS:
        for var in variantes(proj, eixo):
            d = proj[eixo][var]
            an = d["analise"]
            m = an.margens
            par = ("—" if var == "nativo" else
                   f"T_I {d['parametro']:.3f}" if var.startswith("lqi")
                   else f"w_o {d['parametro']:.2f}")
            g = d["ganhos"]
            gs = (f"P {g['kp']:.4g} I {g['ki']:.4g} D {g['kd']:.3g} ang {g['ang_p']:.4g}"
                  if var == "nativo" else
                  " ".join(f"{k} {float(v):.4g}" for k, v in g.items() if k != "w_o"))
            print(f"{NOMES[eixo]:8s} {var:9s} {par:>10s} {_f(m['wc'])} {_f(m['mf_graus'], 1):>6s} "
                  f"{_f(m['mg_db'], 1):>6s} {polo_lento(an, mod.ts):7.2f} "
                  f"{zeta_dominante(an, mod.ts):5.2f} {_f(an.ruido, 4):>7s}   {gs}")
    print("\nrobustez (margem de fase; '—' = instavel):")
    for (eixo, var), casos in rob.items():
        txt = "  ".join(f"{nome}: {_f(an.margens['mf_graus'], 1) if an.estavel() else '—'}"
                        for nome, an in casos.items())
        print(f"  {NOMES[eixo]:8s} {var:9s} {txt}")
    print("\nequivalencia LQI -> cascata do nativo (FLTT, FLTE e D desligados):")
    for eixo, r in eq.items():
        if r is None:
            g = proj[eixo]["lqi"]["ganhos"]
            print(f"  {NOMES[eixo]:8s} sem cascata equivalente: k_th^2 = "
                  f"{g['k_th'] ** 2:.3f} < 4 k_w k_i = {4 * g['k_w'] * g['k_i']:.3f}")
        else:
            g = r["ganhos"]
            print(f"  {NOMES[eixo]:8s} P {g['kp']:.4g}  I {g['ki']:.4g}  ang {g['ang_p']:.4g}  "
                  f"(maior distancia entre polos: {r['dist_polos']:.2f} rad/s)")
    print("\nprevisao do modelo (perfis de torque dos cenarios; degrau com o alvo do voo nominal):")
    for (eixo, var), s in sims.items():
        mp, md = s["torque_pulso"]["metricas"], s["torque_degrau"]["metricas"]
        mn = s.get("nominal", {}).get("metricas", {})
        print(f"  {NOMES[eixo]:8s} {var:9s} pulso {_f(mp.get('desvio_max_deg'))} graus / "
              f"{_f(mp.get('tempo_recuperacao_s'))} s | degrau {_f(md.get('desvio_max_deg'))} "
              f"graus / {_f(md.get('tempo_recuperacao_s'))} s | nominal sobressinal "
              f"{_f(mn.get('sobressinal_pct'), 1)} %, erro de rastreamento "
              f"{_f(mn.get('erro_rastreio_max_deg'))} graus")


# --------------------------------------------------------------------------
# tabelas LaTeX
# --------------------------------------------------------------------------

CAB_TEX = "% Gerada por modelagem/projeto.py — nao editar a mao.\n"


def _tex(v, casas=2):
    return "---" if v is None else f"{v:.{casas}f}".replace(".", ",").replace("-", "$-$")


def _tex_g(v, sig=4):
    """Numero com ``sig`` algarismos significativos, em notacao da monografia."""
    if v is None:
        return "---"
    s = f"{float(v):.{sig}g}"
    if "e" in s:
        m, e = s.split("e")
        return f"\\num{{{m}e{int(e)}}}".replace(".", ",")
    return s.replace(".", ",").replace("-", "$-$")


def tabela_ganhos(proj):
    linhas = []
    for eixo in EIXOS:
        for var in variantes(proj, eixo)[1:]:
            d = proj[eixo][var]
            g = d["ganhos"]
            par = (f"$T_I = {_tex(d['parametro'], 3)}$ s" if var.startswith("lqi")
                   else f"$\\omega_o = {_tex(d['parametro'], 2)}$ rad/s")
            linhas.append(f"{NOMES[eixo].capitalize()} & {ROTULO[var]} & {par} & "
                          f"{_tex_g(g.get('k_i'))} & {_tex_g(g['k_th'])} & {_tex_g(g['k_w'])} & "
                          f"{_tex_g(g.get('l1'))} & {_tex_g(g.get('l2'))} \\\\")
    return (CAB_TEX + "\\begin{tabular}{lllrrrrr}\n\\toprule\n"
            "Eixo & Variante & Parâmetro & $k_I$ & $k_\\theta$ & $k_\\omega$ & $l_1$ & $l_2$ \\\\\n"
            "\\midrule\n" + "\n".join(linhas) + "\n\\bottomrule\n\\end{tabular}\n")


def tabela_margens(mod, proj, rob):
    linhas = []
    for eixo in EIXOS:
        for var in variantes(proj, eixo):
            an = proj[eixo][var]["analise"]
            m = an.margens
            casos = rob[(eixo, var)]
            mf = [_tex(c.margens["mf_graus"], 1) if c.estavel() else "inst."
                  for c in casos.values()]
            linhas.append(
                f"{NOMES[eixo].capitalize()} & {ROTULO[var]} & {_tex(m['wc'], 1)} & "
                f"{_tex(m['mf_graus'], 1)} & {_tex(m['mg_db'], 1)} & "
                f"{_tex(m['margem_atraso_s'] * 1e3 if m['margem_atraso_s'] else None, 1)} & "
                f"{_tex(polo_lento(an, mod.ts), 2)} & {_tex(an.ruido, 3)} & "
                + " & ".join(mf) + " \\\\")
        linhas.append("\\addlinespace")
    linhas.pop()
    return (CAB_TEX + "\\begin{tabular}{llrrrrrrrrr}\n\\toprule\n"
            " & & & & & & & & \\multicolumn{3}{c}{MF com a planta alterada (\\si{\\degree})} \\\\\n"
            "\\cmidrule(lr){9-11}\n"
            "Eixo & Controlador & $\\omega_c$ (\\si{\\radian\\per\\second}) & MF (\\si{\\degree}) & "
            "MG (\\si{\\decibel}) & Atraso (\\si{\\milli\\second}) & Polo lento (\\si{\\per\\second}) & "
            "Ruído & $b-20\\,\\%$ & $b+20\\,\\%$ & +1 laço \\\\\n"
            "\\midrule\n" + "\n".join(linhas) + "\n\\bottomrule\n\\end{tabular}\n")


def tabela_previsao(sims):
    linhas = []
    for ensaio, rot in (("torque_pulso", "pulso"), ("torque_degrau", "degrau")):
        for eixo in ("roll", "yaw"):
            for var in VARIANTES + ("lqi_ruido",):
                s = sims.get((eixo, var))
                if s is None:
                    continue
                m = s[ensaio]["metricas"]
                linhas.append(f"{NOMES[eixo].capitalize()} & {rot} & {ROTULO[var]} & "
                              f"{_tex(m.get('desvio_max_deg'))} & "
                              f"{_tex(m.get('tempo_recuperacao_s'))} & "
                              f"{_tex(m.get('rmse_deg'), 3)} & {_tex(m.get('u_max_abs'), 3)} \\\\")
        linhas.append("\\addlinespace")
    for eixo in ("roll", "yaw"):
        for var in VARIANTES + ("lqi_ruido",):
            s = sims.get((eixo, var), {}).get("nominal")
            if s is None:
                continue
            m = s["metricas"]
            linhas.append(f"{NOMES[eixo].capitalize()} & degrau de atitude & {ROTULO[var]} & "
                          f"\\multicolumn{{2}}{{r}}{{sobressinal {_tex(m.get('sobressinal_pct'), 1)}\\,\\%}} & "
                          f"\\multicolumn{{2}}{{r}}{{rastreamento {_tex(m.get('erro_rastreio_max_deg'))}\\,\\si{{\\degree}}}} \\\\")
    return (CAB_TEX + "\\begin{tabular}{lllrrrr}\n\\toprule\n"
            "Eixo & Ensaio & Controlador & Desvio máx. (\\si{\\degree}) & "
            "Recuperação (\\si{\\second}) & RMSE (\\si{\\degree}) & $u_{\\max}$ \\\\\n"
            "\\midrule\n" + "\n".join(linhas) + "\n\\bottomrule\n\\end{tabular}\n")


def tabela_portao(veredito):
    """O nativo, o melhor de cada familia de estrutura fixa (so com ruido, MF e MF
    robusta) e o LQG do portao, na planta real: a tabela do Cap. 5."""
    linhas = []
    for eixo, v in veredito.items():
        nat = v["frs"]["nativo"]
        casos = [("Nativo (AutoTune)", nat)]
        casos += [(f"{ROTULO_FAMILIA[f]}, melhor", v["relaxados"][f]) for f in ("pid", "lqi", "eso")]
        if v["melhores"]["lqg"] is not None or v["relaxados"].get("lqg") is None:
            casos.append(("LQG (portão)", v["melhores"]["lqg"]))
        else:
            # nenhum LQG cumpre todas as restricoes: o que voa e o melhor so com
            # ruido, MF e MF robusta, como as familias de estrutura fixa acima
            casos.append(("LQG, melhor (o que voa)", v["relaxados"]["lqg"]))
        for rot, pt in casos:
            if pt is None:
                linhas.append(f"{NOMES[eixo].capitalize()} & {rot} & \\multicolumn{{8}}{{c}}{{---}} \\\\")
                continue
            mt = pt["metricas"]
            rec = mt["torque_pulso"]["tempo_recuperacao_s"]
            rec_d = mt["torque_degrau"]["tempo_recuperacao_s"]
            linhas.append(
                f"{NOMES[eixo].capitalize()} & {rot} & {_tex(pt['ruido'] / nat['ruido'])} & "
                f"{_tex(pt['mf'], 1)} & {_tex(mt['torque_pulso']['desvio_max_deg'], 3)} & "
                f"{_tex(mt['torque_pulso']['rmse_deg'], 3)} & {_tex(rec)} & "
                f"{_tex(mt['torque_degrau']['desvio_max_deg'], 3)} & {_tex(rec_d)} & "
                f"{_tex(pt['estocastico']['rms_th'], 3)} \\\\")
        linhas.append("\\addlinespace")
    linhas.pop()
    return (CAB_TEX + "\\begin{tabular}{llrrrrrrrr}\n\\toprule\n"
            " & & & & \\multicolumn{3}{c}{Pulso de torque} & \\multicolumn{2}{c}{Degrau de torque} & \\\\\n"
            "\\cmidrule(lr){5-7}\\cmidrule(lr){8-9}\n"
            "Eixo & Controlador & Ruído & MF (\\si{\\degree}) & Desvio (\\si{\\degree}) & "
            "RMSE (\\si{\\degree}) & Recup. (\\si{\\second}) & Desvio (\\si{\\degree}) & "
            "Recup. (\\si{\\second}) & Estocástico (\\si{\\degree}) \\\\\n"
            "\\midrule\n" + "\n".join(linhas) + "\n\\bottomrule\n\\end{tabular}\n")


# --------------------------------------------------------------------------
# figuras
# --------------------------------------------------------------------------

EIXOS_FIG = ("roll", "yaw")


def _estilo():
    import matplotlib
    matplotlib.use("Agg")
    from analise import estilo as E
    E.aplicar()
    matplotlib.rcParams.update({"font.family": "serif", "axes.grid": True})
    return E


def _cor(E, var):
    return E.SERIE[VARIANTES.index("lqi" if var == "lqi_ruido" else var)]


def _hz(ax, marcas=(0.01, 0.1, 1, 10, 100)):
    import matplotlib.ticker
    ax.set_xscale("log")
    lo, hi = ax.get_xlim()
    m = [x for x in marcas if lo <= x <= hi]
    ax.set_xticks(m)
    ax.set_xticklabels([f"{x:g}".replace(".", ",") for x in m])
    ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())


def _salvar(fig, destino):
    fig.tight_layout()
    destino.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(destino)
    import matplotlib.pyplot as plt
    plt.close(fig)
    return destino


def desenhar_polos(mod, proj, destino):
    """Polos de malha fechada: taxa de decaimento x frequencia amortecida."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.9))
    marcas = {"nativo": "o", "lqi": "s", "eso": "^"}
    for ax, eixo in zip(eixos, EIXOS_FIG):
        for var in VARIANTES:
            s = polos_continuos(proj[eixo][var]["analise"].polos_z, mod.ts)
            s = s[s.imag >= -1e-9]
            ax.scatter(-s.real, s.imag, marker=marcas[var], s=26, color=_cor(E, var),
                       edgecolor="white", linewidth=0.5, label=ROTULO[var], zorder=3)
        wc = proj[eixo]["nativo"]["analise"].margens["wc"]
        ax.axvline(wc, color=E.MUDO, lw=0.8, ls=":", zorder=1)
        ax.text(wc * 1.08, ax.get_ylim()[1] * 0.93 if ax.get_ylim()[1] > 0 else 1,
                "$\\omega_c$", color=E.TINTA_2, fontsize=8, va="top")
        ax.set_xscale("log")
        ax.set_xlabel("Taxa de decaimento $-\\mathrm{Re}(s)$ (1/s)")
        ax.set_title(NOMES[eixo].capitalize(), loc="left")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    eixos[0].set_ylabel("Frequência $\\mathrm{Im}(s)$ (rad/s)")
    eixos[0].legend(loc="upper left")
    return _salvar(fig, destino)


def desenhar_malha(mod, proj, destino):
    """Bode da malha aberta na entrada da planta, com filtro e segurador."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(2, 2, figsize=(6.3, 4.4), sharex="col")
    for j, eixo in enumerate(EIXOS_FIG):
        for var in VARIANTES:
            an = proj[eixo][var]["analise"]
            f = an.w / (2 * np.pi)
            fase = np.degrees(np.unwrap(np.angle(an.L)))
            # fase levada a faixa em que cruza -180 graus perto do cruzamento
            i_c = np.argmin(np.abs(np.abs(an.L) - 1.0) + (an.w < 1.0) * 1e9)
            fase -= 360.0 * np.round((fase[i_c] + 180.0 - 90.0) / 360.0)
            eixos[0, j].semilogx(f, 20 * np.log10(np.abs(an.L)), color=_cor(E, var),
                                 label=ROTULO[var])
            eixos[1, j].semilogx(f, fase, color=_cor(E, var))
        eixos[0, j].axhline(0, color=E.MUDO, lw=0.8)
        eixos[1, j].axhline(-180, color=E.MUDO, lw=0.8)
        eixos[0, j].set_ylim(-60, 80)
        eixos[1, j].set_ylim(-360, 0)
        eixos[1, j].set_yticks([-360, -270, -180, -90, 0])
        eixos[0, j].set_title(NOMES[eixo].capitalize(), loc="left")
        eixos[1, j].set_xlabel("Frequência (Hz)")
        for ax in eixos[:, j]:
            for lado in ("top", "right"):
                ax.spines[lado].set_visible(False)
        eixos[1, j].set_xlim(0.01, 1 / (2 * mod.ts))
        _hz(eixos[1, j])
    eixos[0, 0].set_ylabel("Ganho de $L$ (dB)")
    eixos[1, 0].set_ylabel("Fase de $L$ (°)")
    eixos[0, 0].legend(loc="upper right")
    return _salvar(fig, destino)


def desenhar_sensibilidade(mod, proj, destino):
    """|Theta/D|: quanto um torque de cada frequencia desvia a atitude."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8))
    for ax, eixo in zip(eixos, EIXOS_FIG):
        for var in VARIANTES:
            an = proj[eixo][var]["analise"]
            ax.loglog(an.w / (2 * np.pi), np.degrees(np.abs(an.S_d)), color=_cor(E, var),
                      label=ROTULO[var])
        topo = max(np.degrees(np.abs(proj[eixo][v]["analise"].S_d)).max() for v in VARIANTES)
        ax.set_ylim(topo * 1e-3, topo * 2.0)
        ax.set_xlim(0.01, 1 / (2 * mod.ts))
        _hz(ax)
        ax.set_xlabel("Frequência (Hz)")
        ax.set_title(NOMES[eixo].capitalize(), loc="left")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    eixos[0].set_ylabel("$|\\Theta/D|$ (° por rad/s²)")
    eixos[0].legend(loc="lower left")
    return _salvar(fig, destino)


def desenhar_tempo_torque(sims, destino):
    """Erro de atitude no pulso e no degrau de torque, simulados no modelo."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(2, 2, figsize=(6.3, 4.6))
    casos = (("torque_pulso", "roll", (-0.3, 3.0)), ("torque_pulso", "yaw", (-0.3, 4.0)),
             ("torque_degrau", "roll", (-0.3, 4.0)), ("torque_degrau", "yaw", (-1.0, 30.0)))
    for ax, (ensaio, eixo, xlim) in zip(eixos.flat, casos):
        for var in VARIANTES + ("lqi_ruido",):
            if (eixo, var) not in sims:
                continue
            s = sims[(eixo, var)][ensaio]
            ax.plot(s["t"], s["erro"], color=_cor(E, var), label=ROTULO[var],
                    ls="--" if var == "lqi_ruido" else "-")
        ax.axhline(0, color=E.MUDO, lw=0.8, zorder=0)
        ax.set_xlim(*xlim)
        amp = sims[(eixo, "nativo")][ensaio]["amp"]
        tipo = "pulso" if "pulso" in ensaio else "degrau"
        ax.set_title(f"{NOMES[eixo].capitalize()}, {tipo} de "
                     f"{amp:g} rad/s²".replace(".", ","), loc="left")
        ax.set_xlabel("Tempo desde o início do torque (s)")
        ax.set_ylabel("Erro de atitude (°)")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    eixos[0, 0].legend(loc="upper right")
    eixos[0, 1].legend(handles=eixos[0, 1].get_lines()[3:4], loc="lower right")
    return _salvar(fig, destino)


def desenhar_degrau(sims, destino):
    """Degrau de atitude com o alvo registrado no voo nominal do nativo."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8))
    for ax, eixo, xlim in zip(eixos, EIXOS_FIG, ((-0.05, 0.4), (-0.2, 2.5))):
        ref = None
        for var in VARIANTES:
            s = sims[(eixo, var)].get("nominal")
            if s is None:
                continue
            ref = s
            ax.plot(s["t"], s["y"], color=_cor(E, var), label=ROTULO[var])
        if ref is None:
            continue
        ax.plot(ref["t"], ref["ref"], color=E.REFERENCIA, lw=E.LARGURA_REF, ls="--",
                label="Alvo", zorder=1)
        ax.set_xlim(*xlim)
        ax.set_xlabel("Tempo desde o degrau (s)")
        ax.set_title(NOMES[eixo].capitalize(), loc="left")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    eixos[0].set_ylabel("Ângulo (°)")
    eixos[1].legend(loc="lower right")
    return _salvar(fig, destino)


def desenhar_eso(sims, destino):
    """Perturbacao injetada e a estimativa do ESO, simuladas."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8))
    for ax, (ensaio, eixo, xlim) in zip(eixos, (("torque_pulso", "roll", (-0.2, 1.2)),
                                                ("torque_degrau", "yaw", (-0.5, 3.0)))):
        s = sims[(eixo, "eso")][ensaio]
        r = s["sim"]
        ax.plot(r["t"] - s["t0"], r["d"], color=E.REFERENCIA, lw=E.LARGURA_REF, ls="--",
                label="Injetada $d$")
        ax.plot(r["t_laco"] - s["t0"], r["d_hat"], color=_cor(E, "eso"),
                label="Estimada $\\hat d$")
        ax.set_xlim(*xlim)
        tipo = "pulso" if "pulso" in ensaio else "degrau"
        ax.set_title(f"{NOMES[eixo].capitalize()}, {tipo}", loc="left")
        ax.set_xlabel("Tempo desde o início do torque (s)")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    eixos[0].set_ylabel("Perturbação (rad/s²)")
    eixos[0].legend(loc="upper right")
    return _salvar(fig, destino)


def desenhar_compromisso(mod, proj, destino):
    """Rapidez do modo mais lento contra margem de fase e ganho de ruido, nas varreduras."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(2, 2, figsize=(6.3, 4.8))
    marcas = {"lqi": "s", "eso": "^"}
    for j, eixo in enumerate(EIXOS_FIG):
        an_n = proj[eixo]["nativo"]["analise"]
        for i, (chave, rot) in enumerate((("mf", "Margem de fase (°)"),
                                          ("ruido", "Ganho de ruído"))):
            ax = eixos[i, j]
            for var in ("lqi", "eso"):
                pts = [v for v in proj[eixo][var]["varredura"] if v["viavel"] and v[chave]]
                x = np.array([v[chave] for v in pts])
                y = np.array([-v["lento"] for v in pts])
                ok = np.array([v["passa"] for v in pts])
                cor = _cor(E, var)
                ax.scatter(x[ok], y[ok], marker=marcas[var], s=18, color=cor,
                           label=f"{ROTULO[var]} (passa)", zorder=3)
                ax.scatter(x[~ok], y[~ok], marker=marcas[var], s=18, facecolor="none",
                           edgecolor=cor, label=f"{ROTULO[var]} (não passa)", zorder=3)
                esc = proj[eixo][var]
                ax.scatter([esc[chave]], [-esc["lento"]], marker="o", s=110, facecolor="none",
                           edgecolor=E.TINTA, linewidth=1.0, zorder=4)
            xn = an_n.margens["mf_graus"] if chave == "mf" else an_n.ruido
            ax.scatter([xn], [-polo_lento(an_n, mod.ts)], marker="*", s=90,
                       color=_cor(E, "nativo"), label="Nativo", zorder=4)
            if chave == "mf":
                ax.axvline(xn, color=E.MUDO, lw=0.8, ls=":")
            ax.set_yscale("log")
            ax.set_xlabel(rot)
            if j == 0:
                ax.set_ylabel("Rapidez do modo lento (1/s)")
            if i == 0:
                ax.set_title(NOMES[eixo].capitalize(), loc="left")
            for lado in ("top", "right"):
                ax.spines[lado].set_visible(False)
    eixos[0, 0].legend(loc="lower left", fontsize=7)
    return _salvar(fig, destino)


def desenhar_robustez(mod, proj, destino):
    """Respostas a torque com a planta alterada (b +-20 %, +1 laco): faixa e nominal."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.9))
    for ax, (ensaio, eixo, xlim) in zip(eixos, (("torque_pulso", "roll", (-0.2, 3.0)),
                                                ("torque_degrau", "yaw", (-1.0, 30.0)))):
        cen = f"{GRUPO[eixo]}/{ensaio}"
        for var in VARIANTES:
            g = proj[eixo][var]["ganhos"] if var != "nativo" else None
            curvas = []
            for _, fator, atraso in CASOS_ROBUSTEZ:
                s = simular_torque(mod, eixo, novo_controlador(mod, eixo, var, g), cen,
                                   b_planta=mod.b[eixo] * fator, atraso_lacos=atraso)
                curvas.append(s["erro"])
            nom = simular_torque(mod, eixo, novo_controlador(mod, eixo, var, g), cen)
            cor = _cor(E, var)
            ax.fill_between(nom["t"], np.min(curvas, axis=0), np.max(curvas, axis=0),
                            color=cor, alpha=0.18, lw=0)
            ax.plot(nom["t"], nom["erro"], color=cor, label=ROTULO[var])
        ax.axhline(0, color=E.MUDO, lw=0.8, zorder=0)
        ax.set_xlim(*xlim)
        tipo = "pulso" if "pulso" in ensaio else "degrau"
        ax.set_title(f"{NOMES[eixo].capitalize()}, {tipo}", loc="left")
        ax.set_xlabel("Tempo desde o início do torque (s)")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    eixos[0].set_ylabel("Erro de atitude (°)")
    eixos[0].legend(loc="upper right")
    return _salvar(fig, destino)




def rastreio(mod, proj, lqg, eixos=("roll", "yaw")):
    """Degrau de atitude (alvo do voo nominal do nativo) na planta real, por controlador.

    A comparacao justa do rastreamento e com a MESMA alimentacao direta: o PID +
    FF (cc_pid_ff) e o LQI ja a tem. Para o LQG, com um estimador so (o lento) e
    com o estimador rapido da manobra.
    """
    res = {}
    for eixo in eixos:
        alvo = alvo_nominal(eixo)
        if alvo is None or lqg[eixo]["lqg_ruido"] is None:
            continue
        g_nat = V.ganhos(mod.p, "ATC", eixo)
        g = lqg[eixo]["lqg_ruido"]["ganhos"]
        casos = (("Nativo", novo_controlador(mod, eixo, "nativo")),
                 ("PID + FF", C.CascataPID(g_nat, eixo, mod.ts, raiz=False,
                                           ff=(mod.a, mod.b[eixo]))),
                 ("LQI", novo_controlador(mod, eixo, "lqi", proj[eixo]["lqi"]["ganhos"])),
                 ("LQG, um estimador", lqg_de_ganhos(mod, eixo, {**g, "L_r": None})),
                 ("LQG, com manobra", lqg_de_ganhos(mod, eixo, g)))
        res[eixo] = [(nome, simular_degrau(mod, eixo, ctl, alvo, planta=planta_real(eixo))["metricas"])
                     for nome, ctl in casos]
    return res


def tabela_rastreio(res):
    linhas = []
    for eixo, casos in res.items():
        for nome, m in casos:
            linhas.append(f"{NOMES[eixo].capitalize()} & {nome} & {_tex(m.get('sobressinal_pct'), 1)} & "
                          f"{_tex(m.get('erro_rastreio_max_deg'), 3)} & "
                          f"{_tex(m.get('erro_rastreio_rms_deg'), 3)} \\\\")
        linhas.append("\\addlinespace")
    linhas.pop()
    return (CAB_TEX + "\\begin{tabular}{llrrr}\n\\toprule\n"
            " & & & \\multicolumn{2}{c}{Erro de rastreamento (\\si{\\degree})} \\\\\n"
            "\\cmidrule(lr){4-5}\n"
            "Eixo & Controlador & Sobressinal (\\%) & máximo & eficaz \\\\\n"
            "\\midrule\n" + "\n".join(linhas) + "\n\\bottomrule\n\\end{tabular}\n")


def desenhar_fronteira(veredito, destino):
    """Frentes de Pareto (metrica primaria x ganho de ruido) de cada familia, na planta real."""
    E = _estilo()
    import matplotlib.pyplot as plt
    cor = {"pid": E.SERIE[0], "lqi": E.SERIE[1], "eso": E.SERIE[2], "lqg": E.SERIE[3]}
    marca = {"pid": "o", "lqi": "s", "eso": "^", "lqg": "D"}
    fig, eixos = plt.subplots(1, len(veredito), figsize=(6.3, 3.0))
    for ax, (eixo, v) in zip(np.atleast_1d(eixos), veredito.items()):
        nat = v["frs"]["nativo"]
        for f in FAMILIAS:
            fr = [p for p in v["frs"][f]["frente"] if p["ruido"] / nat["ruido"] < 3.0]
            if not fr:
                continue
            x = [p["ruido"] / nat["ruido"] for p in fr]
            y = [p["primaria"] / nat["primaria"] for p in fr]
            ax.plot(x, y, color=cor[f], marker=marca[f], ms=3.5, lw=1.2,
                    label=ROTULO_FAMILIA[f])
        lqg = v["melhores"]["lqg"]
        if lqg is not None:
            ax.scatter([lqg["ruido"] / nat["ruido"]], [lqg["primaria"] / nat["primaria"]],
                       s=110, facecolor="none", edgecolor=E.TINTA, linewidth=1.0, zorder=5,
                       label="LQG escolhido")
        ax.scatter([1.0], [1.0], marker="*", s=120, color=E.TINTA, zorder=6, label="Nativo")
        ax.axvline(1.0, color=E.MUDO, lw=0.8, ls=":")
        ax.axhline(1.0, color=E.MUDO, lw=0.8, ls=":")
        import matplotlib.ticker
        ax.set_xscale("log")
        ax.set_xlim(0.3, 3.0)
        ax.set_xticks([0.5, 1, 2])
        ax.set_xticklabels(["0,5", "1", "2"])
        ax.xaxis.set_minor_formatter(matplotlib.ticker.NullFormatter())
        # a regiao de interesse: perto do nativo; acima de 2x so ha o ramo de
        # pouco ruido das familias, que nao muda a leitura
        ax.set_ylim(0.6, 2.0)
        ax.yaxis.set_major_formatter(matplotlib.ticker.FuncFormatter(
            lambda v, _p: f"{v:g}".replace(".", ",")))
        ens = "pulso" if CRITERIO[eixo]["primaria"][0] == "torque_pulso" else "degrau"
        ax.set_title(f"{NOMES[eixo].capitalize()}: desvio no {ens} de torque", loc="left")
        ax.set_xlabel("Ganho de ruído (× nativo)")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    np.atleast_1d(eixos)[0].set_ylabel("Desvio máximo (× nativo)")
    # no painel da rolagem o canto de cima a direita fica vazio
    np.atleast_1d(eixos)[0].legend(loc="upper right", fontsize=7)
    return _salvar(fig, destino)


def desenhar_ltr(pontos, veredito, destino):
    """MF e ganho de ruido do LQG escolhido ao longo de q_d: o caminho da LTR."""
    E = _estilo()
    import matplotlib.pyplot as plt
    fig, eixos = plt.subplots(1, 2, figsize=(6.3, 2.8))
    for (eixo, v), c in zip(veredito.items(), (E.SERIE[0], E.SERIE[1])):
        lqg = v["melhores"]["lqg"]
        if lqg is None:
            continue
        nat = v["frs"]["nativo"]
        cam = sorted((p for p in pontos if p["familia"] == "lqg" and p["eixo"] == eixo
                      and p.get("estavel") and p["par"]["r"] == lqg["par"]["r"]
                      and p["par"]["pw"] == lqg["par"]["pw"]), key=lambda p: p["par"]["q_d"])
        q = [p["par"]["q_d"] for p in cam]
        eixos[0].semilogx(q, [p["mf"] for p in cam], color=c, label=NOMES[eixo].capitalize())
        eixos[0].axhline(nat["mf"], color=c, lw=0.8, ls=":")
        eixos[1].loglog(q, [p["ruido"] / nat["ruido"] for p in cam], color=c)
        for ax, y in ((eixos[0], lqg["mf"]), (eixos[1], lqg["ruido"] / nat["ruido"])):
            ax.scatter([lqg["par"]["q_d"]], [y], s=60, facecolor="none", edgecolor=E.TINTA,
                       zorder=5)
    eixos[1].axhline(1.0, color=E.MUDO, lw=0.8, ls=":")
    eixos[0].set_ylabel("Margem de fase (°)")
    eixos[1].set_ylabel("Ganho de ruído (× nativo)")
    for ax in eixos:
        ax.set_xlabel("Intensidade da perturbação $q_d$")
        for lado in ("top", "right"):
            ax.spines[lado].set_visible(False)
    eixos[0].legend(loc="lower right")
    return _salvar(fig, destino)


# --------------------------------------------------------------------------

def main():
    import warnings
    warnings.filterwarnings("ignore", category=linalg.LinAlgWarning)
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--parm", type=Path, help="params.parm do voo que define o modelo "
                    "(padrao: o ensaio mais recente do nativo)")
    ap.add_argument("--tensao-repouso", type=float,
                    help="BAT.VoltR em V (padrao: a mediana do mesmo voo)")
    ap.add_argument("--figuras", type=Path, help="pasta onde gravar as figuras (PDF)")
    ap.add_argument("--tabelas", type=Path, help="pasta onde gravar as tabelas (.tex)")
    ap.add_argument("--portao", action="store_true",
                    help="LQG: varre as familias (PID, LQI, ESO, LQG) na planta real, decide o "
                         "portao e projeta cc_lqg e cc_lqg_mesmo_ruido (~15 min)")
    ap.add_argument("--cache", type=Path,
                    help="com --portao: arquivo (pickle) das varreduras; reaproveitado se existir")
    ap.add_argument("--fc-giro", type=float,
                    help="variante de veiculo: outro INS_GYRO_FILTER (Hz) no modelo")
    ap.add_argument("--ganhos-nativo", type=Path,
                    help="YAML de controlador (nativo_<sufixo>.yaml) com os ganhos do nativo "
                         "desse veiculo, do AutoTune")
    ap.add_argument("--sufixo",
                    help="com --portao --escrever-yaml: grava so cc_lqi_<sufixo>, "
                         "cc_eso_<sufixo> e cc_lqg_<sufixo>")
    ap.add_argument("--ablacao", type=Path, metavar="CACHE",
                    help="com --portao: projeta as variantes de ablacao do LQG (rolagem) e, com "
                         "--escrever-yaml, grava cc_lqg_sem_*.yaml; CACHE e o pickle da varredura")
    ap.add_argument("--escrever-yaml", action="store_true",
                    help="grava cc_lqi, cc_eso, cc_lqi_mesmo_ruido, cc_lqi_sem_ff, cc_pid_ff e "
                         "nativo_lqi (.yaml) em ensaios/controladores/")
    args = ap.parse_args()

    mod = carregar_modelo(args.parm, args.tensao_repouso)
    params_veiculo = {}
    if args.fc_giro:
        import dataclasses
        mod = dataclasses.replace(mod, fc_giro=args.fc_giro)
        params_veiculo["INS_GYRO_FILTER"] = args.fc_giro
    if args.ganhos_nativo:
        # os ganhos do nativo desse veiculo (do AutoTune) no lugar dos do voo
        g = yaml.safe_load(args.ganhos_nativo.read_text(encoding="utf-8"))["ganhos"]
        mod.p.update({k: float(v) for k, v in g.items()})
    proj = projetar(mod, relaxar=bool(args.sufixo))
    rob = robustez(mod, proj)
    eq = equivalencia(mod, proj)
    sims = simulacoes(mod, proj)
    relatorio(mod, proj, rob, eq, sims)

    if args.escrever_yaml and not args.sufixo:
        for c in escrever_yamls(mod, proj, eq):
            print(f"yaml: {c}")
    if args.portao:
        import pickle
        if args.cache and args.cache.exists():
            pontos = pickle.loads(args.cache.read_bytes())
        else:
            pontos = varrer(mod, ("roll", "yaw")) + varrer(mod, ("pitch",), ("lqg",))
            if args.cache:
                args.cache.write_bytes(pickle.dumps(pontos))
        if args.sufixo and not any(p["eixo"] == "pitch" and p["familia"] == "lqi" for p in pontos):
            # a variante voa tambem o LQI e o ESO, que precisam da arfagem (o
            # nativo de arfagem ja esta nos pontos)
            pontos += [p for p in varrer(mod, ("pitch",), ("lqi", "eso"))
                       if p["familia"] != "nativo"]
            if args.cache:
                args.cache.write_bytes(pickle.dumps(pontos))
        _CTX["mod"] = mod
        ver = portao(mod, pontos, proj)
        imprimir_portao(ver)
        lqg = projetar_lqg(mod, proj, pontos, relaxar=bool(args.sufixo))
        print("\nLQG projetados (planta real):")
        for eixo in EIXOS:
            for var in ("lqg", "lqg_ruido"):
                d = lqg[eixo][var]
                if d is None:
                    print(f"  {NOMES[eixo]:8s} {var:9s}: nenhum ponto cumpre")
                    continue
                g = d["ganhos"]
                print(f"  {NOMES[eixo]:8s} {var:9s}: q_d {g['q_d']:.4g} r {g['r']:.4g} pw "
                      f"{g['pw']:g}  k_th {g['k_th']:.4g} k_w {g['k_w']:.4g}")
        if args.sufixo:
            print("\nLQI e LQR+ESO da variante (mesmo criterio do LQG):")
            fixas = projetar_fixas(mod, proj, pontos)
        if args.escrever_yaml and args.sufixo:
            for c in escrever_yamls_variante(mod, lqg, fixas, args.sufixo, params_veiculo):
                print(f"yaml: {c}")
        elif args.escrever_yaml:
            for c in escrever_yamls_lqg(mod, lqg):
                print(f"yaml: {c}")
        if args.ablacao and not args.sufixo:
            print("\nablacao do LQG de mesmo ruido (rolagem):")
            abl = projetar_ablacao(mod, proj, "roll", cache=args.ablacao)
            if args.escrever_yaml:
                for c in escrever_yamls_ablacao(mod, lqg, abl):
                    print(f"yaml: {c}")
        ras = rastreio(mod, proj, lqg)
        print("\nrastreamento (degrau de atitude, planta real): sobressinal % / erro max / eficaz")
        for eixo, casos in ras.items():
            for nome, m in casos:
                print(f"  {NOMES[eixo]:8s} {nome:20s} {m.get('sobressinal_pct', float('nan')):5.1f} / "
                      f"{m.get('erro_rastreio_max_deg', float('nan')):.3f} / "
                      f"{m.get('erro_rastreio_rms_deg', float('nan')):.3f}")
        if args.tabelas:
            (args.tabelas / "projeto_portao.tex").write_text(tabela_portao(ver), encoding="utf-8")
            print(f"tabela: {args.tabelas / 'projeto_portao.tex'}")
            (args.tabelas / "projeto_rastreio.tex").write_text(tabela_rastreio(ras),
                                                               encoding="utf-8")
            print(f"tabela: {args.tabelas / 'projeto_rastreio.tex'}")
        if args.figuras:
            print(f"figura: {desenhar_fronteira(ver, args.figuras / 'projeto_fronteira.pdf')}")
            print(f"figura: {desenhar_ltr(pontos, ver, args.figuras / 'projeto_ltr.pdf')}")
    if args.tabelas:
        args.tabelas.mkdir(parents=True, exist_ok=True)
        for nome, texto in (("projeto_ganhos.tex", tabela_ganhos(proj)),
                            ("projeto_margens.tex", tabela_margens(mod, proj, rob)),
                            ("projeto_previsao.tex", tabela_previsao(sims))):
            (args.tabelas / nome).write_text(texto, encoding="utf-8")
            print(f"tabela: {args.tabelas / nome}")
    if args.figuras:
        d = args.figuras
        for caminho in (desenhar_polos(mod, proj, d / "projeto_polos.pdf"),
                        desenhar_malha(mod, proj, d / "projeto_malha.pdf"),
                        desenhar_sensibilidade(mod, proj, d / "projeto_sensibilidade.pdf"),
                        desenhar_tempo_torque(sims, d / "projeto_tempo_torque.pdf"),
                        desenhar_degrau(sims, d / "projeto_degrau.pdf"),
                        desenhar_eso(sims, d / "projeto_eso.pdf"),
                        desenhar_compromisso(mod, proj, d / "projeto_compromisso.pdf"),
                        desenhar_robustez(mod, proj, d / "projeto_robustez.pdf")):
            print(f"figura: {caminho}")


if __name__ == "__main__":
    main()
