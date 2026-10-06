"""Referencia em Python do injetor de vento — espelho exato de ``lua/vento.lua``.

Serve a dois propositos:

1. **Reprodutibilidade.** Regera o perfil de vento a partir dos mesmos parametros
   e da mesma semente e o confronta com a mensagem ``WND`` do log. Se as duas
   series coincidem, o ensaio e reproduzivel por construcao, e nao por sorte.
2. **Consistencia entre os dois caminhos.** E o mesmo gerador que a proposta
   pede que exista no ambiente de projeto em Python, disponivel para alimentar
   simulacoes do modelo nao-linear fora do SITL.

Qualquer alteracao em ``lua/vento.lua`` tem de ser espelhada aqui, e o teste de
``comparar_com_log`` e o que denuncia o esquecimento.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# bits de WND_MODE
MODO_MEDIO = 1
MODO_RAJADA = 2
MODO_DRYDEN = 4
MODO_TQ_PULSO = 8
MODO_TQ_COS = 16
MODO_TQ_DEG = 32
MODO_TQ_CHIRP = 64
MODO_TQ_QUALQUER = MODO_TQ_PULSO | MODO_TQ_COS | MODO_TQ_DEG | MODO_TQ_CHIRP

# Park-Miller minstd, identico ao do Lua
MINSTD_M = 2147483647
MINSTD_A = 16807

TAXA_HZ = 50.0
DT = 1.0 / TAXA_HZ


class RngMinstd:
    """LCG de Park-Miller + Box-Muller, bit a bit igual ao do vento.lua.

    Usa inteiros Python (exatos) — o maior produto intermediario e
    16807*2147483646 ~ 3.6e13, dentro do que o Lua tambem representa sem perda.
    """

    def __init__(self, semente):
        self.estado = int(abs(semente)) % (MINSTD_M - 1) + 1

    def uniforme(self):
        self.estado = (MINSTD_A * self.estado) % MINSTD_M
        return self.estado / MINSTD_M

    def normais(self):
        """Duas normais padrao por chamada — mesma ordem de consumo do Lua."""
        u1 = self.uniforme()
        u2 = self.uniforme()
        r = np.sqrt(-2.0 * np.log(u1))
        a = 2.0 * np.pi * u2
        return r * np.cos(a), r * np.sin(a)


@dataclass
class Perfil:
    """Parametros WND_* de um cenario."""

    modo: int = 0
    spd: float = 0.0
    dir: float = 0.0
    dirz: float = 0.0
    ga: float = 0.0
    gt0: float = 10.0
    gdur: float = 3.0
    tsig: float = 0.0
    tlu: float = 200.0
    seed: int = 1
    t0: float = 8.0
    tqx: float = 0.0
    tqy: float = 0.0
    tqz: float = 0.0
    tqt0: float = 20.0
    tqdur: float = 0.4
    tqf0: float = 0.2
    tqf1: float = 10.0

    @classmethod
    def de_dict(cls, d):
        """Aceita as chaves como aparecem no YAML do cenario (``WND_SPD`` ou ``spd``).

        ``WND_MODE`` vira ``modo``, porque ``mode`` colidiria com o vocabulario de
        modos de voo usado no resto dos ensaios.
        """
        campos = set(cls.__annotations__)
        inteiros = {"modo", "seed"}
        limpo = {}
        for k, v in d.items():
            nome = k.lower().removeprefix("wnd_")
            if nome == "mode":
                nome = "modo"
            if nome in campos:
                limpo[nome] = int(v) if nome in inteiros else float(v)
        return cls(**limpo)


def forma_torque(perfil: Perfil, t):
    """Fator de forma do disturbio de torque em ``t`` — espelho do Lua.

    As tres formas se somam quando mais de um bit esta ligado.
    """
    tqdur = max(perfil.tqdur, 0.01)
    dentro = (t >= perfil.tqt0) & (t <= perfil.tqt0 + tqdur)
    f = np.zeros_like(np.asarray(t, dtype=float))
    if perfil.modo & MODO_TQ_PULSO:
        f = f + np.where(dentro, 1.0, 0.0)
    if perfil.modo & MODO_TQ_COS:
        f = f + np.where(
            dentro,
            0.5 * (1.0 - np.cos(2.0 * np.pi * (np.asarray(t) - perfil.tqt0) / tqdur)),
            0.0)
    if perfil.modo & MODO_TQ_DEG:
        f = f + np.where(np.asarray(t) >= perfil.tqt0, 1.0, 0.0)

    if perfil.modo & MODO_TQ_CHIRP:
        tc = np.asarray(t, dtype=float) - perfil.tqt0
        fade = tqdur * 0.1
        wmin = 2.0 * np.pi * max(perfil.tqf0, 0.01)
        wmax = 2.0 * np.pi * max(perfil.tqf1, 0.01)
        B = np.log(wmax / wmin)

        janela = np.zeros_like(tc)
        subida = (tc > 0) & (tc <= fade)
        plano = (tc > fade) & (tc <= tqdur - fade)
        descida = (tc > tqdur - fade) & (tc <= tqdur)
        janela[subida] = 0.5 - 0.5 * np.cos(np.pi * tc[subida] / fade)
        janela[plano] = 1.0
        janela[descida] = 0.5 - 0.5 * np.cos(
            np.pi * (tc[descida] - (tqdur - fade)) / fade + np.pi)

        if abs(B) < 1e-9:
            fase = wmin * tc
        else:
            fase = (wmin * tqdur / B) * (np.exp(B * np.clip(tc, 0, tqdur) / tqdur) - 1.0)
        dentro_ch = (tc > 0) & (tc <= tqdur)
        f = f + np.where(dentro_ch, janela * np.sin(fase), 0.0)

    return f


def gerar(perfil: Perfil, duracao_s: float, dt: float = DT):
    """Gera o perfil de disturbio e devolve (t, wn, we, tqx, tqy, tqz).

    Espelha o passo fixo do Lua: o gerador e uma funcao do INDICE DE PASSO, nao
    do instante em que o script foi escalonado. O passo ``k`` (1-indexado, como
    no Lua) corresponde ao tempo de perfil ``t = k*dt``, e e esse valor que o
    firmware grava no campo ``Tp`` da mensagem WND.

    ``duracao_s`` e a duracao do PERFIL, contada do primeiro passo.
    """
    n = int(round(duracao_s / dt))
    if n <= 0:
        z = np.zeros(0)
        return z, z, z, z, z, z

    va = max(perfil.spd, 1.0)
    lu = max(perfil.tlu, 1.0)
    lv = lu * 0.5
    sigma = perfil.tsig
    gdur = max(perfil.gdur, 0.01)

    # longitudinal: Ornstein-Uhlenbeck, discretizacao exata
    u_phi = np.exp(-(va / lu) * dt)
    u_g = sigma * np.sqrt(max(1.0 - u_phi * u_phi, 0.0))

    # lateral: Dryden de 2a ordem por Tustin. K = sigma*sqrt(3a), SEM o fator
    # 1/pi — com ele a saida sai em 0.56*sigma (conferido numericamente).
    a = va / lv
    K = sigma * np.sqrt(3.0 * a)
    z = a / np.sqrt(3.0)
    c = 2.0 / dt
    A0 = c * c + 2.0 * a * c + a * a
    v_b0 = (K * c + K * z) / A0
    v_b1 = (2.0 * K * z) / A0
    v_b2 = (-K * c + K * z) / A0
    v_a1 = (-2.0 * c * c + 2.0 * a * a) / A0
    v_a2 = (c * c - 2.0 * a * c + a * a) / A0
    v_esc = 1.0 / np.sqrt(dt)

    rng = RngMinstd(perfil.seed)
    du_x = 0.0
    dv_u1 = dv_u2 = dv_y1 = dv_y2 = 0.0

    t = np.empty(n)
    wn = np.empty(n)
    we = np.empty(n)

    psi = np.radians(perfil.dir + 180.0)
    cs, sn = np.cos(psi), np.sin(psi)

    for k in range(n):
        tk = (k + 1) * dt          # passo 1-indexado, igual ao Lua
        vel_long = 0.0
        vel_lat = 0.0

        if perfil.modo & MODO_MEDIO:
            vel_long += perfil.spd

        if perfil.modo & MODO_RAJADA:
            tg = tk - perfil.gt0
            if 0.0 <= tg <= gdur:
                vel_long += 0.5 * perfil.ga * (1.0 - np.cos(2.0 * np.pi * tg / gdur))

        if (perfil.modo & MODO_DRYDEN) and sigma > 0.0:
            w0, w1 = rng.normais()
            du_x = u_phi * du_x + u_g * w0
            uin = w1 * v_esc
            y = (v_b0 * uin + v_b1 * dv_u1 + v_b2 * dv_u2
                 - v_a1 * dv_y1 - v_a2 * dv_y2)
            dv_u2, dv_u1 = dv_u1, uin
            dv_y2, dv_y1 = dv_y1, y
            vel_long += du_x
            vel_lat += y

        t[k] = tk
        wn[k] = vel_long * cs - vel_lat * sn
        we[k] = vel_long * sn + vel_lat * cs

    # torque: deterministico, sem RNG, entao sai vetorizado de uma vez
    ft = forma_torque(perfil, t)
    return t, wn, we, perfil.tqx * ft, perfil.tqy * ft, perfil.tqz * ft


def comparar_com_log(perfil: Perfil, dados, tolerancia=1e-3):
    """Confronta o perfil regerado com a mensagem WND do log.

    O alinhamento usa o campo ``Tp`` (tempo do perfil) gravado pelo injetor, e
    NAO a posicao da amostra no log. A diferenca importa: o laco do Lua nao roda
    na taxa nominal — medido em voo, ~44 Hz em vez de 50 —, entao contar amostras
    desalinharia a referencia e acusaria erro onde nao ha. Com ``Tp``, cada
    amostra do log encontra exatamente o passo que a gerou.
    """
    wnd = dados.get("WND")
    if wnd is None or len(wnd.get("t", [])) < 2:
        return {"ok": False, "motivo": "log sem mensagem WND"}
    if "Tp" not in wnd:
        return {"ok": False,
                "motivo": "WND sem o campo Tp — log gravado por uma versao "
                          "anterior do vento.lua"}

    tp = np.asarray(wnd["Tp"], dtype=float)
    idx = np.round(tp / DT).astype(int) - 1          # Tp = (passo)*DT, 1-indexado
    valido = idx >= 0
    if not valido.any():
        return {"ok": False, "motivo": "nenhum passo valido em Tp"}

    _, wn_ref, we_ref, tx_ref, ty_ref, tz_ref = gerar(perfil, float(tp.max()) + DT)
    valido &= idx < len(wn_ref)
    idx_v = idx[valido]
    if len(idx_v) == 0:
        return {"ok": False, "motivo": "referencia mais curta que o log"}

    res = {
        "amostras": int(len(idx_v)),
        "taxa_efetiva_hz": float(len(tp) / max(tp[-1] - tp[0], 1e-9)),
    }
    piores = []
    # o torque so entra na conferencia quando o log realmente o traz: assim um
    # log antigo, sem os campos Tx/Ty/Tz, nao passa a falhar por isso
    pares = [("Wn", wn_ref, "wn"), ("We", we_ref, "we")]
    if "Tx" in wnd:
        pares += [("Tx", tx_ref, "tqx"), ("Ty", ty_ref, "tqy"), ("Tz", tz_ref, "tqz")]
    for campo, ref, nome in pares:
        d = np.asarray(wnd[campo], dtype=float)[valido] - ref[idx_v]
        res[f"err_max_{nome}"] = float(np.max(np.abs(d)))
        res[f"err_rms_{nome}"] = float(np.sqrt(np.mean(d ** 2)))
        piores.append(res[f"err_max_{nome}"])
    res["ok"] = max(piores) <= tolerancia
    return res
