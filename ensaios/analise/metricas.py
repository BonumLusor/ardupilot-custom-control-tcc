"""Metricas de desempenho de um ensaio.

Sao exatamente as que a proposta do TCC tabela:

  transitorio   tempo de subida, sobressinal, tempo de acomodacao, erro em
                regime permanente
  disturbio     desvio maximo de atitude, tempo de recuperacao, RMSE
  custo         esforco de controle acumulado, integral do modulo e do quadrado

Toda metrica devolve tambem a janela de tempo em que foi calculada. Isso nao e
zelo excessivo: sem a janela registrada, dois ensaios com a mesma etiqueta de
metrica podem ter medido coisas diferentes, e a comparacao entre controladores
perde o sentido.

As metricas saem POR EIXO (``degrau_roll``, ``degrau_yaw``, ``disturbio_yaw``,
...), so para os eixos que o ensaio exercita: os que recebem degrau ou torque.
"""

from __future__ import annotations

import numpy as np

from .eixos import COLUNAS, EIXOS, ordenar


# Folga entre o fim da analise do degrau e o inicio do disturbio. Sem ela, a
# ultima amostra da janela do degrau ja e a primeira do disturbio, e o pico dele
# entra como "sobressinal" do degrau — foi exatamente o que aconteceu no primeiro
# ensaio de torque deste projeto.
GUARDA_ANTES_DISTURBIO_S = 0.5


def _limpar(t, y):
    """Descarta amostras nao finitas, que aparecem fora do alcance de cada mensagem."""
    ok = np.isfinite(t) & np.isfinite(y)
    return t[ok], y[ok]


def detectar_degrau(t, ref, amplitude_esperada=None, minimo=2.0,
                    janela_s=0.6, plato_ini_s=2.0, plato_dur_s=4.0):
    """Instante e patamares do degrau de referencia.

    Dois cuidados, ambos aprendidos com dados reais deste projeto:

    1. **Nao se detecta degrau amostra a amostra.** O ArduPilot rampeia o alvo de
       atitude pelo modelador de entrada: num degrau de 10 graus, o maior salto
       entre amostras consecutivas fica em ~0,7 grau.
    2. **Nao se usa a maior variacao.** Logo apos o degrau, o alvo modelado dispara
       ate ~2x o valor comandado antes de assentar. A maior variacao da serie e
       esse transitorio, e nao o degrau — um detector por maximo aponta o lugar
       errado com toda a confianca.

    Por isso, quando o ensaio informa ``amplitude_esperada`` (vem do YAML do
    cenario), a deteccao procura a BORDA DE SUBIDA rumo a esse valor: o primeiro
    cruzamento de 50% da amplitude precedido por um trecho estavel perto de zero.
    Sem essa informacao, cai no detector por janela, que serve para inspecionar um
    log solto.

    Devolve (t_degrau, patamar_antes, patamar_depois) ou None quando nao ha degrau.
    """
    t, ref = _limpar(t, ref)
    if len(t) < 10:
        return None
    dt = float(np.median(np.diff(t)))
    w = max(int(janela_s / dt), 1)
    if len(ref) <= w:
        return None

    i = None
    if amplitude_esperada is not None and abs(amplitude_esperada) >= minimo:
        alvo = abs(float(amplitude_esperada))
        n_estavel = max(int(1.5 / dt), 1)
        baixo = np.abs(ref) < 0.25 * alvo
        # soma corrida de "esteve baixo": exige um trecho calmo antes da borda
        calmo = np.convolve(baixo.astype(float), np.ones(n_estavel), mode="full")[:len(ref)]
        cruzou = np.nonzero((np.abs(ref) >= 0.5 * alvo) & (calmo >= n_estavel * 0.9))[0]
        if len(cruzou):
            i = int(cruzou[0])

    if i is None:
        d = ref[w:] - ref[:-w]
        j = int(np.argmax(np.abs(d)))
        if abs(d[j]) < minimo:
            return None
        i = j

    #  ANCORAGEM NO COMANDO, e nao no cruzamento de 50%.
    #  O ``ref`` aqui e o alvo ja MODELADO pelo ArduPilot (ATC_INPUT_TC), nao o
    #  degrau que o ensaio mandou. Cronometrar a partir do cruzamento de 50%
    #  descarta a parte inicial da subida e devolve um tempo de subida menor do
    #  que o real — medido neste projeto, ~0,15 s a menos. O instante do comando
    #  e onde o alvo DEIXA o patamar inicial: dali a diante o firmware ja sabia
    #  do degrau.
    n_antes = max(int(1.0 / dt), 1)
    antes = float(np.median(ref[max(i - n_antes, 0):i + 1]))

    limiar_saida = abs(0.02 * (float(np.median(ref[i:i + n_antes])) - antes)) + 1e-6
    j = i
    while j > 0 and abs(ref[j - 1] - antes) > limiar_saida:
        j -= 1
    t_deg = float(t[j])
    i = j

    # Patamar depois: janela ja acomodada, saltando o transitorio do modelador de
    # entrada. Nao se usa "o resto do log": ele continua pelo pouso, onde a
    # referencia volta a se mexer por conta propria.
    j0 = min(i + max(int(plato_ini_s / dt), 1), len(ref) - 1)
    j1 = min(j0 + max(int(plato_dur_s / dt), 1), len(ref))
    if j1 - j0 < 3:
        return None
    depois = float(np.median(ref[j0:j1]))

    # Fim do patamar: onde a referencia deixa de valer o degrau. O perfil devolve
    # a aeronave a 0 grau antes do disturbio, e sem este limite a descida entra na
    # janela do degrau — o sinal sai da faixa de acomodacao e o tempo de
    # acomodacao vira "—" num ensaio que acomodou em 0,3 s.
    t_fim = None
    if abs(depois - antes) > 1e-6:
        limiar = antes + 0.5 * (depois - antes)
        caindo = np.nonzero((np.arange(len(ref)) > j0)
                            & ((ref - limiar) * np.sign(depois - antes) < 0))[0]
        if len(caindo):
            t_fim = float(t[caindo[0]])
    return t_deg, antes, depois, t_fim


def metricas_degrau(t, ref, y, amplitude_esperada=None, banda=0.02,
                    duracao_analise_s=8.0, ate=None):
    """Tempo de subida (10-90%), sobressinal, acomodacao e erro em regime.

    ``banda`` e a faixa de acomodacao como fracao da amplitude do degrau (2% por
    padrao).

    A analise fica confinada aos ``duracao_analise_s`` seguintes ao degrau E,
    quando ``ate`` e informado, termina antes do disturbio comecar. Sem esse
    recorte a janela do degrau alcanca o disturbio, e o pico dele entra no
    sobressinal do degrau — dois ensaios diferentes somados num numero so.
    """
    deg = detectar_degrau(t, ref, amplitude_esperada=amplitude_esperada)
    if deg is None:
        return {}
    t_deg, y0, y1, t_fim_plato = deg
    amp = y1 - y0
    if abs(amp) < 1e-6:
        return {}

    fim = t_deg + duracao_analise_s
    if ate is not None:
        fim = min(fim, float(ate))
    if t_fim_plato is not None:
        fim = min(fim, t_fim_plato - GUARDA_ANTES_DISTURBIO_S)
    tt, yy = _limpar(t, y)
    sel = (tt >= t_deg) & (tt <= fim)
    tt, yy = tt[sel], yy[sel]
    if len(tt) < 5:
        return {}

    # normaliza para 0 -> 1, o que faz sinal do degrau deixar de importar
    norm = (yy - y0) / amp

    def primeiro_cruzamento(alvo):
        idx = np.nonzero(norm >= alvo)[0]
        return float(tt[idx[0]] - t_deg) if len(idx) else None

    t10 = primeiro_cruzamento(0.1)
    t90 = primeiro_cruzamento(0.9)
    pico = float(np.max(norm))
    i_pico = int(np.argmax(norm))

    # acomodacao: ultimo instante em que sai da banda, medido do fim para tras
    fora = np.nonzero(np.abs(norm - 1.0) > banda)[0]
    t_acom = float(tt[fora[-1]] - t_deg) if len(fora) and fora[-1] < len(tt) - 1 else None

    # regime permanente: media do ultimo 20% da janela
    n_fim = max(len(yy) // 5, 1)
    ess = float(np.mean(yy[-n_fim:]) - y1)

    res = {
        "t_degrau_s": t_deg,
        "amplitude_deg": float(amp),
        "tempo_subida_s": (t90 - t10) if (t10 is not None and t90 is not None) else None,
        "sobressinal_pct": float(max(pico - 1.0, 0.0) * 100.0),
        "t_pico_s": float(tt[i_pico] - t_deg),
        "tempo_acomodacao_s": t_acom,
        "erro_regime_deg": ess,
        "janela_s": [t_deg, float(tt[-1])],
    }

    #  ERRO DE RASTREAMENTO: a metrica que de fato distingue controladores.
    #  Sobressinal e acomodacao medidos contra o alvo modelado caracterizam
    #  sobretudo o modelador de entrada, que e feedforward e IDENTICO em todos os
    #  controladores — dois PIDs bem diferentes devolvem numeros parecidos. Ja o
    #  quanto a aeronave FICA PARA TRAS do alvo e trabalho exclusivo da
    #  realimentacao.
    tr, er = _limpar(t, ref)
    sel_r = (tr >= t_deg) & (tr <= fim)
    if sel_r.sum() > 3:
        ty, yy2 = _limpar(t, y)
        alvo = np.interp(tr[sel_r], ty, yy2) - er[sel_r]
        res["erro_rastreio_max_deg"] = float(np.max(np.abs(alvo)))
        res["erro_rastreio_rms_deg"] = float(np.sqrt(np.mean(alvo ** 2)))

    return res


def metricas_disturbio(t, erro, janela, frac_recuperacao=0.1):
    """Desvio maximo, tempo de recuperacao e RMSE dentro da janela de disturbio.

    O tempo de recuperacao conta do instante do pico ate o erro entrar e
    PERMANECER abaixo de ``frac_recuperacao`` do pico. Exigir permanencia, e nao
    apenas o primeiro cruzamento, evita creditar uma recuperacao a uma passagem
    pelo zero no meio de uma oscilacao.
    """
    t_ini, t_fim = janela
    tt, ee = _limpar(t, erro)
    sel = (tt >= t_ini) & (tt <= t_fim)
    tt, ee = tt[sel], ee[sel]
    if len(tt) < 5:
        return {}

    absr = np.abs(ee)
    i_pico = int(np.argmax(absr))
    pico = float(absr[i_pico])
    t_pico = float(tt[i_pico])

    t_rec = None
    if pico > 1e-6:
        limiar = frac_recuperacao * pico
        depois = absr[i_pico:]
        # ultimo indice acima do limiar: dali em diante o erro nunca mais sobe
        acima = np.nonzero(depois > limiar)[0]
        if len(acima) and acima[-1] < len(depois) - 1:
            t_rec = float(tt[i_pico + acima[-1] + 1] - t_pico)

    return {
        "desvio_max_deg": pico,
        "t_desvio_max_s": t_pico,
        "tempo_recuperacao_s": t_rec,
        "rmse_deg": float(np.sqrt(np.mean(ee ** 2))),
        "erro_medio_abs_deg": float(np.mean(absr)),
        "janela_s": [float(tt[0]), float(tt[-1])],
    }


def metricas_deriva(grade, col, janela):
    """Deriva de posicao e velocidade dentro da janela do disturbio.

    Existe porque, no modelo de multirrotor do SITL, o vento horizontal entra
    como ARRASTO — forca no centro de massa e uma parcela de arrasto de quantidade
    de movimento em cada rotor — e nao como torque. O efeito dominante e a
    translacao: sob uma rajada de 10 m/s a atitude se desvia centesimos de grau,
    enquanto a aeronave anda metros. Medir so atitude daria a impressao, falsa, de
    que o disturbio nao fez nada.

    (A proposta ja antecipa isso: o torque parasita de centro de pressao fora do
    centro de massa foi deliberadamente desprezado no modelo nominal.)
    """
    t_ini, t_fim = janela
    sel = (grade >= t_ini) & (grade <= t_fim)
    if sel.sum() < 5:
        return {}

    res = {}
    if "vn" in col and "ve" in col:
        vn, ve = col["vn"][sel], col["ve"][sel]
        vel = np.hypot(vn, ve)
        vel = vel[np.isfinite(vel)]
        if len(vel):
            res["vel_horiz_max_ms"] = float(np.max(vel))
            res["vel_horiz_media_ms"] = float(np.mean(vel))

    if "pn" in col and "pe" in col:
        pn, pe = col["pn"][sel], col["pe"][sel]
        ok = np.isfinite(pn) & np.isfinite(pe)
        if ok.sum() > 2:
            pn, pe = pn[ok], pe[ok]
            # deslocamento a partir da posicao no INICIO da janela: e a deriva
            # atribuivel ao disturbio, e nao a posicao absoluta no mundo
            d = np.hypot(pn - pn[0], pe - pe[0])
            res["deriva_max_m"] = float(np.max(d))
            res["deriva_final_m"] = float(d[-1])

    if res:
        res["janela_s"] = [float(t_ini), float(t_fim)]
    return res


def esforco_controle(t, u, janela=None):
    """Integral do modulo e do quadrado da saida do controlador.

    E o indicador de custo energetico associado ao desempenho: dois controladores
    com a mesma rejeicao mas esforcos diferentes nao sao equivalentes.
    """
    tt, uu = _limpar(t, u)
    if janela is not None:
        sel = (tt >= janela[0]) & (tt <= janela[1])
        tt, uu = tt[sel], uu[sel]
    if len(tt) < 2:
        return {}
    return {
        "esforco_abs": float(np.trapezoid(np.abs(uu), tt)),
        "esforco_quad": float(np.trapezoid(uu ** 2, tt)),
        "u_max_abs": float(np.max(np.abs(uu))),
        "janela_s": [float(tt[0]), float(tt[-1])],
    }


def _pico(v):
    """Maior modulo finito da serie, ou 0 se nao houver nenhum."""
    v = np.asarray(v, dtype=float)
    v = v[np.isfinite(v)]
    return float(np.max(np.abs(v))) if len(v) else 0.0


def tem_torque(perfil, col=None, limiar=1e-4):
    """True se o cenario aplica disturbio de TORQUE."""
    if perfil is not None:
        return bool(abs(perfil.tqx) > 0 or abs(perfil.tqy) > 0 or abs(perfil.tqz) > 0)
    if col is not None and "tq_mod" in col:
        v = col["tq_mod"][np.isfinite(col["tq_mod"])]
        return bool(len(v) and np.max(v) > limiar)
    return False


def eixos_com_torque(perfil, col=None, limiar=1e-4):
    """Eixos com componente de torque nao nula, na ordem canonica."""
    if perfil is not None:
        amp = {"roll": perfil.tqx, "pitch": perfil.tqy, "yaw": perfil.tqz}
        return [e for e in EIXOS if abs(amp[e]) > 0]
    if col is None:
        return []
    return [e for e in EIXOS
            if COLUNAS[e]["torque"] in col and _pico(col[COLUNAS[e]["torque"]]) > limiar]


def eixos_disturbio(perfil, col=None):
    """Eixos de atitude em que se mede rejeicao de disturbio.

    Com torque, sao os eixos de componente nao nula: num ensaio de torque so em
    guinada, a rolagem nao foi perturbada e "rejeicao em rolagem" mediria ruido.
    Vento sozinho continua medido em rolagem, como sempre foi — no SITL ele nao
    gera momento em eixo nenhum (README, "Dois achados").
    """
    if tem_torque(perfil, col):
        return eixos_com_torque(perfil, col)
    if tem_vento(perfil, col):
        return ["roll"]
    return []


def tem_vento(perfil, col=None, limiar_ms=0.05):
    """True se o cenario aplica vento com intensidade nao nula."""
    if perfil is not None:
        return bool(perfil.spd > 0 or perfil.ga > 0 or perfil.tsig > 0)
    if col is not None and "w_mod" in col:
        w = col["w_mod"][np.isfinite(col["w_mod"])]
        return bool(len(w) and np.max(w) > limiar_ms)
    return False


def perfil_tem_disturbio(perfil, col=None, limiar_ms=0.05):
    """True se o cenario aplica qualquer disturbio — vento ou torque."""
    return tem_vento(perfil, col, limiar_ms) or tem_torque(perfil, col)


def janela_disturbio(grade, col, perfil=None):
    """Janela em que o disturbio age, com folga para a recuperacao.

    A janela e localizada pelo TEMPO DE PERFIL gravado no log (coluna ``tp``), e
    nao por um deslocamento suposto a partir do inicio da grade: a decolagem dura
    o que durar, e um offset suposto poe a janela em cima do trecho errado — o
    que, calado, produz um "desvio maximo" de centesimos de grau e a impressao de
    que o disturbio nao fez nada.

    **O TORQUE TEM PRECEDENCIA sobre o vento.** Num cenario com os dois, e o
    torque que perturba a malha de atitude; centrar a janela na rajada mediria o
    trecho errado.

    A janela e a mesma para todos os eixos. Isso mantem a rolagem de um ensaio
    combinado na mesma janela da rolagem do ensaio so de rolagem — RMSE e erro
    medio dependem do tamanho dela —, e a guinada cabe: o pulso de guinada
    recupera em ~1 s, mais rapido que o de rolagem (medido nos dois pulsos).
    """
    tp = col.get("tp")

    if tem_torque(perfil, col) and tp is not None and perfil is not None:
        ini = perfil.tqt0
        # o degrau sustentado nao termina; os demais ganham folga para recuperar
        if perfil.modo & 32:
            fim = float(tp[-1])
        else:
            fim = perfil.tqt0 + max(perfil.tqdur * 8, 4.0)
        dentro = np.nonzero((tp >= ini) & (tp <= fim))[0]
        if len(dentro) > 5:
            return float(grade[dentro[0]]), float(grade[dentro[-1]])

    if perfil is not None and (perfil.modo & 2) and perfil.ga > 0 and tp is not None:
        # a janela se estende alem da rajada para caber a recuperacao
        ini, fim = perfil.gt0, perfil.gt0 + perfil.gdur * 4
        dentro = np.nonzero((tp >= ini) & (tp <= fim))[0]
        if len(dentro) > 5:
            return float(grade[dentro[0]]), float(grade[dentro[-1]])

    if "w_mod" in col:
        w = col["w_mod"]
        ok = np.isfinite(w)
        if ok.any() and np.nanmax(w) > 1e-3:
            acima = np.nonzero(ok & (w > 0.5 * np.nanmax(w)))[0]
            if len(acima):
                return float(grade[acima[0]]), float(grade[min(acima[-1] + len(acima), len(grade) - 1)])
    return float(grade[0]), float(grade[-1])


def calcular(grade, col, perfil=None, degraus=None, inercia_kgm2=None):
    """Reune todas as metricas do ensaio num unico dicionario.

    ``degraus`` vem do YAML do cenario — {eixo: amplitude em graus}, com zero nos
    eixos sem degrau — e e o que ancora a deteccao de cada degrau no que foi de
    fato comandado. ``None`` (log solto, sem cenario) faz o detector procurar
    degrau em todos os eixos.

    ``inercia_kgm2`` — {eixo: momento de inercia} do veiculo simulado — da ao
    torque injetado o equivalente em N.m, que e a grandeza fisica do disturbio.
    """
    if len(grade) == 0:
        return {}

    res = {"duracao_s": float(grade[-1] - grade[0]),
           "eixos_ativos": []}           # preenchido no fim; so vai para o JSON

    # a janela do disturbio e calculada primeiro, para servir de limite superior
    # a analise dos degraus; vale para todos os eixos perturbados
    eixos_dist = [e for e in eixos_disturbio(perfil, col) if COLUNAS[e]["erro"] in col]
    jan = janela_disturbio(grade, col, perfil) if eixos_dist else None

    for eixo in EIXOS:
        c = COLUNAS[eixo]
        if c["ref"] not in col or c["med"] not in col:
            continue
        amp = None if degraus is None else float(degraus.get(eixo) or 0.0)
        if degraus is not None and not amp:
            continue                 # o cenario nao comanda degrau neste eixo
        d = metricas_degrau(grade, col[c["ref"]], col[c["med"]],
                            amplitude_esperada=amp,
                            ate=(jan[0] - GUARDA_ANTES_DISTURBIO_S) if jan else None)
        # degrau comandado e nao encontrado fica registrado vazio: e diferente de
        # "nao havia degrau", e a analise avisa
        if d or degraus is not None:
            res[f"degrau_{eixo}"] = d

    # As metricas de disturbio so existem quando ha disturbio. Num cenario nominal
    # elas ainda produziriam numeros — o transitorio do degrau viraria "desvio
    # maximo" e a deriva de voar inclinado viraria "deriva sob disturbio" — com
    # rotulos que dizem outra coisa. Numero com rotulo errado e pior que numero
    # nenhum.
    for eixo in eixos_dist:
        res[f"disturbio_{eixo}"] = metricas_disturbio(grade, col[COLUNAS[eixo]["erro"]], jan)
    if jan is not None:
        # A deriva so e calculada quando ha VENTO. Num cenario so de torque ela
        # mediria a translacao provocada pela inclinacao comandada — um numero
        # verdadeiro respondendo a uma pergunta que ninguem fez.
        if tem_vento(perfil, col):
            res["deriva"] = metricas_deriva(grade, col, jan)
        res["janela_disturbio_s"] = list(jan)

    # Eixos exercitados: com degrau comandado (ou detectado) ou com torque. Um
    # ensaio sem nenhum dos dois — vento puro, por exemplo — conta como de
    # rolagem, que e como sempre foi analisado.
    ativos = set(eixos_dist)
    ativos.update(e for e in EIXOS if res.get(f"degrau_{e}") or (degraus or {}).get(e))
    res["eixos_ativos"] = ordenar(ativos) or ["roll"]

    # o esforco so e integrado nos eixos exercitados: o de um eixo parado e ruido
    for eixo in res["eixos_ativos"]:
        if COLUNAS[eixo]["saida"] in col:
            res[f"esforco_{eixo}"] = esforco_controle(grade, col[COLUNAS[eixo]["saida"]])

    # o bloco de vento so existe quando ha vento: tres zeros num ensaio de torque
    # sao ruido visual, nao informacao
    if "w_mod" in col and tem_vento(perfil, col):
        w = col["w_mod"][np.isfinite(col["w_mod"])]
        if len(w):
            res["vento"] = {
                "modulo_max_ms": float(np.max(w)),
                "modulo_medio_ms": float(np.mean(w)),
                "modulo_std_ms": float(np.std(w)),
            }

    if "tq_mod" in col and tem_torque(perfil, col):
        v = col["tq_mod"]
        ok = np.isfinite(v)
        if ok.any():
            res["torque"] = {"modulo_max_rads2": float(np.max(v[ok]))}
            #  O impulso angular (integral do torque = variacao de velocidade
            #  angular injetada) so caracteriza severidade em forma UNIPOLAR. Numa
            #  varredura o sinal troca de sinal o tempo todo: a integral do modulo
            #  mediria "quanto tempo se excitou", nao a severidade, e o numero
            #  soaria enorme sem querer dizer nada. Para o chirp vale a faixa de
            #  frequencia; a analise fica em analise/frequencia.py.
            chirp = perfil is not None and bool(perfil.modo & 64)
            if chirp:
                res["torque"]["chirp_f0_hz"] = float(perfil.tqf0)
                res["torque"]["chirp_f1_hz"] = float(perfil.tqf1)
                res["torque"]["chirp_dur_s"] = float(perfil.tqdur)
            else:
                imp = float(np.trapezoid(v[ok], grade[ok]))
                res["torque"]["impulso_ang_rads"] = imp
                res["torque"]["impulso_ang_degs"] = float(np.degrees(imp))

            #  Por eixo. Num ensaio combinado o modulo mistura rolagem e guinada,
            #  eixos com autoridades que diferem por dezenas de vezes: 15 rad/s2 e
            #  pouco em rolagem e muito em guinada. O equivalente em N.m
            #  (I x alfa) e a grandeza fisica que a monografia declara.
            for eixo in eixos_com_torque(perfil, col):
                serie = col.get(COLUNAS[eixo]["torque"])
                if serie is None:
                    continue
                ok_e = np.isfinite(serie)
                if not ok_e.any():
                    continue
                pico = _pico(serie)
                res["torque"][f"{eixo}_pico_rads2"] = pico
                if inercia_kgm2 and inercia_kgm2.get(eixo):
                    res["torque"][f"{eixo}_pico_nm"] = pico * float(inercia_kgm2[eixo])
                if not chirp:
                    imp_e = float(np.trapezoid(np.abs(serie[ok_e]), grade[ok_e]))
                    res["torque"][f"{eixo}_impulso_degs"] = float(np.degrees(imp_e))

    return res


def achatar(m, prefixo=""):
    """Achata o dicionario aninhado em {coluna: valor} para o consolidado.csv."""
    plano = {}
    for k, v in m.items():
        nome = f"{prefixo}{k}"
        if isinstance(v, dict):
            plano.update(achatar(v, prefixo=f"{nome}."))
        elif isinstance(v, (list, tuple)):
            continue          # janelas ficam so no JSON, nao poluem a tabela
        else:
            plano[nome] = v
    return plano
