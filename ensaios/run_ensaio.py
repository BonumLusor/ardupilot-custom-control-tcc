#!/usr/bin/env python3
"""Orquestra um ensaio completo: sobe o SITL, voa, injeta vento, analisa.

Um ensaio = um cenario x um controlador. Cada corrida e independente e deixa a
propria pasta de resultado com tudo que a torna auditavel depois: o .BIN, os dois
YAMLs, o dump de parametros, as metricas, as series, o painel e o GIF. O
resumo.txt abre com o veiculo (frame, massa, braco, inercia) e os ganhos que
voaram em cada eixo.

Os cenarios ficam em uma pasta por eixo — cenarios/rolagem/, cenarios/arfagem/,
cenarios/guinada/ — e uma com os tres ao mesmo tempo, cenarios/rolagem_arfagem_guinada/,
todas com os mesmos ensaios (nominal, torque_pulso, torque_degrau, torque_chirp). O
resultado repete a organizacao: resultados/guinada/torque_pulso/nativo/<carimbo>/.
O SITL roda sem janela; a saida do binario fica no sitl.log de cada ensaio.

    ./run_ensaio.py                                  # TODOS os cenarios x TODOS os controladores
    ./run_ensaio.py --cenario guinada                # uma pasta inteira
    ./run_ensaio.py --cenario guinada/torque_pulso --controlador nativo
    ./run_ensaio.py --cenario torque_pulso           # o pulso em todas as pastas
    ./run_ensaio.py --analisar resultados --sem-gif  # reanalisa tudo o que houver la

IMPORTANTE: roda em SITL PURO, sem Gazebo. Com --model JSON o SIM_JSON zera o
vento internamente e nenhum SIM_WIND_* teria efeito fisico.
"""

from __future__ import annotations

import argparse
import datetime as dt
import math
import os
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

#  Todo tempo decorrido neste arquivo usa time.monotonic(), nunca time.time().
#  O relogio de parede do WSL ressincroniza e pode ANDAR PARA TRAS no meio de uma
#  corrida; com ele, um "tempo decorrido" vira negativo e o campo uint32 de
#  time_boot_ms do SET_ATTITUDE_TARGET estoura, derrubando o ensaio no ar. Os
#  laços de timeout tambem esperariam a mais. O carimbo da pasta de resultado
#  continua vindo de datetime.now(), que e data de calendario e nao intervalo.
RAIZ = Path(__file__).resolve().parent
PROJETO = RAIZ.parent
ARDUPILOT = PROJETO / "ardupilot"
LOGS_SITL = ARDUPILOT / "ArduCopter" / "logs"
CENARIOS = RAIZ / "cenarios"
CONTROLADORES = RAIZ / "controladores"
RESULTADOS = RAIZ / "resultados"
# Ajustes de linha de comando para voos de MEDICAO do procedimento (Cap. 3), nao
# para os ensaios das tabelas: --param soma parametros com a palavra final,
# --fila-antiga volta a ler a mensagem mais antiga da fila no laco do perfil (o
# comportamento de antes da correcao) e --destino tira os voos da arvore das tabelas.
PARAMS_EXTRA = {}
FILA_FRESCA = True

# Os cenarios ficam em uma pasta por eixo exercitado, e os ensaios de cada pasta
# tem os mesmos nomes. A ordem aqui e a de execucao e de listagem; pastas e
# ensaios novos entram depois, em ordem alfabetica.
ORDEM_GRUPOS = ("rolagem", "arfagem", "guinada", "rolagem_arfagem_guinada",
                "ruido/rnd10", "ruido/rnd20", "ruido/rnd40", "ruido/rnd80", "vento",
                "grande/30graus", "grande/45graus", "saturacao/guinada",
                "erro_modelo/empuxo_fraco", "erro_modelo/braco_longo",
                "erro_modelo/braco_curto", "falha_motor")
ORDEM_ENSAIOS = ("nominal", "combinado", "torque_pulso", "torque_degrau", "torque_chirp",
                 "torque_estocastico", "leve", "severa")

sys.path.insert(0, str(RAIZ))

# --------------------------------------------------------------------------
# preambulo do SITL: sem isto o vento nao chega a aeronave como foi comandado
# --------------------------------------------------------------------------
PARAMS_BASE = {
    # o injetor de vento e um script Lua
    "SCR_ENABLE": 1,
    # SIM_WIND_T=1 (NO_LIMIT): o padrao e a lei de raiz quadrada, que a 5 m
    # entregaria so ~29% do vento comandado
    "SIM_WIND_T": 1,
    # piso do firmware (MAX(wind_change_tc, 0.1)); o padrao 5 s destruiria a rajada
    "SIM_WIND_TC": 0.1,
    # a turbulencia IIR nativa fica desligada — a nossa e a do vento.lua
    "SIM_WIND_TURB": 0,
    # Logging: tudo que a analise le, e nada do que ela nao le.
    # bits 0,1,2,3,4,5,6,7,9,10,12,13,17 = atitude rapida e media, GPS, PM,
    # CTUN, NTUN, entrada RC, IMU, bateria, saida RC, PID, bussola, motores.
    # Ficam DE FORA os bits 18/19/21 (Fast IMU, Raw IMU, notch harmonico), que
    # sozinhos levam o log de dezenas de MB para mais de 1 GB por ensaio.
    "LOG_BITMASK": 145151,
    # o log comeca no armamento, que e exatamente a janela do ensaio: o perfil de
    # vento tambem so dispara ali
    "LOG_DISARMED": 0,
    #  Frame X, explicito. O -f X ja carrega isto pelo copter-X.parm; repetir aqui
    #  garante que firmware e fisica concordem mesmo que os defaults do ArduPilot
    #  mudem, e deixa o valor registrado no params.parm do ensaio.
    "FRAME_CLASS": 1,
    "FRAME_TYPE": 1,
    #  Modelador de entrada rapido (padrao 0.1 s). O modelador transforma o degrau
    #  comandado numa rampa suave ANTES da realimentacao ver qualquer coisa, e e
    #  feedforward identico em todos os controladores. Com o padrao, a aeronave
    #  segue o alvo modelado com erro de pico de 0,5 grau e as metricas do degrau
    #  passam a caracterizar o modelador em vez do controlador. Encurtando a
    #  constante, o comando chega mais perto de um degrau de verdade e a malha
    #  fechada tem de trabalhar.
    "ATC_INPUT_TC": 0.02,
}

# type_mask do SET_ATTITUDE_TARGET: ignora as tres taxas de corpo, usa a atitude
# e o campo de thrust. Com GUID_OPTIONS no padrao, thrust=0.5 e taxa de subida 0.
TYPEMASK_ATITUDE = 0b00000111
THRUST_NEUTRO = 0.5

# eixos que o perfil de voo sabe degrauzar, com a chave do YAML de cada um
# (voo.degrau_roll_deg, voo.degrau_pitch_deg, voo.degrau_yaw_deg)
EIXOS_DEGRAU = ("roll", "pitch", "yaw")
NOMES_EIXO = {"roll": "rolagem", "pitch": "arfagem", "yaw": "guinada"}


def log(msg):
    print(f"[ensaio] {msg}", flush=True)


def _rel(caminho):
    """Caminho relativo ao projeto, para o log; absoluto se estiver fora dele."""
    try:
        return Path(caminho).relative_to(PROJETO).as_posix()
    except ValueError:
        return str(caminho)


# --------------------------------------------------------------------------
# catalogos
# --------------------------------------------------------------------------

def carregar_yaml(caminho):
    with open(caminho, encoding="utf-8") as f:
        return yaml.safe_load(f)


def params_vento_inicial(vento):
    """Parametros WND_* como devem ficar ANTES do ensaio comecar.

    O vento fica desligado (``WND_MODE=0``) durante a decolagem, e o gatilho vai
    para o modo manual (``WND_T0=-1``), em que o perfil zera o relogio quando o
    ``WND_MODE`` deixa de ser zero. Quem faz isso e o ``voar_perfil``, no instante
    em que passa a comandar atitude.
    """
    p = dict(vento)
    p["WND_MODE"] = 0
    p["WND_T0"] = -1
    return p


def degraus_do_voo(voo):
    """Amplitude do degrau comandado em cada eixo, em graus, a partir do YAML.

    Eixo sem a chave no YAML fica em zero: os cenarios antigos, so de rolagem,
    continuam valendo como estao.
    """
    return {e: float((voo or {}).get(f"degrau_{e}_deg") or 0.0) for e in EIXOS_DEGRAU}


def descrever_degraus(degraus):
    """'10 deg em rolagem + 30 deg em guinada', ou None se nao ha degrau."""
    partes = [f"{a:g} deg em {NOMES_EIXO[e]}" for e, a in degraus.items() if a]
    return " + ".join(partes) if partes else None


def _ordem_cenario(id_cenario):
    """Chave de ordenacao: grupos e ensaios na ordem da tabela do README."""
    partes = id_cenario.split("/")
    grupo, nome = ("/".join(partes[:-1]), partes[-1])
    pos_g = ORDEM_GRUPOS.index(grupo) if grupo in ORDEM_GRUPOS else len(ORDEM_GRUPOS)
    pos_n = ORDEM_ENSAIOS.index(nome) if nome in ORDEM_ENSAIOS else len(ORDEM_ENSAIOS)
    return (pos_g, grupo, pos_n, nome)


def listar_cenarios():
    """Identificadores de todos os cenarios: o caminho em cenarios/, sem .yaml.

    A pasta diz o eixo (rolagem/, guinada/, rolagem_arfagem_guinada/...) e o arquivo diz o
    ensaio (nominal, torque_pulso, ...): 'guinada/torque_pulso'. E esse mesmo
    caminho que organiza resultados/, e e ele que vai na coluna "cenario" do
    consolidado.
    """
    ids = [p.relative_to(CENARIOS).with_suffix("").as_posix()
           for p in CENARIOS.rglob("*.yaml")]
    return sorted(ids, key=_ordem_cenario)


def selecionar_cenarios(seletores=None):
    """Cenarios que casam com algum seletor, na ordem de ``listar_cenarios``.

    Um seletor pode ser o cenario inteiro ('guinada/torque_pulso'), uma pasta
    ('guinada': todos dela) ou o nome do ensaio ('torque_pulso': o pulso de
    todas as pastas). Sem seletor, ou com 'todos', vem tudo.
    """
    todos = listar_cenarios()
    if not seletores or "todos" in seletores:
        return todos
    escolhidos = set()
    for s in seletores:
        s = s.strip("/").removesuffix(".yaml")
        achados = [c for c in todos
                   if c == s or c.startswith(s + "/") or c.split("/")[-1] == s]
        if not achados:
            raise SystemExit(
                f"cenario '{s}' nao existe. Disponiveis (pasta/ensaio): "
                f"{', '.join(todos)}")
        escolhidos.update(achados)
    return [c for c in todos if c in escolhidos]


def listar_controladores():
    """Controladores de controladores/, com o nativo — a linha de base — primeiro."""
    nomes = sorted(p.stem for p in CONTROLADORES.glob("*.yaml"))
    return sorted(nomes, key=lambda n: (n != "nativo", n))


def selecionar_controladores(nomes=None):
    """Controladores pedidos, na ordem em que vieram; sem nome, ou 'todos', todos."""
    todos = listar_controladores()
    if not nomes or "todos" in nomes:
        return todos
    for n in nomes:
        if n not in todos:
            raise SystemExit(
                f"controlador '{n}' nao existe. Disponiveis: {', '.join(todos)}\n"
                f"(cc_lqi, cc_eso, cc_lqi_mesmo_ruido e nativo_lqi saem de "
                f"modelagem/projeto.py --escrever-yaml)")
    return list(dict.fromkeys(nomes))


def carregar_cenario(id_cenario):
    p = CENARIOS / f"{id_cenario}.yaml"
    if not p.exists():
        raise SystemExit(f"cenario '{id_cenario}' nao existe. Disponiveis: "
                         f"{', '.join(listar_cenarios())}")
    return carregar_yaml(p), p


def carregar_controlador(nome):
    p = CONTROLADORES / f"{nome}.yaml"
    if not p.exists():
        selecionar_controladores([nome])      # levanta o erro com a lista
    return carregar_yaml(p), p


def id_cenario_da_pasta(pasta, cen=None):
    """'guinada/torque_pulso' de uma pasta resultados/<cenario>/<controlador>/<carimbo>.

    Sai do caminho, que e onde o rodar() o pos. Fora de resultados/, ou numa
    pasta gravada antes da organizacao por eixo, cai no slug do YAML copiado ou
    no nome da pasta.
    """
    pasta = Path(pasta).resolve()
    bases = [RESULTADOS.resolve()] + [a for a in pasta.parents if a.name == "resultados"]
    for base in bases:
        try:
            partes = pasta.relative_to(base).parts
        except ValueError:
            continue
        if len(partes) >= 3:
            return "/".join(partes[:-2])
    return (cen or {}).get("slug") or pasta.parent.parent.name


# --------------------------------------------------------------------------
# SITL
# --------------------------------------------------------------------------

class Sitl:
    """O SITL de um ensaio: o sim_vehicle.py que subimos e o binario que ele roda.

    O binario nao e filho direto do sim_vehicle.py — passa pelo
    run_in_terminal_window.sh —, entao e achado pela variavel ENSAIO_SITL, que
    ele herda e que mais ninguem tem. E isso que permite dizer se ELE esta vivo,
    e nao "algum arducopter": a busca antiga pelo nome casava "arducopter
    --model", e a linha de comando antes do reboot e "arducopter -w --model" —
    um SITL vivo e mudo era dado como morto.
    """

    def __init__(self, proc, marca, instancia, saida):
        self.proc = proc
        self.marca = marca
        self.instancia = instancia
        self.saida = saida
        self.visto = False         # o binario ja apareceu alguma vez?

    def processos(self):
        """Processos vivos do binario deste SITL."""
        return processos_sitl(lambda amb: amb.get("ENSAIO_SITL") == self.marca)

    def vivo(self):
        vivo = bool(self.processos())
        self.visto = self.visto or vivo
        return vivo

    def reiniciou(self):
        """True se o binario ja passou por um reboot.

        O SITL se reexecuta no reboot (execv, no mesmo processo) e tira o -w da
        propria linha de comando (HAL_SITL_Class.cpp), para nao apagar os
        parametros de novo. Como o ensaio sempre sobe com -w, a ausencia dele e
        prova exata de reboot — comparar o tempo desde o boot nao serve: antes
        da primeira conexao o relogio simulado nem anda, e os dois tempos saem
        parecidos.
        """
        procs = self.processos()
        try:
            return bool(procs) and all("-w" not in p.cmdline() for p in procs)
        except Exception:
            return False

    def verificar(self):
        """SystemExit, com o fim da saida do SITL, se ele morreu.

        Logo depois de subir, o binario ainda nao existe: o sim_vehicle.py confere
        a compilacao antes de lanca-lo. So conta como morte o binario sumir
        depois de ter aparecido, ou o sim_vehicle.py terminar sem lanca-lo. O
        reboot nao conta: o SITL reinicia com execv, no mesmo processo.
        """
        if self.vivo():
            return
        if self.visto:
            raise SystemExit("o binario do SITL morreu. Fim da saida dele (sitl.log):\n"
                             + self.ultimas_linhas())
        if self.proc.poll() is not None:
            raise SystemExit("o sim_vehicle.py terminou sem subir o SITL. Fim da saida "
                             "(sitl.log):\n" + self.ultimas_linhas())

    def ultimas_linhas(self, n=12):
        """Fim da saida do SITL (sim_vehicle.py e binario), para as mensagens de erro."""
        if not self.saida:
            return ""
        try:
            linhas = Path(self.saida).read_text(encoding="utf-8", errors="replace").splitlines()
        except OSError:
            return ""
        linhas = [l for l in linhas if l.strip() and not l.startswith("Embedding file")]
        return "\n".join("    " + l for l in linhas[-n:])

    def estado(self):
        """Uma linha sobre o binario: vivo ou nao, PID, e se ja reiniciou."""
        procs = self.processos()
        if not procs:
            return "binario do SITL nao esta mais rodando"
        partes = []
        for p in procs:
            try:
                reiniciou = "-w" not in p.cmdline()
                partes.append(f"PID {p.pid} {p.status()}, "
                              f"{'ja reiniciou' if reiniciou else 'ainda sem reboot'}")
            except Exception:              # o processo pode sair entre as chamadas
                continue
        return "binario vivo: " + "; ".join(partes) if partes else "binario saindo"


def processos_sitl(filtro):
    """Processos 'arducopter' vivos cujo ambiente passa no ``filtro``."""
    import psutil
    achados = []
    for p in psutil.process_iter(["name"]):
        try:
            if p.info["name"] != "arducopter" or p.status() == psutil.STATUS_ZOMBIE:
                continue
            if filtro(p.environ()):
                achados.append(p)
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            continue
    return achados


def porta_ocupada(porta):
    """True se ja ha alguem escutando na porta TCP local."""
    import socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(1.0)
        return s.connect_ex(("127.0.0.1", porta)) == 0


def subir_sitl(speedup, instancia=0, limpar_eeprom=True, saida=None):
    """Sobe o sim_vehicle.py sem janela e devolve o ``Sitl``.

    Sem MAVProxy nao ha para quem encaminhar telemetria: o ``--out`` do
    sim_vehicle.py e argumento DO MAVProxy. Quem escuta e o proprio binario SITL,
    em TCP 5760 (+10 por instancia) — e e nesse endereco que nos conectamos.
    """
    porta = porta_sitl(instancia)
    # SITL esquecido por um lote interrompido a forca: e nosso (tem a marca) e
    # e da mesma instancia, entao pode ir. Sem isso o binario novo morre no bind
    # da porta, e o erro parece do ensaio novo.
    orfaos = processos_sitl(lambda amb: "ENSAIO_SITL" in amb
                            and amb.get("ENSAIO_INSTANCIA") == str(instancia))
    for p in orfaos:
        log(f"encerrando SITL esquecido de um ensaio anterior (PID {p.pid})")
        p.kill()
    if orfaos:
        time.sleep(1)
    if porta_ocupada(porta):
        raise SystemExit(
            f"ja ha algo escutando na porta {porta} — outro SITL aberto (o "
            f"run-sitl.sh, por exemplo). Feche-o, ou use --instancia 1.")

    cmd = [
        sys.executable,
        str(ARDUPILOT / "Tools" / "autotest" / "sim_vehicle.py"),
        "-v", "ArduCopter",
        #  Frame X: carrega copter-X.parm (FRAME_TYPE 1) e roda --model X, que o
        #  SITL casa com quad_x_motors. E a mesma configuracao de motores do
        #  ambiente Gazebo do projeto, entao os dois medem o mesmo veiculo.
        "-f", "X",
        "--no-mavproxy",
        "-I", str(instancia),
        "--speedup", str(speedup),
    ]
    if limpar_eeprom:
        cmd.append("-w")

    log(f"subindo SITL: speedup={speedup} instancia={instancia} (TCP {porta})")
    marca = f"{os.getpid()}-{time.monotonic_ns()}"
    env = dict(os.environ)
    env["PATH"] = f"{ARDUPILOT / 'Tools' / 'autotest'}:{env.get('PATH', '')}"
    env["ENSAIO_SITL"] = marca
    env["ENSAIO_INSTANCIA"] = str(instancia)
    #  Sem janela: o run_in_terminal_window.sh do ArduPilot usa o "terminal" de
    #  SITL_RITW_TERMINAL antes de procurar xterm, e o nosso roda o binario em
    #  segundo plano com a saida no sitl.log (ver sitl_sem_janela.sh). A saida do
    #  binario era justamente o que faltava para diagnosticar quando ele morria.
    env["SITL_RITW_TERMINAL"] = str(RAIZ / "sitl_sem_janela.sh")
    env["SITL_SAIDA"] = str(saida) if saida else os.devnull

    # A saida vai para ARQUIVO, nunca para um PIPE. Com stdout=PIPE e ninguem
    # lendo, o SITL trava assim que enche os 64 KB do buffer do pipe — e ele fala
    # bastante a cada reboot, que e exatamente quando isso acontecia. O arquivo e
    # aberto em modo de acrescimo porque o binario escreve no mesmo sitl.log.
    destino = open(saida, "a") if saida else subprocess.DEVNULL
    proc = subprocess.Popen(
        cmd, cwd=str(ARDUPILOT / "ArduCopter"), env=env,
        stdout=destino, stderr=subprocess.STDOUT,
        text=True, start_new_session=True)
    return Sitl(proc, marca, instancia, saida)


def derrubar_sitl(sitl):
    """Encerra o SITL e so volta quando o binario saiu de fato.

    O proximo ensaio usa a mesma porta: se o binario deste ainda estiver de pe,
    o novo morre no bind.
    """
    if sitl is None:
        return
    proc = sitl.proc
    if proc.poll() is None:
        log("encerrando SITL")
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            proc.wait(timeout=20)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
    fim = time.monotonic() + 10
    restos = sitl.processos()
    while restos and time.monotonic() < fim:
        time.sleep(0.2)
        restos = sitl.processos()
    for p in restos:
        log(f"o binario do SITL nao saiu com SIGTERM; forcando (PID {p.pid})")
        try:
            p.kill()
        except Exception:
            pass


# --------------------------------------------------------------------------
# voo
# --------------------------------------------------------------------------

def porta_sitl(instancia=0):
    """Porta TCP em que o binario SITL escuta."""
    return 5760 + 10 * instancia


def conectar(instancia=0, timeout=180, sitl=None):
    """Conecta ao SITL por TCP, reabrindo o socket ate o firmware aceitar.

    Depois de um reboot o SITL derruba o socket e volta a escutar, entao insistir
    na conexao — e nao apenas no heartbeat — e o que faz o reinicio funcionar.
    Se o binario do SITL tiver morrido, desiste na hora, com o fim da saida dele
    na mensagem: insistir 3 minutos num processo inexistente so atrasa o
    diagnostico.
    """
    from pymavlink import mavutil
    alvo = f"tcp:127.0.0.1:{porta_sitl(instancia)}"
    log(f"conectando em {alvo} (ate {timeout}s)")
    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        if sitl is not None:
            sitl.verificar()
        try:
            m = mavutil.mavlink_connection(alvo, retries=1)
        except Exception:
            time.sleep(2)
            continue
        if m.wait_heartbeat(timeout=10) is not None:
            log(f"conectado: sistema {m.target_system}, componente {m.target_component}")
            pedir_streams(m)
            return m
        m.close()
        if sitl is not None:
            log(f"sem heartbeat em 10 s; {sitl.estado()}")
        time.sleep(2)
    detalhe = f"\nFim da saida do SITL (sitl.log):\n{sitl.ultimas_linhas()}" if sitl else ""
    raise SystemExit(f"nao consegui heartbeat do SITL em {timeout} s.{detalhe}")


def pedir_streams(m, hz=10):
    """Pede a telemetria ao SITL.

    Quem normalmente faz isso e o MAVProxy. Como rodamos com --no-mavproxy,
    ninguem pede — e sem pedido o SITL nao envia ATTITUDE, GLOBAL_POSITION_INT
    nem EKF_STATUS_REPORT, e o ensaio fica esperando mensagens que nunca vem.
    """
    from pymavlink import mavutil
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, hz, 1)
    time.sleep(0.5)


def set_param(m, nome, valor, tentativas=5):
    """Escreve um parametro e confirma pelo PARAM_VALUE de volta."""
    from pymavlink import mavutil
    for _ in range(tentativas):
        m.mav.param_set_send(m.target_system, m.target_component,
                             nome.encode(), float(valor),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        fim = time.monotonic() + 2.0
        while time.monotonic() < fim:
            msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if msg and msg.param_id.strip("\x00") == nome:
                # o parametro e float32: 5856.64 volta como 5856.6401, entao a
                # tolerancia cresce com o valor
                if abs(msg.param_value - float(valor)) < max(1e-4, 1e-6 * abs(float(valor))):
                    return True
                break
    return False


def esperar_param(m, nome, timeout=60):
    """Espera um parametro EXISTIR no veiculo.

    Os WND_* nao existem ate o vento.lua rodar e registrar a tabela dinamica, o
    que leva alguns segundos depois do reboot. Escrever antes disso falha nos 16
    de uma vez, e a mensagem de erro acusa o symlink ou o SCR_ENABLE — que nao
    sao o problema. Esperar o parametro aparecer elimina a corrida.
    """
    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        m.mav.param_request_read_send(m.target_system, m.target_component,
                                      nome.encode(), -1)
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
        if msg and msg.param_id.strip("\x00") == nome:
            return True
    return False


def aplicar_params(m, params, rotulo=""):
    faltaram = []
    for nome, valor in params.items():
        if not set_param(m, nome, valor):
            faltaram.append(nome)
    if faltaram:
        raise SystemExit(
            f"nao consegui aplicar {rotulo}: {', '.join(faltaram)}.\n"
            "Se sao os WND_*, o vento.lua nao carregou — confira o symlink "
            "ardupilot/ArduCopter/scripts e se SCR_ENABLE=1 sobreviveu ao reboot.")
    log(f"aplicados {len(params)} parametros {rotulo}")


def _esperar_sitl_fechar(m, timeout=5.0):
    """Le e descarta o que chegar ate o SITL fechar a conexao, ou ate o timeout.

    O SITL fecha a conexao ao se reexecutar no reboot. Esperar por isso, em vez
    de fechar do nosso lado, e o que evita o travamento descrito em
    ``reiniciar``. Devolve True se o SITL fechou.
    """
    import select
    sock = m.port
    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        prontos, _, _ = select.select([sock], [], [], 0.1)
        if not prontos:
            continue
        try:
            dados = sock.recv(65536)
        except (BlockingIOError, InterruptedError):
            continue
        except OSError:
            return True                 # conexao derrubada pelo outro lado
        if not dados:
            return True                 # EOF: o SITL fechou
    return False


def reiniciar(m, instancia=0, sitl=None, tentativas=3):
    """Reinicia o firmware, reconecta e confere que o reboot aconteceu.

    O REBOOT DO SITL TRAVA SE A CONEXAO CAI NO MEIO DELE. Antes de reiniciar, o
    AP_Vehicle::reboot() chama hal.serial(0)->flush() para garantir que o
    COMMAND_ACK saia (AP_Vehicle.cpp). O UARTDriver::_flush() do SITL repete
    _timer_tick() ate esvaziar o buffer ou passar 1 s de tempo SIMULADO — mas
    nada nesse laco faz o tempo simulado andar. Com a conexao TCP ja fechada, o
    _timer_tick() nao escreve nada, o buffer nunca esvazia e o laco nunca acaba:
    o SITL fica a 100% de CPU, sem reiniciar e ignorando SIGTERM.

    O codigo antigo fechava a conexao logo depois de mandar o pedido, e caia
    nessa corrida de vez em quando (reproduzido: ~30% dos reboots). Conforme o
    lado que ganhava, o SITL travava ("o processo do SITL morreu", porque a
    busca antiga nao reconhecia o processo travado) ou descartava o pedido e nao
    reiniciava ("os parametros WND_* nunca apareceram").

    Por isso: espera o COMMAND_ACK, depois le ate o PROPRIO SITL fechar a
    conexao — o que ele faz ao se reexecutar — e so entao fecha do nosso lado.
    Por fim confere, pela linha de comando do binario, que o reboot aconteceu.
    """
    from pymavlink import mavutil
    reboot = mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
    for tentativa in range(1, tentativas + 1):
        log("reiniciando o firmware" + (f" (tentativa {tentativa})" if tentativa > 1 else ""))
        m.mav.command_long_send(m.target_system, m.target_component, reboot,
                                0, 1, 0, 0, 0, 0, 0, 0)
        resultado = None
        fim = time.monotonic() + 3
        while time.monotonic() < fim:
            ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=1)
            if ack is not None and ack.command == reboot:
                resultado = ack.result
                break
        if resultado is None:
            log("o SITL nao confirmou o pedido de reboot (sem COMMAND_ACK)")
        elif resultado != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            log(f"o SITL recusou o reboot (COMMAND_ACK result={resultado})")
        fechou = _esperar_sitl_fechar(m)
        try:
            m.close()
        except Exception:
            pass
        # fechada pelo SITL, a porta ja reabre em seguida; sem isso, da tempo
        # para ele cair e voltar a escutar antes de insistir
        time.sleep(2 if fechou else 8)
        m = conectar(instancia, sitl=sitl)
        if sitl is None or sitl.reiniciou():
            return m
        log("o firmware NAO reiniciou: o binario ainda roda com a linha de comando "
            "do primeiro boot (-w)")
    raise SystemExit(f"o firmware nao reiniciou depois de {tentativas} pedidos de reboot")


def esperar_pronto(m, timeout=300):
    """Espera fix 3D do GPS e convergencia do EKF."""
    from pymavlink import mavutil
    PRECISA = (mavutil.mavlink.ESTIMATOR_ATTITUDE
               | mavutil.mavlink.ESTIMATOR_VELOCITY_HORIZ
               | mavutil.mavlink.ESTIMATOR_POS_HORIZ_REL
               | mavutil.mavlink.ESTIMATOR_POS_HORIZ_ABS
               | mavutil.mavlink.ESTIMATOR_POS_VERT_ABS)
    log("esperando GPS 3D e EKF")
    fim = time.monotonic() + timeout
    gps_ok = ekf_ok = False
    ultimo_aviso = 0.0
    while time.monotonic() < fim:
        msg = m.recv_match(type=["GPS_RAW_INT", "EKF_STATUS_REPORT"],
                           blocking=True, timeout=3)
        if msg is None:
            pedir_streams(m)             # o pedido expira; renova
            continue
        if msg.get_type() == "GPS_RAW_INT":
            gps_ok = msg.fix_type >= 3
        else:
            ekf_ok = (msg.flags & PRECISA) == PRECISA
        if gps_ok and ekf_ok:
            log("GPS 3D e EKF prontos")
            time.sleep(1)
            return True
        if time.monotonic() - ultimo_aviso > 15:
            log(f"  ainda esperando: gps={gps_ok} ekf={ekf_ok}")
            ultimo_aviso = time.monotonic()
    raise SystemExit(f"o veiculo nunca ficou pronto (gps={gps_ok} ekf={ekf_ok})")


def armar_e_decolar(m, altitude, timeout=180, rc_override=None):
    from pymavlink import mavutil
    log(f"GUIDED, armar, decolar para {altitude} m")
    m.set_mode("GUIDED")
    aplicar_rc_override(m, rc_override)
    time.sleep(1)

    for tentativa in range(20):
        aplicar_rc_override(m, rc_override)
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM \
                and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            break
        time.sleep(2)
    else:
        raise SystemExit("o veiculo recusou armar — veja os pre-arm no log")
    log("armado")

    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, altitude)

    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        msg = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
        if msg and msg.relative_alt / 1000.0 >= altitude * 0.95:
            log(f"altitude atingida: {msg.relative_alt / 1000.0:.1f} m")
            return True
    raise SystemExit("nao alcancou a altitude de decolagem")


def aplicar_rc_override(m, mapa):
    """Forca canais de RC por MAVLink.

    Existe por um motivo especifico: o ``AC_CustomControl`` NAO entra em operacao
    so por ter ``CC_TYPE`` diferente de zero. Ele so assume quando uma chave
    auxiliar de RC vai para HIGH (``RC_Channel_Copter.cpp:671``, opcao 109). Sem
    isso o ensaio roda com o PID nativo enquanto os parametros dizem o contrario —
    silenciosamente, que e o pior jeito de errar uma comparacao.

    Os overrides expiram (RC_OVERRIDE_TIME), entao sao reenviados ao longo do voo.
    Canais com 0 ficam como estao.
    """
    if not mapa:
        return
    canais = [0] * 18
    for ch, valor in mapa.items():
        i = int(ch) - 1
        if 0 <= i < 18:
            canais[i] = int(valor)
    m.mav.rc_channels_override_send(m.target_system, m.target_component, *canais)


def tempo_sim(m, timeout=5.0, fresco=False):
    """Tempo de simulacao do veiculo, em segundos, lido do proprio veiculo.

    Com ``fresco=True`` a fila de recepcao e esvaziada antes da leitura. Isso
    importa ao ESTABELECER o zero do perfil: o ``set_param`` que dispara o vento
    fica ~1 s ocupado esperando o ACK, e nesse tempo o veiculo enfileira mensagens
    de atitude. Ler a primeira da fila daria um zero atrasado, e o perfil inteiro
    herdaria o deslocamento — foi por isso que o degrau pedido em t=6 s aparecia
    em tp=7 s no log.

    Toda a temporizacao do ensaio se apoia neste relogio, e nao no relogio de
    parede dividido pelo speedup. O motivo e concreto: com ``--speedup 5`` o SITL
    nao entrega 5x de fato (medido: ~1,4x sob carga de log), e um perfil marcado
    no relogio de parede terminava cedo demais — o pouso comecava no meio do
    ensaio e o controlador de posicao mandava 30 graus de rolagem para frear a
    deriva, que entrava no log como se fosse resposta ao disturbio.
    """
    if fresco:
        while m.recv_match(blocking=False) is not None:
            pass
    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        msg = m.recv_match(type="ATTITUDE", blocking=True, timeout=1)
        if msg is not None:
            return msg.time_boot_ms / 1000.0
    return None


def esperar_sim(m, segundos, timeout_parede=300):
    """Espera ``segundos`` de tempo SIMULADO."""
    t0 = tempo_sim(m)
    if t0 is None:
        time.sleep(segundos)
        return
    fim_parede = time.monotonic() + timeout_parede
    while time.monotonic() < fim_parede:
        t = tempo_sim(m)
        if t is None or t - t0 >= segundos:
            return


def _quaternion(roll_deg, pitch_deg, yaw_deg):
    r, p, y = (math.radians(v) / 2.0 for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return [cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy]


def voar_perfil(m, voo, vento, speedup, taxa_hz=10.0, rc_override=None):
    """Transmite atitude comandada durante toda a janela do ensaio.

    Comandar atitude o tempo todo — e nao deixar o veiculo em manutencao de
    posicao — e o que isola a malha de ATITUDE. Em manutencao de posicao, o
    controlador de posicao reagiria ao vento e a medida deixaria de ser da malha
    que se quer parametrizar.

    O PERFIL DE VENTO E DISPARADO AQUI, e nao no armamento. A diferenca e
    decisiva: com o vento ligado durante a decolagem, o veiculo inclina ~9 graus
    em manutencao de posicao so para se segurar contra a rajada media, e essa
    inclinacao entra no log como se fosse um degrau de referencia. Ligando o
    vento junto com o streaming, a janela do ensaio comeca limpa e o unico degrau
    do log e o que nos comandamos.

    O degrau pode ser em rolagem, em guinada ou nos dois ao mesmo tempo
    (``voo.degrau_roll_deg``, ``voo.degrau_yaw_deg``); todos compartilham
    ``t_degrau_s`` e ``t_retorno_s``. O de guinada e somado ao rumo travado.
    """
    duracao = float(voo["duracao_s"])
    t_degrau = float(voo.get("t_degrau_s") or 0.0)
    degraus = degraus_do_voo(voo)
    zero = {e: 0.0 for e in EIXOS_DEGRAU}
    # t_retorno_s ausente = mantem o degrau ate o fim (comportamento antigo)
    t_retorno = voo.get("t_retorno_s")
    t_retorno = float(t_retorno) if t_retorno is not None else None

    # o rumo e travado no valor atual: um rumo solto giraria o eixo do degrau em
    # relacao a direcao do vento no meio do ensaio. O degrau de guinada e
    # relativo a ele, e o retorno volta exatamente a ele.
    att = m.recv_match(type="ATTITUDE", blocking=True, timeout=5)
    yaw0 = math.degrees(att.yaw) if att else 0.0
    desc = descrever_degraus(degraus)
    perfil_txt = f"degrau de {desc} em t={t_degrau}s" if desc else "sem degrau"
    if desc and t_retorno is not None:
        perfil_txt += f", retorno em t={t_retorno}s"
    log(f"streaming de atitude por {duracao}s ({perfil_txt}, "
        f"rumo travado em {yaw0:.1f} deg)")

    # dispara o injetor: com WND_T0 < 0 o vento.lua zera o relogio do perfil na
    # transicao de WND_MODE de 0 para diferente de zero
    modo = int(vento.get("WND_MODE", 0))
    if modo and not set_param(m, "WND_MODE", modo):
        raise SystemExit("nao consegui disparar o perfil de vento (WND_MODE)")
    log(f"perfil de vento disparado (WND_MODE={modo})")

    # ritmo de envio no relogio de parede; o PERFIL, porem, e marcado no relogio
    # de simulacao lido do veiculo
    dt_envio = 1.0 / (taxa_hz * max(speedup, 1))
    # o zero do perfil e lido DEPOIS de disparar o vento e com a fila esvaziada,
    # senao o degrau sai deslocado em relacao ao Tp gravado no log
    t0_sim = tempo_sim(m, fresco=True)
    if t0_sim is None:
        raise SystemExit("sem ATTITUDE do veiculo — nao da para marcar o perfil")

    t0_parede = time.monotonic()
    teto_parede = t0_parede + 60 + duracao * 3     # rede de seguranca
    ultimo_aviso = 0.0
    degrau_avisado = False
    retorno_avisado = False
    t_sim = 0.0

    while t_sim < duracao and time.monotonic() < teto_parede:
        # SEMPRE a leitura mais recente. Lendo a primeira da fila, o atraso
        # cresce ao longo do voo: o laco consome ATTITUDE um pouco mais devagar
        # do que o veiculo os produz, a fila acumula, e o relogio do orquestrador
        # fica para tras. Medido antes da correcao: o degrau atrasava 0,5 s e o
        # retorno, 1,8 s — um erro de TAXA, nao um deslocamento fixo.
        agora = tempo_sim(m, timeout=2.0, fresco=FILA_FRESCA)
        if agora is not None:
            t_sim = agora - t0_sim

        # Tres atos no mesmo voo: nivelado, degrau, e de volta a nivelado ANTES
        # do disturbio. Manter o degrau ate o fim faria o disturbio incidir sobre
        # uma aeronave inclinada e acelerando de lado — ponto de operacao dificil
        # de defender num ensaio de rejeicao.
        if t_retorno is not None and t_sim >= t_retorno:
            alvo = zero
            if desc and not retorno_avisado:
                log(f"  retorno em t_sim={t_sim:.1f}s")
                retorno_avisado = True
        elif t_sim >= t_degrau:
            alvo = degraus
            if desc and not degrau_avisado:
                log(f"  degrau aplicado em t_sim={t_sim:.1f}s")
                degrau_avisado = True
        else:
            alvo = zero

        q = _quaternion(alvo["roll"], alvo["pitch"], yaw0 + alvo["yaw"])
        # MAVLink define time_boot_ms como uint32 e espera que ele de a volta
        ms = int((time.monotonic() - t0_parede) * 1000) % (2 ** 32)
        m.mav.set_attitude_target_send(
            ms, m.target_system,
            m.target_component, TYPEMASK_ATITUDE, q, 0.0, 0.0, 0.0, THRUST_NEUTRO)
        aplicar_rc_override(m, rc_override)      # os overrides expiram; renova
        if t_sim - ultimo_aviso >= 15:
            log(f"  perfil em t_sim={t_sim:.0f}/{duracao:.0f}s")
            ultimo_aviso = t_sim
        # o ritmo ja vem da leitura bloqueante de ATTITUDE acima; dormir de novo
        # so reduziria a taxa de comando pela metade
        time.sleep(dt_envio * 0.1)

    parede = time.monotonic() - t0_parede
    log(f"perfil concluido: {t_sim:.1f}s simulados em {parede:.1f}s de parede "
        f"(aceleracao efetiva {t_sim / max(parede, 1e-6):.1f}x)")


def pousar(m, timeout=180):
    log("pousando")
    m.set_mode("LAND")
    fim = time.monotonic() + timeout
    while time.monotonic() < fim:
        hb = m.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
        from pymavlink import mavutil
        if hb and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            log("desarmado no solo")
            time.sleep(2)
            return True
    log("aviso: nao confirmei o desarme; seguindo mesmo assim")
    return False


def extrair_params(caminho_bin, destino):
    """Dump completo de parametros, tirado do proprio .BIN.

    O ArduPilot grava todos os parametros no log como mensagens PARM. Ler dali e
    melhor que baixar por MAVLink em todos os aspectos: e instantaneo, nao disputa
    o enlace com a telemetria, e — o que mais importa — o que sai e exatamente o
    que estava valendo NAQUELE voo, e nao o que o veiculo tinha depois de pousar.
    """
    from pymavlink import DFReader
    leitor = DFReader.DFReader_binary(str(caminho_bin))
    vistos = {}
    while True:
        msg = leitor.recv_msg()
        if msg is None:
            break
        if msg.get_type() == "PARM":
            vistos[msg.Name] = msg.Value
    with open(destino, "w", encoding="utf-8") as f:
        for nome in sorted(vistos):
            f.write(f"{nome},{vistos[nome]:.8g}\n")
    return len(vistos)


# --------------------------------------------------------------------------
# analise
# --------------------------------------------------------------------------

def analisar(pasta, gerar_gif=True):
    """Analisa uma pasta de resultado e escreve todas as saidas."""
    from analise.leitura import (ler_log, serie_unificada, controlador_ativo,
                                 ler_params, torque_segurado)
    from analise import (metricas as mod_metricas, painel, animacao, tabelas,
                         frequencia, veiculo)
    from analise.dryden_ref import Perfil, comparar_com_log
    from analise.eixos import COLUNAS, EIXOS, nome as nome_eixo

    pasta = Path(pasta)
    bins = sorted(pasta.glob("*.BIN"))
    if not bins:
        raise SystemExit(f"nenhum .BIN em {pasta}")
    caminho_bin = bins[0]

    cen = carregar_yaml(pasta / "cenario.yaml") if (pasta / "cenario.yaml").exists() else {}
    ctl = carregar_yaml(pasta / "controlador.yaml") if (pasta / "controlador.yaml").exists() else {}
    id_cen = id_cenario_da_pasta(pasta, cen)
    nome_cen = cen.get("nome", id_cen)
    nome_ctl = ctl.get("nome", pasta.parent.name)
    msg_pid = ctl.get("msg_pid", "PIDR")

    # Os parametros que valeram NESTE voo: geometria do frame, massa, inercia e
    # ganhos em uso. Uma pasta sem params.parm ganha um, tirado do proprio .BIN.
    if not (pasta / "params.parm").exists():
        extrair_params(caminho_bin, pasta / "params.parm")
    parm = ler_params(pasta / "params.parm")
    frame_type = int(parm.get("FRAME_TYPE", 1))
    desc_veiculo = veiculo.descrever(parm, frame_type=frame_type)

    log(f"lendo {caminho_bin.name}")
    dados = ler_log(caminho_bin)
    grade, col = serie_unificada(
        dados, msg_pid=msg_pid, duracao_s=(cen.get("voo") or {}).get("duracao_s"))
    if len(grade) == 0:
        raise SystemExit("log sem dados utilizaveis")

    perfil = Perfil.de_dict(cen.get("vento", {})) if cen.get("vento") else None

    # confere, pelo proprio log, que rodou o controlador pedido
    cc_tipo, cc_ativo = controlador_ativo(dados, janela=(float(grade[0]), float(grade[-1])))
    esperado = ctl.get("espera_cc_tipo")
    if esperado:
        if cc_tipo != esperado:
            log(f"AVISO: o log nao registra o AC_CustomControl no tipo {esperado} "
                f"(achou {cc_tipo}). Sem mensagem CC, o backend nunca assumiu — "
                "o voo saiu com o PID nativo. Confira RC6_OPTION=109 e o "
                "rc_override do YAML do controlador.")
        elif cc_ativo < 0.5:
            log(f"AVISO: CC_TYPE={cc_tipo} esta no log, mas o controlador ficou "
                f"ativo em apenas {cc_ativo * 100:.0f}% da janela do ensaio "
                "(a chave auxiliar caiu, ou nunca subiu). Boa parte dos numeros "
                "abaixo e do PID nativo.")
        else:
            log(f"controlador confirmado no log: CC_TYPE={cc_tipo}, "
                f"ativo em {cc_ativo * 100:.0f}% da janela do ensaio")

    # degraus que o cenario comandou; sem cenario, o detector procura em todos
    degraus = degraus_do_voo(cen["voo"]) if cen.get("voo") else None
    m = mod_metricas.calcular(grade, col, perfil, degraus=degraus,
                              inercia_kgm2=veiculo.inercias(desc_veiculo))
    m["controlador_no_log"] = {"cc_tipo": cc_tipo, "fracao_ativa": cc_ativo}
    for eixo in EIXOS:
        if f"degrau_{eixo}" in m and not m[f"degrau_{eixo}"]:
            log(f"AVISO: o cenario comanda degrau de {nome_eixo(eixo)}, mas ele nao "
                "aparece na atitude comandada do log — veja o painel")

    # reprodutibilidade: a serie regerada tem de bater com o WND do log
    if perfil is not None and "WND" in dados:
        m["reprodutibilidade"] = comparar_com_log(perfil, dados)

    #  Chirp: a serie temporal sozinha nao responde nada. A resposta em
    #  frequencia e o resultado do ensaio, e a coerencia diz onde acreditar nela.
    #  Uma estimativa por eixo varrido: torque do eixo -> atitude do mesmo eixo.
    #  A entrada e o torque segurado entre as escritas do injetor, como a fisica
    #  o aplica: a coluna da serie e interpolada e atrasaria a fase em pi*f*T.
    if perfil is not None and (perfil.modo & 64):
        segurado = torque_segurado(dados, grade)
        for eixo in mod_metricas.eixos_com_torque(perfil, col):
            c = COLUNAS[eixo]
            est = frequencia.estimar(grade, col, perfil, entrada=c["torque"],
                                     saida=c["med"],
                                     rotulo=f"{nome_eixo(eixo)} / {c['torque']}",
                                     u=segurado.get(c["torque"]))
            if est is None:
                continue
            m[tabelas.chave_resposta_freq(eixo)] = frequencia.metricas(est)
            arquivo = ("resposta_freq.png" if eixo == "roll"
                       else f"resposta_freq_{eixo}.png")
            frequencia.desenhar(est, pasta / arquivo, controlador=nome_ctl, cenario=nome_cen)
            log(f"{arquivo} gravada")

    # o que voou, junto das metricas: entra no consolidado e permite ler uma
    # varredura de ganho (ou de massa) linha a linha
    m["veiculo"], m["ganhos"] = veiculo.para_metricas(
        desc_veiculo, veiculo.ganhos_em_uso(parm))

    tabelas.salvar_metricas(m, pasta / "metricas.json")
    n_lin, n_col = tabelas.salvar_series(grade, col, pasta / "series.csv")
    log(f"series.csv: {n_lin} linhas x {n_col} colunas")

    preambulo = veiculo.linhas_resumo(parm, ctl, m["controlador_no_log"],
                                      frame_type=frame_type)
    texto = tabelas.salvar_resumo(m, pasta / "resumo.txt",
                                  cabecalho=f"{nome_ctl}  |  {nome_cen}",
                                  preambulo=preambulo)
    print()
    print(texto)

    # eixos em que o AC_CustomControl comandou de fato: neles o painel nao mostra
    # os termos do PIDR/PIDY, que sao do PID nativo rodando em sombra
    eixos_cc = ()
    if cc_tipo and cc_ativo >= 0.5:
        eixos_cc = tuple(e for e in EIXOS if veiculo.fonte_ganhos(parm, e) != "ATC_")

    # sem disturbio, sem faixa sombreada: sombrear o ensaio inteiro dizia
    # "janela do disturbio" num ensaio que nao tem disturbio nenhum
    jan = m.get("janela_disturbio_s")
    fig = painel.desenhar(grade, col, m, controlador=nome_ctl, cenario=nome_cen,
                          janela_disturbio=tuple(jan) if jan else None,
                          frame_type=frame_type, eixos_cc=eixos_cc,
                          titulo=f"Ensaio {id_cen} · {pasta.name}")
    painel.salvar(fig, pasta / "painel.png", pasta / "painel.pdf")
    log("painel.png e painel.pdf gravados")

    if gerar_gif:
        log("renderizando o GIF (pode levar ~1 min)")
        n = animacao.gerar(grade, col, pasta / "ensaio.gif",
                           controlador=nome_ctl, cenario=nome_cen, metricas=m,
                           frame_type=frame_type)
        log(f"ensaio.gif: {n} quadros")

    linha = tabelas.linha_consolidado(
        m, ensaio=pasta.name, cenario=id_cen,
        controlador=ctl.get("slug", nome_ctl),
        semente=(cen.get("vento") or {}).get("WND_SEED"))
    total = tabelas.anexar_consolidado(RESULTADOS / "consolidado.csv", linha)
    tabelas.escrever_tabela_tex(
        RESULTADOS / "consolidado.csv", RESULTADOS / "tabela.tex",
        legenda="Metricas dos ensaios de rejeicao de disturbio em SITL.",
        rotulo="tab:ensaios-vento")
    log(f"consolidado.csv agora tem {total} ensaio(s); tabela.tex atualizada")
    return m


# --------------------------------------------------------------------------
# ensaio completo
# --------------------------------------------------------------------------

class FalhaDeAnalise(Exception):
    """O voo terminou e o .BIN esta na pasta; quem falhou foi a analise.

    Fica separado das falhas de voo porque repetir o voo nao resolve: o lote nao
    tenta de novo, e a pasta continua valida para um --analisar depois.
    """


def voar(cen, ctl, pasta, speedup, instancia):
    """Sobe o SITL, voa o perfil e deixa na pasta o .BIN e o params.parm."""
    voo = cen["voo"]
    # nome -> mtime: passado de LOG_MAX_FILES (500) o firmware volta ao 00000001.BIN
    # e sobrescreve, entao o log novo pode ter o nome de um antigo
    antes = ({p.name: p.stat().st_mtime for p in LOGS_SITL.glob("*.BIN")}
             if LOGS_SITL.exists() else {})

    sitl = None
    try:
        sitl = subir_sitl(speedup, instancia, saida=pasta / "sitl.log")
        m = conectar(instancia, sitl=sitl)

        # UM unico reboot. SCR_ENABLE e CC_TYPE sao os dois parametros que exigem
        # reinicio, entao vao juntos: cada reboot e uma chance de o SITL nao voltar
        # a escutar, e o ensaio ja tem pontos de falha suficientes.
        # params: do cenario (o ruido do giroscopio, por exemplo) vao junto, e os
        # do controlador tem a palavra final
        params_cen = dict(cen.get("params") or {})
        params_ctl = dict(ctl.get("params") or {})
        aplicar_params(m, {**PARAMS_BASE, **params_cen, **params_ctl, **PARAMS_EXTRA},
                       "de base, do cenario, do controlador e da linha de comando")
        m = reiniciar(m, instancia, sitl=sitl)

        # Os ganhos vao DEPOIS do reboot: os CC2_* do backend so existem quando
        # o firmware sobe com CC_TYPE=2 e instancia o AC_CustomControl_PID. Ganho
        # nao precisa de reinicio — o controlador le o valor a cada passo.
        ganhos = ctl.get("ganhos") or {}
        if ganhos:
            aplicar_params(m, ganhos, "de ganhos do controlador")

        # Os WND_* so existem DEPOIS do reboot, e so quando o vento.lua ja rodou e
        # registrou a tabela dinamica — o que leva alguns segundos.
        log("esperando o vento.lua registrar os parametros WND_*")
        if not esperar_param(m, "WND_MODE"):
            raise SystemExit(
                "os parametros WND_* nunca apareceram — o vento.lua nao carregou. "
                "Confira o symlink ardupilot/ArduCopter/scripts e se SCR_ENABLE=1 "
                "sobreviveu ao reboot; os erros de Lua saem no sitl.log. Fim dele:\n"
                + sitl.ultimas_linhas())
        aplicar_params(m, params_vento_inicial(cen["vento"]), "do cenario (WND_*)")

        rc_override = ctl.get("rc_override") or {}
        if rc_override:
            log(f"chaves de RC forcadas: {rc_override}")

        esperar_pronto(m)
        aplicar_rc_override(m, rc_override)
        armar_e_decolar(m, float(voo["altitude_m"]), rc_override=rc_override)
        esperar_sim(m, float(voo["espera_apos_decolagem_s"]))
        voar_perfil(m, voo, cen["vento"], speedup, rc_override=rc_override)
        pousar(m)
        m.close()
    finally:
        derrubar_sitl(sitl)

    time.sleep(2)
    novos = sorted(p for p in LOGS_SITL.glob("*.BIN")
                   if p.stat().st_mtime > antes.get(p.name, float("-inf")))
    if not novos:
        raise SystemExit(f"nenhum .BIN novo em {LOGS_SITL} — o voo nao gerou log")
    origem = max(novos, key=lambda p: p.stat().st_mtime)
    destino_bin = pasta / origem.name
    shutil.copy(origem, destino_bin)
    log(f"log copiado: {origem.name} ({origem.stat().st_size / 1e6:.1f} MB)")

    n_par = extrair_params(destino_bin, pasta / "params.parm")
    log(f"params.parm: {n_par} parametros extraidos do log")


# backends do TCC (firmware/) por CC_TYPE
BACKENDS_TCC = {3: "AC_CustomControl_LQR", 4: "AC_CustomControl_LQG"}


def conferir_firmware(ctl):
    """O backend que o controlador pede esta no firmware?

    Os backends AC_CustomControl_LQR (CC_TYPE 3) e AC_CustomControl_LQG (CC_TYPE
    4) vivem em firmware/ e entram no submodulo por firmware/instalar.sh. Sem
    eles, o SITL subiria, o CC_TYPE cairia no "default" do init() e o voo sairia
    com o PID nativo — o que so se veria no aviso do espera_cc_tipo, depois de
    voar.
    """
    backend = BACKENDS_TCC.get(ctl.get("espera_cc_tipo"))
    if backend and not (ARDUPILOT / "libraries" / "AC_CustomControl" / f"{backend}.cpp").exists():
        raise SystemExit(
            f"o controlador {ctl.get('slug')} usa o backend {backend}, que nao "
            "esta instalado no ardupilot/. Rode firmware/instalar.sh (uma vez); o "
            "sim_vehicle.py recompila no proximo voo.")


def voa_no_cenario(nome_controlador, id_cenario):
    """O controlador voa esse cenario? Diz a chave ``cenarios:`` do YAML dele.

    Sem a chave, voa todos. Com ela, so os cenarios que comecam por algum dos
    prefixos da lista ('rolagem', 'ruido/rnd20', 'guinada/torque_degrau'): as
    variantes de um controlador feitas para uma familia de ensaios (os ganhos
    do filtro de 10 Hz, o escalonamento por empuxo) nao precisam voar o lote
    inteiro, e o lote sem argumentos continua sendo "tudo o que faz sentido".
    """
    ctl, _ = carregar_controlador(nome_controlador)
    prefixos = ctl.get("cenarios")
    if not prefixos:
        return True
    return any(id_cenario == pr or id_cenario.startswith(pr.rstrip("/") + "/")
               for pr in prefixos)


def rodar(id_cenario, nome_controlador, speedup, instancia, gerar_gif=True):
    """Um ensaio: resultados/<pasta>/<ensaio>/<controlador>/<carimbo>/."""
    cen, p_cen = carregar_cenario(id_cenario)
    ctl, p_ctl = carregar_controlador(nome_controlador)
    conferir_firmware(ctl)

    carimbo = dt.datetime.now().strftime("%Y-%m-%d_%H%M%S")
    pasta = RESULTADOS / id_cenario / ctl["slug"] / carimbo
    pasta.mkdir(parents=True, exist_ok=True)
    shutil.copy(p_cen, pasta / "cenario.yaml")
    shutil.copy(p_ctl, pasta / "controlador.yaml")
    log(f"resultado em {_rel(pasta)}")

    try:
        voar(cen, ctl, pasta, speedup, instancia)
    except BaseException as exc:
        # Sem .BIN, a pasta so serve de diagnostico. O sufixo deixa claro, ao
        # listar resultados/, que aquele voo nao valeu, e a tira do caminho do
        # --analisar. O motivo vai num arquivo porque a mensagem do terminal se
        # perde num lote longo; a saida do binario fica no sitl.log ao lado.
        falhou = pasta.with_name(pasta.name + "_falhou")
        pasta.rename(falhou)
        motivo = ("interrompido com Ctrl+C" if isinstance(exc, KeyboardInterrupt)
                  else str(exc) or type(exc).__name__)
        (falhou / "falha.txt").write_text(motivo + "\n", encoding="utf-8")
        log(f"voo nao concluido; a pasta ficou como {falhou.name}")
        raise

    try:
        return analisar(pasta, gerar_gif=gerar_gif)
    except (SystemExit, Exception) as exc:
        raise FalhaDeAnalise(f"{exc} (o .BIN esta em {_rel(pasta)})") from exc


def rodar_lote(cenarios, controladores, speedup, instancia, gerar_gif=True,
               tentativas=2):
    """Roda todos os pares cenario x controlador, um SITL por vez.

    Uma falha nao derruba o lote: o par e tentado de novo e, se falhar outra vez,
    fica no resumo do fim, e o lote segue. A nova tentativa existe por causa do
    reboot do SITL, que de vez em quando nao volta (visto uma vez em 16 voos na
    validacao destes ensaios) — e nao vale perder uma noite de lote por isso.
    Ctrl+C interrompe tudo.

    Devolve True se todos os pares terminaram.
    """
    plano = [(c, k) for c in cenarios for k in controladores if voa_no_cenario(k, c)]
    log(f"{len(plano)} ensaio(s): {len(cenarios)} cenario(s) x "
        f"{len(controladores)} controlador(es) ({', '.join(controladores)})")
    for c in cenarios:
        log(f"  {c}")

    situacao = []
    t0 = time.monotonic()
    for i, (cenario, controlador) in enumerate(plano, 1):
        for tentativa in range(1, tentativas + 1):
            log(f"===== [{i}/{len(plano)}] {cenario} x {controlador}"
                + (f" (tentativa {tentativa}/{tentativas})" if tentativa > 1 else "")
                + " =====")
            try:
                rodar(cenario, controlador, speedup, instancia, gerar_gif=gerar_gif)
                situacao.append((cenario, controlador, "ok", ""))
                break
            except FalhaDeAnalise as exc:
                log(f"FALHOU na analise: {exc}")
                situacao.append((cenario, controlador, "FALHOU", f"analise: {exc}"))
                break
            except (SystemExit, Exception) as exc:
                motivo = str(exc) or type(exc).__name__
                log(f"FALHOU no voo: {motivo}")
                if tentativa == tentativas:
                    situacao.append((cenario, controlador, "FALHOU", f"voo: {motivo}"))

    minutos = (time.monotonic() - t0) / 60.0
    falhas = [s for s in situacao if s[2] != "ok"]
    log(f"===== fim do lote: {len(situacao) - len(falhas)} de {len(situacao)} "
        f"ensaio(s) concluidos em {minutos:.0f} min =====")
    for cenario, controlador, estado, motivo in situacao:
        log(f"  {estado:6s} {cenario:38s} {controlador:10s} {motivo}".rstrip())
    if not falhas:
        log(f"resultados em {_rel(RESULTADOS)}/; "
            "consolidado.csv e tabela.tex atualizados")
    return not falhas


def pastas_de_resultado(caminhos):
    """Pastas de ensaio (as que tem .BIN) nos caminhos dados ou abaixo deles.

    Aceitar a pasta de cima — resultados/, ou resultados/guinada/ — e o que
    torna o --analisar pratico com os resultados organizados em pastas. Os voos
    de resultados/autotune/ (autotune.py) ficam de fora: nao sao ensaios.
    """
    autotune = (RESULTADOS / "autotune").resolve()
    achadas = []
    for c in caminhos:
        c = Path(c)
        if not c.exists():
            raise SystemExit(f"{c} nao existe")
        candidatas = [c] if any(c.glob("*.BIN")) else sorted(
            {b.parent for b in c.rglob("*.BIN")})
        for p in candidatas:
            if p.name.endswith("_falhou") or autotune in p.resolve().parents:
                continue
            if p not in achadas:
                achadas.append(p)
    return achadas


def analisar_varias(caminhos, gerar_gif=True):
    """Reanalisa cada pasta de ensaio encontrada; uma falha nao para as outras."""
    pastas = pastas_de_resultado(caminhos)
    if not pastas:
        raise SystemExit(f"nenhuma pasta com .BIN em: {', '.join(map(str, caminhos))}")
    falhas = []
    for i, pasta in enumerate(pastas, 1):
        if len(pastas) > 1:
            log(f"===== [{i}/{len(pastas)}] {pasta} =====")
        try:
            analisar(pasta, gerar_gif=gerar_gif)
        except (SystemExit, Exception) as exc:
            log(f"FALHOU: {exc}")
            falhas.append((pasta, exc))
    if len(pastas) > 1:
        log(f"===== {len(pastas) - len(falhas)} de {len(pastas)} pasta(s) reanalisadas =====")
        for pasta, exc in falhas:
            log(f"  FALHOU {pasta}: {exc}")
    return not falhas


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cenario", nargs="+", metavar="CENARIO",
                    help="o que rodar: um cenario (guinada/torque_pulso), uma pasta "
                         "(rolagem, arfagem, guinada, rolagem_arfagem_guinada) ou um ensaio de todas "
                         "as pastas (torque_pulso). Sem a opcao: todos")
    ap.add_argument("--controlador", nargs="+",
                    help="um ou mais controladores de controladores/. Sem a opcao: todos")
    ap.add_argument("--speedup", type=int, default=5,
                    help="aceleracao do SITL (padrao 5)")
    ap.add_argument("--instancia", type=int, default=0,
                    help="instancia do SITL; define a porta TCP 5760+10*I")
    ap.add_argument("--tentativas", type=int, default=2,
                    help="tentativas por ensaio quando o voo falha (padrao 2)")
    ap.add_argument("--sem-gif", action="store_true", help="pula a renderizacao do GIF")
    ap.add_argument("--analisar", metavar="PASTA", nargs="+",
                    help="so reanalisa resultados existentes: pastas de ensaio ou "
                         "pastas acima delas (resultados/, resultados/guinada/...)")
    ap.add_argument("--listar", action="store_true",
                    help="lista os cenarios e controladores disponiveis")
    ap.add_argument("--param", action="append", default=[], metavar="NOME=VALOR",
                    help="parametro do veiculo aplicado por ultimo, acima do cenario e do "
                         "controlador (repete). So para voos de medicao: use com --destino")
    ap.add_argument("--fila-antiga", action="store_true",
                    help="le a mensagem mais antiga da fila no laco do perfil, como antes "
                         "da correcao da temporizacao (Cap. 3). So para medir o atraso")
    ap.add_argument("--destino", metavar="PASTA",
                    help="raiz dos resultados no lugar de resultados/ (pasta, consolidado.csv "
                         "e tabela.tex). Para voos que nao devem entrar nas tabelas")
    args = ap.parse_args()

    global RESULTADOS, FILA_FRESCA
    for item in args.param:
        nome, sep, valor = item.partition("=")
        if not sep:
            raise SystemExit(f"--param espera NOME=VALOR, veio {item!r}")
        PARAMS_EXTRA[nome.strip()] = float(valor)
    FILA_FRESCA = not args.fila_antiga
    if args.destino:
        RESULTADOS = Path(args.destino).resolve()
        RESULTADOS.mkdir(parents=True, exist_ok=True)
    if (PARAMS_EXTRA or args.fila_antiga) and not args.destino and not args.analisar:
        raise SystemExit("--param e --fila-antiga sao para voos de medicao: "
                         "use --destino para nao misturar com os ensaios das tabelas")
    if PARAMS_EXTRA:
        log(f"parametros da linha de comando: {PARAMS_EXTRA}")
    if args.fila_antiga:
        log("leitura ANTIGA da fila no laco do perfil (medicao do atraso)")

    if args.listar:
        print("cenarios (pasta/ensaio):")
        grupo_antes = None
        for c in listar_cenarios():
            grupo, _, nome = c.rpartition("/")
            if grupo != grupo_antes:
                print(f"  {grupo or '.'}/")
                grupo_antes = grupo
            print(f"    {nome:16s} {carregar_cenario(c)[0].get('nome', '')}")
        print("controladores:")
        for n in listar_controladores():
            print(f"  {n:18s} {carregar_controlador(n)[0].get('nome', '')}")
        return

    if args.analisar:
        if not analisar_varias(args.analisar, gerar_gif=not args.sem_gif):
            sys.exit(1)
        return

    ok = rodar_lote(selecionar_cenarios(args.cenario),
                    selecionar_controladores(args.controlador),
                    args.speedup, args.instancia, gerar_gif=not args.sem_gif,
                    tentativas=max(args.tentativas, 1))
    if not ok:
        sys.exit(1)


if __name__ == "__main__":
    main()
