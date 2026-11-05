# TCC — Controle de Atitude de Quadrotor no ArduPilot (SITL + Gazebo)

Ambiente de simulação do TCC: firmware **ArduPilot** rodando em SITL (*Software-In-The-Loop*),
com a física e o visual 3D vindos do **Gazebo Harmonic**, e comando pelo **MAVProxy**.

O planejamento do trabalho está em [`diretivas/geral.md`](diretivas/geral.md);
a instalação do zero, em [`diretivas/setup.md`](diretivas/setup.md);
a integração com o VSCode, em [`diretivas/vscode.md`](diretivas/vscode.md).

---

## Estrutura

```
~/faculdade/TCC/
├── ardupilot/              # firmware (submódulo) — aqui mora o AC_CustomControl
├── ardupilot_gazebo/       # plugin ponte Gazebo ↔ SITL (submódulo)
├── firmware/               # os backends AC_CustomControl_LQR e _LQG do TCC e o instalar.sh
├── ensaios/                # injetor de distúrbios, cenários, controladores, análise
├── modelagem/              # modelo do SITL, projeto do controlador, validação
├── monografia/             # o texto do TCC (LaTeX), com as figuras e tabelas geradas
├── diretivas/              # roteiro do TCC, setup, guia do VSCode
├── .vscode/tasks.json      # tarefa "▶ Simular" (sobe Gazebo + SITL juntos)
├── run-gazebo.sh           # lançador do Gazebo
├── run-sitl.sh             # lançador do SITL
└── proposta.tex            # proposta do TCC (LaTeX)
```

---

## Pré-requisitos (uma vez só)

### 0. Inicializar os submódulos do ArduPilot

O firmware depende de vários submódulos (waf, MAVLink, ChibiOS...). Sem eles o build morre logo
no começo com `No such file or directory: .../modules/waf/waf-light`.

```bash
cd ~/faculdade/TCC/ardupilot
git submodule update --init --recursive
```

Confira com `git submodule status --recursive`: linhas começando com `-` significam **não
inicializado**; com `+`, **no commit errado**. Só está certo quando todas começam com espaço.

> **Se o download cair no meio** (`RPC failed; curl 92`, `early EOF`): o git aborta e deixa vários
> submódulos clonados **mas com o diretório vazio**. O sintoma é o build morrer com
> `source not found: 'modules/littlefs/lfs.c'` ou equivalente. **Rode o mesmo comando de novo** —
> os objetos já baixados são reaproveitados e só falta o checkout.
>
> Não tente consertar com `git reset --hard` dentro do submódulo: isso restaura os arquivos no
> HEAD do branch padrão, e **não** no commit que o ArduPilot fixa. O littlefs, por exemplo, vem na
> v2.9.3 e o build falha com `error: unused variable 'bd' [-Werror=unused-variable]`.
>
> `ChibiOS` e `gsoap` são os maiores e os que mais falham — e **nenhum dos dois entra no build
> SITL** (ChibiOS é o RTOS de placas reais; gsoap é ONVIF de câmera). Se só eles resistirem, pule-os
> listando os outros explicitamente:
>
> ```bash
> git submodule update --init --recursive -- \
>   modules/mavlink modules/waf modules/littlefs modules/lwip modules/gtest \
>   modules/gbenchmark modules/DroneCAN modules/Micro-CDR \
>   modules/Micro-XRCE-DDS-Client modules/CrashDebug
> ```

### 1. Compilar o plugin `ardupilot_gazebo`

Sem isso o Iris aparece no Gazebo mas **nunca conecta** ao SITL.

```bash
sudo apt install -y libgz-sim8-dev rapidjson-dev cmake
cd ~/faculdade/TCC/ardupilot_gazebo
export GZ_VERSION=harmonic
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Resultado esperado: `ardupilot_gazebo/build/libArduPilotPlugin.so` — é esse diretório que o
`run-gazebo.sh` publica em `GZ_SIM_SYSTEM_PLUGIN_PATH`.

### 2. Firmware SITL

Os backends LQR e LQG do TCC vivem em `firmware/`, fora do submódulo. O
`firmware/instalar.sh` os liga ao submódulo e aplica o `firmware/ardupilot.patch` (frontend,
mensagem `CCTQ` com a atitude verdadeira, `SIM_TWIST_TIME` em microssegundos). Depois,
compile:

```bash
cd ~/faculdade/TCC
firmware/instalar.sh
cd ardupilot
./waf configure --board sitl && ./waf copter
```

O `sim_vehicle.py` chamado à mão recompila o que mudou (5–15 min na primeira vez). O
`ensaios/run_ensaio.py` **não** recompila (`--no-rebuild`): todo voo usa o binário compilado
antes, e o hash dele vai no `versao.json` do voo. Depois de mexer no firmware, rode
`./waf copter`.

O build termina com `'copter' finished successfully` e produz
`ardupilot/build/sitl/bin/arducopter` (~5,6 MB) — é esse binário que o `sim_vehicle.py` executa.

Para acompanhar um build em andamento: `pgrep -af "waf copter"` diz se ainda roda, e
`find build/sitl -name '*.o' | wc -l` conta o progresso (um Copter completo dá ~1400 objetos).

### 3. Ambiente Python

O MAVProxy e as bibliotecas dos ensaios e da modelagem vivem no venv `~/venv-ardupilot`, com
as versões de `requirements.txt` (Python 3.12):

```bash
python3 -m venv ~/venv-ardupilot
~/venv-ardupilot/bin/pip install -r requirements.txt
```

Se o SITL reclamar `mavproxy.py: command not found`, ative-o antes de subir a simulação:

```bash
source ~/venv-ardupilot/bin/activate
```

### 4. Reproduzir os resultados da monografia

O mesmo caminho do Apêndice C ("Como reproduzir os resultados"):

```bash
source ~/venv-ardupilot/bin/activate
# um ensaio: cenario de ensaios/cenarios/ x controlador de ensaios/controladores/
ensaios/run_ensaio.py --cenario rolagem/torque_impulso --controlador cc_lqg
# o lote da monografia (~560 voos, ~8 h; retomavel: rodar de novo pula o que ja voou)
ensaios/lotes/lote_revisao.sh --l4-inerente
# metricas na atitude verdadeira, tabelas e figuras (so os voos do binario do lote)
cd ensaios
python -m analise.verdade resultados --binario $(cat lotes/binario_lote.sha256)
./tabelas_monografia.py
./figuras_monografia.py
# modelo, projeto e confronto modelo x SITL
cd ../modelagem
./projeto.py --figuras ../monografia/figuras --tabelas ../monografia/tabelas
./validacao.py --binario $(cat ../ensaios/lotes/binario_lote.sha256) \
    --figuras ../monografia/figuras --tabela ../monografia/tabelas/validacao.tex
```

Deixe o computador sem dormir durante o lote: o SITL anda em passo com a física e para
quando o Windows suspende (o lote retoma sozinho, mas perde as horas paradas).

---

## Como rodar

### Pelo VSCode (recomendado)

`Ctrl+Shift+P` → **Run Task** → **▶ Simular (Gazebo + SITL)**

Sobem dois terminais lado a lado: um com o Gazebo, outro com o MAVProxy (onde você digita os
comandos). O `run-sitl.sh` já espera 4 s para o Gazebo subir primeiro.

### Na mão, dois terminais

```bash
# Terminal 1 — física e visual
~/faculdade/TCC/run-gazebo.sh

# Terminal 2 — firmware + MAVProxy
~/faculdade/TCC/run-sitl.sh
```

### Primeiro voo

No terminal do SITL, depois que os *pre-arm checks* passarem (~30 s):

```
mode guided
arm throttle   q+
takeoff 5
```

Para pousar: `mode land`.

---

## O que acontece quando você roda

```
   ┌──────────────────┐   PWM dos 4 motores (UDP 9002)   ┌──────────────────┐
   │  ArduPilot SITL  │ ───────────────────────────────► │  Gazebo Harmonic │
   │  (arducopter)    │ ◄─────────────────────────────── │  + ArduPilotPlugin│
   └──────────────────┘   estado físico: pose, vel,      └──────────────────┘
        ▲     │           giro, acel, IMU simulada
   MAVLink    │ MAVLink
   TCP 5760   ▼ UDP 14550/14551
   ┌──────────────────┐        ┌──────────────────┐
   │    MAVProxy      │        │ QGroundControl   │
   │ (seu terminal)   │        │  (opcional)      │
   └──────────────────┘        └──────────────────┘
```

**Passo a passo do que cada script dispara:**

1. **`run-gazebo.sh`** exporta `GZ_VERSION`, `GZ_SIM_SYSTEM_PLUGIN_PATH` (onde está o
   `libArduPilotPlugin.so`) e `GZ_SIM_RESOURCE_PATH` (modelos e mundos), e abre
   `gz sim -r iris_runway.sdf`. O `-r` já inicia a simulação rodando (sem ele a física fica
   pausada e o EKF nunca converge).
2. O mundo carrega o modelo **`iris_with_ardupilot`**, cujo SDF instancia o
   `ArduPilotPlugin`. O plugin abre um socket **UDP na porta 9002** e imprime
   `Waiting for connection...`.
3. **`run-sitl.sh`** põe `ardupilot/Tools/autotest` no `PATH`, espera 4 s e chama
   `sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON`:
   - `-f gazebo-iris` carrega `default_params/copter.parm` + `default_params/gazebo-iris.parm`
     (frame X, `FRAME_CLASS=1`, `FRAME_TYPE=1`, rangefinder e IRLock do modelo);
   - `--model JSON` desliga a física interna do SITL e o faz falar a **interface JSON** com um
     simulador externo — a planta passa a ser o Gazebo;
   - `--add-param-file .../gazebo-iris.parm` aplica os parâmetros do frame explicitamente.

   > **Por que o `--add-param-file`?** Nesta versão do ArduPilot o `sim_vehicle.py` não passa
   > mais `--defaults`: o binário SITL resolve os parâmetros do frame a partir do
   > `vehicleinfo.json` embutido, **chaveado por `--model`** e não por `-f`. Como o modelo
   > aqui é `JSON`, ele procura um frame chamado `JSON`, não encontra, e os defaults do
   > `gazebo-iris` nunca chegam. Sem eles `FRAME_CLASS` fica em 0 e o veículo reporta
   > `Frame: UNSUPPORTED` e recusa armar.
4. O SITL compila (se preciso), sobe o binário `arducopter` e começa a enviar os **PWMs dos
   quatro motores** para `127.0.0.1:9002`. O Gazebo aplica esses empuxos e devolve, a cada
   passo, o estado físico (pose, velocidades, acelerações, giroscópio). O SITL usa isso para
   alimentar os sensores simulados — daí em diante o firmware não sabe que não está voando.
5. O `sim_vehicle.py` sobe o **MAVProxy** conectado ao SITL por **TCP 5760** e retransmite a
   telemetria em **UDP 14550 e 14551** — é por isso que o QGroundControl conecta sozinho.
6. O EKF converge, o GPS trava o *fix* 3D, os *pre-arm checks* passam e o veículo aceita `arm`.

> **Ordem importa:** o Gazebo tem que estar rodando (e despausado) antes do SITL. Se o Gazebo
> morrer com o SITL vivo, o SITL congela esperando o próximo estado físico.

**Onde ficam os logs:** cada voo gera um `.BIN` em `ardupilot/ArduCopter/logs/`
(é a matéria-prima da análise com `pymavlink`, Fase 0 do roteiro).

---

## Comandos do MAVProxy

Tudo abaixo se digita no **terminal do SITL**. `Tab` completa comandos e `help` lista os módulos.

### Voo básico

| Comando | O que faz |
|---|---|
| `mode <MODO>` | Troca o modo de voo (ex.: `mode guided`). Lista completa abaixo. |
| `arm throttle` | Arma os motores. Só passa se os *pre-arm checks* estiverem OK. |
| `arm safetyoff` | Desliga o *safety switch* simulado (raramente necessário no SITL). |
| `arm list` | Lista quais *pre-arm checks* estão ativos. `arm uncheck <nome>` desativa um deles (ex.: `arm uncheck gps`) — útil para depurar por que o veículo não arma. |
| `disarm` | Desarma. No ar, **corta os motores** — use só no chão. |
| `takeoff 5` | Decola até 5 m. Exige estar **armado e em GUIDED**. |
| `mode land` | Pouso automático no lugar onde está. **É este que você usa para pousar.** |
| `mode rtl` | Volta ao ponto de decolagem e pousa. |
| `land` | **Não** é o pouso normal: envia `DO_LAND_START`, que dispara a sequência de pouso de uma missão AUTO. Sem missão carregada, é rejeitado. `land abort` envia `DO_GO_AROUND`. |
| `status` | Despeja o estado atual: modo, armado, altitude, links, taxas. |
| `alt` | Mostra só a altitude. |

### Modos de voo do Copter (os que interessam ao TCC)

| Modo | Nº | Comportamento |
|---|---|---|
| `stabilize` | 0 | Ângulo comandado pelo piloto, throttle manual. **Malha de atitude pura** — o modo mais direto para avaliar seu controlador. |
| `acro` | 1 | Comando de taxa angular, sem nivelamento automático. |
| `alt_hold` | 2 | Atitude manual + altitude automática. |
| `guided` | 4 | Aceita comandos da GCS (posição, velocidade, `takeoff`). É o modo de trabalho. |
| `loiter` | 5 | Mantém posição e altitude sozinho. |
| `rtl` | 6 | Retorna à origem. |
| `circle` | 7 | Voa em círculo automático. |
| `land` | 9 | Pouso automático. |
| `systemid` | 25 | Injeta *chirp* nos eixos para **identificação de sistema** (Fase 4). |

### Movimento em GUIDED

| Comando | O que faz |
|---|---|
| `guided ALT` | Voa para a posição atual na altitude `ALT`. |
| `guided LAT LON ALT` | Voa até a coordenada global. |
| `guided forward METROS` | Avança N metros no eixo do nariz. |
| `position x y z` | Alvo de posição **relativo ao corpo** (`MAV_FRAME_BODY_NED`), em metros: x = frente, y = direita, z = **para baixo**. Ex.: `position 10 0 -5` → 10 m à frente, 5 m acima. |
| `velocity x y z` | Alvo de velocidade em m/s, mesma convenção de eixos. |
| `posvel ...` | Posição + velocidade no mesmo comando. |
| `setspeed 5` | `DO_CHANGE_SPEED` — muda a velocidade de cruzeiro para 5 m/s. |
| `setyaw ANG VEL MODO` | `CONDITION_YAW`: ângulo, velocidade angular, e `0` = absoluto / `1` = relativo. |
| `changealt 20` | Sobe/desce para 20 m mantendo a posição. |

### Parâmetros (o dia a dia da sintonia)

| Comando | O que faz |
|---|---|
| `param show ATC_RAT_RLL*` | Mostra os parâmetros que casam com o padrão. É assim que você lê os ganhos do PID nativo. |
| `param set ATC_RAT_RLL_P 0.135` | Altera um parâmetro **ao vivo**, sem recompilar. |
| `param fetch` | Rebaixa a lista completa de parâmetros do veículo. |
| `param save meus.parm` / `param load meus.parm` | Salva/carrega um conjunto de parâmetros — use para versionar cada configuração de experimento. |
| `param diff arquivo.parm` | Mostra o que difere entre o veículo e um arquivo. |
| `param download` | Baixa os metadados (descrição, faixa, unidade) dos parâmetros. |

> Alguns parâmetros só valem depois de `reboot` — o MAVProxy avisa quando é o caso
> (`@RebootRequired: True` na definição, como o `CC_TYPE`).

### Telemetria, gráficos e depuração

| Comando | O que faz |
|---|---|
| `watch ATTITUDE` | Imprime toda mensagem MAVLink que casa com o padrão (aceita `*`). |
| `module load graph` | Carrega o módulo de gráficos. |
| `graph ATTITUDE.roll ATTITUDE.pitch` | Plota campos MAVLink em tempo real. Ótimo para ver resposta ao degrau na hora. |
| `module load map` / `module load console` | Sobem o mapa e o console gráfico (o `run-sitl.sh` os omite de propósito — quem faz esse papel é o QGroundControl). |
| `output add udp:IP:14550` | Adiciona uma saída de telemetria (útil se o QGC no Windows não conectar sozinho). |
| `link` | Estado dos links MAVLink. |
| `set streamrate 10` | Taxa das mensagens de telemetria, em Hz. |
| `long <CMD> p1..p7` | Envia um `COMMAND_LONG` cru — escape para qualquer comando MAVLink sem atalho. |
| `reboot` | Reinicia o firmware SITL (sem derrubar o Gazebo). |

### Missões

| Comando | O que faz |
|---|---|
| `wp list` | Lista os waypoints carregados. |
| `wp load missao.txt` / `wp save missao.txt` | Carrega/salva uma missão. |
| `wp clear` | Apaga a missão. |
| `mode auto` | Executa a missão carregada. |

### Logs

| Comando | O que faz |
|---|---|
| `log list` | Lista os logs de dataflash a bordo. |
| `log download <n>` | Baixa o log `n`. (No SITL os `.BIN` já estão em `ardupilot/ArduCopter/logs/`.) |
| `log erase` | Apaga os logs a bordo. |

---

## Parâmetros úteis para o TCC

| Parâmetro | Para que serve |
|---|---|
| `ATC_RAT_RLL_*`, `ATC_RAT_PIT_*`, `ATC_RAT_YAW_*` | Ganhos do **PID de taxa** nativo — sua linha de base de comparação (Fase 3). |
| `ATC_ANG_RLL_P`, `ATC_ANG_PIT_P`, `ATC_ANG_YAW_P` | Ganhos P da malha externa de **ângulo**. |
| `CC_TYPE` | Habilita o **`AC_CustomControl`** (`0` = desligado, `1` = backend Empty, `2` = backend PID de exemplo). Exige `reboot`. Já vem compilado nos builds SITL. |
| `RC6_OPTION = 109` | Mapeia um canal de rádio para **alternar entre seu controlador e o nativo em pleno voo** (Fase 5/6). |
| `SID_AXIS`, `SID_MAGNITUDE`, `SID_F_START_HZ`, `SID_F_STOP_HZ`, `SID_T_REC` | Configuram o *chirp* do modo `systemid` (Fase 4). |
| `LOG_BITMASK` | Escolhe o que vai para o `.BIN`. Planeje isso **antes** dos experimentos. |

> **Atenção com vento:** com `--model JSON` a física é do Gazebo, então os `SIM_WIND_*` do SITL
> **não** movem o drone — o `SIM_JSON` zera o vento internamente
> ([`SIM_JSON.cpp:444`](ardupilot/libraries/SITL/SIM_JSON.cpp#L444)) e o plugin do Gazebo só
> reporta vento aparente de anemômetro, nunca aplica força. Os ensaios de rejeição de distúrbio
> rodam em **SITL puro**, e é isso que o diretório [`ensaios/`](ensaios/) automatiza — veja a
> seção seguinte.

---

## Ensaios de vento e parametrização do controle

O diretório [`ensaios/`](ensaios/) injeta vento reprodutível em SITL puro, registra o vento
aplicado **dentro do próprio `.BIN`** e produz, por corrida, um painel de figuras, um GIF e os
números da corrida.

```bash
source ~/venv-ardupilot/bin/activate

firmware/instalar.sh                                         # uma vez: os backends LQR e LQG no ardupilot/
python ensaios/run_ensaio.py                                 # TODOS os ensaios, em todos os controladores
python ensaios/run_ensaio.py --listar                        # o que existe
python ensaios/run_ensaio.py --cenario guinada               # uma pasta inteira
python ensaios/run_ensaio.py --cenario guinada/torque_pulso --controlador nativo
python ensaios/run_ensaio.py --cenario torque_pulso          # o pulso em todas as pastas
python ensaios/run_ensaio.py --analisar ensaios/resultados --sem-gif   # reprocessa tudo

python modelagem/projeto.py --figuras monografia/figuras --tabelas monografia/tabelas \
    --escrever-yaml                                      # projeto do Cap. 5 e os YAMLs dele
python ensaios/tabelas_monografia.py --desde 2026-10-27  # tabelas do Cap. 6 (monografia/tabelas/)
python ensaios/figuras_monografia.py --desde 2026-10-27  # figuras do Cap. 6 (monografia/figuras/)
python modelagem/validacao.py --desde 2026-10-27 \
    --tabela monografia/tabelas/validacao.tex --figuras monografia/figuras   # validação do Cap. 4
```

O `tabelas_monografia.py` pega o ensaio mais recente de cada cenário × controlador (a partir de
`--desde`) e escreve as tabelas de resultados no formato da monografia. O `validacao.py` simula a
malha fechada com o modelo linear do Cap. 4 e o controlador que voou, alimentada pelo torque
registrado em cada voo, calcula as métricas com o mesmo código da análise e, nas varreduras,
identifica `a` e `b` da planta no domínio da frequência.

Sem argumentos, o `run_ensaio.py` roda os 18 cenários com cada controlador de
`ensaios/controladores/` — 8 controladores, 144 voos, um SITL por vez, ~1 h 50 sem GIF e ~3 h 40
com (`--controlador` e `--cenario` recortam o lote). O SITL roda
**sem janela**, com a saída do binário no `sitl.log` de cada ensaio (ver
[O SITL sem janela](#o-sitl-sem-janela-e-por-que-alguns-voos-falhavam)). Um voo que falha é
tentado de novo (`--tentativas`, padrão 2), e o lote segue até o fim e termina com um resumo do
que passou e do que falhou. A pasta de um voo que não terminou ganha o sufixo `_falhou`, com o
motivo em `falha.txt`; a tentativa seguinte abre outra pasta. `--cenario` e `--controlador`
restringem o lote; `--analisar` aceita uma pasta de ensaio ou qualquer pasta acima dela.

Cada ensaio deixa em `ensaios/resultados/<pasta>/<ensaio>/<controlador>/<carimbo>/` (por
exemplo `resultados/guinada/torque_pulso/nativo/2026-10-06_020447/`): o `.BIN`, os dois YAMLs,
`params.parm`, `metricas.json`, `series.csv`, `resumo.txt`, `painel.png`, `painel.pdf` e
`ensaio.gif`. Acumulando na raiz de `resultados/`, o `consolidado.csv` ganha **uma linha por
ensaio** — é essa tabela que se lê para escolher ganho, e ela traz os ganhos em uso e a massa e
inércia do veículo de cada linha — e o `tabela.tex` sai pronto para `\input{}` na monografia, com
**uma linha por eixo** (um ensaio dos três eixos vira três).

### Os ganhos do PID: AutoTune do ArduPilot

Os ganhos do `nativo` (e, com prefixo `CC2_`, do `cc_pid`) vêm do AutoTune do ArduPilot rodado
no mesmo SITL dos ensaios, e não do `copter.parm`, que não foi sintonizado para este veículo:

```bash
python ensaios/autotune.py                 # rolagem, arfagem e guinada (AUTOTUNE_AXES 7)
python ensaios/autotune.py --eixos 4       # só guinada
```

O voo segue o roteiro do autotest do ArduPilot: decola em GUIDED, passa para LOITER com os
manches no centro, entra em AUTOTUNE e espera o `AutoTune: Success` (~8 min simulados, ~1 min
de parede). Depois **pousa sem sair do AUTOTUNE**, porque só o desarme dentro do modo grava os
ganhos (`AC_AutoTune::disarmed`). Como o SITL dos ensaios sobe com `-w`, o que vale é a pasta
`ensaios/resultados/autotune/<carimbo>/`: o `ganhos.yaml` traz os ganhos de antes e de depois e
os blocos prontos para o `ganhos:` dos dois YAMLs de controlador. O `run_ensaio.py` aplica o
`ganhos:` depois do reboot, porque os `CC2_*` só existem com o backend já instanciado.

O AutoTune grava também o `ATC_ACC_*_MAX`, que limita o alvo modelado. Esse valor vai igual para
os dois controladores, porque quem modela o alvo é sempre o `AC_AttitudeControl` nativo. Na
guinada ele fixa `I = P/10` (`AUTOTUNE_YAW_PI_RATIO_FINAL`), então a recuperação de um torque
sustentado continua lenta, com constante de tempo ~`P/I` = 10 s.

### Cenários

Os cenários ficam em uma pasta por eixo, e cada pasta tem os mesmos ensaios. As famílias de
ensaio mais novas ganham uma pasta própria, com uma subpasta por variante:

```
ensaios/cenarios/
├── rolagem/           nominal (10°)   torque_pulso (15 rad/s²)  torque_degrau (10 rad/s²)  torque_chirp (0,2–10 Hz)  torque_estocastico (σ 10 rad/s²)
├── arfagem/           nominal (10°)   torque_pulso (15 rad/s²)  torque_degrau (10 rad/s²)  torque_chirp (0,2–10 Hz)  torque_estocastico (σ 10 rad/s²)
├── guinada/           nominal (30°)   torque_pulso (3 rad/s²)   torque_degrau (1 rad/s²)   torque_chirp (0,1–5 Hz)   torque_estocastico (σ 1 rad/s²)
├── rolagem_arfagem_guinada/
│                     nominal         torque_pulso              torque_degrau              torque_estocastico   (os três eixos juntos)
├── ruido/             rnd10/ rnd20/ rnd40/ rnd80/: torque_pulso, torque_degrau, torque_estocastico
│                                     (os três eixos, giroscópio com SIM_GYRn_RND = 10 … 80)
├── vento/             rajada (10 m/s, meio cosseno de 3 s, de lado)
├── grande/            30graus/ 45graus/: nominal (degrau de rolagem), combinado (rolagem + arfagem + 90° de guinada)
├── saturacao/guinada/ torque_pulso (12 rad/s² × 0,5 s), torque_degrau (6 rad/s²): além da autoridade do mixer
├── erro_modelo/       empuxo_fraco/ braco_longo/ braco_curto/: nominal, torque_pulso, torque_degrau (três eixos)
└── falha_motor/       leve (SIM_ENGINE_MUL 0,95), severa (0,85): motor 1 fraco por 40 s
```

Um cenário pode trazer um bloco `params:`, aplicado antes do reboot junto com os do controlador
(os do controlador têm a palavra final). É assim que os da pasta `ruido/` ligam
`SIM_GYR1_RND`–`SIM_GYR3_RND`: o SITL soma a cada **sub-amostra** do giroscópio um ruído uniforme
de amplitude `SIM_GYRn_RND` °/s vezes o acelerador (~0,6 no pairado) e entrega a 1 kHz a média de 8
sub-amostras, e é também assim que os de `erro_modelo/` mudam o veículo (ver
[O caminho do ruído](#o-caminho-do-ruído-e-o-ângulo-do-ahrs) e [O LQG](#o-lqg-e-o-portão)).

Um cenário de três níveis (`ruido/rnd20/torque_pulso`) funciona como os de dois: o nome é o
caminho, e os resultados ficam em `resultados/ruido/rnd20/torque_pulso/<controlador>/<carimbo>/`.

O `vento/rajada` é o ensaio que sustenta a divisão entre as famílias do injetor: no SITL a rajada
move a aeronave, mas quase não perturba a atitude (ver [Dois achados](#dois-achados-que-mudam-o-texto-da-monografia)).

O nome de um cenário é o caminho dele: `guinada/torque_pulso`. É esse nome que o `--cenario`
aceita, que organiza `resultados/` e que vai na coluna `cenario` do consolidado. Para um ensaio
novo, basta pôr o YAML na pasta do eixo; ele entra no lote sozinho.

O degrau de um eixo é `voo.degrau_roll_deg` / `voo.degrau_pitch_deg` / `voo.degrau_yaw_deg` no
YAML (o de guinada é somado ao rumo travado no início do perfil); o torque, `WND_TQX` / `WND_TQY` /
`WND_TQZ`. As métricas saem **por
eixo** (`degrau_yaw`, `disturbio_yaw`, `esforco_yaw`, ...), só para os eixos que o ensaio
exercita, e o painel ganha uma coluna de faixas por eixo. Na guinada, as métricas usam o rumo
**relativo** ao inicial (`yaw_rel` e `yaw_des_rel` no `series.csv`), desdobrado antes da
interpolação — sem isso, um degrau que cruza o norte vira um salto de 360° no log.

A arfagem usa as amplitudes da rolagem: no frame X os dois eixos têm o mesmo braço (0,247 m) e a
mesma inércia (`Ixx = Iyy`), então `arfagem/` e `rolagem/` devem dar os mesmos números, e uma
diferença entre eles aponta assimetria no modelo ou no controlador. As amplitudes de guinada são
menores de propósito: a guinada não tem braço de alavanca, o momento vem do torque de reação das
hélices, e a autoridade em guinada do modelo do SITL é da ordem de 10 rad/s², contra centenas em
rolagem e arfagem. Não há chirp combinado: com a mesma varredura em vários eixos,
a resposta de um eixo e o acoplamento vindo dos outros ficam indistinguíveis.

Os ensaios de `rolagem_arfagem_guinada/` usam as amplitudes dos de um eixo só, para cada eixo
se comparar com o seu par. Rolagem e arfagem iguais somam um momento sobre a diagonal do X que
passa por `M1` e `M2`: só `M3` e `M4` têm braço em torno dela, e cada um trabalha o dobro do que
num ensaio de um eixo — é o carregamento mais severo da mistura.

> **Ao comparar `nativo` × `cc_pid`:** as medidas abaixo são de antes do AutoTune, quando o
> backend PID de exemplo voava com 90% dos ganhos *compilados* do ArduPilot
> ([`AC_CustomControl_PID.cpp:323`](ardupilot/libraries/AC_CustomControl/AC_CustomControl_PID.cpp#L323))
> e o nativo com o [`copter.parm`](ardupilot/Tools/autotest/default_params/copter.parm)
> (`ATC_RAT_YAW_P = 0.3`): em guinada o cc_pid tinha **54%** do P de taxa do nativo. Agora os dois
> voam com os ganhos do AutoTune (ver [Os ganhos do PID](#os-ganhos-do-pid-autotune-do-ardupilot)),
> e no `guinada/torque_degrau` saem iguais: 2,55° / 22,2 s e 2,55° / 22,1 s.

Primeiras medidas (uma corrida de cada, semente fixa, ganhos do `copter.parm`):

| ensaio | `nativo` | `cc_pid` |
|---|---|---|
| `guinada/nominal`: sobressinal / acomodação / erro de rastreamento máx. | 14,9% / 1,32 s / 13,3° | 15,4% / 2,02 s / 17,9° |
| `guinada/torque_pulso`: desvio máx. / recuperação | 6,6° / 0,94 s | 8,5° / 1,62 s |
| `guinada/torque_degrau`: desvio máx. / recuperação | 3,2° / 32 s | — |

A arfagem repete a rolagem, como a simetria do frame X prevê. Com o nativo, o degrau deu
0,16 s / 2,3% / 0,38 s (rolagem: 0,16 s / 2,2% / 0,38 s), o pulso 2,03° / 1,82 s (1,99° / 1,88 s),
o torque sustentado 1,47° / 2,36 s (1,47° / 2,38 s), e o chirp, a mesma resposta em frequência.
Só o erro de rastreamento do degrau ficou fora da variação entre corridas: 0,92°, contra 0,77° e
0,80° nas duas corridas de rolagem.

Dois achados: o integrador de guinada é **lento** — sob 1 rad/s² constante o rumo leva ~32 s para
voltar a 10% do desvio, contra 2,4 s na rolagem —, e o acoplamento entre os eixos no SITL é
**pequeno**: nos ensaios combinados, a rejeição de cada eixo fica a poucos por cento da do ensaio
de um eixo só (pulso nos três eixos: 2,15° / 1,96° / 6,46°, contra 1,99° / 2,03° / 6,6°). No
degrau dos três eixos, porém, o próprio alvo de rolagem e arfagem deixa de ser um degrau — girar
em guinada inclinado o faz passar do ponto em ângulos de Euler, e a arfagem comandada chega a
~13° —, então ali sobressinal e subida medem o alvo; compare controladores pelo erro de
rastreamento.

### O `resumo.txt`: veículo e ganhos em uso

Todo `resumo.txt` abre com dois blocos lidos do `params.parm` daquele voo — o que estava valendo
de fato, e não o que o YAML pretendia:

- **VEÍCULO** — frame e posição/sentido de giro de cada motor, massa e peso, distância entre
  eixos, braço, inércias, empuxo por motor (pairado e máximo), curva de empuxo, coeficiente do
  torque de reação, PWM, bateria, taxa do laço e filtros;
- **CONTROLADOR EM USO** — uma tabela rolagem × arfagem × guinada com os ganhos que voaram em cada
  eixo (`ATC_*` no nativo, `CC2_*` nos eixos de `CC_AXIS_MASK` quando o `AC_CustomControl` está
  ativo), a fração da janela em que o backend esteve ativo e os limites do modelador de entrada,
  que são comuns a todos os controladores.

Para ganhar os blocos nos ensaios já gravados, basta reanalisar (`--analisar ensaios/resultados --sem-gif`): as
métricas não mudam, só o texto e as colunas novas do consolidado.

Dois detalhes do modelo de multirrotor do SITL, conferidos no código e nos logs, entram no bloco
como notas e precisam entrar na monografia:

1. **`SIM_FRM_DIAG_SZ` se diz "diagonal motor a motor", mas é o braço.**
   [`SIM_Motor.cpp:206`](ardupilot/libraries/SITL/SIM_Motor.cpp#L206) põe cada motor a
   `DIAG_SZ` do **centro**, então o braço efetivo é 0,35 m e a diagonal efetiva, 0,70 m. Confirmado
   no `rolagem/torque_degrau`: o diferencial medido dos motores devolve ~10,5 rad/s² de contra-torque
   contra os 10 aplicados com braço = `DIAG_SZ`, e só 5,2 com braço = `DIAG_SZ/2`.
2. **A inércia é estimada com metade desse raio.** Sem modelo JSON,
   [`SIM_Frame.cpp:823`](ardupilot/libraries/SITL/SIM_Frame.cpp#L823) usa metade da massa num anel
   de raio `DIAG_SZ/2`: `Ixx = Iyy = 0,02297` e `Izz = 0,04594 kg·m²`. É incoerente como
   geometria, mas é a planta que o SITL integra — e é com ela que o modelo do TCC tem de bater.

### O injetor de distúrbios

[`ensaios/lua/vento.lua`](ensaios/lua/vento.lua) roda dentro do SITL, a 50 Hz de passo fixo, e
registra a verdade do que aplicou na mensagem `WND` do log. Ele injeta **duas famílias de
distúrbio**, e a distinção é o eixo de todo o resto:

| família | como entra | o que perturba |
|---|---|---|
| **Vento** (`SIM_WIND_*`) | força no centro de massa | **posição** — deriva metros; a atitude, centésimos de grau |
| **Torque** (`SIM_TWIST_*`) | somado direto em `rot_accel`, em rad/s² | **atitude** — é o que exercita a malha que você projeta |

O ruído de Dryden e o torque estocástico vêm de um PRNG determinista semeado por `WND_SEED`,
espelhado em [`ensaios/analise/dryden_ref.py`](ensaios/analise/dryden_ref.py): as duas
implementações batem amostra a amostra (`comparar_com_log`; no torque estocástico, a menos de
1e-5 rad/s², o arredondamento do float de 32 bits), e é isso que sustenta a reprodutibilidade dos
ensaios estocásticos. O ArduPilot compila o Lua com `LUA_32BITS` (inteiros e floats de 32 bits,
em todas as placas e no SITL): o produto direto `16807 × estado` do minstd dava a volta, e o
gerador do Lua não era o do Python até 31/10/2026. O `vento.lua` usa desde então o método de
Schrage, exato em 32 bits.

| Parâmetro | Para que serve |
|---|---|
| `WND_MODE` | bitmask somável: `1`=vento médio, `2`=rajada, `4`=Dryden, `8`=torque em pulso, `16`=torque em meio cosseno, `32`=torque em degrau, `64`=torque em varredura (chirp), `128`=torque estocástico, `256`=evento de planta |
| `WND_SPD`, `WND_DIR`, `WND_DIRZ` | vento médio: módulo (m/s), direção **de onde vem** (°), ângulo vertical (°) |
| `WND_GA`, `WND_GT0`, `WND_GDUR` | rajada: amplitude (m/s), início (s) e duração (s) |
| `WND_TSIG`, `WND_TLU` | Dryden: intensidade σ (m/s) e escala longitudinal Lu (m); a lateral é Lu/2 |
| `WND_TQX`, `WND_TQY`, `WND_TQZ` | torque: amplitude em **rad/s²** por eixo do corpo (rolagem, arfagem, guinada) |
| `WND_TQT0`, `WND_TQDUR` | torque: início (s) e duração (s); o degrau ignora a duração |
| `WND_TQTC` | torque estocástico: tempo de correlação (s) do processo de Gauss-Markov; com o bit 128, `WND_TQX/Y/Z` são o **desvio padrão** |
| `WND_EVT`, `WND_EVM`, `WND_EVK` | evento de planta (bit 256), aplicado em `WND_TQT0` e desfeito em `WND_TQT0 + WND_TQDUR`: `1` = motor fraco (`SIM_ENGINE_FAIL = EVM`, `SIM_ENGINE_MUL = EVK`), `2` = perda de empuxo (`SIM_FRM_HOVR_THR = EVK`) |
| `WND_SEED` | semente do PRNG — é o que torna dois ensaios idênticos |
| `WND_T0` | espera após o armamento; **negativo** = modo de bancada, começa sem armar |

O torque em rad/s² **é** o "termo aditivo de perturbação na equação da dinâmica de rotação" da
`proposta.tex` — não uma emulação por assimetria de rotor. O impulso angular de um pulso é
exatamente amplitude × duração, e sai calculado em `metricas.json`.

O torque **estocástico** (bit 128) é a turbulência que gera momento, que o vento deste simulador não
gera: em cada eixo, um processo de Gauss-Markov de variância unitária, discretizado exatamente, que
corre desde o início do perfil e só é aplicado na janela; as normais saem do mesmo minstd, depois
das do Dryden. A métrica é o RMSE na janela, o que um LQG minimiza. **Não use `math.random` no
Lua**: no ArduPilot ele é o `rand()` da libc, que divide o estado com o ruído do IMU do SITL.

O **evento de planta** (bit 256) muda um parâmetro da física em pleno voo e o desfaz antes do
pouso. O SITL relê `SIM_ENGINE_FAIL`/`SIM_ENGINE_MUL` e os `SIM_FRM_*` a cada passo. O motor fraco
é não linear: `MUL` escala `PWM - 1000`, e 0,95 tira ~10 % do empuxo daquele motor no pairado
(~8 rad/s² em rolagem e arfagem). O campo `Ev` da `WND` registra o estado do evento.

> **Três armadilhas do vento nativo do SITL**, todas tratadas pelo `run_ensaio.py` e que
> precisam entrar como ressalva na monografia:
>
> 1. `SIM_WIND_T` vem em `0` (lei de raiz quadrada) — a 5 m de altitude chegariam só
>    `sqrt(5/60) ≈ 29%` do vento comandado. Os ensaios usam `1` (sem limite).
> 2. `SIM_WIND_TC` vem em **5 s**, e o firmware impõe piso de `0.1 s`
>    ([`SIMState.cpp:307`](ardupilot/libraries/AP_HAL/SIMState.cpp#L307)). Com o padrão, nenhuma
>    rajada sobreviveria. Mesmo em `0.1 s` sobra um passa-baixa de ~1,6 Hz em série: a rajada e o
>    vento médio passam intactos, a turbulência é atenuada acima disso.
> 3. O SITL ignora vento nos **primeiros 5 s** de simulação, então `WND_T0` reserva essa folga.

### Adicionar um controlador

Os controladores são configuração, não código: cada arquivo em
[`ensaios/controladores/`](ensaios/controladores/) diz quais parâmetros aplicar e o que esperar
no log. Um controlador novo do lado do firmware é um backend do `AC_CustomControl`; os do TCC
estão descritos nas seções seguintes.

Um YAML de controlador pode trazer a chave `cenarios:`, uma lista de prefixos de cenário
(`rolagem`, `ruido/rnd20`, `guinada/torque_degrau`). Sem ela, o controlador voa todos os cenários;
com ela, só os que começam por algum prefixo. Os controladores de verificação (`cc_pid`,
`cc_pid_ff`, `cc_lqi_sem_ff`, `nativo_lqi`) a usam para ficar nos 18 cenários do lote original,
para os quais existem; assim o lote sem argumentos continua sendo "tudo o que faz sentido".

### O controlador proposto: LQI e LQR + ESO

O backend `AC_CustomControl_LQR` (`CC_TYPE 3`) tem duas variantes, escolhidas por `CC3_MODO`:

| | lei de controle |
|---|---|
| `0` — LQI | `u = u_ff + k_I·x_I + k_θ·e_θ + k_ω·e_ω`, com `x_I` a integral de `e_θ` |
| `1` — LQR + ESO | `u = u_ff + k_θ·e_θ + k_ω·e_ω − d̂/b`, com `d̂` de um observador de estado estendido de `[ω, d]` |

Nas duas, `u_ff = (α_t + a·ω_t)/b` inverte o modelo do Cap. 4 (`ω̇ = −a·ω + b·u + d`), e o alvo
de taxa passa pelo mesmo passa-baixas do giroscópio (`CC3_FLTW`) antes de virar `e_ω` — sem isso,
o atraso do filtro do giroscópio vira, num degrau, um erro de taxa que não existe.

O caminho, do projeto ao voo:

1. **Projeto** — [`modelagem/projeto.py`](modelagem/projeto.py) calcula os ganhos (LQR discreto
   pela equação de Riccati; ESO por Ackermann) com a regra: mesma frequência de cruzamento do
   nativo, margem de fase não menor que a dele, amortecimento dominante ≥ 0,7, e, entre os pontos
   que passam, o modo mais lento mais rápido. A análise linear não transcreve equações: lineariza
   os espelhos em Python de [`modelagem/controladores.py`](modelagem/controladores.py) por
   perturbação e fecha a malha com o modelo, o passa-baixas do giroscópio e o segurador de ordem
   zero. Saem as figuras e tabelas `projeto_*` do Cap. 5 e, com `--escrever-yaml`, os YAMLs
   `cc_lqi`, `cc_eso`, `cc_lqi_mesmo_ruido` (a guinada com o ganho de ruído do nativo, no lugar
   da mesma banda), `nativo_lqi` (a cascata do nativo com os ganhos do LQI — a prova de
   equivalência; só existe em rolagem e arfagem) e o par da alimentação direta, abaixo. O modelo
   do projeto é o de um voo fixo (`VOO_DO_PROJETO`), para os YAMLs regerados serem os que voaram.
2. **Firmware** — o backend vive em [`firmware/`](firmware/), não num fork do ArduPilot.
   [`firmware/instalar.sh`](firmware/instalar.sh) cria os links simbólicos dos dois arquivos em
   `ardupilot/libraries/AC_CustomControl/` e aplica [`ardupilot.patch`](firmware/ardupilot.patch)
   (as quatro edições do frontend: `CUSTOMCONTROL_MAX_TYPES 3`, `CONT_LQR`, o `case` em `init()` e
   o grupo `3_` dos `CC3_*`; e a alimentação direta opcional do `cc_pid`, `CC2_FF*`). Roda quantas vezes quiser; `--remover` devolve o submódulo ao commit
   original. O `run_ensaio.py` recusa um controlador `espera_cc_tipo: 3` sem o backend instalado.
3. **Voo e conferência** — o backend registra `CCLR`/`CCLP`/`CCLY` (integral, `u_ff`, `ω̂`, `d̂`),
   que viram as colunas `x_i_*`, `u_ff_*`, `w_hat_*` e `d_hat_*` do `series.csv`; o painel
   sobrepõe `d̂` ao torque injetado nos voos com ESO. O `validacao.py` simula cada voo com o
   espelho em Python e escreve `validacao_proposto.tex`: é a conferência de que o C++ faz a mesma
   conta (desvio máximo: LQI e ESO a ±1,9 % do modelo; LQG a −1,6…+4,9 %, média +2 %, em rolagem
   e arfagem, e a ±1,8 % em guinada).

O espelho em Python é a referência: para mudar a lei de controle, mude
`modelagem/controladores.py`, depois o `firmware/AC_CustomControl_LQR.cpp` na mesma ordem.

#### A alimentação direta e a comparação justa

No degrau de atitude, quase toda a vantagem do LQI vem da alimentação direta, e não da
realimentação: o nativo voa com `ATC_RAT_*_FF = D_FF = 0`. Ligar esses dois não seria a correção
certa, porque o `AC_PID` os multiplica pelo alvo de taxa **inteiro**
([`AC_PID.cpp`](ardupilot/libraries/AC_PID/AC_PID.cpp), `_target * _kff` e a derivada dele), que
inclui a correção do erro de ângulo — mudariam também a realimentação e a resposta a torque. Por
isso o `ardupilot.patch` dá ao backend PID de exemplo a **mesma** alimentação direta pura do LQI
(`CC2_FF`, `CC2_FF_A`, `CC2_FF_B_*`, desligada por padrão), e dois controladores fecham um 2×2:

| | sem alimentação direta | com alimentação direta |
|---|---|---|
| **PID** (ganhos do AutoTune) | `cc_pid` | `cc_pid_ff` |
| **LQI** | `cc_lqi_sem_ff` (`CC3_FF = 0`, `CC3_FLTW = 0`) | `cc_lqi` |

O `cc_lqi_sem_ff` desliga também o filtro do alvo de taxa, que só faz sentido junto com a
alimentação direta: sozinho, ele só atrasa a referência. Nos ensaios de torque o alvo fica parado,
`u_ff = 0`, e os dois são idênticos ao `cc_pid` e ao `cc_lqi`; a comparação muda só nos nominais
(`resultados_alimentacao_direta.tex` e `comparacao_ff.pdf`).

### O caminho do ruído e o ângulo do AHRS

Quatro fatos do SITL, conferidos no código e nos logs (`python modelagem/validacao.py --ruido
--figuras monografia/figuras`, figura `validacao_ruido_ahrs.pdf`), mudam o modelo de medida e de
atuação:

1. **O giroscópio do IMU0 é a média de 8 sub-amostras** (`INS_FAST_SAMPLE`,
   `AP_InertialSensor_SITL::generate_gyro`, `gyro_accum /= nsamples`): o desvio padrão do ruído a
   1 kHz é `SIM_GYRn_RND × acelerador / √3 / √8`, e não `/ √3`. O `validacao.ruido_do_voo`
   superestimava o ruído em 2,8×; a RND 20 ele é ~2,5 °/s. As razões entre controladores (o
   2,4×/2,5× do LQI) não mudam.
2. **O ângulo do AHRS integra o mesmo ruído, sem o passa-baixas de 20 Hz**: o EKF3 propaga a
   atitude com os delta-ângulos crus (`AP_InertialSensor_Backend.cpp:333-365`). A derivada do
   ângulo tem o ruído branco que o giroscópio filtrado não tem; o modelo com isso bate com o SITL
   a ±9 % (giroscópio filtrado, derivada do ângulo e esforço, nos três eixos), e sem isso erraria
   3×. Contra a atitude verdadeira (mensagem `SIM`), o erro do ângulo tem autocorrelação de 2 a
   3,7 s, mais uma deriva lenta do próprio EKF, maior na guinada.
3. **O backend lê a atitude de um laço atrás**: `run_custom_controller` roda antes de `read_AHRS`
   (`Copter.cpp:115-127`). No log, o `CCLR.Eth` casa com `ANG.DesRoll − ANG.Roll` deslocado de
   exatamente um laço (resíduo de 2–3 mdeg contra 5–14 nos vizinhos).

4. **O esforço vale no mesmo laço**: enquanto o laço principal espera o próximo período,
   `SITL_State::_fdm_input_step` integra o modelo físico com a última saída dos motores e só então
   gera a amostra seguinte do giroscópio; o `u` calculado no laço k age já em [t_k, t_k+1]. Até
   01/11, o `malha.simular` o aplicava um laço depois (a fila de atuação tinha `atraso_lacos + 1`
   posições), e a validação só dava ao LQG o ângulo atrasado do item 3. Nos controladores que só
   usam o giroscópio, um erro compensava o outro (o Cap. 4 ficava 1,4–3,4 % abaixo do SITL); no
   LQG, que prevê o efeito do próprio `u`, não: o replay ficava 5–10 % pessimista em rolagem e
   arfagem. Corrigidos os dois, a validação do Cap. 4 bate a 2,3 % no desvio máximo e a 0,08 dB
   (mediana) e 1,2 dB (máximo) na resposta em frequência, contra 1,9 dB antes. Um laço a mais de
   atuação piora a frequência para 0,57 dB e 2,2 dB. O projeto (a planta linear e o portão) já
   tinha a temporização certa: os pontos escolhidos e os YAMLs que voaram não mudam (conferido a
   20 e a 10 Hz).

`malha.simular` (`ruido_angulo`, `tau_ahrs`, `atraso_angulo`, `vies_giro`) e
`projeto.planta_com_filtro` (`ruido_angulo`, `tau_c`, `atraso_angulo`) modelam os itens 2 e 3, com
as opções desligadas por padrão; o item 4 é a atuação de ambos com `atraso_lacos = 0`.
`projeto.PLANTA_REAL` é a planta com os itens 2 e 3 ligados: é nela que o LQG é projetado, que
todos os controladores são comparados no portão e que todos são validados contra os voos
(`validacao.validar`).

### O LQG e o portão

O backend `AC_CustomControl_LQG` (`CC_TYPE 4`, parâmetros `CC4_*`) usa a lei do LQR + ESO,
`u = u_ff + k_θ·ê_θ + k_ω·ê_ω − d̂/b`, sobre os estados de um **filtro de Kalman em regime** que
modela o caminho de medida acima. Em coordenadas de erro, os estados são `[e_θ, e_ω, f₁, f₂, d, η,
b_g, z]`: o Butterworth do giroscópio (`f`), a perturbação (`d`, passeio aleatório), o erro do
ângulo do AHRS (`η`, a integral do ruído), o viés residual que o EKF deixa no giroscópio (`b_g`) e o
ângulo do laço anterior (`z`). As medidas são o erro de ângulo lido e o erro de taxa filtrado. O
espelho é `controladores.LQG`; Φ e Γ são calculados a bordo a partir de `CC4_A`, `CC4_B`,
`CC4_FLTW`, `CC4_TAUC` e do período do laço, pela mesma série de Taylor (`expm_taylor`); K e L
(`CC4_KTH`, `CC4_KW`, `CC4_L1T…L8T`, `CC4_L1W…L8W`) vêm do projeto.

O que o LQG usa e a cascata descarta: a entrada `u`, conhecida; o modelo do filtro de 20 Hz, que
custa 33° de fase no cruzamento da rolagem; e o ângulo do AHRS, que não passa por ele (um laço
atrasado custa só 7°). O ganho de Kalman sai da Riccati dual, com o ruído do giroscópio da física
(item 1) e a intensidade da perturbação `q_d` como botão — subir `q_d` é a recuperação da malha
(LTR) na entrada da planta.

```bash
python modelagem/projeto.py --portao --cache ensaios/resultados/projeto_portao_cache.pkl \
    --escrever-yaml --tabelas monografia/tabelas --figuras monografia/figuras   # ~20 min
```

O `--portao` varre, na planta real, quatro famílias com o mesmo número de botões — **PID** (a
cascata do nativo com ganhos livres, FLTE inclusive), **LQI**, **LQR + ESO** e **LQG** —, desenha
as fronteiras rejeição × ruído (`projeto_fronteira.pdf`) e decide o portão (`projeto_portao.tex`):

- **P1, ganho real:** o melhor LQG com ruído no esforço ≤ o do nativo (na análise com ruído
  branco e numa simulação de 600 s no tempo), margem de fase e margem com a planta alterada ≥ as
  do nativo, nada pior que o nativo (2 %), e ≥ 10 % melhor na métrica primária (pico do pulso na
  rolagem, contra o LQI; pico do degrau de torque na guinada, contra o nativo);
- **P2, atribuição:** ≥ 5 % melhor que o melhor ponto de cada família de estrutura fixa sob as
  mesmas restrições (e, relaxado, só com ruído e margens);
- **P3, margens:** algum ponto do LQG cumpre P1.

Os achados da varredura, que entram no Cap. 5:

- o **AutoTune já está no ótimo da família PID** quanto ao pico do pulso de rolagem; a
  recuperação rápida da guinada (1–2 s em vez de 22 s) um PID ressintonizado também alcança
  (I/P = 3 em vez de 0,1), mas não o pico do degrau de torque;
- o ganho do LQG em rolagem vem da fase do ângulo do AHRS; em guinada, do corte de ruído com o
  modelo, e só existe se o estimador descontar o ângulo do AHRS por ~1,5 s ou mais. Por isso o
  `CC4_TAUC` é por eixo (0,3 s em rolagem e arfagem, 2,0 s em guinada; ver o comentário de
  `projeto.TAU_ESTIMADOR`): com o τ físico (3 s) em todos os eixos, o estimador trocava um viés do
  giroscópio por um erro de ângulo estático (73 mdeg em rolagem no SITL).

O `cc_lqg` segue a regra do LQI e do ESO (mesma banda do nativo); o `cc_lqg_mesmo_ruido` é o
escolhido pelo portão, nos três eixos.

> **`CC_TYPE` sozinho não liga o controlador.** O `AC_CustomControl` só assume quando uma chave
> auxiliar de RC vai para HIGH ([`RC_Channel_Copter.cpp:671`](ardupilot/ArduCopter/RC_Channel_Copter.cpp#L671),
> opção `109`). Sem isso o voo sai com o PID nativo enquanto os parâmetros dizem o contrário — em
> silêncio. Por isso o YAML do `cc_pid` traz `RC6_OPTION: 109` e um bloco `rc_override`, e a
> análise confere no log, pela mensagem `CC`, se o backend esteve mesmo ativo durante o ensaio.

### Dois achados que mudam o texto da monografia

**1. No SITL, vento horizontal NÃO perturba a malha de atitude — não é "pouco", é zero.** Cada
rotor calcula a entrada de ar como `MAX(0, -motor_vel.projected(thrust_vector).z)`
([`SIM_Motor.cpp:81`](ardupilot/libraries/SITL/SIM_Motor.cpp#L81)): só a componente **axial**, ao
longo do eixo de empuxo. Um vento horizontal projeta quase nada nesse eixo e, sobretudo, é
**uniforme sobre a aeronave** — os quatro rotores recebem a mesma contribuição. Efeito igual nos
quatro significa variação de empuxo total e nunca diferencial; sem diferencial não há momento. O
arrasto de corpo, por sua vez, é aplicado no centro de massa: braço zero, torque zero.

Medido aqui, num mesmo voo, comparando a rajada contra trechos calmos:

| trecho | \|erro\|max | \|p\|max |
|---|---|---|
| calmo antes | 0,0312° | 0,206°/s |
| **rajada de 10 m/s** | **0,0164°** | **0,178°/s** |
| calmo depois | 0,0140° | 0,167°/s |

A aeronave fica *mais quieta* durante a rajada que antes dela. Já um pulso de torque de
15 rad/s² por 0,3 s produz **2,80° de desvio** e 1,86 s de recuperação — cem vezes mais.

A `proposta.tex` antecipa isso: o torque parasita de centro de pressão deslocado "havia sido
deliberadamente desprezado" no modelo nominal. A consequência prática é a divisão de trabalho
entre os ensaios:

- **vento** → mede deriva de posição, e põe o controlador a operar fora do equilíbrio simétrico;
- **torque** → mede rejeição de distúrbio da malha de atitude, que é o objeto do TCC;
- **degrau de atitude** → mede rastreamento, que é como o vento realmente chega à malha de
  atitude num voo em `LOITER`: o controlador de posição converte o empurrão em comando de atitude.

Na monografia isso precisa ser escrito com precisão. Não é "vento não perturba atitude" — num
quadrotor real ele perturba, por batimento de pá e centro de pressão deslocado. É "no modelo de
multirrotor do SITL o vento entra como força no centro de massa, e portanto não produz distúrbio
de atitude".

**2. Ensaio marcado no relógio de simulação, nunca no de parede — e sempre a leitura mais
recente.** Com `--speedup 5` o SITL entrega ~4,5x de fato, e sob carga de log chegou a ~1,4x. Um
perfil temporizado por `time.sleep(duração/speedup)` terminava cedo, o pouso começava no meio do
ensaio, e os 30° de rolagem que o controlador de posição comanda para frear a deriva entravam no
log como se fossem resposta ao distúrbio.

Ler o relógio do veículo não bastou: consumindo a **primeira** mensagem da fila, o atraso
*cresce* ao longo do voo, porque o laço consome `ATTITUDE` um pouco mais devagar do que o veículo
os produz. Medido: o degrau atrasava 0,50 s e o retorno, 1,82 s — erro de taxa, não deslocamento
fixo. Esvaziando a fila antes de cada leitura, os desvios caem para 0,12 s e 0,16 s, que é o piso
imposto pela taxa de comando de 10 Hz. O distúrbio, temporizado pelo relógio do próprio injetor
Lua, cai em 0,00 s.

### Frame X, o mesmo do Gazebo

Os ensaios rodam com `-f X`, que carrega `copter-X.parm` (`FRAME_TYPE 1`) e a física `--model X`.
É a mesma configuração de motores do ambiente Gazebo (`gazebo-iris.parm`), então os dois medem o
mesmo veículo.

Onde cada motor fica, contado do nariz no sentido horário
([`AP_MotorsMatrix.cpp:592`](ardupilot/libraries/AP_Motors/AP_MotorsMatrix.cpp#L592)):

| saída | posição |
|---|---|
| `RCOU.C1` | 45° — dianteiro direito |
| `RCOU.C2` | −135° — traseiro esquerdo |
| `RCOU.C3` | −45° — dianteiro esquerdo |
| `RCOU.C4` | 135° — traseiro direito |

O torque de rolagem sai daí: `m1 − m2 − m3 + m4`; o de arfagem, `m1 − m2 + m3 − m4` (frente
menos trás). O de guinada sai do sentido de giro — `M1` e `M2` giram no sentido anti-horário
(CCW), `M3` e `M4` no horário (CW) — e é `(m1 + m2) − (m3 + m4)`: o torque de reação de um rotor
CCW gira a aeronave no sentido horário, então acelerar o par CCW guina para a direita. O painel e a vista de cima do GIF **derivam**
essas fórmulas e a posição dos rotores da geometria em `analise/estilo.py`, indexada pelo
`FRAME_TYPE` que o `params.parm` do ensaio registrou. Chumbar a fórmula combinaria eixos
diferentes assim que o frame mudasse, e o gráfico mostraria ruído com cara de sinal.

Medido sob torque sustentado de 10 rad/s²: os dois motores da direita sobem (1613,8 e 1614,1) e os
dois da esquerda descem (1594,8 e 1595,1).

### Relógio monotônico, nunca o de parede

Todo tempo decorrido no `run_ensaio.py` usa `time.monotonic()`. O relógio de parede do WSL
ressincroniza e pode **andar para trás** no meio de uma corrida — aconteceu aqui, na virada do dia.
Com `time.time()`, um "tempo decorrido" vira negativo, o campo `uint32` de `time_boot_ms` do
`SET_ATTITUDE_TARGET` estoura e o ensaio morre em pleno voo com um `struct.error`. Os laços de
timeout também esperariam a mais. Só o carimbo da pasta de resultado vem de `datetime.now()`, que
é data de calendário e não intervalo.

### A atitude vem da `ANG`, não da `ATT`

No ArduCopter 4.8 a mensagem `ATT` é gravada **sempre a 10 Hz**
([`Copter.cpp`](ardupilot/ArduCopter/Copter.cpp), `ten_hz_logging_loop`). Com o bit
`ATTITUDE_FAST` do `LOG_BITMASK`, a atitude alvo e a medida na taxa do laço vão na `ANG`
(`AC_AttitudeControl::Write_ANG`), com os mesmos campos. A análise lê a `ANG` e monta a grade
do `series.csv` a 400 Hz. Antes disso ela interpolava a `ATT` de 10 Hz, e um degrau de rolagem,
que com os ganhos do AutoTune sobe em ~45 ms, cabia em uma ou duas amostras: o sobressinal do
mesmo degrau saía entre 4% e 22% conforme a posição das amostras, e o desvio do pulso, até 13%
menor. Todas as pastas de resultado foram reanalisadas em 27/10/2026; as medidas citadas mais
acima neste README, de antes disso, ainda são as da `ATT`.

### O SITL sem janela, e por que alguns voos falhavam

O `sim_vehicle.py` não roda o binário direto: chama o `run_in_terminal_window.sh` do ArduPilot,
que abre um xterm sempre que há `DISPLAY`. O `run_ensaio.py` aponta `SITL_RITW_TERMINAL` para
[`ensaios/sitl_sem_janela.sh`](ensaios/sitl_sem_janela.sh), que roda o binário em segundo plano e
grava a saída dele no `sitl.log` do ensaio — no xterm ela se perdia junto com a janela. O wrapper
mantém um processo pai vivo de propósito: o SITL se encerra sozinho quando perde o pai
([`SITL_State.cpp:95`](ardupilot/libraries/AP_HAL_SITL/SITL_State.cpp#L95)).

Os lotes perdiam ~1 voo em cada 5 a 15, sempre logo depois do reboot que ativa `SCR_ENABLE` e
`CC_TYPE`, com "o processo do SITL morreu" ou, mais raro, "os parâmetros WND_* nunca apareceram".
As duas mensagens vêm de um **bug do SITL do ArduPilot**, que o orquestrador disparava:

1. Antes de reiniciar, o `AP_Vehicle::reboot()` chama `hal.serial(0)->flush()` para garantir que o
   `COMMAND_ACK` saia ([`AP_Vehicle.cpp:983`](ardupilot/libraries/AP_Vehicle/AP_Vehicle.cpp#L983)).
2. O `UARTDriver::_flush()` do SITL repete `_timer_tick()` até esvaziar o buffer ou passar 1 s de
   tempo **simulado** ([`UARTDriver.cpp:272`](ardupilot/libraries/AP_HAL_SITL/UARTDriver.cpp#L272))
   — só que nada nesse laço faz o tempo simulado andar.
3. Se o cliente TCP já fechou a conexão, o `_timer_tick()` não escreve nada, o buffer nunca esvazia
   e o laço nunca acaba: o SITL fica a 100% de CPU, sem reiniciar e ignorando SIGTERM.

O orquestrador antigo fechava a conexão logo depois de mandar o pedido de reboot. Conforme quem
chegava primeiro ao SITL, ele travava no laço acima ou descartava o pedido e seguia sem reiniciar
(daí os `WND_*` que nunca apareciam). E o travado era dado como "morto" porque a verificação
procurava `arducopter --model`, e o processo que não reiniciou ainda tinha a linha de comando do
primeiro boot, `arducopter -w --model ...`.

Evidência: com o reboot antigo e a saída do binário capturada, o travamento apareceu em 3 de 8 e
2 de 7 ciclos; rodando o SITL dentro do gdb, o backtrace mostrou a thread principal presa em
`AP_Vehicle::reboot → UARTDriver::_flush → handle_writing_from_writebuffer_to_device`. Com o reboot
novo, 0 de 40 e 0 de 30 ciclos, e 3 de 3 voos de três eixos. O reboot novo espera o `COMMAND_ACK`,
lê até o **próprio SITL** fechar a conexão (ele fecha ao se reexecutar) e confere o reboot pela
saída do `-w` da linha de comando; o binário é achado por uma variável de ambiente própria, e não
pelo nome; e o encerramento espera o binário sair, com SIGKILL se ele não atender. De quebra, cada
ensaio ficou ~6 s mais curto.

### O perfil de voo tem três atos

Cada ensaio segue a mesma sequência, e a ordem importa:

| tp | o que acontece | o que mede |
|---|---|---|
| `t_degrau_s` | rolagem ou arfagem 0 → 10°, e/ou guinada +30° | **transitório**: subida, sobressinal, acomodação |
| `t_retorno_s` | volta a nivelado e ao rumo inicial | — |
| `WND_GT0` / `WND_TQT0` | distúrbio | **rejeição**: desvio máximo, recuperação, RMSE |

O retorno a 0° não é detalhe. Sem ele, o distúrbio incidiria sobre uma aeronave inclinada 10° e
acelerando de lado a ~4,3 m/s — ponto de operação difícil de defender numa banca, e com deriva
crescendo até o pouso. Com ele, o distúrbio chega a **−0,01° de rolagem e 0,92 m/s**. Deixar
`t_retorno_s` de fora do YAML mantém o degrau até o fim, que é o comportamento antigo.

---

## Encerrar

1. `Ctrl+C` no terminal do **SITL**;
2. depois feche o **Gazebo**.

Nessa ordem. Se o Gazebo cair primeiro, o SITL trava esperando o próximo estado físico.

---

## Solução de problemas

| Sintoma | Causa | Correção |
|---|---|---|
| Iris aparece no Gazebo mas o SITL fica esperando | Plugin não compilado ou `GZ_SIM_SYSTEM_PLUGIN_PATH` errado | Conferir se existe `ardupilot_gazebo/build/libArduPilotPlugin.so` |
| Gazebo com tela preta ou travando | OpenGL via GPU falhando no WSLg | Descomentar `LIBGL_ALWAYS_SOFTWARE=1` em `run-gazebo.sh` |
| `mavproxy.py: command not found` | Venv não ativo | `source ~/venv-ardupilot/bin/activate` antes de rodar |
| `sim_vehicle.py: command not found` | `PATH` sem `Tools/autotest` | Use `run-sitl.sh`, que já exporta o `PATH` |
| *Pre-arm*: EKF ou GPS nunca prontos | Simulação do Gazebo pausada | Clique em ▶ no Gazebo (o `-r` do script já deveria evitar isso) |
| `takeoff` não faz nada | Não está em GUIDED, ou não está armado | `mode guided` → `arm throttle` → `takeoff 5` |
| `AP: Frame: UNSUPPORTED` e `PreArm: Motors: Check frame class and type` | `FRAME_CLASS`/`FRAME_TYPE` não aplicados (ver nota abaixo) | `param set FRAME_CLASS 1`, `param set FRAME_TYPE 1`, `reboot` |
| Erro de modelo/mundo não encontrado | `GZ_SIM_RESOURCE_PATH` ausente | Rodar pelo `run-gazebo.sh`, não pelo `gz sim` direto |
| QGroundControl não conecta | NAT do WSL bloqueando UDP | `networkingMode=mirrored` no `.wslconfig`, ou `output add udp:IP_DO_WINDOWS:14550` |
| `No such file: modules/waf/waf-light`, ou `source not found: 'modules/littlefs/lfs.c'` | Submódulos não inicializados, ou inicializados pela metade | Ver o **passo 0** dos pré-requisitos |
| Ensaio: "ja ha algo escutando na porta 5760" | Outro SITL aberto (o `run-sitl.sh`, por exemplo) | Feche-o, ou rode o ensaio com `--instancia 1` |
| Ensaio: pasta `_falhou` | O voo não terminou | O motivo está no `falha.txt`, e a saída do SITL no `sitl.log` ao lado |

---

## Licença

GPLv3 (`LICENSE`), a mesma do ArduPilot. O submódulo `ardupilot/` e o `ardupilot_gazebo/`
seguem as licenças deles. O material do minicurso (`Material_participante/`) é de terceiros e
fica fora do repositório.
