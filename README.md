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

Não precisa fazer nada à mão: o `sim_vehicle.py` compila sozinho na primeira execução
(5–15 min; nas seguintes só recompila o que mudou). Para forçar:

```bash
cd ~/faculdade/TCC/ardupilot
./waf configure --board sitl && ./waf copter
```

O build termina com `'copter' finished successfully` e produz
`ardupilot/build/sitl/bin/arducopter` (~5,6 MB) — é esse binário que o `sim_vehicle.py` executa.

Para acompanhar um build em andamento: `pgrep -af "waf copter"` diz se ainda roda, e
`find build/sitl -name '*.o' | wc -l` conta o progresso (um Copter completo dá ~1400 objetos).

### 3. Ambiente Python

O MAVProxy vive no venv `~/venv-ardupilot`. Se o SITL reclamar
`mavproxy.py: command not found`, ative-o antes de subir a simulação:

```bash
source ~/venv-ardupilot/bin/activate
```

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
arm throttle
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
> **não** movem o drone — o SITL apenas *recebe* o vento reportado pelo simulador externo. Para
> experimentos de rejeição de distúrbio no Gazebo, use um plugin de vento no `.sdf` do mundo, ou
> rode esses ensaios no SITL puro (sem `--model JSON`), onde os `SIM_WIND_*` valem.

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
