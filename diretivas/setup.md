# Guia Passo a Passo — Ambiente WSL2 + ArduPilot SITL + Gazebo

**Objetivo:** partir de um Windows limpo e chegar a um quadrotor (Iris) voando no Gazebo, controlado pelo ArduPilot SITL padrão (sem nenhuma modificação no firmware ainda). Este é exatamente o entregável da **Fase 0** do seu roteiro de TCC.

**Tempo estimado:** 2 a 4 horas (a maior parte é download e compilação).

**Arquitetura final:**

```
┌──────────────────────── Windows ────────────────────────┐
│  QGroundControl (opcional, GCS gráfica)                 │
│  ┌──────────────────── WSL2 (Ubuntu) ────────────────┐  │
│  │  ArduPilot SITL  ◄──JSON──►  Gazebo Harmonic      │  │
│  │       ▲                      (física + 3D)        │  │
│  │       └── MAVProxy (console de comando)           │  │
│  └───────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────┘
```

---

## Pré-requisitos no Windows

- **Windows 11** (recomendado) ou **Windows 10 versão 19044+**. O Windows 11 traz o **WSLg**, que exibe janelas gráficas do Linux (o Gazebo) automaticamente, sem gambiarras de servidor X.
- **Virtualização habilitada na BIOS/UEFI** (Intel VT-x ou AMD-V). Verifique no Gerenciador de Tarefas → Desempenho → CPU → "Virtualização: Habilitado". Se estiver desabilitada, ative na BIOS antes de continuar.
- Pelo menos **~30 GB livres** em disco e **8 GB de RAM** (16 GB é confortável).
- GPU com driver atualizado (o WSLg usa a GPU do Windows para acelerar o OpenGL do Gazebo).

---

## Parte 1 — Instalar o WSL2 com Ubuntu

### 1.1. Instalar

Abra o **PowerShell como Administrador** (botão direito → "Executar como administrador") e rode:

```powershell
wsl --install -d Ubuntu-22.04
```

> **Por que 22.04?** É a versão mais testada com o toolchain do ArduPilot e com o Gazebo Harmonic. O Ubuntu 24.04 também funciona, mas o 22.04 dá menos surpresa.

Se o comando reclamar que o WSL não está habilitado, rode primeiro:

```powershell
wsl --install
```

e reinicie o computador quando pedido; depois repita o comando com `-d Ubuntu-22.04`.

### 1.2. Criar o usuário Linux

Ao terminar, uma janela do Ubuntu abre pedindo **username** e **senha**. Escolha um nome curto sem espaços (ex.: `joao`). Essa senha será usada em todo `sudo`.

### 1.3. Verificar que é WSL **2** (não 1)

No PowerShell:

```powershell
wsl -l -v
```

A coluna `VERSION` deve mostrar `2`. Se mostrar `1`:

```powershell
wsl --set-version Ubuntu-22.04 2
```

### 1.4. Atualizar o WSL e o Ubuntu

No PowerShell:

```powershell
wsl --update
```

Dentro do Ubuntu (abra pelo menu Iniciar → "Ubuntu 22.04"):

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y git build-essential
```

### 1.5. Testar o suporte gráfico (WSLg)

Ainda no Ubuntu:

```bash
sudo apt install -y x11-apps
xeyes
```

Se aparecer uma janelinha com dois olhos que seguem o mouse, o suporte gráfico está OK — o Gazebo vai abrir do mesmo jeito. Feche com `Ctrl+C`.

> **Se não abrir nada:** atualize o driver da sua GPU no Windows, rode `wsl --update` de novo e reinicie o WSL com `wsl --shutdown` (no PowerShell) antes de reabrir o Ubuntu.

### 1.6. Regra de ouro do WSL: trabalhe dentro do Linux

Mantenha **todos os repositórios dentro do filesystem do Linux** (ex.: `/home/joao/...`), **nunca** em `/mnt/c/...`. Compilar a partir de `/mnt/c` é 10–50× mais lento e causa erros esquisitos de permissão.

Para acessar seus arquivos do Linux pelo Windows Explorer, digite na barra de endereço: `\\wsl$\Ubuntu-22.04\home\seu_usuario`.

---

## Parte 2 — Instalar o ArduPilot e rodar o primeiro SITL

### 2.1. Clonar o repositório (com submódulos)

```bash
cd ~/faculdade/TCC
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

> O `--recurse-submodules` é essencial — o ArduPilot depende de vários submódulos (MAVLink, ChibiOS etc.). O clone completo leva alguns minutos.
>
> Se esquecer a flag, corrija com: `git submodule update --init --recursive`

### 2.2. Rodar o script de instalação do ambiente

```bash
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Esse script instala o compilador, Python, MAVProxy e tudo mais que o build precisa. Leva um bom tempo (10–30 min). Ao final:

```bash
. ~/.profile
```

> **Importante:** feche e reabra o terminal do Ubuntu depois disso (ou rode o `. ~/.profile`), para que o `PATH` passe a incluir as ferramentas do ArduPilot (`sim_vehicle.py` etc.).

### 2.3. Verificar a instalação

```bash
which sim_vehicle.py
```

Deve imprimir algo como `/home/joao/faculdade/TCC/ardupilot/Tools/autotest/sim_vehicle.py`. Se não imprimir nada, o `.profile` não foi carregado — feche e reabra o terminal.

### 2.4. Compilar e rodar o primeiro SITL (sem Gazebo ainda)

```bash
cd ~/faculdade/TCC/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter --console --map -w
```

O que acontece:
1. Na **primeira vez**, ele compila o firmware para SITL (5–15 min). Nas próximas, só recompila o que mudou.
2. Abre a janela do **console do MAVProxy** e um **mapa**.
3. A flag `-w` zera os parâmetros para o padrão (use só na primeira vez).

Espere o console mostrar mensagens de inicialização e o GPS travar (aparece algo como `GPS: u-blox... 3D fix` e os pré-arm checks passarem — pode levar ~30 s).

### 2.5. Primeiro voo (no terminal do MAVProxy)

Digite, um comando por vez, no prompt do MAVProxy:

```
mode guided
arm throttle
takeoff 10
```

O ícone no mapa deve subir para 10 m de altitude. Para pousar:

```
mode land
```

Para encerrar tudo: `Ctrl+C` no terminal onde rodou o `sim_vehicle.py`.

✅ **Checkpoint 1:** se você decolou e pousou, o ArduPilot SITL está funcional. Isso já é metade da Fase 0 do TCC.

---

## Parte 3 — Instalar o Gazebo Harmonic

O simulador nativo do SITL não tem física 3D nem visual. O Gazebo entra como **planta física externa**, conversando com o SITL pela interface JSON.

### 3.1. Adicionar o repositório oficial do Gazebo

No Ubuntu:

```bash
sudo apt update
sudo apt install -y curl lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
```

### 3.2. Instalar o Gazebo Harmonic

```bash
sudo apt install -y gz-harmonic
```

> **Por que Harmonic?** <br>É a versão LTS recomendada pela documentação do ArduPilot e a mais testada com o plugin oficial. (Garden e Ionic também são suportados, mas fique no Harmonic para o TCC.)

### 3.3. Testar o Gazebo sozinho

```bash
gz sim -v4 shapes.sdf
```

Deve abrir uma janela 3D com esferas, cubos e cilindros. Clique no ▶ (play) no canto inferior esquerdo e veja a física rodando. Feche a janela.

> **Se a janela abrir preta, travar ou o Gazebo crashar:** é quase sempre problema de renderização OpenGL no WSL. Force renderização por software:
>
> ```bash
> export LIBGL_ALWAYS_SOFTWARE=1
> gz sim -v4 shapes.sdf
> ```
>
> Se resolver, adicione a linha `export LIBGL_ALWAYS_SOFTWARE=1` ao final do seu `~/.bashrc`. Fica mais lento que com GPU, mas 100% estável.

---

## Parte 4 — Compilar o plugin `ardupilot_gazebo`

Este plugin é a ponte entre o Gazebo e o SITL: ele recebe os comandos de motor do ArduPilot e devolve o estado físico simulado (interface JSON).

### 4.1. Instalar dependências de build

```bash
sudo apt update
sudo apt install -y libgz-sim8-dev rapidjson-dev cmake
```

> `libgz-sim8-dev` é a versão do dev-package correspondente ao **Harmonic** (gz-sim 8). Se um dia migrar para Ionic, seria `libgz-sim9-dev`.

(Opcional, só se for usar câmeras simuladas com streaming mais adiante:)

```bash
sudo apt install -y libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

### 4.2. Clonar e compilar o plugin

```bash
cd ~/faculdade/TCC
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
export GZ_VERSION=harmonic
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Se terminar sem erros, o arquivo `libArduPilotPlugin.so` estará em `~/faculdade/TCC/ardupilot_gazebo/build/`.

### 4.3. Configurar as variáveis de ambiente do Gazebo

O Gazebo precisa saber **onde achar o plugin** e **onde achar os modelos/mundos**. Adicione ao `~/.bashrc`:

```bash
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/faculdade/TCC/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/faculdade/TCC/ardupilot_gazebo/models:$HOME/faculdade/TCC/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

> **Confira:** `echo $GZ_SIM_SYSTEM_PLUGIN_PATH` deve mostrar o caminho do `build`. Esse passo é o erro nº 1 de quem monta esse ambiente — se as variáveis não estiverem setadas, o drone aparece no Gazebo mas **nunca conecta** ao SITL.

---

## Parte 5 — Voar o Iris no Gazebo com o SITL

Agora você vai precisar de **dois terminais do Ubuntu** abertos ao mesmo tempo (abra duas janelas do app Ubuntu, ou use o Windows Terminal com abas).

### 5.1. Terminal 1 — Gazebo

```bash
gz sim -v4 -r iris_runway.sdf
```

Deve abrir o mundo 3D com uma pista e o quadrotor **Iris** parado nela. A flag `-r` já inicia a simulação rodando (play).

> Se aparecer erro de modelo/mundo não encontrado, as variáveis do passo 4.3 não estão ativas neste terminal — rode `source ~/.bashrc` e tente de novo.

### 5.2. Terminal 2 — ArduPilot SITL conectado ao Gazebo

```bash
cd ~/faculdade/TCC/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map
```

O que cada flag faz:
- `-f gazebo-iris` → carrega o conjunto de parâmetros do frame Iris para Gazebo;
- `--model JSON` → diz ao SITL para usar a **interface JSON** e esperar a física vinda de fora (do Gazebo), em vez da física interna;
- `--console --map` → console do MAVProxy + mapa.

No console do MAVProxy, aguarde as mensagens de inicialização. No terminal do Gazebo você deve ver algo como `Waiting for connection...` seguido de conexão estabelecida. Aguarde os **pre-arm checks** passarem (EKF pronto, GPS fix — dá ~30 s).

### 5.3. Decolar

No prompt do MAVProxy (Terminal 2):

```
mode guided
arm throttle
takeoff 5
```

🚁 As hélices giram na janela 3D e o Iris sobe a 5 m. Experimente:

```
# voar até uma posição relativa (também dá para clicar com botão direito no mapa → "Fly to")
mode circle        # voa em círculo
mode guided        # volta ao guiado
mode land          # pousa
```

✅ **Checkpoint 2 (Fase 0 completa):** SITL + Gazebo + MAVProxy funcionando em conjunto. Esta é exatamente a configuração sobre a qual seu `AC_CustomControl` vai rodar mais adiante — nada muda no fluxo, só o firmware que você compilará.

### 5.4. Encerrar corretamente

1. `Ctrl+C` no terminal do `sim_vehicle.py`;
2. Feche a janela do Gazebo (ou `Ctrl+C` no terminal dele).

Sempre nessa ordem — se o Gazebo morrer primeiro, o SITL fica esperando a física e "congela".

---

## Parte 6 (opcional) — QGroundControl no Windows

O MAVProxy resolve tudo, mas uma GCS gráfica ajuda muito a visualizar parâmetros e missões.

### 6.1. Instalar

Baixe o instalador do QGroundControl para **Windows** no site oficial (qgroundcontrol.com → Downloads) e instale normalmente no Windows (não no WSL).

### 6.2. Conectar ao SITL que roda no WSL

Nas versões recentes do WSL2 no Windows 11, o tráfego UDP local costuma atravessar direto: muitas vezes o QGC **conecta sozinho** ao abrir (ele escuta UDP 14550, e o MAVProxy retransmite para `localhost:14550` por padrão).

**Se não conectar automaticamente:**

1. Descubra o IP do **Windows** visto de dentro do WSL:
   ```bash
   ip route show | grep -i default | awk '{ print $3}'
   ```
2. Rode o SITL adicionando uma saída extra para esse IP (troque `172.x.x.x` pelo resultado):
   ```bash
   sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map --out udp:172.x.x.x:14550
   ```
3. O QGC no Windows conecta na hora.

**Alternativa definitiva (Windows 11 22H2+):** ative o modo de rede espelhada do WSL criando o arquivo `C:\Users\SEU_USUARIO\.wslconfig` com:

```ini
[wsl2]
networkingMode=mirrored
```

Depois `wsl --shutdown` no PowerShell e reabra o Ubuntu. Com isso, WSL e Windows compartilham `localhost` e tudo conecta sem `--out`.

---

## Parte 7 — Ambiente Python do TCC (projeto e análise)

Aproveite para já deixar pronto o ambiente da parte matemática (Fases 1–4 do roteiro):

```bash
sudo apt install -y python3-venv
python3 -m venv ~/faculdade/TCC/tcc-venv
source ~/faculdade/TCC/tcc-venv/bin/activate
pip install control numpy scipy sympy matplotlib pandas pymavlink jupyter
```

Teste rápido:

```bash
python3 -c "import control; print(control.__version__)"
jupyter lab --no-browser
```

Copie a URL com token que aparecer e abra no navegador **do Windows** — o Jupyter roda no WSL e você usa pela interface web normalmente.

> Dica: sempre ative o venv (`source ~/faculdade/TCC/tcc-venv/bin/activate`) antes de trabalhar nos notebooks. Não instale essas libs com `sudo pip` no Python do sistema — o script do ArduPilot gerencia o dele e vocês podem entrar em conflito.

---

## Solução de problemas frequentes

| Sintoma | Causa provável | Correção |
|---|---|---|
| `sim_vehicle.py: command not found` | `.profile` não recarregado após o script de prereqs | Feche e reabra o terminal, ou `. ~/.profile` |
| Erros de submódulo no build | Clone sem `--recurse-submodules` | `git submodule update --init --recursive` |
| Gazebo abre tela preta / crasha | OpenGL via GPU falhando no WSLg | `export LIBGL_ALWAYS_SOFTWARE=1` (e põe no `.bashrc`); atualizar driver de vídeo no Windows e `wsl --update` |
| Iris aparece no Gazebo mas não arma / SITL "esperando" | Plugin não carregado (variáveis `GZ_SIM_*` ausentes) ou Gazebo aberto antes do `source ~/.bashrc` | Conferir `echo $GZ_SIM_SYSTEM_PLUGIN_PATH`; reabrir o terminal do Gazebo |
| `cmake` não acha `gz-sim8` | `libgz-sim8-dev` não instalado ou `GZ_VERSION` errado | `sudo apt install libgz-sim8-dev` e `export GZ_VERSION=harmonic` antes do `cmake` |
| Pre-arm: `EKF` / `GPS` não fica pronto | Simulação do Gazebo pausada | Clique em ▶ na janela do Gazebo (ou use `-r` ao abrir) |
| Tudo muito lento | Repositórios em `/mnt/c/...` | Mova para `~/` (filesystem nativo do Linux) |
| QGC não conecta | NAT do WSL bloqueando UDP | Usar `--out udp:IP_DO_WINDOWS:14550` ou `networkingMode=mirrored` |

---

## Resumo dos comandos do dia a dia

Depois de tudo instalado, sua rotina de simulação se resume a:

```bash
# Terminal 1
gz sim -v4 -r iris_runway.sdf

# Terminal 2
cd ~/faculdade/TCC/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --map

# No MAVProxy
mode guided
arm throttle
takeoff 5
```

## Próximos passos (ligando com o roteiro do TCC)

1. **Ler os logs:** cada voo gera um `.bin` em `~/faculdade/TCC/ardupilot/ArduCopter/logs/`. Escreva o notebook que abre um log com `pymavlink` e plota roll/pitch — é o segundo entregável da Fase 0.
2. **Ler as docs:** "Copter Attitude Control" e "Adding a Custom Attitude Controller" em ardupilot.org/dev.
3. **Explorar parâmetros:** no MAVProxy, `param show ATC_RAT_RLL*` mostra os ganhos do PID nativo que será sua linha de base.