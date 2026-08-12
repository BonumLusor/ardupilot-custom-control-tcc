# Ambiente de trabalho no VSCode — ArduPilot + Gazebo + QGroundControl

**Objetivo:** parar de gerenciar 5 janelas soltas. O VSCode vira o **centro de tudo** — editor de C++ e Python, terminais integrados, git — e sobe a simulação inteira (Gazebo + SITL) com **um único comando**.

## O resultado final

Você fica com **3 janelas**, e todas fazem sentido:

```
┌─────────────────────────────┐   ┌──────────────┐   ┌──────────────────┐
│  VSCode (conectado ao WSL)  │   │   Gazebo     │   │  QGroundControl  │
│  ├ código C++ / Python      │   │  (física 3D) │   │  (mapa, HUD,     │
│  ├ terminal Gazebo  ┐ split │   │              │   │   parâmetros)    │
│  └ terminal SITL    ┘       │   │              │   │                  │
│    (MAVProxy p/ digitar)    │   │              │   │                  │
└─────────────────────────────┘   └──────────────┘   └──────────────────┘
```

Os dois terminais (Gazebo e SITL) ficam **lado a lado dentro do VSCode**. O QGroundControl (no Windows) substitui os popups de mapa e console do MAVProxy — por isso vamos **remover** o `--console` e o `--map`, eliminando duas janelas.

---

## Passo 1 — Instalar o VSCode e conectar ao WSL

1. Instale o **VS Code** no **Windows** (não no WSL): baixe em code.visualstudio.com e instale normalmente.
2. Abra o VS Code e instale a extensão **WSL** (id: `ms-vscode-remote.remote-wsl`) — pela aba de extensões (`Ctrl+Shift+X`), procure por "WSL".
3. Agora o pulo do gato: **abra um terminal do Ubuntu** e rode, dentro da pasta do TCC:

```bash
cd ~/faculdade/TCC
code .
```

Na primeira vez, isso instala o "VS Code Server" dentro do WSL e abre uma janela do VSCode **já conectada ao Linux**, com a pasta `~/faculdade/TCC` aberta. No canto inferior esquerdo aparece um selo verde escrito **WSL: Ubuntu** — é como você confirma que está editando dentro do Linux, e não no Windows.

> A partir de agora, sempre que quiser trabalhar, é só `cd ~/faculdade/TCC && code .` — ou reabrir pelo menu "Arquivo → Abrir Recentes".

---

## Passo 2 — Extensões recomendadas

Crie o arquivo abaixo. Quando o VSCode reabrir a pasta, ele **sugere instalar** essas extensões automaticamente (aceite).

**Arquivo:** `~/faculdade/TCC/.vscode/extensions.json`

```json
{
  "recommendations": [
    "ms-vscode-remote.remote-wsl",
    "ms-vscode.cpptools",
    "ms-python.python",
    "ms-toolsai.jupyter"
  ]
}
```

O que cada uma faz:
- **remote-wsl** — a ponte VSCode ↔ WSL (já instalada no Passo 1).
- **cpptools** — IntelliSense de C++ (autocompletar, ir para definição) — essencial na Fase 5, quando você mexer no `AC_CustomControl`.
- **python** + **jupyter** — rodar seus notebooks de modelagem/controle direto no VSCode, sem abrir o Jupyter no navegador.

> Instale as extensões **no WSL** (o VSCode diferencia; ao clicar em instalar com o selo WSL ativo, ele instala do lado certo).

---

## Passo 3 — Scripts lançadores (o coração da automação)

A jogada aqui é ter dois scriptzinhos **autossuficientes**: eles definem sozinhos as variáveis de ambiente de que precisam, então funcionam de forma confiável, seja chamados pelo VSCode, seja rodados na mão em qualquer terminal.

**Arquivo:** `~/faculdade/TCC/run-gazebo.sh`

```bash
#!/bin/bash
# Lançador do Gazebo — define as próprias variáveis (não depende do .bashrc)
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HOME/faculdade/TCC/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$HOME/faculdade/TCC/ardupilot_gazebo/models:$HOME/faculdade/TCC/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH"

# Descomente a linha abaixo se o Gazebo abrir tela preta ou travar no WSL:
# export LIBGL_ALWAYS_SOFTWARE=1

exec gz sim -v4 -r iris_runway.sdf
```

**Arquivo:** `~/faculdade/TCC/run-sitl.sh`

```bash
#!/bin/bash
# Lançador do ArduPilot SITL — garante o PATH e conecta ao Gazebo via JSON
export PATH="$HOME/faculdade/TCC/ardupilot/Tools/autotest:$HOME/.local/bin:$PATH"

# Pequena folga para o Gazebo subir primeiro (quando os dois sobem juntos).
sleep 4

cd "$HOME/faculdade/TCC/ardupilot/ArduCopter" || exit 1

# Sem --console e sem --map: quem faz mapa/telemetria agora é o QGroundControl.
# O MAVProxy continua neste terminal para você digitar arm/takeoff/etc.
exec sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON
```

Torne os dois executáveis (uma vez só), num terminal:

```bash
chmod +x ~/faculdade/TCC/run-gazebo.sh ~/faculdade/TCC/run-sitl.sh
```

> **Por que scripts autossuficientes?** As variáveis `GZ_SIM_*` que você pôs no `.bashrc` só carregam em terminais interativos. Tarefas automatizadas nem sempre são interativas, então um script que exporta as próprias variáveis nunca falha por "plugin não encontrado" — é a causa nº 1 de o drone aparecer no Gazebo mas não conectar.

---

## Passo 4 — Rodar tudo com um comando (tasks.json)

Este arquivo cria três "tarefas" no VSCode: subir o Gazebo, subir o SITL, e uma terceira que sobe **os dois de uma vez**, em terminais lado a lado.

**Arquivo:** `~/faculdade/TCC/.vscode/tasks.json`

```json
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "🌍 Gazebo",
      "type": "shell",
      "command": "${env:HOME}/faculdade/TCC/run-gazebo.sh",
      "isBackground": true,
      "presentation": { "group": "sim", "panel": "dedicated", "reveal": "always" },
      "problemMatcher": []
    },
    {
      "label": "🚁 SITL",
      "type": "shell",
      "command": "${env:HOME}/faculdade/TCC/run-sitl.sh",
      "isBackground": true,
      "presentation": { "group": "sim", "panel": "dedicated", "reveal": "always" },
      "problemMatcher": []
    },
    {
      "label": "▶ Simular (Gazebo + SITL)",
      "dependsOn": ["🌍 Gazebo", "🚁 SITL"],
      "dependsOrder": "parallel",
      "problemMatcher": []
    }
  ]
}
```

**Como executar:**
- Abra a paleta de comandos: `Ctrl+Shift+P`
- Digite **"Run Task"** → Enter
- Escolha **"▶ Simular (Gazebo + SITL)"**

O Gazebo abre num terminal e o SITL em outro, **lado a lado** na área de terminais do VSCode (é o `"group": "sim"` que os cola juntos). Você digita `arm throttle`, `takeoff 5` etc. no terminal do SITL (onde roda o MAVProxy).

> Se o VSCode perguntar algo sobre "background task", pode seguir — as tarefas são processos longos (ficam rodando), e é esse o comportamento esperado.

---

## Passo 5 (opcional) — Atalho de teclado

Para subir a simulação com uma combinação de teclas em vez de navegar no menu:

**Arquivo:** `~/faculdade/TCC/.vscode/keybindings.json` **não** — atalhos ficam no arquivo global. Abra `Ctrl+Shift+P` → "Preferences: Open Keyboard Shortcuts (JSON)" e adicione:

```json
{
  "key": "ctrl+alt+s",
  "command": "workbench.action.tasks.runTask",
  "args": "▶ Simular (Gazebo + SITL)"
}
```

Agora `Ctrl+Alt+S` sobe tudo.

---

## Passo 6 — Conectar o QGroundControl (que você já instalou)

O SITL roda dentro do WSL; o QGC roda no Windows. Para eles se enxergarem sem gambiarra, ative o **modo de rede espelhada** do WSL (Windows 11 22H2+), que faz WSL e Windows compartilharem `localhost`:

1. No Windows, crie/edite o arquivo `C:\Users\SEU_USUARIO\.wslconfig` com:

```ini
[wsl2]
networkingMode=mirrored
```

2. No **PowerShell**, reinicie o WSL:

```powershell
wsl --shutdown
```

3. Reabra o Ubuntu/VSCode e suba a simulação. O QGroundControl no Windows **conecta sozinho** (ele escuta a porta UDP 14550, e o MAVProxy transmite para lá por padrão).

**Se mesmo assim não conectar:** descubra o IP do Windows visto do WSL com `ip route show | grep default | awk '{print $3}'` e acrescente `--out udp:ESSE_IP:14550` no final da linha `exec sim_vehicle.py ...` dentro do `run-sitl.sh`.

---

## O fluxo do dia a dia (depois de tudo pronto)

1. Abrir o Ubuntu → `cd ~/faculdade/TCC && code .`
2. No VSCode: `Ctrl+Alt+S` (ou Run Task → "▶ Simular")
3. Abrir o QGroundControl no Windows (conecta sozinho)
4. No terminal do SITL, dentro do VSCode:
   ```
   mode guided
   arm throttle
   takeoff 5
   ```

Editar código, ver os terminais, acompanhar o mapa no QGC — tudo à mão, sem caçar janelas.

---

## (Opcional, para a Fase 5) — IntelliSense de C++ do ArduPilot

Quando você começar a editar o `AC_CustomControl`, o autocompletar do C++ fica muito melhor se o VSCode conhecer os caminhos de include do ArduPilot. O jeito limpo é gerar um `compile_commands.json` na configuração do build:

```bash
cd ~/faculdade/TCC/ardupilot
./waf configure --board sitl
# o arquivo aparece em build/sitl/compile_commands.json
```

Depois, crie `~/faculdade/TCC/.vscode/c_cpp_properties.json` apontando para ele:

```json
{
  "configurations": [
    {
      "name": "ArduPilot SITL",
      "compileCommands": "${workspaceFolder}/ardupilot/build/sitl/compile_commands.json",
      "intelliSenseMode": "linux-gcc-x64"
    }
  ],
  "version": 4
}
```

Aí o "ir para definição" (F12) passa a navegar pelo código do ArduPilot inteiro — vai te poupar muito tempo lendo as bibliotecas `AC_AttitudeControl` e afins.

---

## Estrutura de arquivos que você terá

```
~/faculdade/TCC/
├── .vscode/
│   ├── extensions.json          (extensões recomendadas)
│   ├── tasks.json               (subir Gazebo + SITL com 1 comando)
│   └── c_cpp_properties.json    (opcional, IntelliSense — Fase 5)
├── run-gazebo.sh                (lançador do Gazebo)
├── run-sitl.sh                  (lançador do SITL)
├── ardupilot/                   (firmware)
├── ardupilot_gazebo/            (plugin)
└── tcc-venv/                    (ambiente Python)
```