# Roteiro de TCC — Controle de um Quadrotor com ArduPilot (SITL, 100% Open Source)

**Curso:** Engenharia de Controle e Automação
**Plataforma:** ArduPilot (GPLv3) em simulação SITL (Software-In-The-Loop)
**Arquitetura escolhida:** controlador de atitude próprio em **C++** (`AC_CustomControl`) + projeto, identificação e análise em **Python** (ou Octave). Planta podendo ser a nativa do SITL ou um modelo próprio em Python via interface JSON.

> Este documento é um **roteiro de trabalho e de estudo**. Ele diz *o que fazer*, *o que aprender antes de cada etapa* e *por onde começar*. Use-o como checklist vivo: marque o que concluir e ajuste o cronograma à sua realidade.

---

## 1. Visão geral do projeto

### 1.1. A tese em uma frase
> Projetar, implementar e validar em simulação um controlador de atitude alternativo para um quadrotor no ArduPilot, comparando seu desempenho com o controlador PID nativo, usando exclusivamente software livre.

### 1.2. O que você vai entregar
1. Um **modelo dinâmico** do quadrotor (não-linear) e sua **versão linearizada** em torno do voo pairado (*hover*).
2. Um **controlador de atitude** projetado por você (recomendado: LQR; alternativas: alocação de polos, LQG, ou *sliding mode* se quiser um viés não-linear/robusto).
3. A **implementação em C++** desse controlador dentro do ArduPilot, via o framework `AC_CustomControl`.
4. Uma **bateria de experimentos em SITL** (resposta ao degrau, rastreamento de referência, rejeição de distúrbio) com **métricas** e **comparação** contra o PID nativo.
5. A **monografia** documentando modelagem, projeto, implementação e resultados.

### 1.3. Por que essa arquitetura
- **Malha interna (atitude/taxa) roda a ~400 Hz** e precisa de temporização apertada → tem que viver em C++ dentro do firmware. É exatamente o que caracteriza "mexer completamente no controle".
- **O projeto do controlador é matemático** (calcular ganhos, sintonizar, identificar a planta) → isso acontece em Python/Octave, onde é fácil e rápido iterar.
- **Tudo é aberto:** o C++ que você toca é o do próprio ArduPilot (GPLv3), não uma ferramenta proprietária.

### 1.4. Escopo (e o que deixar de fora)
- **Dentro:** malha de atitude (roll, pitch, yaw) e, opcionalmente, a malha de altitude. Validação em SITL.
- **Fora (a menos que sobre tempo):** voo real em hardware; controle de trajetória complexo; enxames; visão computacional. Deixe isso como "trabalhos futuros".
- **Ponto de decisão:** se, ao longo do caminho, sua contribuição migrar para o nível de **guiagem/trajetória** (seguir caminhos, MPC de posição, evasão), aí o Python *offboard* via MAVLink passa a ser a escolha natural — e o C++ deixa de ser necessário. Malha interna → C++; malha externa → Python.

---

## 2. Ferramentas a serem utilizadas

Todas livres e multiplataforma. Recomendo **Ubuntu 22.04+** (nativo ou via **WSL2** no Windows).

### 2.1. Tabela de ferramentas

| Ferramenta | Papel no TCC | Camada |
|---|---|---|
| **Ubuntu / WSL2 + git** | Sistema operacional e versionamento | Base |
| **ArduPilot (repo)** | Firmware; onde mora seu controlador (`AC_CustomControl`) | C++ |
| **WAF + gcc** | Sistema de build do ArduPilot | C++ |
| **SITL + `sim_vehicle.py`** | Roda o firmware sem hardware | Simulação |
| **MAVProxy** | GCS de linha de comando (sobe junto com o SITL) | Controle/telemetria |
| **QGroundControl** ou **Mission Planner** | GCS gráfica (mapa, missões, parâmetros) | Controle |
| **Python 3 + Jupyter** | Ambiente de projeto e análise | Python |
| **`python-control`** | Projeto de controle (LQR, polos, Bode, step) — API tipo MATLAB | Python |
| **`numpy` / `scipy`** | Álgebra linear, integração, `scipy.signal` para ID | Python |
| **`sympy`** | Derivar o modelo e **linearizar simbolicamente** (Jacobiana) | Python |
| **`matplotlib` / `pandas`** | Gráficos e tabelas dos resultados | Python |
| **`pymavlink`** | Ler os logs `.bin` (dataflash) e controle *offboard* | Python |
| **Gazebo (Harmonic) + `ardupilot_gazebo`** | Física 3D e visualização realista (opcional) | Simulação |
| **JSBSim / FlightGear** | FDM e visualização alternativos (opcional) | Simulação |
| **Octave (+ pacotes `control`, `signal`)** | Alternativa ao Python, se preferir | Projeto/análise |

### 2.2. Divisão de responsabilidades
- **C++ (dentro do ArduPilot):** apenas a **conta final** do controlador — algo como `u = -K·x` mais o tratamento de reset de integrador. Sem ponteiros, sem gerência de memória, sem *templates*. O esqueleto (classe, parâmetros, integração com o build) já vem pronto no *backend* vazio de exemplo.
- **Python:** modelagem, linearização, cálculo dos ganhos, identificação do sistema, e **toda** a análise de resultados.

### 2.3. Ordem de instalação (faça nesta sequência)
1. Ubuntu/WSL2 funcionando + `git`.
2. Clonar o ArduPilot e rodar o script de setup do ambiente (`Tools/environment_install/`).
3. Compilar e rodar um SITL de exemplo (Copter) — confirmar que decola em modo GUIDED pelo MAVProxy.
4. Ambiente Python: `pip install control numpy scipy sympy matplotlib pandas pymavlink jupyter`.
5. (Opcional) Gazebo Harmonic + plugin `ardupilot_gazebo`.
6. QGroundControl para uma GCS gráfica.

---

## 3. Fundamentação teórica a aprofundar

Organizada por área, com **o que estudar** e **onde isso aparece no seu TCC**. Priorize na ordem apresentada.

### 3.1. Dinâmica de corpo rígido (base da modelagem)
- **Referenciais** (inercial vs corpo) e transformações entre eles.
- **Representações de atitude:** ângulos de Euler (roll φ, pitch θ, yaw ψ), matriz de rotação/DCM e **quatérnios** (o ArduPilot usa quatérnios internamente).
- **Equações de Newton-Euler:** translação (`F = m·a`) e rotação (`τ = I·ω̇ + ω × Iω`).
- **Onde usa:** capítulo de modelagem — é o coração do seu modelo do quadrotor.

### 3.2. Sistemas lineares e espaço de estados (base do controle moderno)
- Forma `ẋ = Ax + Bu`, `y = Cx + Du`.
- **Linearização** por expansão de Taylor / Jacobiana em torno de um ponto de equilíbrio.
- **Estabilidade** por autovalores de A; polos e zeros.
- **Controlabilidade** e **observabilidade** (matrizes de Kalman).
- **Onde usa:** linearização e projeto do controlador.

### 3.3. Controle clássico (a linha de base e a validação)
- **PID** (o que cada ganho faz; anti-windup).
- **Lugar das raízes** e **resposta em frequência** (Bode, Nyquist).
- **Margens de estabilidade** (ganho e fase) — como você "prova" que seu controlador é robusto.
- **Onde usa:** entender o PID nativo do ArduPilot (sua base de comparação) e analisar seu próprio controlador.

### 3.4. Controle moderno (a sua contribuição, se for LQR/estimador)
- **LQR** (Regulador Linear Quadrático): matrizes de peso Q e R, equação de Riccati, cálculo do ganho K.
- **Alocação de polos** (*pole placement*).
- **Estimação de estado:** observador de Luenberger / filtro de Kalman (se precisar estimar estados não medidos).
- **Onde usa:** projeto do controlador de atitude.

### 3.5. Controle digital (implementação embarcada)
- **Amostragem** e **transformada Z**.
- **Discretização** (ZOH, Tustin) — passar do controlador contínuo `K(s)` para o discreto que roda a 400 Hz.
- Efeitos de tempo de amostragem e atraso.
- **Onde usa:** ao portar o controlador de Python para o C++ do ArduPilot.

### 3.6. Identificação de sistemas (validar a planta)
- **Estimação de resposta em frequência** a partir de sinais de excitação (*chirp*).
- **Mínimos quadrados** para ajuste de parâmetros.
- **Onde usa:** rodar o modo *System ID* do ArduPilot e ajustar/validar seu modelo.

### 3.7. (Opcional) Controle não-linear — se optar por *sliding mode* etc.
- **Estabilidade de Lyapunov**, superfícies de deslizamento, robustez a incertezas.
- **Onde usa:** projeto de um controlador não-linear em vez do LQR.

### 3.8. Recursos abertos recomendados
- **ArduPilot Developer Docs** (ardupilot.org/dev) — em especial a página "Adding Custom Attitude Controller to Copter", "Copter Attitude Control" e "System Identification Mode".
- **Åström & Murray, *Feedback Systems*** — livro gratuito online; é inclusive a base conceitual da biblioteca `python-control`.
- **Documentação da `python-control`** — exemplos de `lqr`, `place`, `ss`, `bode`, `step_response`.
- **Teses abertas de modelagem de quadrotor** — procure por Bouabdallah ("Modelling, identification and control of a quadrotor"), Sabatino (KTH, "Quadrotor control: modeling, nonlinear control design, and simulation") e as notas de Randal Beard (BYU, "Quadrotor Dynamics and Control"). São referências clássicas, gratuitas e fazem exatamente a modelagem que você precisa.
- **Vídeos de intuição** — série de controle do Brian Douglas (gratuita no YouTube), ótima para consolidar Bode, LQR e realimentação de estados.

---

## 4. Modelagem do sistema

Objetivo: obter o **modelo dinâmico não-linear** do quadrotor, `ẋ = f(x, u)`. Faça isso simbolicamente no `sympy` — assim a linearização (Seção 5) sai quase de graça.

### 4.1. Referenciais
- **Inercial (NED):** North-East-Down. É o referencial de mundo do ArduPilot.
- **Corpo (FRD):** Forward-Right-Down, preso ao drone.
- A **matriz de rotação** `R(φ, θ, ψ)` leva vetores do corpo para o inercial.

### 4.2. Vetor de estado
Um conjunto usual de 12 estados:
```
x = [ p_N, p_E, p_D,      (posição, inercial)
      u,   v,   w,        (velocidade, corpo ou inercial — defina e mantenha)
      φ,   θ,   ψ,        (atitude — ângulos de Euler)
      p,   q,   r ]        (velocidade angular, corpo)
```
Entradas: `u = [ T, τ_φ, τ_θ, τ_ψ ]` — empuxo total e os três torques.

### 4.3. Equações de movimento (Newton-Euler)
**Translação** (o empuxo age ao longo do eixo −z do corpo, para cima):
```
m · v̇ = R · [0, 0, −T]ᵀ + [0, 0, m·g]ᵀ + F_arrasto
```
**Rotação** (torques vêm da diferença de empuxo entre os rotores):
```
I · ω̇ = τ − ω × (I · ω)
```
onde `I = diag(Ixx, Iyy, Izz)` é a matriz de inércia (assumindo simetria).

**Cinemática de atitude** (relaciona velocidades angulares do corpo às taxas de Euler):
```
[φ̇, θ̇, ψ̇]ᵀ = W(φ, θ) · [p, q, r]ᵀ
```

### 4.4. Modelo dos atuadores e alocação (*mixer*)
- Cada rotor `i` gera **empuxo** `T_i = k_T · ω_i²` e **torque de reação** `Q_i = k_Q · ω_i²`.
- A **matriz de alocação** mapeia `(T, τ_φ, τ_θ, τ_ψ)` para as velocidades dos quatro rotores, em função da geometria (comprimento do braço `l`, configuração "+" ou "×").
- **Dinâmica de motor/ESC:** aproxime por um **atraso de primeira ordem** (constante de tempo do motor). Para uma primeira versão, pode até desprezar.

### 4.5. Parâmetros a levantar
`m` (massa), `Ixx, Iyy, Izz` (inércias), `l` (braço), `k_T`, `k_Q` (coeficientes de empuxo/torque). Fontes: datasheets, medições, ou os próprios parâmetros do modelo do SITL (ex.: frame `iris`/`gazebo-iris`). Documente cada valor e sua origem.

### 4.6. Efeitos que você *pode* desprezar (justifique na monografia)
Arrasto aerodinâmico do corpo, *blade flapping*, efeito solo, giroscópico dos rotores. Desprezá-los é padrão numa primeira modelagem; mencione-os como fontes de erro modelo-realidade.

**Entregável desta seção:** documento com o modelo não-linear completo, a matriz de alocação e a tabela de parâmetros. Um *notebook* `sympy` com `f(x, u)` simbólico.

---

## 5. Linearização

Objetivo: obter o **modelo linear em espaço de estados** em torno do voo pairado, para poder aplicar controle linear (LQR, polos).

### 5.1. Ponto de equilíbrio (*hover*)
No voo pairado: `φ = θ = 0`, `p = q = r = 0`, velocidades nulas, `ψ = ψ₀` (constante), e empuxo de equilíbrio `T₀ = m·g`. Esse é o ponto onde você lineariza.

### 5.2. Linearização por Jacobiana
Com o `f(x, u)` simbólico do `sympy`, calcule:
```
A = ∂f/∂x |_(x₀, u₀)        B = ∂f/∂u |_(x₀, u₀)
```
Use `sympy.Matrix.jacobian(...)`, substitua o ponto de equilíbrio, e exporte `A` e `B` numéricos para o Python.

### 5.3. Estrutura desacoplada resultante
Perto do *hover*, o sistema se **desacopla** em subsistemas quase independentes:
- **Altitude (z):** duplo integrador acionado pelo empuxo.
- **Atitude (cada eixo φ, θ, ψ):** torque → aceleração angular → taxa → ângulo (cadeia integradora).
- **Posição horizontal:** acopla-se à atitude (deslocamento em x depende do *pitch*; em y, do *roll*).

Isso justifica a **arquitetura em cascata** que o ArduPilot usa (Seção 6) e permite projetar por malhas.

### 5.4. Verificações de projeto
- **Controlabilidade** (posto da matriz `[B  AB  A²B ...]`) e **observabilidade**.
- **Autovalores de A** (o sistema em malha aberta é, tipicamente, marginalmente estável/instável — daí a necessidade do controlador).

### 5.5. Discretização
Como o controlador roda em tempo discreto a `Ts = 1/400 s`, discretize o modelo (ZOH) com `control.c2d(...)` para projetar/validar no domínio discreto. Guarde tanto a versão contínua quanto a discreta.

### 5.6. Validação da linearização
Compare a resposta do **modelo linear** com a do **modelo não-linear** (simulado em Python) para pequenas perturbações. Devem coincidir perto do equilíbrio e divergir para grandes ângulos — mostre isso num gráfico. É um resultado valioso para a banca.

**Entregável desta seção:** matrizes `A, B, C` (contínuas e discretas), checagem de controlabilidade, e o gráfico de validação linear vs não-linear.

---

## 6. Malha de controle

Objetivo: projetar o controlador em Python, entendê-lo à luz da arquitetura do ArduPilot, e validar em simulação **antes** de escrever C++.

### 6.1. A cascata do ArduPilot e onde você entra
O ArduPilot refina o comando em cascata: **posição → velocidade → aceleração → atitude → taxa angular → motores** (bibliotecas `AC_PosControl` e `AC_AttitudeControl`). O controlador de atitude é um **P de ângulo → PID de taxa**, usando quatérnios.

Seu controlador, via `AC_CustomControl`, **substitui a malha de atitude+taxa**: ele recebe o alvo de atitude já modelado e produz as entradas `roll/pitch/yaw` para o *mixer* dos motores. A função é chamada **depois** da malha de taxa principal e **antes** da biblioteca de motores, minimizando latência.

### 6.2. Linha de base: entender o PID nativo
Antes de inventar, **caracterize o PID que já existe** (ganhos `ATC_RAT_*` e `ATC_ANG_*`). Rode experimentos com ele e guarde os resultados — é a sua base de comparação.

### 6.3. Sua contribuição: projeto do controlador
Recomendado para começar: **LQR** sobre o modelo linearizado.
1. Monte `(A, B)` da malha de atitude (subsistema).
2. Escolha `Q` (penaliza erro de estado) e `R` (penaliza esforço de controle).
3. Calcule `K = control.lqr(A, B, Q, R)`.
4. Simule a malha fechada (`step_response`, rastreamento de referência) e **sintonize Q e R** iterativamente.
5. Verifique **margens de estabilidade** (`control.margin`, Bode).

Alternativas conforme sua ambição: **alocação de polos** (mais didático), **LQG** (LQR + Kalman, se houver ruído/estados não medidos), ou ***sliding mode*** (não-linear, robusto — exige Lyapunov, mais avançado).

### 6.4. Identificação do sistema (validar/refinar a planta)
Use o **modo System ID** do ArduPilot: ele injeta *chirp* nos eixos e você registra a resposta. Depois, em Python (`scipy.signal`, `python-control`), estime a **resposta em frequência** e **ajuste a função de transferência**. Compare com o modelo analítico da Seção 4 — concordância (ou discrepância explicada) é um capítulo forte de metodologia.

### 6.5. Cuidados de implementação
- **Anti-windup** no termo integral.
- **Reset do integrador** ao alternar entre seu controlador e o nativo (o próprio guia do `AC_CustomControl` alerta que isso é responsabilidade sua — e é um bom ponto de discussão técnica).
- **Discretização** do controlador projetado (Seção 5.5) para a taxa de 400 Hz.

**Entregável desta seção:** controlador projetado, com ganhos definidos e **resultados de simulação em Python** (malha fechada) já mostrando desempenho aceitável; modelo identificado via System ID.

---

## 7. Aplicação final

Objetivo: levar o controlador projetado para dentro do ArduPilot, rodar em SITL, e produzir os resultados da monografia.

### 7.1. Implementação em C++ (`AC_CustomControl`)
1. Copie o *backend* de exemplo vazio (`AC_CustomControl_XYZ`) para criar o seu.
2. Registre o novo tipo no *enum* de controladores e inclua o *header*.
3. Preencha a função `update()`: leia a atitude alvo (`get_attitude_target_quat()`), a taxa alvo (`rate_bf_targets()`) e a medida do giroscópio (`get_gyro_latest()`), calcule sua lei de controle e retorne o `Vector3f` de saída `(roll, pitch, yaw)` para o *mixer*.
4. Implemente o `reset()` corretamente (integradores, alvos).

> O volume real de C++ é pequeno: a lei de controle é aritmética sobre vetores já fornecidos. Você não está "aprendendo C++", está preenchendo uma função.

### 7.2. Parâmetros ajustáveis e *logging*
- Exponha seus ganhos como **parâmetros `AP_Float`** — assim você os **sintoniza ao vivo** pela GCS, **sem recompilar**. Só recompila quando muda a *estrutura* do algoritmo.
- Adicione **mensagens de log próprias** para os sinais internos (alvo, erro e saída de cada eixo). São eles que virarão os gráficos da monografia. Planeje quais sinais registrar antes de começar.

### 7.3. Chave de comparação em voo
Atribua uma opção de canal de rádio (ex.: `RC6_OPTION`) para **alternar em pleno voo** entre o seu controlador e o PID nativo. Isso gera comparações lado a lado diretas nos mesmos dados.

### 7.4. Compilar e rodar
```bash
cd ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -v ArduCopter --console --map
# (ou -f gazebo-iris --model JSON, para física/visual 3D no Gazebo)
```

### 7.5. Bateria de experimentos
- **Resposta ao degrau** de atitude (comando de roll/pitch).
- **Rastreamento de referência** (trajetória de atitude/posição).
- **Rejeição de distúrbio:** vento simulado (parâmetros `SIM_WIND_*`), ou falhas (`SIM_RC_FAIL`).
- **Robustez:** variar parâmetros do modelo (massa, inércia) e observar a degradação.

### 7.6. Métricas
Tempo de subida, sobressinal (*overshoot*), tempo de acomodação, erro em regime, **RMSE** de rastreamento e **esforço de controle**. Tabele tudo.

### 7.7. Análise e comparação
Parseie os logs `.bin` com `pymavlink` → `pandas`/`matplotlib`. Gere os gráficos de **seu controlador vs PID nativo** nas mesmas condições. Discuta as diferenças à luz da teoria.

### 7.8. (Opcional) Voo real
Só se sobrar tempo e com **muita cautela e segurança**: simulação-em-hardware (*Simulation on Hardware*) ou voo real com área controlada, failsafes e supervisão. Deixe explícito na monografia se ficará só em simulação.

**Entregável desta seção:** controlador rodando em SITL, resultados dos experimentos, tabelas de métricas e gráficos comparativos.

---

## 8. Cronograma em fases — por onde começar

Fases pensadas para ~1 semestre; **elas se sobrepõem de propósito** (estude a teoria da fase seguinte enquanto executa a atual). Ajuste as durações ao seu calendário.

### ▶️ COMECE AQUI — Fase 0 · Ambiente e reconhecimento (Semanas 1–2)
- Instalar Ubuntu/WSL2, clonar o ArduPilot, compilar e **rodar o primeiro SITL**.
- **Decolar em modo GUIDED** pelo MAVProxy (`mode guided`, `arm throttle`, `takeoff 5`).
- Montar o ambiente Python e escrever um *notebook* que **abre um log `.bin`** e plota um sinal.
- Ler as páginas de docs: "Copter Attitude Control" e "Adding Custom Attitude Controller".
- **Entregável:** SITL funcional que você comanda + *notebook* lendo log.

### Fase 1 · Modelagem (Semanas 3–6)
- Estudar dinâmica de corpo rígido e representações de atitude (Seção 3.1).
- Derivar `f(x, u)` no `sympy`; montar a matriz de alocação; levantar parâmetros.
- **Entregável:** modelo não-linear documentado + *notebook* simbólico.

### Fase 2 · Linearização (Semanas 5–8)
- Estudar espaço de estados e linearização (Seção 3.2).
- Calcular `A, B`; checar controlabilidade; discretizar; **validar linear vs não-linear**.
- **Entregável:** modelo linear (contínuo e discreto) + gráfico de validação.

### Fase 3 · Projeto do controlador (Semanas 8–12)
- Estudar controle clássico e moderno (Seções 3.3–3.4).
- Caracterizar o **PID nativo**; projetar o **LQR** (ou escolha equivalente); simular malha fechada; sintonizar; checar margens.
- **Entregável:** controlador projetado com resultados de simulação em Python.

### Fase 4 · Identificação (Semanas 10–14)
- Estudar identificação (Seção 3.6).
- Rodar **System ID** no SITL; extrair resposta em frequência; **comparar com o modelo analítico**.
- **Entregável:** modelo identificado + comparação.

### Fase 5 · Implementação em C++ (Semanas 13–18)
- Estudar controle digital (Seção 3.5) para a discretização.
- Criar o *backend* `AC_CustomControl_XYZ`; expor ganhos `AP_Float`; adicionar *logging*; **fazer voar em SITL**.
- **Entregável:** controlador próprio rodando no SITL.

### Fase 6 · Validação e experimentos (Semanas 17–22)
- Rodar a bateria de experimentos; coletar métricas; **comparar com o PID nativo**.
- **Entregável:** resultados, tabelas e gráficos comparativos.

### Fase 7 · Escrita e fechamento (Semanas 21–24)
- Redigir a monografia; consolidar figuras; (opcional) tentar voo/hardware.
- **Entregável:** TCC finalizado.

---

## 9. Riscos e conselhos práticos

- **O gargalo do C++ não é a linguagem, é o *build*.** Reserve os primeiros dias para fazer o ambiente compilar (WSL2 + toolchain). Depois disso, iterar é tranquilo — e você sintoniza ganhos ao vivo sem recompilar.
- **Simule sempre antes de escrever C++.** Chegar no `AC_CustomControl` com o controlador já validado em Python economiza semanas.
- **Planeje o *logging* cedo.** Descobrir no fim que faltou registrar um sinal obriga a refazer experimentos.
- **Cuide do reset do integrador** ao trocar de controlador — é fonte comum de transitórios estranhos e um bom tópico de discussão.
- **Escopo enxuto vence.** Um LQR de atitude bem modelado, identificado e comparado com o PID é um TCC completo e forte. Não tente MPC + enxame + visão de uma vez.
- **Versione tudo com git** desde o dia 1 (seu *fork* do ArduPilot, seus *notebooks*, a monografia).

---

## 10. Resumo do fluxo (mapa mental)

```
[Teoria] → [Modelagem sympy] → [Linearização python-control] → [Validação linear×não-linear]
                                                                        │
                                                                        ▼
        [PID nativo: baseline] ← [Projeto LQR + simulação] ← [Identificação: System ID]
                                                                        │
                                                                        ▼
                          [Implementação C++ em AC_CustomControl]
                                                                        │
                                                                        ▼
         [SITL: experimentos] → [Logs via pymavlink] → [Métricas + comparação] → [Monografia]

  Ferramentas de projeto/análise: Python (control, sympy, scipy, matplotlib, pymavlink)
  Ferramenta de controle em tempo real: C++ (AC_CustomControl)
  Simulação: SITL (nativo) e/ou Gazebo via interface JSON — tudo open source
```

---

*Documento de planejamento — adapte livremente conforme o projeto evolui.*