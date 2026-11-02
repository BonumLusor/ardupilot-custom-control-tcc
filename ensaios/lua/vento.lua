--[[
   vento.lua — injetor de disturbios reproduziveis para ensaios em SITL.

   Gera um perfil de disturbio e o escreve nos parametros de simulacao do SITL,
   registrando a verdade do que foi aplicado no proprio .BIN (mensagem WND), para
   que a entrada e a resposta do controlador fiquem na mesma base de tempo.

   Sao DUAS familias de disturbio, e a distincao importa:

     VENTO (SIM_WIND_*)  entra no modelo como FORCA no centro de massa. Braco
                         zero, torque zero: move a aeronave, mas praticamente nao
                         perturba a atitude. Serve para ensaios de deriva e para
                         por o controlador a operar fora do ponto de equilibrio
                         simetrico.
     TORQUE (SIM_TWIST_*) entra somado direto em rot_accel, em rad/s^2. E o termo
                         aditivo de perturbacao na equacao da dinamica de rotacao,
                         e e ele que exercita a malha de ATITUDE.

   PARCELAS DE VENTO
     1. Vento medio      — vetor constante (WND_SPD, WND_DIR).
     2. Rajada           — meio ciclo de cosseno: A/2*(1 - cos(2*pi*(t-t0)/T)).
     3. Turbulencia      — Dryden. Componente longitudinal u pela discretizacao
                           EXATA do processo de Ornstein-Uhlenbeck; componente
                           lateral v pelo filtro de 2a ordem de Dryden discretizado
                           por Tustin. Ambos entregam desvio-padrao igual a WND_TSIG
                           (normalizacao verificada numericamente antes do porte).

   PARCELAS DE TORQUE (por eixo, WND_TQX/Y/Z em rad/s^2)
     4. Pulso retangular — impulso angular = amplitude * duracao, conhecido de
                           antemao.
     5. Meio cosseno     — mesma forma da rajada; impulso = amplitude * duracao/2.
     6. Degrau sustentado— momento constante que nao termina; expoe erro em regime
                           e a suficiencia da acao integral.

   REPRODUTIBILIDADE
     O ruido branco vem de um LCG de Park-Miller (minstd) semeado por WND_SEED, com
     transformacao de Box-Muller. E deliberadamente escrito a mao, e nao tirado de
     math.random, para que a mesma sequencia possa ser regerada em Python por
     ensaios/analise/dryden_ref.py e conferida contra o log.

   PRE-REQUISITOS no veiculo (o run_ensaio.py aplica; se rodar a mao, aplique voce):
     SIM_WIND_T    = 1     sem isso a lei de raiz quadrada corta o vento a ~29% a 5 m
     SIM_WIND_TC   = 0.1   piso do firmware; o padrao 5 s destruiria a rajada
     SIM_WIND_TURB = 0     desliga a turbulencia IIR nativa; a nossa e que vale
     O SITL ignora vento nos primeiros 5 s apos o boot (wind_start_delay_micros),
     entao WND_T0 deve reservar essa folga.

   ATENCAO: so funciona em SITL puro. Com --model JSON (Gazebo) o SIM_JSON zera o
   vento internamente e nada disto tem efeito fisico.
--]]

local PARAM_TABLE_KEY = 72
local PARAM_TABLE_PREFIX = "WND_"

local TAXA_HZ = 50
local DT = 1.0 / TAXA_HZ
local INTERVALO_MS = math.floor(1000 / TAXA_HZ)

local GRAUS = 180.0 / math.pi
local RAD = math.pi / 180.0

-- bits de WND_MODE
local MODO_MEDIO  = 1
local MODO_RAJADA = 2
local MODO_DRYDEN = 4
local MODO_TQ_PULSO = 8      -- torque: pulso retangular
local MODO_TQ_COS   = 16     -- torque: meio cosseno
local MODO_TQ_DEG   = 32     -- torque: degrau sustentado
local MODO_TQ_CHIRP = 64     -- torque: varredura em frequencia
local MODO_TQ_ESTOC = 128    -- torque: estocastico (Gauss-Markov por eixo)
local MODO_EVENTO   = 256    -- evento de planta: motor fraco ou perda de empuxo
local MODO_TQ_QUALQUER = MODO_TQ_PULSO | MODO_TQ_COS | MODO_TQ_DEG | MODO_TQ_CHIRP
                         | MODO_TQ_ESTOC

-------------------------------------------------------------------------------
-- parametros
-------------------------------------------------------------------------------

local function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('vento: parametro %s nao encontrado', name))
   return p
end

local function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value),
          string.format('vento: nao consegui criar %s%s', PARAM_TABLE_PREFIX, name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 24), 'vento: nao consegui criar a tabela de parametros')

--[[ bitmask, combinavel. 0 desliga o injetor.
       1  vento medio          |   8  torque: pulso retangular
       2  rajada 1-cosseno     |  16  torque: meio cosseno
       4  turbulencia Dryden   |  32  torque: degrau sustentado
                              |  64  torque: varredura em frequencia (chirp)
                              | 128  torque: estocastico (Gauss-Markov)
                              | 256  evento de planta (motor fraco, empuxo)
     As formas de torque compartilham amplitude, inicio e duracao; ligar mais
     de uma ao mesmo tempo as SOMA.                                            --]]
local WND_MODE = bind_add_param('MODE', 1, 0)
-- vento medio: modulo (m/s) e direcao de ONDE VEM (graus, convencao do SIM_WIND_DIR)
local WND_SPD  = bind_add_param('SPD', 2, 0)
local WND_DIR  = bind_add_param('DIR', 3, 0)
-- componente vertical (graus): 0 = puramente horizontal, 90 = ascendente pura
local WND_DIRZ = bind_add_param('DIRZ', 4, 0)
-- rajada 1-cosseno: amplitude (m/s), inicio (s, relativo ao gatilho), duracao (s)
local WND_GA   = bind_add_param('GA', 5, 0)
local WND_GT0  = bind_add_param('GT0', 6, 10)
local WND_GDUR = bind_add_param('GDUR', 7, 3)
-- Dryden: intensidade sigma (m/s) e escala de comprimento longitudinal Lu (m)
local WND_TSIG = bind_add_param('TSIG', 8, 0)
local WND_TLU  = bind_add_param('TLU', 9, 200)
-- semente do PRNG: e o que torna dois ensaios identicos
local WND_SEED = bind_add_param('SEED', 10, 1)
--[[ gatilho do relogio do perfil:
       >= 0  o relogio zera no armamento e o vento comeca em t = WND_T0
       <  0  o relogio zera quando WND_MODE deixa de ser zero, sem precisar armar
             (modo de bancada, para conferir a forma do perfil no chao)          --]]
local WND_T0   = bind_add_param('T0', 11, 8)

--[[ Disturbio de TORQUE: aceleracao angular no corpo, em rad/s^2, por eixo
     (x = rolagem, y = arfagem, z = guinada).

     Entra no SITL por SIM_TWIST_*, que o firmware soma DIRETAMENTE em rot_accel
     (SIM_Aircraft.cpp:1360). E, portanto, o termo aditivo de perturbacao na
     equacao da dinamica de rotacao — nao uma emulacao por assimetria de rotor.

     Existe porque vento NAO perturba atitude neste simulador: o arrasto e
     aplicado como forca no centro de massa, braco zero, torque zero. Quem quiser
     medir rejeicao da malha de atitude precisa de um disturbio que gere momento.
--]]
local WND_TQX   = bind_add_param('TQX', 12, 0)
local WND_TQY   = bind_add_param('TQY', 13, 0)
local WND_TQZ   = bind_add_param('TQZ', 14, 0)
local WND_TQT0  = bind_add_param('TQT0', 15, 20)
local WND_TQDUR = bind_add_param('TQDUR', 16, 0.4)
--[[ Varredura em frequencia (chirp): senoide de frequencia crescente, de TQF0 a
     TQF1 ao longo de WND_TQDUR. Uma corrida entrega a resposta em frequencia
     inteira — margem de ganho e de fase — que degrau e pulso nao dao.

     A progressao e EXPONENCIAL e a janela e de meio cosseno nas pontas, igual ao
     Chirp do ArduPilot (AP_Math/chirp.cpp), para que estes ensaios sejam
     comparaveis aos do modo SYSTEMID nativo.

     O torque nao passa pelo filtro de vento do SITL (SIM_WIND_TC, piso de 0.1 s
     = ~1.6 Hz): SIM_TWIST_* entra direto em rot_accel. Entao a varredura pode
     subir bem acima disso sem distorcao — que e justamente onde a margem de fase
     se decide.                                                               --]]
local WND_TQF0  = bind_add_param('TQF0', 17, 0.2)    -- Hz, inicio
local WND_TQF1  = bind_add_param('TQF1', 18, 10.0)   -- Hz, fim

--[[ Torque ESTOCASTICO (bit 128): em cada eixo, um processo de Gauss-Markov
     (Ornstein-Uhlenbeck) de variancia unitaria e tempo de correlacao WND_TQTC,
     discretizado EXATAMENTE a DT, vezes WND_TQX/Y/Z — que aqui sao o DESVIO
     PADRAO do torque, em rad/s^2. O processo corre desde o inicio do perfil
     (estacionario muito antes de TQT0) e o torque so e aplicado entre TQT0 e
     TQT0 + TQDUR. E a turbulencia que gera momento, que o vento deste simulador
     nao gera; a metrica e o erro eficaz, o que um LQG minimiza.            --]]
local WND_TQTC  = bind_add_param('TQTC', 19, 0.1)    -- s

--[[ EVENTO DE PLANTA (bit 256): em TQT0 o injetor muda um parametro da fisica
     do SITL e o desfaz em TQT0 + TQDUR (antes do pouso):
       WND_EVT = 1  motor fraco: SIM_ENGINE_FAIL = WND_EVM (mascara de motores)
                    e SIM_ENGINE_MUL = WND_EVK (fator sobre PWM - 1000)
       WND_EVT = 2  perda uniforme de empuxo: SIM_FRM_HOVR_THR = WND_EVK
     Os dois sao relidos pelo SITL a cada passo (SITL_State, SIM_Frame).     --]]
local WND_EVT   = bind_add_param('EVT', 20, 0)
local WND_EVM   = bind_add_param('EVM', 21, 0)
local WND_EVK   = bind_add_param('EVK', 22, 1.0)

-- parametros do SITL que o injetor escreve
local SIM_WIND_SPD   = bind_param('SIM_WIND_SPD')
local SIM_WIND_DIR   = bind_param('SIM_WIND_DIR')
local SIM_WIND_DIR_Z = bind_param('SIM_WIND_DIR_Z')
local SIM_TWIST_X    = bind_param('SIM_TWIST_X')
local SIM_TWIST_Y    = bind_param('SIM_TWIST_Y')
local SIM_TWIST_Z    = bind_param('SIM_TWIST_Z')
local SIM_TWIST_TIME = bind_param('SIM_TWIST_TIME')
local SIM_ENGINE_FAIL = bind_param('SIM_ENGINE_FAIL')
local SIM_ENGINE_MUL  = bind_param('SIM_ENGINE_MUL')
local SIM_FRM_HOVR_THR = bind_param('SIM_FRM_HOVR_THR')

-- O twist se auto-desarma quando SIM_TWIST_TIME expira, zerando o proprio
-- parametro. Armamos uma vez com um valor longo e modulamos so as amplitudes a
-- cada ciclo: assim a forma de onda e nossa, e nao um pulso retangular imposto
-- pelo firmware.
local TWIST_ARMADO_MS = 3600000

-------------------------------------------------------------------------------
-- PRNG determinista: Park-Miller minstd + Box-Muller
-- O ArduPilot compila o Lua com LUA_32BITS (Tools/ardupilotwaf/boards.py), em
-- todas as placas e no SITL: inteiros e floats de 32 bits. O produto direto
-- 16807*estado passa de 2^31 e da a volta, e o gerador deixa de ser o minstd
-- (foi o que se viu no primeiro ensaio de torque estocastico: o log nao batia
-- com analise.dryden_ref). O metodo de Schrage calcula o MESMO minstd sem que
-- nenhum produto intermediario passe de 2^31: exato em 32 bits.
-------------------------------------------------------------------------------

local MINSTD_M = 2147483647
local MINSTD_A = 16807
local MINSTD_Q = 127773            -- M // A
local MINSTD_R = 2836              -- M % A

local rng_estado = 1

local function rng_semear(semente)
   local s = math.floor(math.abs(semente)) % (MINSTD_M - 1)
   rng_estado = s + 1              -- garante estado em [1, M-1], nunca 0
end

local function rng_uniforme()      -- em (0, 1)
   local hi = rng_estado // MINSTD_Q
   local lo = rng_estado % MINSTD_Q
   local x = MINSTD_A * lo - MINSTD_R * hi
   if x <= 0 then
      x = x + MINSTD_M
   end
   rng_estado = x
   return rng_estado / MINSTD_M
end

-- devolve DUAS normais padrao independentes por chamada (uma para cada canal de
-- Dryden), o que mantem o consumo de numeros aleatorios fixo em 2 uniformes por
-- passo e torna a sequencia trivialmente reproduzivel em Python.
local function rng_normais()
   local u1 = rng_uniforme()
   local u2 = rng_uniforme()
   local r = math.sqrt(-2.0 * math.log(u1))
   local a = 2.0 * math.pi * u2
   return r * math.cos(a), r * math.sin(a)
end

-------------------------------------------------------------------------------
-- estado do perfil
-------------------------------------------------------------------------------

local perfil_ativo = false
local t_gatilho_ms = nil          -- millis() do instante em que o relogio zerou
local armado_antes = false
local modo_antes = 0

-- parcelas congeladas no inicio do perfil (um ensaio nao muda de perfil no meio)
local p = {}

-- estados dos filtros de Dryden
local du_x = 0.0                                  -- OU exato (longitudinal)
-- estados do torque estocastico (variancia unitaria, por eixo)
local es_x, es_y, es_z = 0.0, 0.0, 0.0
-- evento de planta: 0 antes, 1 aplicado, 2 desfeito; e os valores originais
local evento = 0
local evento_orig = {}
local dv_u1, dv_u2, dv_y1, dv_y2 = 0.0, 0.0, 0.0, 0.0   -- Tustin 2a ordem (lateral)

-- contador de passos do gerador e ultima saida calculada
local passo = 0
local wn_atual, we_atual = 0.0, 0.0
local tq_x, tq_y, tq_z = 0.0, 0.0, 0.0

-- teto de passos por chamada: se o script travar, o perfil retoma sem tentar
-- recuperar minutos de turbulencia num unico ciclo
local MAX_PASSOS_POR_CICLO = 25

local function iniciar_perfil(agora_ms)
   p.modo = math.floor(WND_MODE:get() or 0)
   p.spd = WND_SPD:get() or 0
   p.dir = WND_DIR:get() or 0
   p.dirz = WND_DIRZ:get() or 0
   p.ga = WND_GA:get() or 0
   p.gt0 = WND_GT0:get() or 0
   p.gdur = math.max(WND_GDUR:get() or 0, 0.01)
   p.sigma = WND_TSIG:get() or 0
   p.lu = math.max(WND_TLU:get() or 200, 1.0)
   p.t0 = WND_T0:get() or 0
   p.tqx = WND_TQX:get() or 0
   p.tqy = WND_TQY:get() or 0
   p.tqz = WND_TQZ:get() or 0
   p.tqt0 = WND_TQT0:get() or 0
   p.tqdur = math.max(WND_TQDUR:get() or 0, 0.01)
   p.tqf0 = math.max(WND_TQF0:get() or 0.2, 0.01)
   p.tqf1 = math.max(WND_TQF1:get() or 10.0, 0.01)
   -- fade de 10% em cada ponta, para a varredura nao comecar nem terminar num
   -- degrau de amplitude
   p.tq_fade = p.tqdur * 0.1
   p.tq_wmin = 2.0 * math.pi * p.tqf0
   p.tq_wmax = 2.0 * math.pi * p.tqf1
   p.tq_B = math.log(p.tq_wmax / p.tq_wmin)
   -- torque estocastico: OU exato de variancia unitaria
   p.tqtc = math.max(WND_TQTC:get() or 0.1, 1e-3)
   p.es_phi = math.exp(-DT / p.tqtc)
   p.es_g = math.sqrt(math.max(1.0 - p.es_phi * p.es_phi, 0.0))
   p.evt = math.floor(WND_EVT:get() or 0)
   p.evm = math.floor(WND_EVM:get() or 0)
   p.evk = WND_EVK:get() or 1.0

   -- Velocidade de referencia dos filtros de Dryden. Um quadrotor em voo pairado
   -- tem velocidade propria quase nula, entao quem "varre" a escala de turbulencia
   -- e o proprio vento medio. Piso de 1 m/s para nao dividir por zero quando se
   -- pede turbulencia sem vento medio.
   p.va = math.max(p.spd, 1.0)

   -- MIL-F-8785C, forma de Dryden: a escala lateral e metade da longitudinal.
   p.lv = p.lu * 0.5

   -- longitudinal: OU exato. Variancia estacionaria = sigma^2 por construcao.
   p.u_phi = math.exp(-(p.va / p.lu) * DT)
   p.u_g = p.sigma * math.sqrt(math.max(1.0 - p.u_phi * p.u_phi, 0.0))

   --[[ lateral: H(s) = K*(s + z) / (s + a)^2, com a = Va/Lv, z = a/sqrt(3) e
        K = sigma*sqrt(3a). A ausencia do fator 1/pi nao e descuido: com ele a
        saida sai em 0.56*sigma. A normalizacao foi conferida numericamente
        (tres sementes, 6e4 s de serie) antes de escrever isto.                --]]
   local a = p.va / p.lv
   local K = p.sigma * math.sqrt(3.0 * a)
   local z = a / math.sqrt(3.0)
   local c = 2.0 / DT                       -- Tustin
   local A0 = c * c + 2.0 * a * c + a * a
   p.v_b0 = (K * c + K * z) / A0
   p.v_b1 = (2.0 * K * z) / A0
   p.v_b2 = (-K * c + K * z) / A0
   p.v_a1 = (-2.0 * c * c + 2.0 * a * a) / A0
   p.v_a2 = (c * c - 2.0 * a * c + a * a) / A0
   -- ruido branco de PSD unitaria amostrado a DT tem variancia 1/DT
   p.v_esc = 1.0 / math.sqrt(DT)

   du_x = 0.0
   dv_u1, dv_u2, dv_y1, dv_y2 = 0.0, 0.0, 0.0, 0.0
   es_x, es_y, es_z = 0.0, 0.0, 0.0
   evento = 0
   passo = 0
   wn_atual, we_atual = 0.0, 0.0
   tq_x, tq_y, tq_z = 0.0, 0.0, 0.0
   rng_semear(WND_SEED:get() or 1)

   -- arma o injetor de torque; as amplitudes sao moduladas a cada ciclo
   if (p.modo & MODO_TQ_QUALQUER) ~= 0 then
      SIM_TWIST_X:set(0)
      SIM_TWIST_Y:set(0)
      SIM_TWIST_Z:set(0)
      SIM_TWIST_TIME:set(TWIST_ARMADO_MS)
   end

   t_gatilho_ms = agora_ms
   perfil_ativo = true

   gcs:send_text(6, string.format(
      'vento: perfil modo=%d spd=%.1f raj=%.1f sig=%.2f tq=%.2f/%.2f/%.2f seed=%d',
      p.modo, p.spd, p.ga, p.sigma, p.tqx, p.tqy, p.tqz,
      math.floor(WND_SEED:get() or 1)))
end

local function desfazer_evento()
   if evento == 1 then
      if p.evt == 1 then
         SIM_ENGINE_FAIL:set(evento_orig.fail)
         SIM_ENGINE_MUL:set(evento_orig.mul)
      elseif p.evt == 2 then
         SIM_FRM_HOVR_THR:set(evento_orig.hovr)
      end
      evento = 2
      gcs:send_text(6, 'vento: evento de planta desfeito')
   end
end

local function aplicar_evento()
   evento_orig = {fail = SIM_ENGINE_FAIL:get(), mul = SIM_ENGINE_MUL:get(),
                  hovr = SIM_FRM_HOVR_THR:get()}
   if p.evt == 1 then
      SIM_ENGINE_MUL:set(p.evk)
      SIM_ENGINE_FAIL:set(p.evm)
   elseif p.evt == 2 then
      SIM_FRM_HOVR_THR:set(p.evk)
   end
   evento = 1
   gcs:send_text(6, string.format('vento: evento de planta %d (mascara %d, valor %.3f)',
                                  p.evt, p.evm, p.evk))
end

local function parar_perfil()
   desfazer_evento()
   perfil_ativo = false
   t_gatilho_ms = nil
   SIM_WIND_SPD:set(0)
   SIM_WIND_DIR_Z:set(0)
   SIM_TWIST_X:set(0)
   SIM_TWIST_Y:set(0)
   SIM_TWIST_Z:set(0)
   SIM_TWIST_TIME:set(0)
end

-------------------------------------------------------------------------------
-- geracao do vento
-------------------------------------------------------------------------------

--[[ Avanca o gerador EXATAMENTE UM passo de DT e devolve o vetor velocidade do
     ar em NED (para onde o ar vai), em m/s.

     PASSO FIXO, DESACOPLADO DO ESCALONADOR. O laco do Lua nao roda nos 20 ms
     pedidos — medido em voo, entrega ~44 Hz, com dt entre 15 e 55 ms. Se o
     gerador avancasse uma vez por chamada, o filtro de Dryden veria um dt
     diferente do que assume nos coeficientes (estatistica errada) e a sequencia
     dependeria do escalonamento (nao reproduzivel). Por isso o gerador tem seu
     proprio contador de passos: quem chama e que calcula quantos passos de DT ja
     se passaram no tempo simulado e pede exatamente esse tanto.

     Convencao: WND_DIR e SIM_WIND_DIR sao a direcao de ONDE o vento VEM. O vetor
     velocidade aponta, portanto, para o sentido oposto — e e ele que vai para o
     log e para a seta do GIF.

     Os eixos de Dryden acompanham o vento medio: u e longitudinal (ao longo da
     direcao media) e v e lateral (perpendicular, girado +90 graus).           --]]
--[[ Fator de forma do disturbio de torque em t, entre 0 e 1.

     Tres formas, selecionadas por bit e SOMADAS quando mais de uma esta ligada:

       pulso   amplitude constante entre tqt0 e tqt0+tqdur. O impulso angular e
               exatamente amplitude*duracao, entao a variacao de velocidade
               angular injetada e conhecida de antemao — e a forma mais facil de
               defender como "perturbacao impulsiva de torque".
       cosseno meio ciclo de cosseno na mesma janela, igual a rajada de vento.
               Mais suave; o impulso exige integrar a curva (vale amplitude*dur/2).
       degrau  amplitude constante a partir de tqt0, sem terminar. E o analogo em
               atitude do que o vento medio faz em posicao: expoe erro em regime
               sob momento constante, e portanto a suficiencia da acao integral.
--]]
local function forma_torque(t)
   local modo = p.modo
   local f = 0.0
   local dentro = (t >= p.tqt0) and (t <= p.tqt0 + p.tqdur)

   if (modo & MODO_TQ_PULSO) ~= 0 and dentro then
      f = f + 1.0
   end
   if (modo & MODO_TQ_COS) ~= 0 and dentro then
      f = f + 0.5 * (1.0 - math.cos(2.0 * math.pi * (t - p.tqt0) / p.tqdur))
   end
   if (modo & MODO_TQ_DEG) ~= 0 and t >= p.tqt0 then
      f = f + 1.0
   end

   --[[ Chirp: progressao EXPONENCIAL de frequencia, com janela de meio cosseno
        nas pontas — a mesma forma do Chirp::update do ArduPilot
        (AP_Math/chirp.cpp), para estes ensaios serem comparaveis aos do modo
        SYSTEMID nativo. A janela existe para a varredura nao comecar nem
        terminar num degrau de amplitude, que contaminaria o espectro.

        E o unico termo BIPOLAR: os outros tres so somam para um lado.       --]]
   if (modo & MODO_TQ_CHIRP) ~= 0 then
      local tc = t - p.tqt0
      if tc > 0.0 and tc <= p.tqdur then
         local janela
         if tc <= p.tq_fade then
            janela = 0.5 - 0.5 * math.cos(math.pi * tc / p.tq_fade)
         elseif tc <= p.tqdur - p.tq_fade then
            janela = 1.0
         else
            janela = 0.5 - 0.5 * math.cos(
               math.pi * (tc - (p.tqdur - p.tq_fade)) / p.tq_fade + math.pi)
         end
         local fase
         if p.tq_B < 1e-9 then          -- f0 == f1: senoide de frequencia fixa
            fase = p.tq_wmin * tc
         else
            fase = (p.tq_wmin * p.tqdur / p.tq_B)
                   * (math.exp(p.tq_B * tc / p.tqdur) - 1.0)
         end
         f = f + janela * math.sin(fase)
      end
   end

   return f
end


local function avancar_um_passo()
   passo = passo + 1
   local t = passo * DT              -- tempo do perfil, exato por construcao
   local modo = p.modo
   local vel_long = 0.0              -- ao longo do eixo do vento medio
   local vel_lat = 0.0               -- perpendicular

   if (modo & MODO_MEDIO) ~= 0 then
      vel_long = vel_long + p.spd
   end

   if (modo & MODO_RAJADA) ~= 0 then
      local tg = t - p.gt0
      if tg >= 0.0 and tg <= p.gdur then
         vel_long = vel_long + 0.5 * p.ga * (1.0 - math.cos(2.0 * math.pi * tg / p.gdur))
      end
   end

   if (modo & MODO_DRYDEN) ~= 0 and p.sigma > 0.0 then
      local w0, w1 = rng_normais()

      -- longitudinal: OU exato
      du_x = p.u_phi * du_x + p.u_g * w0

      -- lateral: Dryden de 2a ordem por Tustin
      local uin = w1 * p.v_esc
      local y = p.v_b0 * uin + p.v_b1 * dv_u1 + p.v_b2 * dv_u2
                - p.v_a1 * dv_y1 - p.v_a2 * dv_y2
      dv_u2, dv_u1 = dv_u1, uin
      dv_y2, dv_y1 = dv_y1, y

      vel_long = vel_long + du_x
      vel_lat = vel_lat + y
   end

   -- do referencial do vento medio para NED. O vento medio VEM de p.dir, logo
   -- SOPRA para p.dir + 180 graus.
   local psi = (p.dir + 180.0) * RAD
   local cs, sn = math.cos(psi), math.sin(psi)

   -- torque estocastico: tres normais por passo (duas chamadas, a quarta e
   -- descartada), sorteadas DEPOIS das do Dryden e so com o bit ligado, para a
   -- sequencia ser a mesma em ensaios.analise.dryden_ref
   if (modo & MODO_TQ_ESTOC) ~= 0 then
      local n1, n2 = rng_normais()
      local n3, _ = rng_normais()
      es_x = p.es_phi * es_x + p.es_g * n1
      es_y = p.es_phi * es_y + p.es_g * n2
      es_z = p.es_phi * es_z + p.es_g * n3
   end

   -- torque: mesma base de tempo, mesmo passo fixo
   local ft = forma_torque(t)
   tq_x, tq_y, tq_z = p.tqx * ft, p.tqy * ft, p.tqz * ft
   if (modo & MODO_TQ_ESTOC) ~= 0 and t >= p.tqt0 and t <= p.tqt0 + p.tqdur then
      tq_x = tq_x + p.tqx * es_x
      tq_y = tq_y + p.tqy * es_y
      tq_z = tq_z + p.tqz * es_z
   end

   return vel_long * cs - vel_lat * sn,
          vel_long * sn + vel_lat * cs
end

-------------------------------------------------------------------------------
-- laco principal
-------------------------------------------------------------------------------

local function update()
   local agora_ms = millis():tofloat()
   local modo = math.floor(WND_MODE:get() or 0)
   local armado = arming:is_armed()
   local t0_cfg = WND_T0:get() or 0

   -- gatilho
   if modo == 0 then
      if perfil_ativo then parar_perfil() end
   elseif t0_cfg >= 0 then
      if armado and not armado_antes then
         iniciar_perfil(agora_ms)              -- zera no armamento
      elseif not armado and perfil_ativo then
         parar_perfil()                        -- desarmou: encerra o perfil
      end
   else
      if modo_antes == 0 then
         iniciar_perfil(agora_ms)              -- modo de bancada: zera ao ligar o MODE
      end
   end
   armado_antes = armado
   modo_antes = modo

   if not perfil_ativo then
      return update, INTERVALO_MS
   end

   -- no modo de bancada (WND_T0 < 0) o perfil comeca de imediato
   local espera = (p.t0 >= 0.0) and p.t0 or 0.0
   local t = (agora_ms - t_gatilho_ms) * 0.001 - espera
   if t < 0.0 then
      return update, INTERVALO_MS               -- ainda na folga anterior ao perfil
   end

   -- Poe o gerador em dia com o tempo simulado, um passo de DT por vez. O laco
   -- do Lua nao entrega os 20 ms pedidos (medido: ~44 Hz), entao o numero de
   -- passos por ciclo varia — mas a SEQUENCIA gerada nao, e e isso que torna o
   -- ensaio reproduzivel.
   local alvo = math.floor(t / DT)
   local restantes = MAX_PASSOS_POR_CICLO
   while passo < alvo and restantes > 0 do
      wn_atual, we_atual = avancar_um_passo()
      restantes = restantes - 1
   end
   if passo < alvo then
      passo = alvo                              -- desistiu de recuperar; segue em frente
   end

   -- evento de planta, no relogio do perfil
   if (p.modo & MODO_EVENTO) ~= 0 then
      local tp = passo * DT
      if evento == 0 and tp >= p.tqt0 then
         aplicar_evento()
      elseif evento == 1 and tp > p.tqt0 + p.tqdur then
         desfazer_evento()
      end
   end

   local wn, we = wn_atual, we_atual
   local spd = math.sqrt(wn * wn + we * we)

   -- de volta para a convencao do firmware: direcao de ONDE o vento vem
   local dir = math.fmod(math.atan(-we, -wn) * GRAUS + 360.0, 360.0)
   local wd = spd * math.sin(p.dirz * RAD)      -- componente vertical resultante

   SIM_WIND_SPD:set(spd)
   SIM_WIND_DIR:set(dir)
   SIM_WIND_DIR_Z:set(p.dirz)

   if (p.modo & MODO_TQ_QUALQUER) ~= 0 then
      SIM_TWIST_X:set(tq_x)
      SIM_TWIST_Y:set(tq_y)
      SIM_TWIST_Z:set(tq_z)
   end

   -- Verdade do vento, na mesma base de tempo de ATT/RATE/PIDR. O campo Tp e o
   -- tempo do PERFIL (passo * DT): e ele que permite conferir o log contra a
   -- referencia em Python sem depender de quando cada amostra foi gravada.
   logger:write('WND', 'Tp,Wn,We,Wd,Spd,Dir,Tx,Ty,Tz,Mode,Ev', 'fffffffffHB',
                passo * DT, wn, we, wd, spd, dir, tq_x, tq_y, tq_z, p.modo, evento)

   return update, INTERVALO_MS
end

gcs:send_text(6, 'vento: injetor carregado (WND_MODE=0 para desligar)')

return update, INTERVALO_MS
