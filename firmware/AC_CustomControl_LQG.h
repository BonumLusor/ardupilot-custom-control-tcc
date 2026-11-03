#pragma once

/// @file    AC_CustomControl_LQG.h
/// @brief   Backend LQG do TCC: a lei do LQR + ESO sobre os estados de um
///          filtro de Kalman em regime.
///
/// Porte linha a linha do espelho LQG de modelagem/controladores.py, que e a
/// referencia. O estimador modela o passa-baixas do giroscopio, o ruido que o
/// angulo do AHRS integra sem esse filtro e o laco de atraso com que o backend
/// le a atitude (run_custom_controller roda antes de read_AHRS). As matrizes do
/// modelo (Phi, Gam) sao calculadas a bordo a partir de CC4_A, CC4_B, CC4_FLTW,
/// CC4_TAUC e do periodo do laco; os ganhos (K e L) vem de modelagem/projeto.py
/// pelos YAMLs de ensaios/controladores/.
///
/// Este arquivo vive no repositorio do TCC (firmware/) e entra no ArduPilot por
/// um link simbolico criado por firmware/instalar.sh.

#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_LQG_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter2p.h>

#include "AC_CustomControl_Backend.h"

// estados do estimador, em coordenadas de erro: e_th, e_w, f1, f2, d, eta, bg, z
#define CC_LQG_N 8

class AC_CustomControl_LQG : public AC_CustomControl_Backend {
public:
    AC_CustomControl_LQG(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    Vector3f update() override;
    void reset(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // Phi e Gam de um eixo e de um dos estimadores (0 lento, 1 rapido), se a,
    // b, fc, tau_c ou dt mudaram
    void calcular_modelo(uint8_t i, uint8_t q);
    // um passo de um estimador
    void estimar(uint8_t i, uint8_t q, float e_th, float e_w);
    // um passo de um eixo; devolve o esforco normalizado u
    float passo(uint8_t i, float e_th, float e_w, float u_ff, float w_t);
    void registrar(uint8_t i, float e_th, float e_w, float u_ff, float u, float w_t);

    float _dt;

    // parametros (CC4_*)
    AP_Int8     _ff;        // alimentacao direta pela inversao do modelo
    AP_Float    _a;         // polo da planta, 1/s
    AP_Float    _fltw;      // passa-baixas do giroscopio e do alvo de taxa, Hz
    AP_Vector3f _tauc;      // erro do angulo do AHRS no estimador, s, por eixo
    AP_Vector3f _b;         // eficacia de controle por eixo, rad/s2
    AP_Vector3f _kth;
    AP_Vector3f _kw;
    AP_Vector3f _emax;      // limite do erro de angulo, graus (0: sem limite)
    AP_Vector3f _lt[CC_LQG_N];  // ganho de Kalman sobre a inovacao do angulo
    AP_Vector3f _lw[CC_LQG_N];  // ganho de Kalman sobre a inovacao da taxa
    // estimador rapido (manobra)
    AP_Vector3f _taur;      // erro do angulo do AHRS no estimador rapido, s
    AP_Vector3f _wman;      // velocidade alvo a partir da qual o rapido assume, graus/s (0 desliga)
    AP_Float    _tman;      // constante de tempo da volta ao lento, s
    AP_Vector3f _lrt[CC_LQG_N];
    AP_Vector3f _lrw[CC_LQG_N];

    // modelo e estado, por eixo e por estimador (0 lento, 1 rapido)
    float _phi[2][3][CC_LQG_N][CC_LQG_N];
    float _gam[2][3][CC_LQG_N];
    float _modelo_de[2][3][5];  // a, b, fc, tau_c e dt com que o modelo foi calculado
    float _xe[2][3][CC_LQG_N];  // estimativas correntes
    float _x[3][CC_LQG_N];      // a estimativa usada na lei (a mistura)
    float _m[3];                // indicador de manobra
    float _alfa[3];             // peso do estimador rapido
    float _u_fb_ant[3];         // realimentacao aplicada no laco anterior
    float _inov[3][2];          // inovacoes do ultimo passo (log)
    float _w_t_ant[3];          // alvo de taxa do laco anterior (aceleracao alvo)
    bool _primeiro;             // primeiro passo depois de um reset
    LowPassFilter2pFloat _filtro_alvo[3];
};

#endif  // AP_COPTER_CUSTOMCONTROL_LQG_ENABLED
