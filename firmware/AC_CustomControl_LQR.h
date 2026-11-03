#pragma once

/// @file    AC_CustomControl_LQR.h
/// @brief   Backend do controlador de atitude proposto no TCC: LQI ou LQR + ESO.
///
/// Porte linha a linha dos espelhos LQI e LQRESO de modelagem/controladores.py,
/// que sao a referencia: os ganhos saem de modelagem/projeto.py e os YAMLs de
/// ensaios/controladores/ os aplicam como CC3_*.
///
/// Este arquivo vive no repositorio do TCC (firmware/) e entra no ArduPilot por
/// um link simbolico criado por firmware/instalar.sh.

#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_LQR_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <Filter/LowPassFilter2p.h>

#include "AC_CustomControl_Backend.h"

class AC_CustomControl_LQR : public AC_CustomControl_Backend {
public:
    AC_CustomControl_LQR(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt);

    Vector3f update() override;
    void reset(void) override;

    static const struct AP_Param::GroupInfo var_info[];

protected:
    // um passo de um eixo; devolve o esforco normalizado u
    float passo_lqi(uint8_t i, float e_th, float e_w, float u_ff, bool limite);
    float passo_eso(uint8_t i, float e_th, float e_w, float u_ff, float giro);
    void registrar(uint8_t i, float e_th, float e_w, float u_ff, float u);

    float _dt;

    // parametros (CC3_*)
    AP_Int8     _modo;      // 0: LQI, 1: LQR + ESO
    AP_Int8     _ff;        // alimentacao direta pela inversao do modelo
    AP_Float    _a;         // polo da planta, 1/s
    AP_Float    _fltw;      // passa-baixas do alvo de taxa, Hz (o do giroscopio)
    AP_Vector3f _b;         // eficacia de controle por eixo, rad/s2
    AP_Vector3f _ki;
    AP_Vector3f _kth;
    AP_Vector3f _kw;
    AP_Vector3f _l1;        // ganhos do ESO
    AP_Vector3f _l2;
    AP_Vector3f _emax;      // limite do erro de angulo, graus (0: sem limite)

    // estado, por eixo
    float _x_i[3];          // integral do erro de angulo (LQI)
    float _u_ant[3];        // u do laco anterior (integracao condicional)
    float _w_hat[3];        // estimativas do ESO
    float _d_hat[3];
    float _w_t_ant[3];      // alvo de taxa do laco anterior (aceleracao alvo)
    bool _primeiro;         // primeiro passo depois de um reset
    LowPassFilter2pFloat _filtro_alvo[3];
};

#endif  // AP_COPTER_CUSTOMCONTROL_LQR_ENABLED
