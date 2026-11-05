#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_LQR_ENABLED

#include "AC_CustomControl_LQR.h"

#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_LQR::var_info[] = {
    // @Param: MODO
    // @DisplayName: Variante do controlador
    // @Description: 0 = LQI (realimentacao de [x_I, e_th, e_w]); 1 = LQR + ESO (realimentacao de [e_th, e_w] e cancelamento da perturbacao estimada)
    // @Values: 0:LQI, 1:LQR+ESO
    // @User: Advanced
    AP_GROUPINFO("MODO", 1, AC_CustomControl_LQR, _modo, 0),

    // @Param: FF
    // @DisplayName: Alimentacao direta
    // @Description: Soma u_ff = (a_t + A*w_t)/B, a inversao do modelo de atitude, ao esforco
    // @Values: 0:Desligada, 1:Ligada
    // @User: Advanced
    AP_GROUPINFO("FF", 2, AC_CustomControl_LQR, _ff, 1),

    // @Param: A
    // @DisplayName: Polo da planta
    // @Description: a do modelo dw/dt = -a*w + b*u + d (arrasto de momento e rotores)
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("A", 3, AC_CustomControl_LQR, _a, 1.988f),

    // @Param: FLTW
    // @DisplayName: Filtro do alvo de taxa
    // @Description: Passa-baixas aplicado ao alvo de velocidade angular antes de compara-lo ao giroscopio; o mesmo do giroscopio (INS_GYRO_FILTER). 0 desliga
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FLTW", 4, AC_CustomControl_LQR, _fltw, 20.0f),

    // @Param: B_X
    // @DisplayName: Eficacia de controle em rolagem
    // @Units: rad/s/s
    // @Param: B_Y
    // @DisplayName: Eficacia de controle em arfagem
    // @Units: rad/s/s
    // @Param: B_Z
    // @DisplayName: Eficacia de controle em guinada
    // @Units: rad/s/s
    // @User: Advanced
    AP_GROUPINFO("B", 5, AC_CustomControl_LQR, _b, 0),

    // @Param: KI_X
    // @DisplayName: Ganho da integral do erro de angulo, rolagem
    // @Param: KI_Y
    // @DisplayName: Ganho da integral do erro de angulo, arfagem
    // @Param: KI_Z
    // @DisplayName: Ganho da integral do erro de angulo, guinada
    // @User: Advanced
    AP_GROUPINFO("KI", 6, AC_CustomControl_LQR, _ki, 0),

    // @Param: KTH_X
    // @DisplayName: Ganho do erro de angulo, rolagem
    // @Param: KTH_Y
    // @DisplayName: Ganho do erro de angulo, arfagem
    // @Param: KTH_Z
    // @DisplayName: Ganho do erro de angulo, guinada
    // @User: Advanced
    AP_GROUPINFO("KTH", 7, AC_CustomControl_LQR, _kth, 0),

    // @Param: KW_X
    // @DisplayName: Ganho do erro de velocidade angular, rolagem
    // @Param: KW_Y
    // @DisplayName: Ganho do erro de velocidade angular, arfagem
    // @Param: KW_Z
    // @DisplayName: Ganho do erro de velocidade angular, guinada
    // @User: Advanced
    AP_GROUPINFO("KW", 8, AC_CustomControl_LQR, _kw, 0),

    // @Param: ESO_L1_X
    // @DisplayName: Ganho l1 do observador, rolagem
    // @Param: ESO_L1_Y
    // @DisplayName: Ganho l1 do observador, arfagem
    // @Param: ESO_L1_Z
    // @DisplayName: Ganho l1 do observador, guinada
    // @User: Advanced
    AP_GROUPINFO("ESO_L1", 9, AC_CustomControl_LQR, _l1, 0),

    // @Param: ESO_L2_X
    // @DisplayName: Ganho l2 do observador, rolagem
    // @Param: ESO_L2_Y
    // @DisplayName: Ganho l2 do observador, arfagem
    // @Param: ESO_L2_Z
    // @DisplayName: Ganho l2 do observador, guinada
    // @User: Advanced
    AP_GROUPINFO("ESO_L2", 10, AC_CustomControl_LQR, _l2, 0),

    // @Param: EMAX_X
    // @DisplayName: Limite do erro de angulo, rolagem
    // @Description: Satura o erro de angulo antes da realimentacao. 0 desliga
    // @Units: deg
    // @Param: EMAX_Y
    // @DisplayName: Limite do erro de angulo, arfagem
    // @Units: deg
    // @Param: EMAX_Z
    // @DisplayName: Limite do erro de angulo, guinada
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("EMAX", 11, AC_CustomControl_LQR, _emax, 0),

    AP_GROUPEND
};

AC_CustomControl_LQR::AC_CustomControl_LQR(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i = 0; i < 3; i++) {
        _x_i[i] = _u_ant[i] = _w_hat[i] = _d_hat[i] = _w_t_ant[i] = 0.0f;
    }
    _primeiro = true;
}

Vector3f AC_CustomControl_LQR::update()
{
    // no chao, nada acumula (como no backend PID de exemplo)
    switch (_motors->get_spool_state()) {
        case AP_Motors::SpoolState::SHUT_DOWN:
        case AP_Motors::SpoolState::GROUND_IDLE:
            reset();
            break;
        case AP_Motors::SpoolState::THROTTLE_UNLIMITED:
        case AP_Motors::SpoolState::SPOOLING_UP:
        case AP_Motors::SpoolState::SPOOLING_DOWN:
            break;
    }

    // erro de atitude e alvo de velocidade angular, como no AC_CustomControl_PID
    Quaternion attitude_body, attitude_target;
    _ahrs->get_quat_body_to_ned(attitude_body);
    attitude_target = _att_control->get_attitude_target_quat();
    Vector3f attitude_error;
    float thrust_angle_rad, thrust_error_angle_rad;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, thrust_angle_rad, thrust_error_angle_rad);
    const Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    const Vector3f w_t = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();
    const Vector3f giro = _ahrs->get_gyro_latest();

    if (_primeiro) {
        // sem historico: aceleracao alvo nula e filtro partindo do alvo atual
        for (uint8_t i = 0; i < 3; i++) {
            _w_t_ant[i] = w_t[i];
            _filtro_alvo[i].set_cutoff_frequency(1.0f / _dt, _fltw);
            _filtro_alvo[i].reset(w_t[i]);
        }
        _primeiro = false;
    }

    // o motor saturou no laco anterior? (integracao condicional do LQI)
    const bool limite[3] = {_motors->limit.roll, _motors->limit.pitch, _motors->limit.yaw};

    Vector3f u;
    for (uint8_t i = 0; i < 3; i++) {
        float e_th = attitude_error[i];
        const float e_max = radians(_emax.get()[i]);
        if (is_positive(e_max)) {
            e_th = constrain_float(e_th, -e_max, e_max);
        }
        const float e_w = _filtro_alvo[i].apply(w_t[i]) - giro[i];
        const float a_t = (w_t[i] - _w_t_ant[i]) / _dt;
        _w_t_ant[i] = w_t[i];
        const float b = _b.get()[i];
        const float u_ff = (_ff > 0 && is_positive(b)) ? (a_t + _a * w_t[i]) / b : 0.0f;

        u[i] = (_modo == 1) ? passo_eso(i, e_th, e_w, u_ff, giro[i], limite[i])
                            : passo_lqi(i, e_th, e_w, u_ff, limite[i]);
        registrar(i, e_th, e_w, u_ff, u[i]);
    }
    return u;
}

// LQI: u = u_ff + k_i*x_I + k_th*e_th + k_w*e_w, com x_I[k+1] = x_I[k] + dt*e_th[k]
float AC_CustomControl_LQR::passo_lqi(uint8_t i, float e_th, float e_w, float u_ff, bool limite)
{
    const float u = u_ff + _ki.get()[i] * _x_i[i] + _kth.get()[i] * e_th + _kw.get()[i] * e_w;
    if (!(limite && e_th * _u_ant[i] > 0.0f)) {
        _x_i[i] += _dt * e_th;
    }
    _u_ant[i] = u;
    return u;
}

// LQR + ESO: u = u_ff + k_th*e_th + k_w*e_w - d_hat/b, com o observador
//   [w; d][k+1] = [[fi, beta], [0, 1]] [w; d][k] + [beta*b; 0] u_apl[k]
//                 + [l1; l2] (giro[k] - w_hat[k])
float AC_CustomControl_LQR::passo_eso(uint8_t i, float e_th, float e_w, float u_ff, float giro, bool limite)
{
    const float b = _b.get()[i];
    const float u = u_ff + _kth.get()[i] * e_th + _kw.get()[i] * e_w
                    - (is_positive(b) ? _d_hat[i] / b : 0.0f);
    const float u_apl = constrain_float(u, -1.0f, 1.0f);
    const float fi = expf(-_a * _dt);
    const float beta = is_positive(_a.get()) ? (1.0f - fi) / _a : _dt;
    const float inov = giro - _w_hat[i];
    const float w_hat = fi * _w_hat[i] + beta * (b * u_apl + _d_hat[i]) + _l1.get()[i] * inov;
    // com o eixo saturado na alocacao (sinal do laco anterior), o esforco aplicado nao
    // e o u_apl que o observador supoe: d_hat fica parado, para nao absorver a
    // diferenca como perturbacao (windup do observador, revisao de 3/11)
    if (!limite) {
        _d_hat[i] += _l2.get()[i] * inov;
    }
    _w_hat[i] = w_hat;
    return u;
}

void AC_CustomControl_LQR::registrar(uint8_t i, float e_th, float e_w, float u_ff, float u)
{
    // @LoggerMessage: CCLR,CCLP,CCLY
    // @Description: Sinais internos do backend LQR, por eixo (rolagem, arfagem, guinada)
    // @Field: TimeUS: Time since system startup
    // @Field: Eth: erro de angulo
    // @Field: Ew: erro de velocidade angular
    // @Field: XI: integral do erro de angulo (LQI)
    // @Field: Uff: alimentacao direta
    // @Field: U: esforco calculado
    // @Field: Wh: velocidade angular estimada pelo ESO
    // @Field: Dh: perturbacao estimada pelo ESO, em rad/s/s
    static const char *const nomes[3] = {"CCLR", "CCLP", "CCLY"};
    AP::logger().WriteStreaming(nomes[i], "TimeUS,Eth,Ew,XI,Uff,U,Wh,Dh", "Qfffffff",
                                AP_HAL::micros64(), e_th, e_w, _x_i[i], u_ff, u,
                                _w_hat[i], _d_hat[i]);
}

// Transferencia sem salto: o estado novo assume o que o integrador do nativo
// segurava no instante da troca (o frontend so zera esse integrador depois).
// No LQI, x_I = i/k_i; no ESO, o integrador em regime segura -d/b, e d_hat = -b*i.
void AC_CustomControl_LQR::reset(void)
{
    const float i_nativo[3] = {
        _att_control->get_rate_roll_pid().get_i(),
        _att_control->get_rate_pitch_pid().get_i(),
        _att_control->get_rate_yaw_pid().get_i(),
    };
    const Vector3f giro = _ahrs->get_gyro_latest();
    for (uint8_t i = 0; i < 3; i++) {
        const float ki = _ki.get()[i];
        _x_i[i] = is_zero(ki) ? 0.0f : i_nativo[i] / ki;
        _u_ant[i] = i_nativo[i];
        _d_hat[i] = -_b.get()[i] * i_nativo[i];
        _w_hat[i] = giro[i];
    }
    _primeiro = true;
}

// O que o I do PID de taxa nativo segura em regime: -d/b. No LQI, k_i x_I (a lei em
// regime, com erros nulos); no ESO, -d_hat/b. Usado na volta ao nativo (CC_TRANSF).
bool AC_CustomControl_LQR::integrador_equivalente(Vector3f &i)
{
    for (uint8_t k = 0; k < 3; k++) {
        const float b = _b.get()[k];
        float v;
        if (_modo == 1) {
            v = is_positive(b) ? -_d_hat[k] / b : 0.0f;
        } else {
            v = _ki.get()[k] * _x_i[k];
        }
        i[k] = v;
    }
    return true;
}

#endif  // AP_COPTER_CUSTOMCONTROL_LQR_ENABLED
