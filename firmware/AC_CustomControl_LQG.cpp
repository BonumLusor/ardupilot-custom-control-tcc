#include "AC_CustomControl_config.h"

#if AP_COPTER_CUSTOMCONTROL_LQG_ENABLED

#include "AC_CustomControl_LQG.h"

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Math/AP_Math.h>

// table of user settable parameters
const AP_Param::GroupInfo AC_CustomControl_LQG::var_info[] = {
    // @Param: FF
    // @DisplayName: Alimentacao direta
    // @Description: Soma u_ff = (a_t + A*w_t)/B, a inversao do modelo de atitude, ao esforco
    // @Values: 0:Desligada, 1:Ligada
    // @User: Advanced
    AP_GROUPINFO("FF", 1, AC_CustomControl_LQG, _ff, 1),

    // @Param: A
    // @DisplayName: Polo da planta
    // @Description: a do modelo dw/dt = -a*w + b*u + d
    // @Units: 1/s
    // @User: Advanced
    AP_GROUPINFO("A", 2, AC_CustomControl_LQG, _a, 1.988f),

    // @Param: FLTW
    // @DisplayName: Passa-baixas do giroscopio
    // @Description: Corte do Butterworth do giroscopio (INS_GYRO_FILTER), que o estimador modela e que tambem filtra o alvo de taxa
    // @Units: Hz
    // @User: Advanced
    AP_GROUPINFO("FLTW", 3, AC_CustomControl_LQG, _fltw, 20.0f),

    // @Param: TAUC_X
    // @DisplayName: Erro do angulo do AHRS no estimador, rolagem
    // @Description: Constante de tempo do erro do angulo do AHRS (a integral do ruido do giroscopio) no estimador. Um valor de projeto, em geral menor que o fisico (~3 s): com ele o estimador confia no AHRS abaixo de 1/(2 pi TAUC). 0 usa 0,3 s
    // @Units: s
    // @Param: TAUC_Y
    // @DisplayName: Erro do angulo do AHRS no estimador, arfagem
    // @Units: s
    // @Param: TAUC_Z
    // @DisplayName: Erro do angulo do AHRS no estimador, guinada
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("TAUC", 4, AC_CustomControl_LQG, _tauc, 0),

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
    AP_GROUPINFO("B", 5, AC_CustomControl_LQG, _b, 0),

    // @Param: KTH_X
    // @DisplayName: Ganho do erro de angulo estimado, rolagem
    // @Param: KTH_Y
    // @DisplayName: Ganho do erro de angulo estimado, arfagem
    // @Param: KTH_Z
    // @DisplayName: Ganho do erro de angulo estimado, guinada
    // @User: Advanced
    AP_GROUPINFO("KTH", 6, AC_CustomControl_LQG, _kth, 0),

    // @Param: KW_X
    // @DisplayName: Ganho do erro de taxa estimado, rolagem
    // @Param: KW_Y
    // @DisplayName: Ganho do erro de taxa estimado, arfagem
    // @Param: KW_Z
    // @DisplayName: Ganho do erro de taxa estimado, guinada
    // @User: Advanced
    AP_GROUPINFO("KW", 7, AC_CustomControl_LQG, _kw, 0),

    // @Param: EMAX_X
    // @DisplayName: Limite do erro de angulo, rolagem
    // @Description: Satura o erro de angulo medido antes do estimador. 0 desliga
    // @Units: deg
    // @Param: EMAX_Y
    // @DisplayName: Limite do erro de angulo, arfagem
    // @Units: deg
    // @Param: EMAX_Z
    // @DisplayName: Limite do erro de angulo, guinada
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("EMAX", 8, AC_CustomControl_LQG, _emax, 0),

    // Ganho de Kalman L (8 x 2), uma coluna de 3 eixos por parametro: L<i>T e o
    // ganho do estado i sobre a inovacao do angulo, L<i>W sobre a da taxa. Os
    // estados, na ordem: e_th, e_w, f1, f2, d, eta, bg, z.

    // @Param: L1T_X
    // @DisplayName: Kalman, estado 1 (erro de angulo), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L1T", 9, AC_CustomControl_LQG, _lt[0], 0),
    // @Param: L2T_X
    // @DisplayName: Kalman, estado 2 (erro de taxa), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L2T", 10, AC_CustomControl_LQG, _lt[1], 0),
    // @Param: L3T_X
    // @DisplayName: Kalman, estado 3 (filtro f1), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L3T", 11, AC_CustomControl_LQG, _lt[2], 0),
    // @Param: L4T_X
    // @DisplayName: Kalman, estado 4 (filtro f2), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L4T", 12, AC_CustomControl_LQG, _lt[3], 0),
    // @Param: L5T_X
    // @DisplayName: Kalman, estado 5 (perturbacao), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L5T", 13, AC_CustomControl_LQG, _lt[4], 0),
    // @Param: L6T_X
    // @DisplayName: Kalman, estado 6 (erro do AHRS), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L6T", 14, AC_CustomControl_LQG, _lt[5], 0),
    // @Param: L7T_X
    // @DisplayName: Kalman, estado 7 (vies do giroscopio), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L7T", 15, AC_CustomControl_LQG, _lt[6], 0),
    // @Param: L8T_X
    // @DisplayName: Kalman, estado 8 (angulo do laco anterior), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("L8T", 16, AC_CustomControl_LQG, _lt[7], 0),
    // @Param: L1W_X
    // @DisplayName: Kalman, estado 1 (erro de angulo), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L1W", 17, AC_CustomControl_LQG, _lw[0], 0),
    // @Param: L2W_X
    // @DisplayName: Kalman, estado 2 (erro de taxa), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L2W", 18, AC_CustomControl_LQG, _lw[1], 0),
    // @Param: L3W_X
    // @DisplayName: Kalman, estado 3 (filtro f1), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L3W", 19, AC_CustomControl_LQG, _lw[2], 0),
    // @Param: L4W_X
    // @DisplayName: Kalman, estado 4 (filtro f2), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L4W", 20, AC_CustomControl_LQG, _lw[3], 0),
    // @Param: L5W_X
    // @DisplayName: Kalman, estado 5 (perturbacao), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L5W", 21, AC_CustomControl_LQG, _lw[4], 0),
    // @Param: L6W_X
    // @DisplayName: Kalman, estado 6 (erro do AHRS), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L6W", 22, AC_CustomControl_LQG, _lw[5], 0),
    // @Param: L7W_X
    // @DisplayName: Kalman, estado 7 (vies do giroscopio), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L7W", 23, AC_CustomControl_LQG, _lw[6], 0),
    // @Param: L8W_X
    // @DisplayName: Kalman, estado 8 (angulo do laco anterior), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("L8W", 24, AC_CustomControl_LQG, _lw[7], 0),

    // Estimador rapido da manobra: roda junto com o lento, com as mesmas medidas
    // e o mesmo u, e a lei usa alfa*x_rapido + (1 - alfa)*x_lento, com alfa indo a
    // 1 quando |w_t| passa de WMAN e voltando a 0 com constante TMAN.

    // @Param: TAUR_X
    // @DisplayName: Erro do angulo do AHRS no estimador rapido, rolagem
    // @Units: s
    // @Param: TAUR_Y
    // @DisplayName: Erro do angulo do AHRS no estimador rapido, arfagem
    // @Units: s
    // @Param: TAUR_Z
    // @DisplayName: Erro do angulo do AHRS no estimador rapido, guinada
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("TAUR", 25, AC_CustomControl_LQG, _taur, 0),

    // @Param: WMAN_X
    // @DisplayName: Manobra: velocidade alvo, rolagem
    // @Description: Velocidade angular alvo a partir da qual o estimador rapido assume. 0 desliga o estimador rapido
    // @Units: deg/s
    // @Param: WMAN_Y
    // @DisplayName: Manobra: velocidade alvo, arfagem
    // @Units: deg/s
    // @Param: WMAN_Z
    // @DisplayName: Manobra: velocidade alvo, guinada
    // @Units: deg/s
    // @User: Advanced
    AP_GROUPINFO("WMAN", 26, AC_CustomControl_LQG, _wman, 0),

    // @Param: TMAN
    // @DisplayName: Manobra: volta ao estimador lento
    // @Description: Constante de tempo com que o peso do estimador rapido volta a zero depois da manobra
    // @Units: s
    // @User: Advanced
    AP_GROUPINFO("TMAN", 27, AC_CustomControl_LQG, _tman, 1.0f),

    // @Param: LR1T_X
    // @DisplayName: Kalman rapido, estado 1 (erro de angulo), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR1T", 28, AC_CustomControl_LQG, _lrt[0], 0),
    // @Param: LR2T_X
    // @DisplayName: Kalman rapido, estado 2 (erro de taxa), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR2T", 29, AC_CustomControl_LQG, _lrt[1], 0),
    // @Param: LR3T_X
    // @DisplayName: Kalman rapido, estado 3 (filtro f1), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR3T", 30, AC_CustomControl_LQG, _lrt[2], 0),
    // @Param: LR4T_X
    // @DisplayName: Kalman rapido, estado 4 (filtro f2), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR4T", 31, AC_CustomControl_LQG, _lrt[3], 0),
    // @Param: LR5T_X
    // @DisplayName: Kalman rapido, estado 5 (perturbacao), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR5T", 32, AC_CustomControl_LQG, _lrt[4], 0),
    // @Param: LR6T_X
    // @DisplayName: Kalman rapido, estado 6 (erro do AHRS), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR6T", 33, AC_CustomControl_LQG, _lrt[5], 0),
    // @Param: LR7T_X
    // @DisplayName: Kalman rapido, estado 7 (vies do giroscopio), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR7T", 34, AC_CustomControl_LQG, _lrt[6], 0),
    // @Param: LR8T_X
    // @DisplayName: Kalman rapido, estado 8 (angulo do laco anterior), inovacao da angulo, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR8T", 35, AC_CustomControl_LQG, _lrt[7], 0),
    // @Param: LR1W_X
    // @DisplayName: Kalman rapido, estado 1 (erro de angulo), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR1W", 36, AC_CustomControl_LQG, _lrw[0], 0),
    // @Param: LR2W_X
    // @DisplayName: Kalman rapido, estado 2 (erro de taxa), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR2W", 37, AC_CustomControl_LQG, _lrw[1], 0),
    // @Param: LR3W_X
    // @DisplayName: Kalman rapido, estado 3 (filtro f1), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR3W", 38, AC_CustomControl_LQG, _lrw[2], 0),
    // @Param: LR4W_X
    // @DisplayName: Kalman rapido, estado 4 (filtro f2), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR4W", 39, AC_CustomControl_LQG, _lrw[3], 0),
    // @Param: LR5W_X
    // @DisplayName: Kalman rapido, estado 5 (perturbacao), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR5W", 40, AC_CustomControl_LQG, _lrw[4], 0),
    // @Param: LR6W_X
    // @DisplayName: Kalman rapido, estado 6 (erro do AHRS), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR6W", 41, AC_CustomControl_LQG, _lrw[5], 0),
    // @Param: LR7W_X
    // @DisplayName: Kalman rapido, estado 7 (vies do giroscopio), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR7W", 42, AC_CustomControl_LQG, _lrw[6], 0),
    // @Param: LR8W_X
    // @DisplayName: Kalman rapido, estado 8 (angulo do laco anterior), inovacao da taxa, rolagem
    // @User: Advanced
    AP_GROUPINFO("LR8W", 43, AC_CustomControl_LQG, _lrw[7], 0),

    // @Param: DBGANG
    // @DisplayName: Depuracao: fonte do angulo medido
    // @Description: 0 usa o angulo e a taxa do AHRS. 1 usa como angulo a integral dos incrementos de angulo nao filtrados do INS, sem as correcoes do EKF, um laco atrasada como o AHRS que o backend le. 2 usa tambem a taxa do INS, sem a deriva estimada pelo EKF. So para a ablacao do diagnostico do ruido (revisao de 3/11)
    // @Values: 0:AHRS,1:Angulo do INS,2:Angulo e taxa do INS
    // @User: Advanced
    AP_GROUPINFO("DBGANG", 44, AC_CustomControl_LQG, _dbgang, 0),

    AP_GROUPEND
};

AC_CustomControl_LQG::AC_CustomControl_LQG(AC_CustomControl& frontend, AP_AHRS_View*& ahrs, AC_AttitudeControl*& att_control, AP_MotorsMulticopter*& motors, float dt) :
    AC_CustomControl_Backend(frontend, ahrs, att_control, motors, dt)
{
    _dt = dt;
    AP_Param::setup_object_defaults(this, var_info);
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < CC_LQG_N; j++) {
            _x[i][j] = _xe[0][i][j] = _xe[1][i][j] = 0.0f;
        }
        for (uint8_t j = 0; j < 5; j++) {
            _modelo_de[0][i][j] = _modelo_de[1][i][j] = -1.0f;
        }
        _u_fb_ant[i] = _w_t_ant[i] = _inov[i][0] = _inov[i][1] = _m[i] = _alfa[i] = 0.0f;
    }
    _primeiro = true;
}

// Phi e Gam do estimador de um eixo, discretizados com segurador de ordem zero:
// a exponencial de [[Ac*dt, Bu*dt], [0, 0]] por serie de Taylor com escala e
// quadratura, a mesma conta de controladores.expm_taylor. Ac e o modelo de
// controladores.matrizes_lqg:
//   e_th' = e_w
//   e_w'  = -a e_w - b u_fb - d
//   f1'   = f2
//   f2'   = wc^2 e_w - wc^2 f1 - sqrt(2) wc f2
//   d'    = 0                  (passeio aleatorio)
//   eta'  = -eta / tau_c
//   bg'   = 0                  (vies do giroscopio, passeio aleatorio)
// e o oitavo estado, z[k+1] = e_th[k] - eta[k], o angulo lido um laco depois.
void AC_CustomControl_LQG::calcular_modelo(uint8_t i, uint8_t q)
{
    // um AP_Vector3f nasce zerado: sem o parametro, 0,3 s
    const float tau_par = (q == 0) ? _tauc.get()[i] : _taur.get()[i];
    const float tau = is_positive(tau_par) ? tau_par : 0.3f;
    const float a = _a, b = _b.get()[i], fc = _fltw;
    const float de[5] = {a, b, fc, tau, _dt};
    bool igual = true;
    for (uint8_t j = 0; j < 5; j++) {
        igual = igual && is_equal(de[j], _modelo_de[q][i][j]);
    }
    if (igual) {
        return;
    }
    for (uint8_t j = 0; j < 5; j++) {
        _modelo_de[q][i][j] = de[j];
    }

    const uint8_t n = 8;            // sete estados continuos + a coluna da entrada
    float M[n][n] = {};
    const float wc = 2.0f * M_PI * fc;
    M[0][1] = 1.0f;
    M[1][1] = -a;
    M[1][4] = -1.0f;
    M[2][3] = 1.0f;
    M[3][1] = wc * wc;
    M[3][2] = -wc * wc;
    M[3][3] = -sqrtf(2.0f) * wc;
    M[5][5] = is_positive(tau) ? -1.0f / tau : 0.0f;
    M[1][7] = -b;
    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            M[r][c] *= _dt;
        }
    }

    // escala: norma (maior soma de linha) abaixo de 0,5
    float norma = 0.0f;
    for (uint8_t r = 0; r < n; r++) {
        float s = 0.0f;
        for (uint8_t c = 0; c < n; c++) {
            s += fabsf(M[r][c]);
        }
        norma = MAX(norma, s);
    }
    uint8_t quad = 0;
    if (norma > 0.5f) {
        quad = (uint8_t)ceilf(log2f(norma / 0.5f));
    }
    const float escala = 1.0f / (float)(1U << quad);
    float X[n][n], E[n][n], termo[n][n], tmp[n][n];
    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            X[r][c] = M[r][c] * escala;
            E[r][c] = termo[r][c] = (r == c) ? 1.0f : 0.0f;
        }
    }
    for (uint8_t k = 1; k <= 12; k++) {
        for (uint8_t r = 0; r < n; r++) {
            for (uint8_t c = 0; c < n; c++) {
                float s = 0.0f;
                for (uint8_t m = 0; m < n; m++) {
                    s += termo[r][m] * X[m][c];
                }
                tmp[r][c] = s / k;
            }
        }
        for (uint8_t r = 0; r < n; r++) {
            for (uint8_t c = 0; c < n; c++) {
                termo[r][c] = tmp[r][c];
                E[r][c] += termo[r][c];
            }
        }
    }
    for (uint8_t j = 0; j < quad; j++) {
        for (uint8_t r = 0; r < n; r++) {
            for (uint8_t c = 0; c < n; c++) {
                float s = 0.0f;
                for (uint8_t m = 0; m < n; m++) {
                    s += E[r][m] * E[m][c];
                }
                tmp[r][c] = s;
            }
        }
        for (uint8_t r = 0; r < n; r++) {
            for (uint8_t c = 0; c < n; c++) {
                E[r][c] = tmp[r][c];
            }
        }
    }

    for (uint8_t r = 0; r < CC_LQG_N; r++) {
        for (uint8_t c = 0; c < CC_LQG_N; c++) {
            _phi[q][i][r][c] = (r < 7 && c < 7) ? E[r][c] : 0.0f;
        }
        _gam[q][i][r] = (r < 7) ? E[r][7] : 0.0f;
    }
    _phi[q][i][7][0] = 1.0f;
    _phi[q][i][7][5] = -1.0f;
}

Vector3f AC_CustomControl_LQG::update()
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
    if (_dbgang > 0) {
        // ablacao (revisao de 3/11, Fase 3c): o angulo medido sai da integral dos
        // incrementos de angulo nao filtrados do INS, sem as correcoes do EKF. A
        // integral parte do AHRS e e lida ANTES de somar o incremento deste laco,
        // e fica um laco atrasada, como o AHRS que o backend le
        if (!_q_int_ok) {
            _q_int = attitude_body;
            _q_int_ok = true;
        }
        const Quaternion q_lida = _q_int;
        Vector3f dang;
        float dang_dt;
        if (AP::ins().get_delta_angle(dang, dang_dt)) {
            _q_int.rotate(dang);
            _q_int.normalize();
        }
        attitude_body = q_lida;
    }
    attitude_target = _att_control->get_attitude_target_quat();
    Vector3f attitude_error;
    float thrust_angle_rad, thrust_error_angle_rad;
    _att_control->thrust_heading_rotation_angles(attitude_target, attitude_body, attitude_error, thrust_angle_rad, thrust_error_angle_rad);
    const Quaternion rotation_target_to_body = attitude_body.inverse() * attitude_target;
    const Vector3f w_t = rotation_target_to_body * _att_control->get_attitude_target_ang_vel();
    // a taxa do AHRS e a do INS mais a deriva estimada pelo EKF (AP_AHRS::get_gyro_latest);
    // com _dbgang = 2, a do INS pura (ablacao)
    const Vector3f giro = (_dbgang >= 2) ? AP::ins().get_gyro() : _ahrs->get_gyro_latest();

    if (_primeiro) {
        // sem historico: aceleracao alvo nula e filtro partindo do alvo atual
        for (uint8_t i = 0; i < 3; i++) {
            _w_t_ant[i] = w_t[i];
            _filtro_alvo[i].set_cutoff_frequency(1.0f / _dt, _fltw);
            _filtro_alvo[i].reset(w_t[i]);
        }
        _primeiro = false;
    }

    // o motor saturou no laco anterior? (a perturbacao estimada fica parada)
    const bool limite[3] = {_motors->limit.roll, _motors->limit.pitch, _motors->limit.yaw};

    Vector3f u;
    for (uint8_t i = 0; i < 3; i++) {
        calcular_modelo(i, 0);
        if (is_positive(_wman.get()[i])) {
            calcular_modelo(i, 1);
        }
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

        u[i] = passo(i, e_th, e_w, u_ff, w_t[i], limite[i]);
        registrar(i, e_th, e_w, u_ff, u[i], w_t[i]);
    }
    return u;
}

// x_pred = Phi x + Gam u_fb_ant;  x = x_pred + L (y - C x_pred), y = [e_th, e_w]
void AC_CustomControl_LQG::estimar(uint8_t i, uint8_t q, float e_th, float e_w, bool limite)
{
    float x_pred[CC_LQG_N];
    for (uint8_t r = 0; r < CC_LQG_N; r++) {
        float s = _gam[q][i][r] * _u_fb_ant[i];
        for (uint8_t c = 0; c < CC_LQG_N; c++) {
            s += _phi[q][i][r][c] * _xe[q][i][c];
        }
        x_pred[r] = s;
    }
    // medidas: o angulo do laco anterior (estado z) e o erro de taxa filtrado,
    // menos o vies do giroscopio (f1 - bg)
    const float inov_t = e_th - x_pred[7];
    const float inov_w = e_w - (x_pred[2] - x_pred[6]);
    const AP_Vector3f *lt = (q == 0) ? _lt : _lrt;
    const AP_Vector3f *lw = (q == 0) ? _lw : _lrw;
    for (uint8_t r = 0; r < CC_LQG_N; r++) {
        // com o eixo saturado na alocacao (sinal do laco anterior), a perturbacao
        // (estado 4) nao e corrigida: o esforco aplicado nao e o que o estimador
        // supoe, e a diferenca viraria perturbacao (windup, revisao de 3/11)
        if (limite && r == 4) {
            _xe[q][i][r] = x_pred[r];
            continue;
        }
        _xe[q][i][r] = x_pred[r] + lt[r].get()[i] * inov_t + lw[r].get()[i] * inov_w;
    }
    if (q == 0) {
        _inov[i][0] = inov_t;
        _inov[i][1] = inov_w;
    }
}

// Os dois estimadores rodam com o mesmo u; a lei usa a mistura
// alfa*x_rapido + (1 - alfa)*x_lento, e u = u_ff + k_th x[0] + k_w x[1] - x[4]/b
float AC_CustomControl_LQG::passo(uint8_t i, float e_th, float e_w, float u_ff, float w_t, bool limite)
{
    estimar(i, 0, e_th, e_w, limite);
    const float w_ref = radians(_wman.get()[i]);
    float alfa = 0.0f;
    if (is_positive(w_ref)) {
        estimar(i, 1, e_th, e_w, limite);
        const float decai = expf(-_dt / MAX(_tman.get(), 0.01f));
        _m[i] = MAX(fabsf(w_t) / w_ref, decai * _m[i]);
        alfa = MIN(_m[i], 1.0f);
    }
    _alfa[i] = alfa;
    for (uint8_t r = 0; r < CC_LQG_N; r++) {
        _x[i][r] = alfa * _xe[1][i][r] + (1.0f - alfa) * _xe[0][i][r];
    }

    const float b = _b.get()[i];
    const float u_fb = _kth.get()[i] * _x[i][0] + _kw.get()[i] * _x[i][1]
                       - (is_positive(b) ? _x[i][4] / b : 0.0f);
    const float u = u_ff + u_fb;
    _u_fb_ant[i] = constrain_float(u, -1.0f, 1.0f) - u_ff;
    return u;
}

void AC_CustomControl_LQG::registrar(uint8_t i, float e_th, float e_w, float u_ff, float u, float w_t)
{
    // As mesmas mensagens e campos do backend LQR, para a analise servir sem
    // mudanca: XI = 0, Wh = velocidade angular estimada, Dh = perturbacao estimada.
    // @LoggerMessage: CCLR,CCLP,CCLY
    // @Description: Sinais internos do backend, por eixo (rolagem, arfagem, guinada)
    // @Field: TimeUS: Time since system startup
    // @Field: Eth: erro de angulo
    // @Field: Ew: erro de velocidade angular
    // @Field: XI: integral do erro de angulo (LQI; 0 no LQG)
    // @Field: Uff: alimentacao direta
    // @Field: U: esforco calculado
    // @Field: Wh: velocidade angular estimada
    // @Field: Dh: perturbacao estimada, em rad/s/s
    static const char *const nomes[3] = {"CCLR", "CCLP", "CCLY"};
    AP::logger().WriteStreaming(nomes[i], "TimeUS,Eth,Ew,XI,Uff,U,Wh,Dh", "Qfffffff",
                                AP_HAL::micros64(), e_th, e_w, 0.0f, u_ff, u,
                                w_t - _x[i][1], _x[i][4]);
    // @LoggerMessage: CCKR,CCKP,CCKY
    // @Description: Estados do Kalman do backend LQG, por eixo
    // @Field: TimeUS: Time since system startup
    // @Field: ThH: erro de angulo estimado
    // @Field: WeH: erro de taxa estimado
    // @Field: EtaH: erro do angulo do AHRS estimado
    // @Field: Bg: vies do giroscopio estimado
    // @Field: It: inovacao do angulo (estimador lento)
    // @Field: Iw: inovacao da taxa (estimador lento)
    // @Field: Al: peso do estimador rapido (manobra)
    static const char *const nomes_k[3] = {"CCKR", "CCKP", "CCKY"};
    AP::logger().WriteStreaming(nomes_k[i], "TimeUS,ThH,WeH,EtaH,Bg,It,Iw,Al", "Qfffffff",
                                AP_HAL::micros64(), _x[i][0], _x[i][1], _x[i][5],
                                _x[i][6], _inov[i][0], _inov[i][1], _alfa[i]);
}

// Transferencia sem salto, como no backend LQR: o integrador do nativo, em
// regime, segura -d/b; a estimativa da perturbacao o herda, e o resto (o vies
// do giroscopio inclusive) parte de zero.
void AC_CustomControl_LQG::reset(void)
{
    const float i_nativo[3] = {
        _att_control->get_rate_roll_pid().get_i(),
        _att_control->get_rate_pitch_pid().get_i(),
        _att_control->get_rate_yaw_pid().get_i(),
    };
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < CC_LQG_N; j++) {
            _x[i][j] = _xe[0][i][j] = _xe[1][i][j] = 0.0f;
        }
        _x[i][4] = _xe[0][i][4] = _xe[1][i][4] = -_b.get()[i] * i_nativo[i];
        _u_fb_ant[i] = i_nativo[i];
        _m[i] = _alfa[i] = 0.0f;
        _inov[i][0] = _inov[i][1] = 0.0f;
    }
    _primeiro = true;
    _q_int_ok = false;
}

#endif  // AP_COPTER_CUSTOMCONTROL_LQG_ENABLED
