/*
    Created by ReE=Rishun on 2019-05-13.
*/

//数値定義
    #define GYRO_OFFSET  0          // ジャイロセンサオフセット値(角速度0[deg/sec]時)
    #define LIGHT_WHITE  55         // 白色の光センサ値
    #define LIGHT_BLACK  0          // 黒色の光センサ値
    #define SONAR_ALERT_DISTANCE 30 // 超音波センサによる障害物検知距離[cm]
    #define TAIL_ANGLE_STAND_UP  90 // 完全停止時の角度[度]
    #define TAIL_ANGLE_DRIVE     10 // バランス走行時の角度[度]
    #define P_GAIN             2.5F // 完全停止用モーター制御比例係数
    #define PWM_ABS_MAX          60 // 完全停止用モーター制御PWM絶対最大値

//自作ヘッダー
    #include "ev3api.h"             //EV3API
    #include "app.h"
    #include "balancer.h"

    #if defined(BUILD_MODULE)
    #include "module_cfg.h"
    #else
    #include "kernel_cfg.h"
    #endif


//ポート番号指定
    /*【センサ指定】
            1：タッチセンサー
            2：ソナーセンサー
            3：カラーセンサー
            4：ジャイロセンサー
    */
    static const sensor_port_t
            touch_sensor    = EV3_PORT_1,
            sonar_sensor    = EV3_PORT_2,
            color_sensor    = EV3_PORT_3,
            gyro_sensor     = EV3_PORT_4;

    /*【モータ指定】
            A：尾モータ
            B：右側モータ
            C：左側モータ
            D：blank
    */
    static const motor_port_t
            left_motor      = EV3_PORT_C,
            right_motor     = EV3_PORT_B,
            tail_motor      = EV3_PORT_A;


//プロトタイプ宣言
    static int sonar_alert(void);
    static void tail_control(signed int angle);
    static void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc);


//メイン関数
void main_task(intptr_t unused)
{
    signed char forward;      /* 前進 */
    signed char turn;         /* 旋回 */
    signed char pwm_L, pwm_R; /* PWM左右 */

    //ポートに対してのモジュール設定
        //センサ設定
        ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
        ev3_sensor_config(color_sensor, COLOR_SENSOR);
        ev3_color_sensor_get_reflect(color_sensor);
        ev3_sensor_config(touch_sensor, TOUCH_SENSOR);
        ev3_sensor_config(gyro_sensor, GYRO_SENSOR);
        //モータ設定
        ev3_motor_config(left_motor, LARGE_MOTOR);
        ev3_motor_config(right_motor, LARGE_MOTOR);
        ev3_motor_config(tail_motor, LARGE_MOTOR);
        ev3_motor_reset_counts(tail_motor);


    //スタート待機
    while(1){

        //停止用に尾を下す
        tail_control(TAIL_ANGLE_STAND_UP);

        if (ev3_touch_sensor_is_pressed(touch_sensor) == 1)
        {
            break; //タッチセンサーが押されたら抜ける
        }
    }


    //ジャイロセンサのオフセット初期化
    ev3_gyro_sensor_reset(gyro_sensor);


    //モーター値初期化
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);


    //倒立振子API初期化
    balance_init();


     //mainループ
     while(1){

         //変数宣言
        int gyro,volt;
         int32_t motor_ang_l=0, motor_ang_r=0;

        //ボタンが押されたら抜ける
        if(ev3_button_is_pressed(BACK_BUTTON)) {
            break;
        }

        //尾を上げる
        tail_control(TAIL_ANGLE_DRIVE);

        //障害物検知
        if(sonar_alert()==1) {
            forward = turn = 0;
        }else {
            forward = 100;      //前進最大：100   後退最大：-100
            if (ev3_color_sensor_get_reflect(color_sensor) >= (LIGHT_WHITE + LIGHT_BLACK)/2)
            {
                turn =  30;     //黒色を拾ったとき
            }
            else
            {
                turn = -30;     //白色を拾ったとき
            }
        }

        //倒立振り子制御APIに渡すパラメータを取得する
         motor_ang_l = ev3_motor_get_counts(left_motor);
         motor_ang_r = ev3_motor_get_counts(right_motor);
         gyro = ev3_gyro_sensor_get_rate(gyro_sensor);
         volt = ev3_battery_voltage_mV();

         //バックラッシュキャンセル
         backlash_cancel(pwm_L, pwm_R, &motor_ang_l, &motor_ang_r);

         /* 倒立振子制御APIを呼び出し、倒立走行するための */
         /* 左右モーター出力値を得る */
         balance_control(
                 (float)forward,
                 (float)turn,
                 (float)gyro,
                 (float)GYRO_OFFSET,
                 (float)motor_ang_l,
                 (float)motor_ang_r,
                 (float)volt,
                 (signed char*)&pwm_L,
                 (signed char*)&pwm_R);


         /* EV3ではモーター停止時のブレーキ設定が事前にできないため */
         /* 出力0時に、その都度設定する */
         if (pwm_L == 0)
         {
             ev3_motor_stop(left_motor, true);
         }
         else
         {
             ev3_motor_set_power(left_motor, (int)pwm_L);
         }

         if (pwm_R == 0)
         {
             ev3_motor_stop(right_motor, true);
         }
         else
         {
             ev3_motor_set_power(right_motor, (int)pwm_R);
         }

         tslp_tsk(4); /* 4msec周期起動 */
     }
    ev3_motor_stop(left_motor, false);
    ev3_motor_stop(right_motor, false);

    ext_tsk();
}


//*****************************************************************************
// 関数名 : tail_control
// 引数 : angle (モーター目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モーターの角度制御
//*****************************************************************************
static void tail_control(signed int angle)
{
    signed int pwm = (signed int)((angle - ev3_motor_get_counts(tail_motor))*P_GAIN); /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }

    if (pwm == 0)
    {
        ev3_motor_stop(tail_motor, true);
    }
    else
    {
        ev3_motor_set_power(tail_motor, (signed char)pwm);
    }
}



//*****************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//*****************************************************************************
static int sonar_alert(void)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40/4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= SONAR_ALERT_DISTANCE) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }

    return alert;
}



//*****************************************************************************
// 関数名 : backlash_cancel
// 引数 : lpwm (左モーターPWM値 ※前回の出力値)
//        rpwm (右モーターPWM値 ※前回の出力値)
//        lenc (左モーターエンコーダー値)
//        renc (右モーターエンコーダー値)
// 返り値 : なし
// 概要 : 直近のPWM値に応じてエンコダー値にバックラッシュ分の値を追加します。
//*****************************************************************************
void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc)
{
    const int BACKLASHHALF = 4;   // バックラッシュの半分[deg]

    if(lpwm < 0) *lenc += BACKLASHHALF;
    else if(lpwm > 0) *lenc -= BACKLASHHALF;

    if(rpwm < 0) *renc += BACKLASHHALF;
    else if(rpwm > 0) *renc -= BACKLASHHALF;
}
