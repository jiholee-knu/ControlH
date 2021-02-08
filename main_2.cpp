//
//  main.cpp
//  test
//
//  Created by solyeonkwon on 2021/02/06.
//

#include <iostream>
#include <istream>
#include <string>
#include <array>
#include <cmath>
#include <set>
#include "pid.h"
#include <stdio.h>
#include <math.h>

using namespace std;

//필요한 값들
double Car_v, WhlAng, WhlAcc ;
double UDP_Ax, UDP_WheelAng, Llinecolor, Rlinecolor; // 좌우 차선 색으로 차선 변경 가능 여부를 판단
double lanel0_x, lanel0_y, lanel0_z, lanel1_x, lanel1_y, lanel1_z, laner0_x, laner0_y, laner0_z, laner1_x, laner1_y, laner1_z;
double LCcmd, level, dist_front, dist_rear, rel_spd ;
double GCS_x, GCS_y, Car_yaw, devDist, devAng ;
double nextP[3] ; 

//PID 제어기 정의   * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
class PIDImpl
{
    public:
        PIDImpl( double dt, double Kp, double Kd, double Ki );
        ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _dt;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};


PID::PID( double dt, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID()
{
    delete pimpl;
}


// Implementation
PIDImpl::PIDImpl( double dt, double Kp, double Kd, double Ki ) :
    _dt(dt),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

double PIDImpl::calculate( double setpoint, double pv )
{
    double error = setpoint - pv;   // Calculate error

    double Pout = _Kp * error;   // Proportional term

    _integral += error * _dt;    // Integral term
    double Iout = _Ki * _integral;

    double derivative = (error - _pre_error) / _dt;    // Derivative term
    double Dout = _Kd * derivative;

    double output = Pout + Iout + Dout;    // Calculate total output

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

double* pathplan(double x0, double y0, double th0)
{
    /*
    make waypoints with RRT Dubins
    get their x, y, theta
    calculate devAng, devDist with the closest point
    */
    double way[3] ; // 0 for ego vehicle, 1 for all waypoints
    double d0, d1;
    double index = 0 ;

    d0 = sqrt( pow((way[0] - x0), 2) +  pow((way[1] - y0), 2)) ; //initialize fist point's distance
    for (int i = 1; i < 49; ++i )                       //choose the closest. 
    {
        d1 = sqrt( pow((way[0] - x0), 2) +  pow((way[1] - y0), 2)) ;
        if ( d1 < d0 )
        {
            d0 = d1;
            index = i ;
        }    
    }
   
   devDist = d0;
   devAng ; //  = waypoint[index][3] - th0 ; Note!!!consider its sign and rad|degree!!!
   double nextPlan[3] ; // = waypoint[path_index] ;

   return nextPlan ;

}

//Stanley method 를 이용한 목표 핸들 각도 계산
double Ctrl_stanley()
{
    double desAng = ( devAng + atan2( devDist, (Car_v + 0.5) ) ) * (-14.8) ; // -1은 deviation을 감소시키는 방향, 14.8은 wheel to steering whl 비율(속도에 따라 14.77~15)
    double max = 7 ; double min = -7 ; //핸들 최대,최소 반경 제한
    if( desAng > max )
        desAng = max;
        else if( desAng < min )
        desAng = min;

    return desAng;
}


// 장애물 회피 prototype : 차선 변경. 장애물 이외에 traffic 고려 X
double Ctrl_LC(double dist_front, double dist_rear, double Rcolor, double Lcolor)
{
    if (dist_front < 35 & dist_front > 0.1)  // 거리 측정. 위험 level 계산
        level = 1 ;
        else if (dist_front >= 35 )
        level = 2 ;
        else if (dist_front == 0)
        level = 0 ;

    double white = 0.0; // 흰 색 차선 정보 필요.

    int Lallow = 0 ; int Rallow = 0 ;
    if ( Lcolor == white )
        Lallow = 1;
    if ( Rcolor == white )
        Rallow = 1;
    
    if (level == 1)     // LCcmd -> 0:유지, 1:좌측, 2:우측, -1:불가(+감속)
    {
        if ( Lallow == 1 )
        LCcmd = 1 ;
        else if ( Lallow == 0 & Rallow == 1 ) // 추월은 좌측 우선으로 설정
        LCcmd = 2 ;
        else if ( Lallow == 0 & Rallow == 0 )
        LCcmd = -1 ;
    }
    
    double LCpoint[3] ;
   
    if ( LCcmd == 1 )
    {
        LCpoint = { + 10,  + 2.6, Car_yaw} ; // GCS 좌표에서 Yaw 방향으로 계산이 필요!!
        double LCflag = 0 ;
        while ((LCflag = 0)) {
            nextP = LCpoint ;  // pathplan 함수의 nextPoint를 LCpoint로 대체, LCpoint로부터의 Local path 생성
        }
    }
    else if ( LCcmd == 2 )
    {
        LCpoint = { + 10,  + 2.6, Car_yaw} ; // GCS 좌표에서 Yaw 방향으로 계산이 필요!!
        double LCflag = 0 ;
        while ( LCflag == 0 )
        {
            nextP = LCpoint ; 
        }
    }

    return 0;
};





// Desired Steering Ang 에 따른 속도 제어. 앞 차량과의 거리 줄어들면 필수 감속. line의 Curvature 계산하여 미리 감속하는 로직 반영 필요.
double Ctrl_Vdes(double desAng, double whlAcc)
{
    double V_des;
    double Angabs = fabs(desAng) ;
    double Accabs = fabs(whlAcc) ;

    if ( level != 1 )
    {
        if (Accabs > 50) //현재 Later Controller Stanley 추종 계수를 strict 하게 잡아서, 커브를 돌 때 휠을 52rad/s^2으로 빠르게 돌리는 현상 발견. 이 경우 감속
            V_des = 30 ;
        else {
            if (Angabs <0.3)  //핸들 각도 별 속도 조절(Veloity Profiler 부재로 임시 조치. 먼저 틀고 감속하면 교차로 좌/우회전시 반경 큼)
                V_des = 50 ;
                else if (Angabs >= 0.3 & Angabs < 0.6)
                    V_des = 46 ;
                else if (Angabs >= 0.5 & Angabs < 0.75)
                    V_des = 42 ;
                else if (Angabs >= 0.85 & Angabs < 1.0)
                    V_des = 38 ;
                else if (Angabs >= 1.0 & Angabs < 1.5)
                    V_des = 35 ;
                else if (Angabs >= 1.5 & Angabs < 2)
                    V_des = 30 ;
                else
                    V_des = 25 ;
        }
    }
    else {
        V_des = Car_v + rel_spd;
    }
    return V_des;
}


// 종방향 속도 추종 : PID
double Ctrl_long(double V_des, double Car_v)
{
    PID pid = PID(0.1, 0.9, 0, 0.005); //dt kp kd ki
    V_des = Ctrl_Vdes(WhlAng, WhlAcc);
    double vel_calib = V_des - 0.7 ; // 과속방지. 테스트 결과 ~49.9km/h
    double UDP_Ax = pid.calculate(vel_calib, Car_v) ; // target & current

    return UDP_Ax;
}


//횡방향 속도 추종 : PID , 튜닝 필요
double Ctrl_lat(double desAng, double curAng)
{
    PID pid = PID(0.1, 1, 0, 0.01); //dt kp kd ki
    double UDP_WheelAng = pid.calculate(desAng, curAng) ; // target & current

    return UDP_WheelAng;
}
