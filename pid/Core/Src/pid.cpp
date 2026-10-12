//
// Created by 35877 on 2026/10/11.
//

#include "pid.h"
#include "math.h"

PID::PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k):
        kp_(kp),ki_(ki),kd_(kd),i_max_(i_max),out_max_(out_max),d_filter_k(d_filter_k),
        error(0),error_int(0),last_error(0) {}

float PID::calc(float ref,float fdb) {
    float last_d_out=d_out;
    error = ref-fdb;
    p_out=kp_*error;

	error_int += error*0.001f;
	i_out=ki_*error_int;
    if(i_out>i_max_) i_out=i_max_;
    if(i_out<-i_max_) i_out=-i_max_;

    d_out=kd_ * (error-last_error)/0.001f;
    d_out=(1.0f-d_filter_k)*last_d_out+d_filter_k*d_out;

    out = p_out+i_out+d_out;
    if (out>out_max_) out=out_max_;
    if(out<-out_max_) out=-out_max_;

	last_error=error;
    return out;
}
void PID::reset() {
    last_error=0;
    error=0;
    error_int=0;
    p_out=0;
    i_out=0;
    d_out=0;
}
void PID::setParam(float kp, float ki, float kd) {
    kp_=kp;
    ki_=ki;
    kd_=kd;
}