//
// Created by 35877 on 2026/10/11.
//

#ifndef CAN_PID_H
#define CAN_PID_H

class PID {
    public:
        PID(float kp, float ki, float kd, float i_max, float out_max, float d_filter_k);
        void reset();
        float calc(float ref,float fdb);
        void setParam(float kp, float ki, float kd);
    private:
    float kp_,ki_,kd_,i_max_,out_max_,d_filter_k;
    float ref,fdb,last_fdb,out;
    float last_error,error,error_int;
    float p_out,i_out,d_out;
};


#endif //CAN_PID_H
