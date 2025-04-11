/* =================================================================================
文件名：PID_GRANDO.H 
===================================================================================*/


#ifndef __PID_H__
#define __PID_H__

#include "IQmathLib.h"

typedef struct {  _iq  Ref;      // 输入：参考设定值
      _iq  Fbk;      // 输入：反馈值
      _iq  Out;      // 输出：控制器输出
      _iq  c1;       // 内部：微分滤波器系数1
      _iq  c2;       // 内部：微分滤波器系数2
    } PID_TERMINALS;
    // 注意：将c1和c2放在这里可以保持结构体大小为8个字

typedef struct {  _iq  Kr;     // 参数：参考设定值权重
      _iq  Kp;     // 参数：比例增益
      _iq  Ki;     // 参数：积分增益
      _iq  Kd;     // 参数：微分增益
      _iq  Km;     // 参数：微分权重
      _iq  Umax;   // 参数：上饱和限制
      _iq  Umin;   // 参数：下饱和限制
    } PID_PARAMETERS;

typedef struct {  _iq  up;     // 数据：比例项
      _iq  ui;     // 数据：积分项
      _iq  ud;     // 数据：微分项
      _iq  v1;     // 数据：饱和前的控制器输出
      _iq  i1;     // 数据：积分器存储器：ui(k-1)
      _iq  d1;     // 数据：微分器存储器：ud(k-1)
      _iq  d2;     // 数据：微分器存储器：d2(k-1)
      _iq  w1;     // 数据：饱和记录：[u(k-1) - v(k-1)]
    } PID_DATA;


typedef struct {  PID_TERMINALS term;
      PID_PARAMETERS param;
      PID_DATA  data;
    } PID_CONTROLLER;

/*-----------------------------------------------------------------------------
PID对象的默认初始化值
-----------------------------------------------------------------------------*/

#define PID_TERM_DEFAULTS {    \
                           0,  \
                           0,  \
                           0,  \
                           0,  \
                           0   \
                   }

#define PID_PARAM_DEFAULTS {         \
                           _IQ(1.0), \
                           _IQ(1.0), \
                           _IQ(0.0), \
                           _IQ(0.0), \
                           _IQ(1.0), \
                           _IQ(1.0), \
                           _IQ(-1.0) \
                   }

#define PID_DATA_DEFAULTS {          \
                           _IQ(0.0), \
                           _IQ(0.0), \
                           _IQ(0.0), \
                           _IQ(0.0), \
                           _IQ(0.0), \
                           _IQ(0.0), \
                           _IQ(0.0), \
                           _IQ(1.0)  \
                   }


/*------------------------------------------------------------------------------
PID宏定义
------------------------------------------------------------------------------*/

#define PID_MACRO(v)                                              \
                                                                  \
/* 比例项 */                                                      \
v.data.up = _IQmpy(v.param.Kr, v.term.Ref) - v.term.Fbk;          \
                                                                  \
/* 积分项 */                                                      \
v.data.ui = _IQmpy(v.param.Ki, _IQmpy(v.data.w1,                  \
(v.term.Ref - v.term.Fbk))) + v.data.i1;                          \
v.data.i1 = v.data.ui;                                            \
                                                                  \
/* 微分项 */                                                      \
v.data.d2 = _IQmpy(v.param.Kd, _IQmpy(v.term.c1,                  \
(_IQmpy(v.term.Ref, v.param.Km) - v.term.Fbk))) - v.data.d2;      \
v.data.ud = v.data.d2 + v.data.d1;                                \
v.data.d1 = _IQmpy(v.data.ud, v.term.c2);                         \
                                                                  \
/* 控制器输出 */                                                  \
v.data.v1 = _IQmpy(v.param.Kp,                                    \
(v.data.up + v.data.ui + v.data.ud));                             \
v.term.Out= _IQsat(v.data.v1, v.param.Umax, v.param.Umin);        \
v.data.w1 = (v.term.Out == v.data.v1) ? _IQ(1.0) : _IQ(0.0);      \
 
#endif // __PID_H__
