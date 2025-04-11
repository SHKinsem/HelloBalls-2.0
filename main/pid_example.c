#include "pid.h"
#include "IQmathLib.h"
#include <stdio.h>

void pid_example(void)
{
    // 1. Declare and initialize a PID controller instance
    PID_CONTROLLER pid = {
        PID_TERM_DEFAULTS,
        PID_PARAM_DEFAULTS,
        PID_DATA_DEFAULTS
    };

    // 2. Configure PID parameters according to your system requirements
    pid.param.Kp = _IQ(2.5);    // 比例增益
    pid.param.Ki = _IQ(0.8);    // 积分增益
    pid.param.Kd = _IQ(0.1);    // 微分增益
    pid.param.Kr = _IQ(1.0);    // 参考权重，通常设为1
    pid.param.Km = _IQ(0.5);    // 微分权重
    pid.param.Umax = _IQ(10.0); // 输出上限
    pid.param.Umin = _IQ(-10.0); // 输出下限

    // 3. Set up differential filter coefficients
    pid.term.c1 = _IQ(0.5);     // 微分滤波器系数1
    pid.term.c2 = _IQ(0.25);    // 微分滤波器系数2

    // 4. Control loop example
    _iq setpoint = _IQ(5.0);    // 目标值设置，例如5.0
    _iq feedback = _IQ(0.0);    // 初始反馈值为0
    
    // 模拟10个控制周期
    for (int i = 0; i < 10; i++) {
        // 设置PID输入值
        pid.term.Ref = setpoint;           // 设置参考输入（目标值）
        pid.term.Fbk = feedback;           // 设置反馈值（当前测量值）
        
        // 执行PID计算
        PID_MACRO(pid);
        
        // 获取控制输出
        _iq control_output = pid.term.Out;
        
        // 在实际应用中，使用control_output控制您的系统
        printf("周期 %d: 设定值 = %f, 反馈值 = %f, 控制输出 = %f\n", 
               i, _IQtoF(setpoint), _IQtoF(feedback), _IQtoF(control_output));
        
        // 模拟系统响应（在实际应用中，这会是实际传感器测量值）
        // 这里我们简单地假设反馈值会向设定值移动一点
        feedback = _IQmpy(_IQ(0.8), feedback) + _IQmpy(_IQ(0.2), setpoint);
    }
}

// 在实际应用中，可以创建多个PID控制器实例用于不同控制回路
void motor_speed_control_example(void)
{
    PID_CONTROLLER speed_pid = {
        PID_TERM_DEFAULTS,
        PID_PARAM_DEFAULTS,
        PID_DATA_DEFAULTS
    };
    
    // 针对电机速度控制的PID参数配置
    speed_pid.param.Kp = _IQ(1.2);
    speed_pid.param.Ki = _IQ(0.5);
    speed_pid.param.Kd = _IQ(0.01);
    speed_pid.param.Umax = _IQ(100.0);  // 最大PWM输出
    speed_pid.param.Umin = _IQ(0.0);    // 最小PWM输出
    
    // 电机速度控制循环将在此处实现...
}
