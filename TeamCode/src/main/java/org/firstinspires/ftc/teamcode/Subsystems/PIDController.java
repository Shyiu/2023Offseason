package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    private double Kp;
    private double Kd;
    private double Ki;

    private double integralSum;
    private double maxIntegralSum;

    private double error;
    private double prevError;
    private double errorChange;

    private double a; //Anything between 0 and 1 with 0 being no filter
    private double filterEstimate;
    private double prevFilterEstimate;


    private ElapsedTime timer;

    public PIDController(double Kp, double Kd, double Ki) { //PID without max integral sum or low-pass filter
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        timer = new ElapsedTime();
    }

    public PIDController(double Kp, double Kd, double Ki, int maxIntegralSum) { //PID with integral max sum
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        timer = new ElapsedTime();
        this.maxIntegralSum = maxIntegralSum;
    }
    public PIDController(double Kp, double Kd, double Ki, double a) { //PID with low-pass filter
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        timer = new ElapsedTime();
        this.a = a;
    }

    public PIDController(double Kp, double Kd, double Ki, double a, int maxIntegralSum) { //PID with everything
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        timer = new ElapsedTime();
        this.a = a;
        this.maxIntegralSum = maxIntegralSum;
    }

    public void reset() {
        integralSum = 0;
    }
    public double update(int target, int currentPos) {
        error = target-currentPos;
        errorChange = error - prevError;

        filterEstimate = (a * prevFilterEstimate) + (1-a) * errorChange;

        double derivative = filterEstimate / timer.seconds();

        integralSum += error * timer.seconds();

        if (maxIntegralSum != 0 && integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        } else if (maxIntegralSum != 0 && integralSum < -maxIntegralSum){
            integralSum = -maxIntegralSum;
        }

        prevError = error;
        prevFilterEstimate = filterEstimate;

        timer.reset();

        return (Kp * error) + (Ki * integralSum) + (Kd * derivative);
    }
}
