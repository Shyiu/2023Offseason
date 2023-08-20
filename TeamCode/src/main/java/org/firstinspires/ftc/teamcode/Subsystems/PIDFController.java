package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDFController extends PIDController {
    private double Kf;

    public PIDFController(double Kp, double Kd, double Ki, double Kf) { //PIDF without max integral sum or low-pass filter
        super(Kp, Kd, Ki);
        this.Kf = Kf;
    }

    public PIDFController(double Kp, double Kd, double Ki, double Kf, int maxIntegralSum) { //PIDF with integral max sum
        super(Kp, Kd, Ki, maxIntegralSum);
        this.Kf = Kf;
    }
    public PIDFController(double Kp, double Kd, double Ki, double Kf, double a) { //PIDF with low-pass filter
        super(Kp, Kd, Ki, a);
        this.Kf = Kf;
    }

    public PIDFController(double Kp, double Kd, double Ki, double Kf, double a, int maxIntegralSum) { //PIDF with everything
        super(Kp, Kd, Ki, a, maxIntegralSum);
        this.Kf = Kf;
    }

    @Override
    public double update(int target, int currentPos) {
        return super.update(target, currentPos) + Kf;
    }
}
