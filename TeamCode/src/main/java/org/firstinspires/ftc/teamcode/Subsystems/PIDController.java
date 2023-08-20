package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp;
    private double Kd;
    private double Ki;
    private double integralSum;
    private double a;
    private ElapsedTime timer;



    public PIDController(double Kp, double Kd, double Ki) {
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        timer = new ElapsedTime();
    }

    public void update(int target, int currentPos) {

    }



    
}
