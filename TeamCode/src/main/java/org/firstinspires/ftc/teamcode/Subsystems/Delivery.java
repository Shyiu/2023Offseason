package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

import java.util.concurrent.TimeUnit;

@Config
public class Delivery extends Claw {
    Servo claw;
    DcMotorEx arm;

    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public double out;
    public static double armSpeed = .1;
    public static double pauseTime = .1;
    private boolean teleOp;
    private double reference = 0;
    private double lastReference = reference;
    private double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;

    private double maxIntegralSum = 3;

    private double a = 0.8; // a can be anything from 0 < a < 1
    private double previousFilterEstimate = 0;
    private double currentFilterEstimate = 0;
    private double encoderPosition;
    private double error;
    private double errorChange;
    private double derivative;
    private boolean direction;
    private Telemetry telemetry;
    public Delivery(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        this.telemetry = telemetry;
        claw = hardwareMap.get(Servo.class, RobotConstants.claw);

        arm = hardwareMap.get(DcMotorEx.class, RobotConstants.arm);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);



    }
    public void init(){
        arm.setPower(armSpeed);
        double currentCurrent = arm.getCurrent(CurrentUnit.MILLIAMPS);
        timer.reset();
        boolean stopped = false;
        while(true){
            if(arm.getVelocity() == 0 && !stopped){
                timer.reset();
                stopped = true;
            }
            else if(arm.getVelocity() != 0){
                stopped = false;
            }
            if(stopped){
                if(timer.time(TimeUnit.SECONDS) > pauseTime){
                    break;
                }
            }
            telemetry.addLine("moving");
            telemetry.addData("Velo", arm.getVelocity());
            telemetry.update();
        }
        telemetry.clearAll();
        arm.setPower(0);
        arm.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("calibrated");
        telemetry.update();
    }

    public void moveArm(double power) {
        if(teleOp) {
            if (arm.getCurrentPosition() > RobotConstants.armMaxPos || arm.getCurrentPosition() < RobotConstants.armMinPos) {
                arm.setPower(0);
            } else {
                arm.setPower(power);
            }
        }
    }
    public void asyncMoveToPosition(int target){
        reference = target;
        direction = arm.getCurrentPosition() < target;
        timer.reset();
    }
    public boolean isBusy(){
        if(direction){
            return arm.getCurrentPosition() >= reference;
        }else{
            return arm.getCurrentPosition() <= reference;
        }
    }
    public void update() {
        // Elapsed timer class from SDK, please use it, it's epic
        // obtain the encoder position
        if (!isBusy()) {
            encoderPosition = arm.getCurrentPosition();
            // calculate the error
            error = reference - encoderPosition;

            errorChange = (error - lastError);

            // filter out hight frequency noise to increase derivative performance
            currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            // rate of change of the error
            derivative = currentFilterEstimate / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());


            // max out integral sum
            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum;
            }

            if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum;
            }

            // reset integral sum upon setpoint changes
            if (reference != lastReference) {
                integralSum = 0;
            }

            out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            arm.setPower(out);

            lastError = error;

            lastReference = reference;

            timer.reset();
        }
        else{
            arm.setPower(0);
        }
    }
}
