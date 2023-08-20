package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;
@Config
public class Delivery extends Claw {
    Servo claw;
    DcMotorEx arm;
    public enum gameStages {
        TELE_OP,
        AUTONOMOUS
    }
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public double out;
    private gameStages game;
    public static int ampsTrigger = 10000000;
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
    public Delivery(HardwareMap hardwareMap, gameStages mode, Telemetry telemetry) {
        super(hardwareMap);
        this.telemetry = telemetry;
        claw = hardwareMap.get(Servo.class, RobotConstants.claw);

        arm = hardwareMap.get(DcMotorEx.class, RobotConstants.arm);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        teleOp = mode.equals(gameStages.TELE_OP);
        if(!teleOp){
            init();
        }

    }
    public void init(){
        arm.setPower(1);
        double currentCurrent = arm.getCurrent(CurrentUnit.MILLIAMPS);
        timer.reset();
        while(arm.getCurrent(CurrentUnit.MILLIAMPS) < 6000){
            telemetry.addData("MilliAmps", arm.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
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
