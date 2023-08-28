package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    public PIDController PID;
    public static double Kp = 0.001;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kg = 0;

    public static double armSpeed = -.1;
    public static double pauseTime = .1;

    private int target = 0;

    private final int MAX_ERROR = 1;

    ElapsedTime timer = new ElapsedTime();

    private Telemetry telemetry;
    public Delivery(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);

        this.telemetry = telemetry;
        claw = hardwareMap.get(Servo.class, RobotConstants.claw);

        arm = hardwareMap.get(DcMotorEx.class, RobotConstants.arm);
        arm.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        PID = new PIDController(Kp, Ki, Kd);

    }
    public void init(){
        arm.setPower(armSpeed);
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
        double adjust_Kg = Kg * Math.cos(Math.PI/180 *6.0/7.0 * (arm.getCurrentPosition()+70));
        arm.setPower(power + adjust_Kg);
    }

    public boolean isBusy(){
        return Math.abs(arm.getCurrentPosition()-target) <= MAX_ERROR;
    }

    public void setPosition(int target) {
        this.target = target;
    }
    public void update() {
        // Elapsed timer class from SDK, please use it, it's epic
        // obtain the encoder position
        this.moveArm(PID.update(arm.getCurrentPosition(), target));
    }

    public int getPosition() {
        return arm.getCurrentPosition();
    }
}
