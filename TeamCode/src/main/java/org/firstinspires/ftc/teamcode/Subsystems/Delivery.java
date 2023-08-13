package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Delivery extends Claw {
    Servo joint;
    Servo servoBase1;
    Servo servoBase2;
    DcMotor arm;

    public Delivery(HardwareMap hardwareMap) {
        super(hardwareMap);

        servoBase1=hardwareMap.get(Servo.class, RobotConstants.rightBase);
        servoBase2 = hardwareMap.get(Servo.class, RobotConstants.leftBase);

        joint = hardwareMap.get(Servo.class, RobotConstants.joint);

        arm = hardwareMap.get(DcMotor.class, RobotConstants.arm);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveArm(double power) {
        if(arm.getCurrentPosition() > RobotConstants.armMaxPos || arm.getCurrentPosition() < RobotConstants.armMinPos) {
            arm.setPower(0);
        } else {
            arm.setPower(power);
        }
    }

    public void turnArm(int degrees) {
        double newPosition = 0; //Some math

        servoBase1.setPosition(newPosition);
        servoBase2.setPosition(newPosition);
    }

    public void turnClaw(int degrees) {
        double newPosition = 0; //Some Math
        joint.setPosition(newPosition);
    }

}
