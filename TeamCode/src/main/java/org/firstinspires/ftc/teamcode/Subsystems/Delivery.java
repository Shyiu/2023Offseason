package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.RobotConstants;

public class Delivery extends Claw {
    DcMotor arm;

    public Delivery(HardwareMap hardwareMap) {
        super(hardwareMap);

        arm = hardwareMap.get(DcMotor.class, RobotConstants.arm);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turnArm(double power) {
        if(arm.getCurrentPosition() > RobotConstants.armMaxPos || arm.getCurrentPosition() < RobotConstants.armMinPos) {
            arm.setPower(0);
        } else {
            arm.setPower(power);
        }
    }

}
