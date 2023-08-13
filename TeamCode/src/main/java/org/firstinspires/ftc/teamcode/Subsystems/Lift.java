package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    protected DcMotor leftMotor; //Testing Required to determine which one to reverse
    protected DcMotor rightMotor;

    protected DigitalChannel magnetSensor;

    public Lift(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, RobotConstants.leftSlide);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor = hardwareMap.get(DcMotor.class, RobotConstants.rightSlide);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        magnetSensor =hardwareMap.get(DigitalChannel.class, RobotConstants.magnetSensor);
        magnetSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setPower(double power) { //Positive is up and negative is down

        if (magnetSensor.getState()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else if (leftMotor.getCurrentPosition() > RobotConstants.maxLiftPosition) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        } else {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
    }

    //Position in encoder ticks
    public void setPosition(int position) {

    }


}
