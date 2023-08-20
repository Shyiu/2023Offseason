package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Config
public class Lift {

    protected DcMotor leftMotor; //Testing Required to determine which one to reverse
    protected DcMotor rightMotor;

    //protected DigitalChannel magnetSensor;

    public static double Kp;
    public static double Ki;
    public static double Kd;
    public static double Kg;

    private PIDFController PIDF;

    public Lift(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, RobotConstants.leftSlide);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor = hardwareMap.get(DcMotor.class, RobotConstants.rightSlide);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //magnetSensor = hardwareMap.get(DigitalChannel.class, RobotConstants.magnetSensor);
        //magnetSensor.setMode(DigitalChannel.Mode.INPUT);

        PIDF = new PIDFController(Kp, Ki, Kd, Kg);
    }

    public void setPower(double power) { //Positive is up and negative is down
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (this.getPosition() > RobotConstants.maxLiftPosition && power > 0) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        } else if (this.getPosition() <= RobotConstants.minLiftPosition && power < 0){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        } else {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
        }
    }

    public int getPosition() {
       return (Math.abs(leftMotor.getCurrentPosition()) + Math.abs(rightMotor.getCurrentPosition()))/2;
    }

    //Position in encoder ticks
    public void setPosition(int position) {
        this.setPower(PIDF.update(position, this.getPosition()));
    }


}
