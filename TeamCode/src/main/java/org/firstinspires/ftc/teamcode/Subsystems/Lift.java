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

    protected DigitalChannel magnetSensor;

    public static double Kp = 0.001;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kg = 0;
    private static int targetPos;
    private final PIDController PID;

    public Lift(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.get(DcMotor.class, RobotConstants.leftSlide);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        rightMotor = hardwareMap.get(DcMotor.class, RobotConstants.rightSlide);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        magnetSensor = hardwareMap.get(DigitalChannel.class, RobotConstants.magnetSensor);
        magnetSensor.setMode(DigitalChannel.Mode.INPUT);
        PID = new PIDController(Kp, Ki, Kd);
    }
    public boolean getMagnet(){
        return magnetSensor.getState();
    }
    public void init(){
        this.setPower(-0.05-Kg);
        while(this.getMagnet()){
            ;
        }
        this.setPower(0);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
            leftMotor.setPower(power+Kg);
            rightMotor.setPower(power+Kg);
        }
    }

    public int getPosition() {
       return (Math.abs(leftMotor.getCurrentPosition()) + Math.abs(rightMotor.getCurrentPosition()))/2;
    }

    //Position in encoder ticks
    public void setPosition(int position) {
        targetPos = position;
    }
    public void update(){
        this.setPower(PID.update(targetPos, this.getPosition()));
    }
    public void setPositionAndUpdate(int position){
        this.setPower(PID.update(position, this.getPosition()));
    }

}
