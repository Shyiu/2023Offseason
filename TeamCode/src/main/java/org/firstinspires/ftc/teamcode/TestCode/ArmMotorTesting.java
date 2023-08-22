package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.Subsystems.Delivery;
@Config
@TeleOp(name="Arm Debug", group="Test Code")
public class ArmMotorTesting extends LinearOpMode {
    DcMotorEx arm;
    Delivery delivery;
    public static double maxSpeed = .5;
    @Override
    public void runOpMode() throws InterruptedException {
        arm = (DcMotorEx) hardwareMap.get(DcMotor.class, RobotConstants.arm);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        delivery = new Delivery(hardwareMap, telemetry);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        ElapsedTime time = new ElapsedTime();
        double currentTime = time.time();
        waitForStart();

        while(opModeIsActive()) {
            if(Math.abs(gamepad1.left_trigger) > 0){
                arm.setPower(maxSpeed * -1);
            }
            else if(Math.abs(gamepad1.right_trigger) > 0){
                arm.setPower(maxSpeed);
            }else{
                arm.setPower(0);
            }
            if(gamepad1.a && time.time() - currentTime > 1){
                delivery.init();
                currentTime = time.time();
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("Position", delivery.getPosition());
            telemetry.addData("Velocity", arm.getVelocity());
            telemetry.update();
        }
    }
}
