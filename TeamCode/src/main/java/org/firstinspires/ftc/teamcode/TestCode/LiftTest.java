package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Lift;


@TeleOp(name="Lift Test", group="Test Code")
public class LiftTest extends LinearOpMode {

    Lift lift;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lift = new Lift(hardwareMap);
        ElapsedTime time = new ElapsedTime();
        time.reset();
        double currentTime = time.time();

        waitForStart();

        while(opModeIsActive()) {
            lift.setPower(-gamepad1.left_stick_y);
            if(gamepad1.a && time.time() - currentTime > 1){
                lift.init();
                currentTime = time.time();
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("Magnet Sensor", lift.getMagnet());
            telemetry.addData("Gamepad1 Left Stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Slide Position", lift.getPosition());
            telemetry.update();
        }
    }
}
