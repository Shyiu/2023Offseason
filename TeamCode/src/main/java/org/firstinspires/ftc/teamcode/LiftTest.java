package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Delivery;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp(name="Lift Test", group="Test Code")
public class LiftTest extends LinearOpMode {

    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lift = new Lift(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            lift.setPower(-gamepad1.left_stick_y);



            telemetry.addData("Status", "Running");
            telemetry.addData("Gamepad1 Left Stick y: ", -gamepad1.left_stick_y);
            telemetry.addData("Slide Position", lift.getPosition());
            telemetry.update();
        }
    }
}
