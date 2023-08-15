package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Delivery;

@TeleOp(name="Delivery Test", group="Test Code")
public class DeliveryTest extends LinearOpMode {

    Delivery delivery;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        delivery = new Delivery(hardwareMap);

        delivery.reset();

        waitForStart();

        while(opModeIsActive()) {
            delivery.turnArm(-gamepad1.left_stick_x);



            telemetry.addData("Status", "Running");
            telemetry.addData("Gamepad1 Left Stick y: ", -gamepad1.left_stick_x);
            telemetry.addData("Right Servo ", delivery.getServoBase1Pos());
            telemetry.addData("Left Servo ", delivery.getServoBase2Pos());
            telemetry.update();
        }
    }
}
