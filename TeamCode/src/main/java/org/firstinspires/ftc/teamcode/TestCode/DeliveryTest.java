package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Delivery;

@TeleOp(name="Delivery Test", group="Test Code")
public class DeliveryTest extends LinearOpMode {

    Delivery delivery;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        delivery = new Delivery(hardwareMap, telemetry);
        delivery.init();


        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
//            delivery.asyncMoveToPosition(1000);
            delivery.moveArm(-gamepad1.left_stick_y/2);

            telemetry.addData("Status", "Running");
            telemetry.addData("Position", delivery.getPosition());
            telemetry.addData("Gamepad1 Left Stick y: ", -gamepad1.left_stick_y);

            telemetry.update();
        }
    }
}