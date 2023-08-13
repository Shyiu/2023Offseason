package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Delivery;

public class DeliveryTest extends LinearOpMode {

    Delivery delivery;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        delivery = new Delivery(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            delivery.moveArm(-gamepad1.left_stick_y);
            delivery.turnArm(-gamepad1.right_stick_x*);
        }


    }
}
