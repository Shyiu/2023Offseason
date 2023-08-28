package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Delivery;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp(name = "Lift + Delivery Test", group = "TeleOp")
public class Test extends LinearOpMode {
    Delivery delivery;
    Lift lift;

    @Override
    public void runOpMode() throws InterruptedException {
        delivery = new Delivery(hardwareMap, telemetry);
        lift = new Lift(hardwareMap);
        delivery.init();
        lift.init();

        waitForStart();

        while(opModeIsActive()) {
            lift.setPower(-gamepad1.left_stick_y);
            delivery.moveArm(-gamepad1.right_stick_y/2);
            delivery.open();

            if (gamepad1.a) {
                delivery.open();
            } else {
                delivery.close();
            }
        }

    }
}
