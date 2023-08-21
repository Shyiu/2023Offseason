package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Delivery;

public class MagnetSensorTesting extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();



        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
//            delivery.asyncMoveToPosition(1000);
            telemetry.addData("Status", "Running");
            telemetry.addData("Gamepad1 Left Stick y: ", -gamepad1.left_stick_x);

            telemetry.update();
            break;

        }    }
}
