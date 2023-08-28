package org.firstinspires.ftc.teamcode.TuningCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Delivery;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Config //Tune PIDF values of the Delivery class using the opmode
@TeleOp(name = "Delivery Tuning", group = "Tuning")
public class DeliveryTuning extends LinearOpMode {

    Delivery delivery;
    public static int target = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        delivery = new Delivery(hardwareMap, telemetry);

        delivery.init();

        waitForStart();

        while(opModeIsActive()){
            delivery.setPosition(target);

            telemetry.addData("position", delivery.getPosition());
            telemetry.addData("target", target);

            delivery.update();
            telemetry.update();
        }


    }
}
