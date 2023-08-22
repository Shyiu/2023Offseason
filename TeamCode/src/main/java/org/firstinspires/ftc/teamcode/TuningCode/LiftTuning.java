package org.firstinspires.ftc.teamcode.TuningCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@Config
@TeleOp(name = "Lift Tuning", group = "Tuning")
public class LiftTuning extends LinearOpMode {

    Lift lift;
    public static int target = 300;


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        lift = new Lift(hardwareMap);

        waitForStart();

        while(opModeIsActive()){
            lift.setPositionAndUpdate(target);


            telemetry.addData("position", lift.getPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }


    }
}
