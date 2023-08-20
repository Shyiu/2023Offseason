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
    public static int target;
    boolean changed = false; //Outside of loop()


    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        lift = new Lift(hardwareMap);

        waitForStart();

        while(opModeIsActive()){

            if(gamepad1.a && !changed) {
                if (lift.getPosition() <= 50) {
                    lift.setPosition(target);
                }
                else {
                    lift.setPower(0);
                }
                changed = true;
            } else if(!gamepad1.a) {
                changed = false;
            }

            telemetry.addData("position", lift.getPosition());
            telemetry.addData("target", target);
            telemetry.update();
        }


    }
}
