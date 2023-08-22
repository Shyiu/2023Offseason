package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp(name="Claw Test", group="Test Code")
public class ClawTest extends LinearOpMode {
    private enum states  {
            WAITING,
            INCREMENT,
            INCREMENT_SMALL,
            DECREMENT,
            DECREMENT_SMALL,
            TIMER
    };
    states state = states.WAITING;
    Claw claw;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        claw = new Claw(hardwareMap);

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double currentTime = timer.time();
        while(opModeIsActive()) {
            switch (state){
                case WAITING:
                    if (Math.abs(gamepad1.right_trigger) > 0){
                        state = states.INCREMENT;
                        break;
                    }
                    if (Math.abs(gamepad1.left_trigger)> 0){
                        state = states.DECREMENT;
                        break;
                    }
                    if(gamepad1.right_bumper){
                        state = states.INCREMENT_SMALL;
                        break;
                    }
                    if(gamepad1.left_bumper){
                        state=states.DECREMENT_SMALL;
                        break;
                    }
                case TIMER:
                    if(timer.time() - currentTime > .5){
                        state = states.WAITING;
                        timer.reset();
                        break;
                    }
                    break;
                case INCREMENT:
                    claw.move(claw.getPosition() + .1);
                    state = states.TIMER;
                    currentTime = timer.time();
                    break;
                case DECREMENT:
                    claw.move(claw.getPosition() - .1);
                    state = states.TIMER;
                    currentTime = timer.time();
                    break;
                case INCREMENT_SMALL:
                    claw.move(claw.getPosition() + .01);
                    state = states.TIMER;
                    currentTime = timer.time();
                    break;
                case DECREMENT_SMALL:
                    claw.move(claw.getPosition() - .01);
                    state = states.TIMER;
                    currentTime = timer.time();
                    break;
            }

            telemetry.addLine("right trigger to increase .1\nleft trigger to decrease .1\nright bumper to increase 0.01\nleft bumper to decrease 0.01\n");
            telemetry.addData("Position", claw.getPosition());
            telemetry.update();
        }
    }
}
