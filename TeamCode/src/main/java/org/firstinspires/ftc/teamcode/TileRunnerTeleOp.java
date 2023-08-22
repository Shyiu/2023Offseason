package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.Lift;

@TeleOp(name="Tele Op", group="Test Code")
public class TileRunnerTeleOp extends LinearOpMode {
    private enum clawStates  {
            OPEN,
            CLOSE
    };
    clawStates clawPosition = clawStates.OPEN;
    Claw claw;
    Lift lift;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        claw = new Claw(hardwareMap);
        lift = new Lift(hardwareMap);
        lift.init(); //remove this when we have an auto
        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        double currentTime = timer.time();
        while(opModeIsActive()) {
            switch (clawPosition){
                case OPEN:
                    if(timer.time() - currentTime > .5){
                        if(gamepad2.x){
                            clawPosition = clawStates.CLOSE;
                            break;
                        }
                    }
                    else if(claw.getPosition() != RobotConstants.clawOpenPos){
                        claw.open();
                        currentTime =timer.time();
                        break;
                    }
                    break;
                case CLOSE:
                    if(timer.time() - currentTime > .5){
                        if(gamepad2.x){
                            clawPosition = clawStates.OPEN;
                            break;
                        }
                    }
                    else if(claw.getPosition() != RobotConstants.clawClosePos){
                        claw.close();
                        currentTime =timer.time();
                        break;
                    }
                    break;
            }

            telemetry.addLine("right trigger to increase .1\nleft trigger to decrease .1\nright bumper to increase 0.01\nleft bumper to decrease 0.01\n");
            telemetry.addData("Position", claw.getPosition());
            telemetry.update();
        }
    }
}
