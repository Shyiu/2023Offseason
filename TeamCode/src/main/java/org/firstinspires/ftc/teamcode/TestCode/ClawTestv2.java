package org.firstinspires.ftc.teamcode.TestCode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;

@Config
@TeleOp(name = "Claw Test", group = "Testing")
public class ClawTestv2 extends LinearOpMode {
    Claw claw;
    public static double pos = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = new Claw(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            claw.move(pos);
        }
    }
}
