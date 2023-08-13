package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    Servo servo;

    public Claw(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, RobotConstants.claw);
    }

    public void open() {
        servo.setPosition(RobotConstants.clawOpenPos);
    }

    public void close() {
        servo.setPosition(RobotConstants.clawClosePos);
    }

    public boolean isGrabbed() { //Needs implementation
        return true;
    }

}
