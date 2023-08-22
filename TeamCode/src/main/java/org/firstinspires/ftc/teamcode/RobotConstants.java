package org.firstinspires.ftc.teamcode;

final public class RobotConstants {

    //Chassis Hardware
    final public static String fr = "frontRight"; //CHUB Motor Port:
    final public static String br = "backRight";  //CHUB Motor Port:
    final public static String fl = "frontLeft";  //CHUB Motor Port:
    final public static String bl = "backLeft";  //CHUB Motor Port:

    //Lift Hardware
    final public static String leftSlide = "leftMotor";  //CHUB Motor Port: 2
    final public static String rightSlide = "rightMotor";  //CHUB Motor Port: 3
    final public static String magnetSensor = "magnet_sensor"; //CHUB Digital Port: 1-2


    //Delivery Hardware
    final public static String arm = "arm"; //EHUB Motor Port:
    final public static String leftBase = "leftBase"; //CHUB Servo Port: 0
    final public static String rightBase = "rightBase"; //CHUB Servo Port: 1
    final public static String joint = "joint"; //EHUB Servo Port:
    final public static String claw = "claw"; //CHUB Servo Port:
    final public static String pot = "pot"; //CUB Analog Port 0

    //Intake Hardware
    final public static String intakeMotor = "intakeMotor"; //EHUB Motor Port:
    final public static String intakeServo = "intakeServo";  //CHUB Servo Port:

    //Lift Constants --> Find through testing
    final public static int maxLiftPosition = 620;
    final public static int minLiftPosition = 35;
    final public static int groundPosition = 0;
    final public static int lowPosition = 0;
    final public static int mediumPosition = 0;
    final public static int highPosition = 0;

    //Claw Constants --> Find through testing
    final public static double clawOpenPos = 0;
    final public static double clawClosePos = 0.22;

    //Delivery Constants --> Find through testing
    final public static int armMaxPos = 0;
    final public static int armMinPos = 0;

    //Lift Constants --> Find through testing
    final public static int[] slidePositions = {35, 360, 610};
    //Camera Names
    final public static String junctionCamera = "junction";
}
