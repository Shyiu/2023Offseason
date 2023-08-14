package org.firstinspires.ftc.teamcode;

final public class RobotConstants {

    //Chassis Hardware
    final public static String fr = "frontRight"; //CHUB Motor Port:
    final public static String br = "backRight";  //CHUB Motor Port:
    final public static String fl = "frontLeft";  //CHUB Motor Port:
    final public static String bl = "backLeft";  //CHUB Motor Port:

    //Lift Hardware
    final public static String leftSlide = "leftSlide";  //EHUB Motor Port:
    final public static String rightSlide = "rightSlide";  //EHUB Motor Port:
    final public static String magnetSensor = "magnetSensor"; //CHUB Digital Port:

    //Delivery Hardware
    final public static String arm = "arm"; //EHUB Motor Port:
    final public static String leftBase = "leftBase"; //CHUB Servo Port: 0
    final public static String rightBase = "rightBase"; //CHUB Servo Port: 1
    final public static String joint = "joint"; //EHUB Servo Port:
    final public static String claw = "claw"; //CHUB Servo Port:

    //Intake Hardware
    final public static String intakeMotor = "intakeMotor"; //EHUB Motor Port:
    final public static String intakeServo = "intakeServo";  //CHUB Servo Port:

    //Lift Constants --> Find through testing
    final public static int maxLiftPosition = 0;
    final public static int groundPosition = 0;
    final public static int lowPosition = 0;
    final public static int highPosition = 0;

    //Claw Constants --> Find through testing
    final public static double clawOpenPos = 0;
    final public static double clawClosePos = 0;

    //Delivery Constants --> Find through testing
    final public static int armMaxPos = 0;
    final public static int armMinPos = 0;

    //Camera Names
    final public static String junctionCamera = "junction";
}
