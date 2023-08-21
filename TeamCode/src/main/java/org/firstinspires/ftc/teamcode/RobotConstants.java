package org.firstinspires.ftc.teamcode;

final public class RobotConstants {

    //Chassis Hardware
    final public static String fr = "frontRight"; //EHUB Motor Port: 0
    final public static String br = "backRight";  //EHUB Motor Port: 1
    final public static String fl = "frontLeft";  //EHUB Motor Port: 2
    final public static String bl = "backLeft";  //EHUB Motor Port: 3

    //Lift Hardware
    final public static String leftSlide = "leftMotor";  //CHUB Motor Port: 2
    final public static String rightSlide = "rightMotor";  //CHUB Motor Port: 3
    final public static String magnetSensor = "magnet_sensor"; //CHUB Digital Port: 1-2

    //Delivery Hardware
    final public static String arm = "arm"; //CHub Motor Port: 1
    final public static String claw = "claw"; //CHUB Servo Port:


    //Lift Constants --> Find through testing
    final public static int maxLiftPosition = 580;
    final public static int minLiftPosition = 100;
    final public static int pickupPosition = 0;
    final public static int groundPosition = 0;
    final public static int lowPosition = 0;
    final public static int mediumPosition = 0;
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
