package org.firstinspires.ftc.teamcode.drive.Robotv7;

public class RobotConstants {
    // -------------------------------------------------------------- ROBOT CONFIG

    public static final String FRONT_LEFT = "frontL";
    public static final String FRONT_RIGHT = "frontR";
    public static final String BACK_LEFT = "backL";
    public static final String BACK_RIGHT = "backR";
    public static final String ARM_R = "armR";
    public static final String ARM_L = "armL";
    public static final String SERVO_CLAW = "claw";
    public static final String SERVO_WRIST = "wrist";
    public static final String SERVO_ELBOW_L = "elbowL";
    public static final String SERVO_ELBOW_R = "elbowR";
    public static final String HUB_IMU = "imu";

    public static final int MAX_OUTTAKE_HEIGHT = 4300;
    public static final int MIN_OUTTAKE_HEIGHT = 0;
    public static final int ARM_ADJUSTMENT_INCREMENT = 50;
    public static final int ARM_BOOST_MODIFIER = 1;
    public static final int ARM_RESET_TIMEOUT = 3;

    public static final double CLAW_CLOSE = 0;
    public static final double CLAW_OPEN = 0.22;
    public static final double WRIST_STANDBY = 0.52;
    public static final double WRIST_ACTIVE = 0.3;
    public static final double ELBOW_STANDBY = 0.52;
    public static final double ELBOW_ACTIVE = 1;

    public static final double MAX_ACCELERATION_DEVIATION = 10; // higher = less smoothing
    public static final double BASE_DRIVE_SPEED_MODIFIER = 1; // higher = less speed
    public static final double PRECISION_DRIVE_SPEED_MODIFIER = 3.35;

    public static final double PPR = 537.7; // gobuilda motor 85203 Series

    // -------------------------------------------------------------- JUNCTION PRESETS

    public static final int JUNCTION_OFF = 30; // will change
    public static final int JUNCTION_LOW = 1650;
    public static final int JUNCTION_MID = 2700;
    public static final int JUNCTION_STANDBY = 3200;
    public static final int JUNCTION_HIGH = 4000;
}
