package org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo;

public class RobotConstants {
    // -------------------------------------------------------------- CONTROLLER CONFIG
    public static final double JOYSTICK_DEADZONE = 0.03;

    // -------------------------------------------------------------- ROBOT CONFIG

    public static final String FRONT_LEFT = "frontL";
    public static final String FRONT_RIGHT = "frontR";
    public static final String BACK_LEFT = "backL";
    public static final String BACK_RIGHT = "backR";
    public static final String ARM_R = "armR";
    public static final String ARM_L = "armL";
    public static final String INTAKE_MOTOR = "intake";
    public static final String SERVO_FLAP = "flap";
    public static final String SERVO_CLAW = "claw";
    public static final String SERVO_WRIST = "wrist";
    public static final String SERVO_ELBOW_L = "elbowL";
    public static final String SERVO_ELBOW_R = "elbowR";
    public static final String SERVO_PLANE = "plane";
    public static final String SERVO_HANG_R = "hangR";
    public static final String SERVO_HANG_L = "hangL";
    public static final String HUB_IMU = "imu";
    public static final String FRONT_CAMERA = "Webcam 1";
    public static final String BACK_CAMERA = "Webcam 2";

    public static final int MAX_OUTTAKE_SPEED = 2800;
    public static final int MAX_OUTTAKE_HEIGHT = 3200;
    public static final int MIN_OUTTAKE_HEIGHT = 0;
    public static final int ARM_ADJUSTMENT_INCREMENT = 25; // used to be 50
    public static final int ARM_BOOST_MODIFIER = 1;
    public static final int ARM_RESET_TIMEOUT = 3;

    public static final double FLAP_CLOSE = 0.8;
    public static final double FLAP_OPEN = 0.44;
    public static final double CLAW_CLOSE = 0.775;
    public static final double CLAW_OPEN = 0.55;
    public static final double WRIST_PICKUP = 0.18;
    public static final double WRIST_PICKUP_BACK = 0.2;
    public static final double WRIST_STANDBY = 0.50;
    public static final double WRIST_ACTIVE = 0.7;
    public static final double ELBOW_PICKUP = 0.29;
    public static final double ELBOW_PICKUP_BACK = 0.77;
    public static final double ELBOW_STANDBY = 0.41;
    public static final double ELBOW_STANDBY_BACK = 0.75;
    public static final double ELBOW_ACTIVE = 0.81;
    public static final double PLANE_STANDBY = 0.1;
    public static final double PLANE_ACTIVE = 0.0;

    public static final double MAX_ACCELERATION_DEVIATION = 10; // higher = less smoothing
    public static final double BASE_DRIVE_SPEED_MODIFIER = 1; // higher = less speed
    public static final double PRECISION_DRIVE_SPEED_MODIFIER = 3.35;

    public static final double MAX_MANUAL_INTAKE_POWER = 0.6;

    public static final double PPR = 375; // gobuilda motor 85203 Series
    public static final double ENCODER_TICKS_PER_TILE = 800; // in encoder ticks

    // -------------------------------------------------------------- JUNCTION PRESETS

    public static final int JUNCTION_LOW = 600;
    public static final int JUNCTION_MID = 1700;
    public static final int JUNCTION_HIGH = 2300;

    // -------------------------------------------------------------- VISION

    public static final boolean USE_VIEWPORT = true;
    public static final boolean USE_DRIVE = true;
    public static final boolean USE_BACK = false;

    public static final double FIELD_LENGTH = 3.58;
    public static final double CAMERA_HEIGHT = 0.313;
    public static final double WALL_TAG_X = 1.005;
    public static final double SMALL_WALL_TAG_X = 0.9;

    public static final double BACKDROP_DEPTH = 1.55;
    public static final double TAG_HEIGHT = 0.12;

    public static final double PIXEL_SPACE = 0.05;

    public static final double ROAD_RUNNER_SCALE = 72 / (FIELD_LENGTH / 2);

    public static final double INTAKE_POWER = 0.5;
    public static final double INTAKE_TIME = 1;

    public static final double TRUSS_WIDTH = 0.9;

    public static final Double[] PATH_Y = {-1.46, -0.83, 0d, 0.83, 1.46};

    public static final double HEADING = Math.PI / 2;

    public static final double BACKDROP_ANGLE = - Math.PI / 2;

    public static final double TAG_WALL_ANGLE = Math.PI / 2;

    public static final boolean STREAMING = false;
}
