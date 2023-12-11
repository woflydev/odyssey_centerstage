package org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo;

import com.acmerobotics.dashboard.config.Config;

@Config
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
    public static final String SERVO_WHATEVER_THE_FUCK_THAT_THING_IS = "presser";
    public static final String SERVO_HANG_R = "hangR";
    public static final String SERVO_HANG_L = "hangL";
    public static final String HUB_IMU = "imu";
    public static final String FRONT_CAMERA = "Webcam 1";
    public static final String BACK_CAMERA = "Webcam 2";

    public static final int PERMISSION_TIMEOUT = 5000;

    public static final int MAX_OUTTAKE_SPEED = 2800;
    public static final int MAX_OUTTAKE_HEIGHT = 3200;
    public static final int MIN_OUTTAKE_HEIGHT = 0;
    public static final int ARM_ADJUSTMENT_INCREMENT = 40; // used to be 50
    public static final int ARM_BOOST_MODIFIER = 1;
    public static final int ARM_RESET_TIMEOUT = 3;

    public static final double FLAP_CLOSE = 0.42;
    public static final double FLAP_OPEN = 1.00;
    public static final double CLAW_CLOSE = 0.865;
    public static final double CLAW_OPEN = 0.685;
    public static final double WRIST_PICKUP = 0.133;
    public static final double WRIST_STANDBY_BACK = 0.69;
    public static final double WRIST_STANDBY = 0.41; // 0.615
    public static final double WRIST_ACTIVE = 0.63;
    public static final double ELBOW_PICKUP = 0.050;
    public static final double ELBOW_STANDBY = 0.143;
    public static final double ELBOW_STANDBY_BACK = 0.6205;
    public static final double ELBOW_ACTIVE = 0.49;
    public static final double PLANE_STANDBY = 0.19;
    public static final double PLANE_ACTIVE = 0;
    public static final double WHATEVER_THE_FUCK_THAT_THING_IS_OFF = 0.6;
    public static final double WHATEVER_THE_FUCK_THAT_THING_IS_ON = 0.35;

    // hanging
    public static final double ELBOW_HANG_STABILIZATION = 0.343;
    public static final double WRIST_HANG_STABILIZATION = 0.19;


    public static final double MAX_ACCELERATION_DEVIATION = 10; // higher = less smoothing
    public static final double BASE_DRIVE_SPEED_MODIFIER = 1; // higher = less speed
    public static final double PRECISION_DRIVE_SPEED_MODIFIER = 3.35;

    public static final double MAX_MANUAL_INTAKE_POWER = 0.65; // note: old 0.55

    public static final double PPR = 375; // gobuilda motor 85203 Series
    public static final double ENCODER_TICKS_PER_TILE = 640; // in encoder ticks


    // -------------------------------------------------------------- RR
    public static final boolean STARTUP_USE_LOCALIZER = false;
    public static final double RR_LATERAL_MULTI = 1.38460328144; // note: old - -1.31460328144
    public static final double RR_HEADING_PID_kP = 4;
    public static final double RR_TRANSLATIONAL_PID_kP = 15; // note: old - 30

    // -------------------------------------------------------------- JUNCTION PRESETS

    public static final int JUNCTION_LOW = 600;
    public static final int JUNCTION_MID = 1200;
    public static final int JUNCTION_HIGH = 2300;

    // -------------------------------------------------------------- VISION

    public static final boolean USE_CAMERA_STREAM = true;

    public static final boolean USE_LIVE_VIEW = true;
    public static final boolean USE_DRIVE = true;
    public static final boolean USE_BACK = true;

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

    public static final double ELBOW_DROPOFF = 0.8;

    public static final double INTAKE_OUTPUT = 0.5;
    public static final double INTAKE_OUTPUT_TIME = 1000;

    public static final int INITIAL_HEIGHT = 15;
    public static final double INITIAL_FORWARD = 0.3;
}
