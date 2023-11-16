package org.firstinspires.ftc.teamcode.drive.Robotv8;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BACKDROP_PURPLE_PIXEL_VARIANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BACKDROP_YELLOW_PIXEL_VARIANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BLUE_PARKING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BLUE_STARTING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.CENTER_SPIKEMARK_ALIGN_TURN;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.INCHES_PER_TILE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.RED_PARKING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.RED_STARTING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.SPIKEMARK_CENTER_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.SPIKE_TO_BACKBOARD_TRANSIT;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.rr.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.vision.CameraLocalizer;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class FSM_Auto_Fullstack extends LinearOpMode {
    private SampleMecanumDrive drive;
    private CameraLocalizer camLoc;
    private FSM_Auto autoState = FSM_Auto.PLAY;
    private FSM_Outtake outtakeState = FSM_Outtake.IDLE;
    private VisionPropPipeline.Randomization randomization;
    private final ElapsedTime autoTimer = new ElapsedTime();
    public final ElapsedTime armTimer = new ElapsedTime();
    public final ElapsedTime outtakeTimer = new ElapsedTime();
    public static Pose2d START_POSE = new Pose2d();
    public static Pose2d PARKING_POSE = new Pose2d();

    public RobotAlliance alliance;
    public RobotStartingPosition startingPosition;
    public RobotParkingLocation parkingLocation;
    public int allianceIndex;
    public int startingPositionIndex;
    public int parkingLocationIndex;
    public int dir;
    public Point r1;
    public Point r2;
    public Point r3;

    public DcMotorEx backLM = null;
    public DcMotorEx backRM = null;
    public DcMotorEx frontLM = null;
    public DcMotorEx frontRM = null;
    public Servo servoFlap = null;
    public Servo servoClaw = null;
    public Servo servoWrist = null;
    public Servo servoElbowR = null;
    public Servo servoElbowL = null;
    public Servo servoPlane = null;
    public CRServo servoHangR = null;
    public CRServo servoHangL = null;
    public DcMotorEx armR = null;
    public DcMotorEx armL = null;
    public IMU imu = null;
    public DcMotorEx intake = null;

    public int targetOuttakePosition = 0;

    // note: constructor ----------------------------------------------------------------
    public FSM_Auto_Fullstack(RobotAlliance alliance, RobotStartingPosition startPos, RobotParkingLocation parkLoc, Point r1, Point r2, Point r3) {
        this.alliance = alliance;
        this.startingPosition = startPos;
        this.parkingLocation = parkLoc;
        this.dir = alliance == RobotAlliance.RED ? 1 : -1;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;

        allianceIndex = this.alliance == RobotAlliance.RED ? 0 : 1;
        startingPositionIndex = this.startingPosition == RobotStartingPosition.BACKDROP ? 0 : 1;
        parkingLocationIndex = this.parkingLocation == RobotParkingLocation.INNER ? 0 : 1;
        START_POSE = this.alliance == RobotAlliance.RED ? RED_STARTING_POSES[startingPositionIndex] : BLUE_STARTING_POSES[startingPositionIndex];
        PARKING_POSE = this.alliance == RobotAlliance.RED ? RED_PARKING_POSES[parkingLocationIndex] : BLUE_PARKING_POSES[parkingLocationIndex];
    }

    // note: custom behaviour -----------------------------------------------------------
    public void runOpMode() throws InterruptedException {
        InitializeBlock();

        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(2000);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(START_POSE);

        VisionPropDetection(300);

        telemetry.addData("TEAM_PROP_LOCATION", randomization);
        telemetry.addData("SELECTED_ALLIANCE", alliance);
        telemetry.update();

        // note: MAIN START
        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            MainLoop();
        }
    }

    public void MainLoop() {
        switch (autoState) {
            case PLAY:
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                MoveElbow(RobotConstants.ELBOW_STANDBY);

                autoTimer.reset();
                switch (randomization) {
                    case LOCATION_1:
                        drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[0])); AutoWait();
                        break;
                    default:
                    case LOCATION_2:
                        drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[1])); AutoWait();
                        break;
                    case LOCATION_3:
                        drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[2])); AutoWait();
                        break;
                }
                outtakeState = FSM_Outtake.IDLE;
                autoState = FSM_Auto.TURNING_TO_BACKDROP;
                break;
            case TURNING_TO_BACKDROP:
                if (!drive.isBusy()) {
                    RaiseAndPrime(100); // note: no delay here
                    ExecuteRotationAsync(180);
                    autoState = FSM_Auto.MOVING_TO_BACKDROP;
                }
                break;
            case MOVING_TO_BACKDROP:
                if (!drive.isBusy()) {
                    drive.followTrajectoryAsync(CalcKinematics(-SPIKE_TO_BACKBOARD_TRANSIT));
                    autoState = FSM_Auto.DEPOSIT_YELLOW;
                }
                break;
            case DEPOSIT_YELLOW:
                if (!drive.isBusy()) {
                    DropAndReset();

                    autoTimer.reset();
                    CenterRobotAtBackboard();
                    autoState = FSM_Auto.TURNING_TO_SPIKEMARK;
                }
                break;
            case TURNING_TO_SPIKEMARK:
                if (!drive.isBusy()) {
                    outtakeState = FSM_Outtake.ACTIVATED; // activates the grab and deploy sequence on next iteration
                    ExecuteRotationAsync(0);
                }
                break;
            case MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE:
                // note: this bit can be synchronous
                if (!drive.isBusy()) {
                    PrimePurple(); // note: takes over from outtake subsystem and forces prime purple position
                    switch (randomization) {
                        case LOCATION_1:
                            // has to drive backwards
                            drive.followTrajectory(CalcKinematics(-BACKDROP_PURPLE_PIXEL_VARIANCE[0]));
                            ExpelPurple();
                            break;
                        default:
                        case LOCATION_2:
                            drive.followTrajectory(CalcKinematics(-BACKDROP_PURPLE_PIXEL_VARIANCE[1]));
                            drive.turn(Math.toRadians(-CENTER_SPIKEMARK_ALIGN_TURN * dir)); // note: always turns counterclockwise
                            ExpelPurple();
                            drive.turn(Math.toRadians(CENTER_SPIKEMARK_ALIGN_TURN * dir));
                            break;
                        case LOCATION_3:
                            drive.followTrajectory(CalcKinematics(-BACKDROP_PURPLE_PIXEL_VARIANCE[2]));
                            ExpelPurple();
                            break;
                    }
                    outtakeState = FSM_Outtake.OUTTAKE_RESET_HARD;
                    autoState = FSM_Auto.MOVING_TO_PARKING;
                }
                break;
            case MOVING_TO_PARKING:
                if (!drive.isBusy()) {
                    ParkRobotAtBackboard();
                    autoState = FSM_Auto.PARKED;
                }
                break;
            case PARKED:
                break;
        }

        drive.update();

        OuttakeSubsystem();
        StatusTelemetry();
    }

    private void ParkRobotAtBackboard() {
        Trajectory parking = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(PARKING_POSE.vec(),PARKING_POSE.getHeading()).build();

        drive.followTrajectoryAsync(parking);
        ExecuteRotationAsync(alliance == RobotAlliance.RED ? 90 : 270); // note: ensure field centric heading on finish
    }

    private void CenterRobotAtBackboard() {
        Trajectory center = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(SPIKEMARK_CENTER_POSES[allianceIndex].vec(), SPIKEMARK_CENTER_POSES[allianceIndex].getHeading())
                .build();

        drive.followTrajectoryAsync(center);
    }

    public void OuttakeSubsystem() {
        // NOTE: modified from original to suit Auto
        switch (outtakeState) {
            case IDLE:
                break;
            case ACTIVATED:
                servoFlap.setPosition(RobotConstants.FLAP_OPEN);
                outtakeTimer.reset();

                outtakeState = FSM_Outtake.FLAP_OPENING;
                break;
            case FLAP_OPENING:
                // amount of time the servo takes to activate from the previous state
                if (outtakeTimer.milliseconds() >= 700) {
                    servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
                    MoveElbow(RobotConstants.ELBOW_STANDBY);
                    outtakeTimer.reset();

                    outtakeState = FSM_Outtake.WRIST_PICKING;
                }
                break;
            case WRIST_PICKING:
                if (outtakeTimer.milliseconds() >= 400) {
                    MoveElbow(RobotConstants.ELBOW_PICKUP);
                    outtakeTimer.reset();

                    outtakeState = FSM_Outtake.ELBOW_PICKING;
                }
                break;
            case ELBOW_PICKING:
                if (outtakeTimer.milliseconds() >= 200) {
                    servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                    outtakeTimer.reset();

                    outtakeState = FSM_Outtake.CLAW_CLOSING;
                }
                break;
            case CLAW_CLOSING:
                if (outtakeTimer.milliseconds() >= 350) {
                    outtakeTimer.reset();
                    //servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                    servoFlap.setPosition(RobotConstants.FLAP_OPEN);
                    MoveElbow(RobotConstants.ELBOW_STANDBY);
                    outtakeState = FSM_Outtake.GRABBED_AND_READY;
                }
                break;
            case GRABBED_AND_READY:
                break;
            case PRIMED_FOR_DEPOSIT:
                break;
            case CLAW_OPENING:
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                outtakeTimer.reset();
                outtakeState = FSM_Outtake.OUTTAKE_RESET;
                break;
            case OUTTAKE_RESET:
                if (outtakeTimer.milliseconds() >= 600) {
                    servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                    MoveElbow(RobotConstants.ELBOW_STANDBY);

                    Delay(100);
                    servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
                    targetOuttakePosition = 10;
                    UpdateOuttake(true, 0);

                    outtakeState = FSM_Outtake.IDLE;
                }
                break;
            case OUTTAKE_RESET_HARD:
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                MoveElbow(RobotConstants.ELBOW_STANDBY);

                Delay(100);
                servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
                targetOuttakePosition = 10;
                UpdateOuttake(true, 0);

                outtakeState = FSM_Outtake.IDLE;
                break;
        }
    }

    private void InitializeBlock() {
        // note: giant initialization block stored here instead of directly in init.
        backLM = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_LEFT);
        backRM = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_RIGHT);

        frontLM = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT); //frontRM.setDirection(DcMotorSimple.Direction.REVERSE); // weird workaround Stanley put in
        intake = hardwareMap.get(DcMotorEx.class, RobotConstants.INTAKE_MOTOR);

        backLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRM.setDirection(DcMotorSimple.Direction.REVERSE);
        backRM.setDirection(DcMotorSimple.Direction.REVERSE);

        armR = hardwareMap.get(DcMotorEx.class, RobotConstants.ARM_R);
        armL = hardwareMap.get(DcMotorEx.class, RobotConstants.ARM_L);

        armR.setDirection(DcMotorSimple.Direction.REVERSE);
        armL.setDirection(DcMotorSimple.Direction.FORWARD);
        armR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armR.setTargetPosition(0);
        armL.setTargetPosition(0);
        armR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        servoFlap = hardwareMap.get(Servo.class, RobotConstants.SERVO_FLAP);
        servoElbowR = hardwareMap.get(Servo.class, RobotConstants.SERVO_ELBOW_R);
        servoElbowL = hardwareMap.get(Servo.class, RobotConstants.SERVO_ELBOW_L);
        servoClaw = hardwareMap.get(Servo.class, RobotConstants.SERVO_CLAW);
        servoWrist = hardwareMap.get(Servo.class, RobotConstants.SERVO_WRIST);
        servoPlane = hardwareMap.get(Servo.class, RobotConstants.SERVO_PLANE);

        servoHangR = hardwareMap.get(CRServo.class, RobotConstants.SERVO_HANG_R);
        servoHangR.setDirection(DcMotorSimple.Direction.FORWARD);
        servoHangR.setPower(0);

        servoHangL = hardwareMap.get(CRServo.class, RobotConstants.SERVO_HANG_L);
        servoHangL.setDirection(DcMotorSimple.Direction.REVERSE);
        servoHangL.setPower(0);

        // -------------------------------------------------------------- IMU INIT

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.addData("Important Information", "PLACE ROBOT FACING AWAY FROM ALLIANCE BOX!");
        telemetry.update();

        //InitCameras();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu = hardwareMap.get(IMU.class, RobotConstants.HUB_IMU);
        imu.initialize(parameters);
        imu.resetYaw();

        camLoc = new CameraLocalizer(hardwareMap, "Webcam 1", "Webcam 2", START_POSE, telemetry);

        Delay(100);
    }

    private void StatusTelemetry() {
        telemetry.addData("Current State", autoState);
        telemetry.addData("Autonomous Clockspeed", autoTimer.seconds());
        telemetry.addData("Robot X", drive.getPoseEstimate().getX());
        telemetry.addData("Robot Y", drive.getPoseEstimate().getY());
        telemetry.addData("Visual Pose", camLoc.getPoseEstimate());
        telemetry.addData("Robot Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.update();
    }

    // note: low level helper functions -------------------------------------------------------
    private void VisionPropDetection(double timeout) {
        OpenCvWebcam webcam;
        VisionPropPipeline pipeline = new VisionPropPipeline( alliance, r1, r2, r3 );

        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
            }

            @Override
            public void onError(int errorCode) {
                // intentional noop
                telemetry.addLine("error, you stupid idiot");
            }
        });

        autoTimer.reset();
        while (autoTimer.seconds() < timeout && !isStarted()) {
            randomization = pipeline.getRandomization();
            telemetry.addData("TEAM_PROP_LOCATION", randomization);
            telemetry.update();
        }
        webcam.closeCameraDevice();
    }

    private void PrimePurple() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
    }

    private void ExpelPurple() {
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(800);
    }

    public void RaiseAndPrime(int height) {
        intake.setPower(0); // make sure intake is not running

        targetOuttakePosition = height;
        UpdateOuttake(false, 0);

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

        MoveElbow(RobotConstants.ELBOW_ACTIVE);

        outtakeState = FSM_Outtake.PRIMED_FOR_DEPOSIT;
        Delay(50); // debounce

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
    }

    public void DropAndReset() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(800); // wait for claw to open

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY);

        Delay(350); // elbow should come down after the slide is near done

        targetOuttakePosition = 10;
        UpdateOuttake(true, 0);
    }

    private void AutoWait() {
        Delay(400);
    }

    public Trajectory CalcKinematics(double tiles) {
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(tiles * INCHES_PER_TILE)
                .build();
    }

    public void ExecuteRotationAsync(double heading) {
        double diff = heading - Math.toDegrees(drive.getPoseEstimate().getHeading());
        drive.turnAsync(diff > 180 ? Math.toRadians(-(360 - diff)) : Math.toRadians(diff));
    }

    public void UpdateOuttake(boolean reset, double delay) { // test new function
        if (reset) {
            Delay(delay);
            armR.setTargetPosition(10);
            armL.setTargetPosition(10);
            targetOuttakePosition = 10;
            armTimer.reset();
            armR.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED);
            armL.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED);
            /*while (armM.getCurrentPosition() >= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

                if (armM.getCurrentPosition() <= 50 || armRuntime.seconds() >= ARM_RESET_TIMEOUT) {
                    break;
                }
            }*/

            if ((armL.getCurrentPosition() <= 20 || armR.getCurrentPosition() <= 20) || armTimer.seconds() >= RobotConstants.ARM_RESET_TIMEOUT) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }

            telemetry.update();
        }

        else {
            Delay(delay);
            armR.setTargetPosition(targetOuttakePosition);
            armL.setTargetPosition(targetOuttakePosition);
            armTimer.reset();
            armR.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED);
            armL.setVelocity(RobotConstants.MAX_OUTTAKE_SPEED); // velocity used to be 1800, could be faster
        }
    }

    public void MoveElbow(double targetPos) {
        servoElbowR.setPosition(targetPos);
        servoElbowL.setPosition(1 - targetPos); // Set to the opposite position
    }

    public void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }
}