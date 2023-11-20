package org.firstinspires.ftc.teamcode.drive.Robotv8;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.AUDIENCE_PURPLE_PIXEL_VARIANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BACKDROP_CENTER_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BACKDROP_DEPOSIT_PUSHBACK_AMOUNT;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BACKDROP_PURPLE_PIXEL_VARIANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BACKDROP_YELLOW_PIXEL_VARIANCE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BLUE_PARKING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BLUE_STARTING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BLUE_YELLOW_PIXEL_BACKUP_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.CAUTION_SPEED;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.CYCLING_STACK_INNER_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.INCHES_PER_TILE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.RED_PARKING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.RED_STARTING_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.RED_YELLOW_PIXEL_BACKDROP_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.SPIKEMARK_CENTER_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.SPIKEMARK_TRANSIT_CENTER_POSES;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.SPIKE_TO_BACKBOARD_TRANSIT;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.STAGE_DOOR_POSES;

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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.*;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.rr.OdysseyMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.vision2.MecanumCameraLocalizer;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

/*
IMPORTANT NOTE:
Audience and Backdrop sequences are not the same.
note that - Backdrop: Yellow (forward) -> Purple (backwards) -> Cycle/Park
note that - Audience: Purple (forward) -> Yellow (backwards) -> Cycle/Park
Thus, resulting state-machines are sequenced differently and may exhibit slightly different behaviours.
This means that different robot setups is required on startup and init. Beware of these variations!
 */

@Config
public class FSM_Auto_Fullstack extends LinearOpMode {
    private OdysseyMecanumDrive drive;
    private MecanumCameraLocalizer localizer;
    private FSM_RootAutoState autoState = FSM_RootAutoState.PLAY;
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
    public RobotTaskFinishBehaviour taskFinishBehaviour;
    public int allianceIndex;
    public int startingPositionIndex;
    public int parkingLocationIndex;
    public int dir;
    public double[] workingBackdropPurpleVariance;
    public double[] workingAudiencePurpleAlign;
    public Pose2d[] workingAudienceYellowBackdropAlign;
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
    public FSM_Auto_Fullstack(RobotAlliance alliance, RobotStartingPosition startPos, RobotParkingLocation parkLoc, RobotTaskFinishBehaviour finishBehaviour, Point r1, Point r2, Point r3) {
        this.alliance = alliance;
        this.startingPosition = startPos;
        this.parkingLocation = parkLoc;
        this.taskFinishBehaviour = finishBehaviour;
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

        drive = new OdysseyMecanumDrive(hardwareMap, START_POSE, telemetry, false);
        drive.setPoseEstimate(START_POSE);

        localizer = new MecanumCameraLocalizer(hardwareMap, "Webcam 1", START_POSE, telemetry, drive, false);
        drive.setLocalizer(localizer);

        VisionPropDetection(300);

        telemetry.addData("TEAM_PROP_LOCATION", randomization);
        telemetry.addData("SELECTED_ALLIANCE", alliance);
        telemetry.update();

        // note: MAIN START
        waitForStart();

        workingBackdropPurpleVariance = SortPurpleVariance(); // note: called after, so that randomization is confirmed
        workingAudiencePurpleAlign = SortPurpleAlignVariance();
        workingAudienceYellowBackdropAlign = SortYellowBackdropAlign();

        while (!isStopRequested() && opModeIsActive()) {
            MainLoop();
        }
    }

    public void MainLoop() {
        /*switch (autoState) {
            case PLAY:
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                MoveElbow(RobotConstants.ELBOW_STANDBY);

                autoTimer.reset();
                switch (randomization) {
                    case LOCATION_1:
                        drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[0], DriveConstants.MAX_VEL)); AutoWait();
                        break;
                    default:
                    case LOCATION_2:
                        drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[1], DriveConstants.MAX_VEL)); AutoWait();
                        break;
                    case LOCATION_3:
                        drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[2], DriveConstants.MAX_VEL)); AutoWait();
                        break;
                }
                outtakeState = FSM_Outtake.IDLE;
                autoState = FSM_RootAutoState.TURNING_TO_BACKDROP;
                break;
            case TURNING_TO_BACKDROP:
                if (!drive.isBusy()) {
                    RaiseAndPrime(300); // note: no delay here
                    ExecuteRotation(180,  true);
                    autoState = FSM_RootAutoState.MOVING_TO_BACKDROP;
                }
                break;
            case MOVING_TO_BACKDROP:
                if (!drive.isBusy()) {
                    drive.followTrajectoryAsync(CalcKinematics(-SPIKE_TO_BACKBOARD_TRANSIT, DriveConstants.MAX_VEL));
                    autoState = FSM_RootAutoState.DEPOSIT_YELLOW;
                }
                break;
            case DEPOSIT_YELLOW:
                if (!drive.isBusy()) {
                    servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                    drive.followTrajectory(CalcKinematics(0.1, DriveConstants.MAX_VEL));
                    DropAndReset();

                    autoTimer.reset();
                    CenterRobotForSpikemark();
                    autoState = FSM_RootAutoState.TURNING_TO_SPIKEMARK;
                }
                break;
            case TURNING_TO_SPIKEMARK:
                if (!drive.isBusy()) {
                    outtakeState = FSM_Outtake.ACTIVATED; // activates the grab and deploy sequence on next iteration
                    //ExecuteRotation(0, true);

                    autoState = FSM_RootAutoState.MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE;
                }
                break;
            case MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE:
                // note: this bit can be synchronous
                if (!drive.isBusy() && outtakeState == FSM_Outtake.GRABBED_AND_READY) {
                    PrimePurple(); // note: takes over from outtake subsystem and forces prime purple position
                    switch (randomization) {
                        case LOCATION_1:
                            // has to drive backwards
                            drive.followTrajectory(CalcKinematics(-workingBackdropPurpleVariance[0], DriveConstants.MAX_VEL));
                            ExpelPurple();
                            break;
                        default:
                        case LOCATION_2:
                            drive.followTrajectory(CalcKinematics(-workingBackdropPurpleVariance[1], DriveConstants.MAX_VEL));
                            drive.turn(Math.toRadians(-BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG * dir)); // note: always turns counterclockwise
                            ExpelPurple();
                            drive.turn(Math.toRadians(BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG * dir));
                            break;
                        case LOCATION_3:
                            Delay(400); // note: purple has to prime first to push prop
                            drive.followTrajectory(CalcKinematics(-workingBackdropPurpleVariance[2], DriveConstants.MAX_VEL));
                            ExpelPurple();
                            break;
                    }
                    outtakeState = FSM_Outtake.OUTTAKE_RESET_HARD;

                    if (taskFinishBehaviour == RobotTaskFinishBehaviour.DO_NOT_CYCLE) {
                        autoState = FSM_RootAutoState.MOVING_TO_PARKING;
                    } else {
                        autoState = FSM_RootAutoState.MOVING_TO_CYCLE;
                    }
                }
                break;
            case MOVING_TO_CYCLE:
                if (!drive.isBusy()) {
                    TrajectorySequence cycleTrajectory = alliance == RobotAlliance.RED ?
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    // note: this ensures robot doesn't crash into truss and goes through stage door on appropriate side
                                    .lineToSplineHeading(STAGE_DOOR_POSES[0])
                                    .waitSeconds(0.05)
                                    .splineTo(CYCLING_STACK_INNER_POSES[0].vec(), CYCLING_STACK_INNER_POSES[0].getHeading())
                                    .build()
                            :
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(STAGE_DOOR_POSES[1])
                                    .waitSeconds(0.05)
                                    .splineTo(CYCLING_STACK_INNER_POSES[1].vec(), CYCLING_STACK_INNER_POSES[1].getHeading())
                                    .build()
                            ;

                    drive.followTrajectorySequence(cycleTrajectory); // note: blocking
                    ExecuteRotation(180, true);
                    autoState = FSM_RootAutoState.INTAKE_PIXELS_FROM_STACK;
                }
                break;
            case INTAKE_PIXELS_FROM_STACK:
                if (!drive.isBusy()) {
                    intake.setPower(0.55);
                    drive.followTrajectory(CalcKinematics(0.3, CAUTION_SPEED));
                    ExecuteRotation(180, false);
                    Delay(2000);
                    intake.setPower(0);

                    TrajectorySequence toBackdropTrajectory = alliance == RobotAlliance.RED ?
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    // note: this ensures robot doesn't crash into truss and goes through stage door on appropriate side
                                    .lineToConstantHeading(STAGE_DOOR_POSES[0].vec())
                                    .waitSeconds(0.05)
                                    .splineToConstantHeading(BACKDROP_CENTER_POSES[0].vec(), BACKDROP_CENTER_POSES[0].getHeading())
                                    .build()
                            :
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToConstantHeading(STAGE_DOOR_POSES[1].vec())
                                    .waitSeconds(0.05)
                                    .splineToConstantHeading(BACKDROP_CENTER_POSES[1].vec(), BACKDROP_CENTER_POSES[1].getHeading())
                                    .build()
                            ;

                    outtakeState = FSM_Outtake.ACTIVATED;
                    Delay(500); // note: allow for some time for flap to open and claw to grab

                    drive.followTrajectorySequenceAsync(toBackdropTrajectory);
                    autoState = FSM_RootAutoState.MOVING_BACK_FROM_CYCLE;
                }
                break;
            case MOVING_BACK_FROM_CYCLE:
                // note: empty for now
                autoState = FSM_RootAutoState.DEPOSIT_WHITE;
                break;
            case DEPOSIT_WHITE:
                // note: if it has stopped at backboard AND is grabbed and ready
                if (!drive.isBusy() && outtakeState == FSM_Outtake.GRABBED_AND_READY) {
                    ExecuteRotation(180, false);
                    RaiseAndPrime(600);
                    Delay(300);
                    servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                    drive.followTrajectory(CalcKinematics(0.1, DriveConstants.MAX_VEL));
                    DropAndReset();
                    outtakeState = FSM_Outtake.IDLE;
                    autoState = FSM_RootAutoState.MOVING_TO_PARKING;
                }
            case MOVING_TO_PARKING:
                if (!drive.isBusy()) {
                    ParkRobotAtBackboard();
                    autoState = FSM_RootAutoState.PARKED;
                }
                break;
            case PARKED:
                // other custom logic if parked
                break;
        }*/

        HandleYellow();
        HandlePurple();
        HandleCycle();
        HandleFinish();

        drive.update();

        OuttakeSubsystem();
        StatusTelemetry();
    }

    // note: high level behaviors -------------------------------------------------------------
    private void HandleYellow() {
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            switch (autoState) {
                case PLAY:
                    EnsureAttachmentNormalization();

                    autoTimer.reset();
                    switch (randomization) {
                        case LOCATION_1:
                            drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[0], DriveConstants.MAX_VEL)); AutoWait();
                            break;
                        default:
                        case LOCATION_2:
                            drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[1], DriveConstants.MAX_VEL)); AutoWait();
                            break;
                        case LOCATION_3:
                            drive.followTrajectoryAsync(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[2], DriveConstants.MAX_VEL)); AutoWait();
                            break;
                    }
                    outtakeState = FSM_Outtake.IDLE;
                    autoState = FSM_RootAutoState.TURNING_TO_BACKDROP;
                    break;
                case TURNING_TO_BACKDROP:
                    if (!drive.isBusy()) {
                        RaiseAndPrime(150); // note: no delay here
                        ExecuteRotation(180,  true);
                        autoState = FSM_RootAutoState.MOVING_TO_BACKDROP;
                    }
                    break;
                case MOVING_TO_BACKDROP:
                    if (!drive.isBusy()) {
                        drive.followTrajectoryAsync(CalcKinematics(-SPIKE_TO_BACKBOARD_TRANSIT, DriveConstants.MAX_VEL));
                        autoState = FSM_RootAutoState.DEPOSIT_YELLOW;
                    }
                    break;
                case DEPOSIT_YELLOW:
                    if (!drive.isBusy()) {
                        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                        drive.followTrajectory(CalcKinematics(BACKDROP_DEPOSIT_PUSHBACK_AMOUNT, DriveConstants.MAX_VEL));
                        Delay(100);
                        DropAndReset();

                        autoTimer.reset();
                        autoState = FSM_RootAutoState.TURNING_TO_SPIKEMARK;
                    }
                    break;
            }
        } else {
            switch (autoState) {
                case MOVING_TO_BACKDROP:
                    if (!drive.isBusy()) {
                        Trajectory centerForTransit = alliance == RobotAlliance.RED ?
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(SPIKEMARK_TRANSIT_CENTER_POSES[0])
                                        .build()
                                :
                                drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(SPIKEMARK_TRANSIT_CENTER_POSES[1])
                                        .build();

                        // note: this can be synchronous
                        drive.followTrajectory(centerForTransit);
                        drive.followTrajectory(CalcKinematics(-3, DriveConstants.MAX_VEL));

                        RaiseAndPrime(300);

                        switch (randomization) {
                            case LOCATION_1:
                                drive.followTrajectory(ConstructYellowBackdropAlign(workingAudienceYellowBackdropAlign[0]));
                                break;
                            default:
                            case LOCATION_2:
                                drive.followTrajectory(ConstructYellowBackdropAlign(workingAudienceYellowBackdropAlign[1]));
                                break;
                            case LOCATION_3:
                                drive.followTrajectory(ConstructYellowBackdropAlign(workingAudienceYellowBackdropAlign[2]));
                                break;
                        }

                        autoState = FSM_RootAutoState.DEPOSIT_YELLOW;
                    }
                case DEPOSIT_YELLOW:
                    if (!drive.isBusy()) {
                        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                        drive.followTrajectory(CalcKinematics(BACKDROP_DEPOSIT_PUSHBACK_AMOUNT, DriveConstants.MAX_VEL));
                        DropAndReset();

                        outtakeState = FSM_Outtake.IDLE;
                        autoState = FSM_RootAutoState.MOVING_TO_PARKING;
                    }
            }
        }
    }

    private void HandlePurple() {
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            switch (autoState) {
                case TURNING_TO_SPIKEMARK:
                    if (!drive.isBusy()) {
                        CenterRobotForSpikemark();
                        outtakeState = FSM_Outtake.ACTIVATED; // activates the grab and deploy sequence on next iteration
                        //ExecuteRotation(0, true); //note: testing linetolinearheading

                        autoState = FSM_RootAutoState.MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE;
                    }
                    break;
                case MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE:
                    // note: this bit can be synchronous
                    if (!drive.isBusy() && outtakeState == FSM_Outtake.GRABBED_AND_READY) {
                        PrimePurple(); // note: takes over from outtake subsystem and forces prime purple position
                        switch (randomization) {
                            case LOCATION_1:
                                // has to drive backwards
                                drive.followTrajectory(CalcKinematics(-workingBackdropPurpleVariance[0], DriveConstants.MAX_VEL));
                                ExpelPurple();
                                break;
                            default:
                            case LOCATION_2:
                                drive.followTrajectory(CalcKinematics(-workingBackdropPurpleVariance[1], DriveConstants.MAX_VEL));
                                drive.turn(Math.toRadians(-BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG * dir)); // note: always turns counterclockwise
                                ExpelPurple();
                                drive.turn(Math.toRadians(BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG * dir));
                                break;
                            case LOCATION_3:
                                Delay(400); // note: purple has to prime first to push prop
                                drive.followTrajectory(CalcKinematics(-workingBackdropPurpleVariance[2], DriveConstants.MAX_VEL));
                                ExpelPurple();
                                break;
                        }
                        outtakeState = FSM_Outtake.OUTTAKE_RESET_HARD;
                        OuttakeSubsystem(); // force call, since next is blocking

                        if (taskFinishBehaviour == RobotTaskFinishBehaviour.DO_NOT_CYCLE) {
                            autoState = FSM_RootAutoState.MOVING_TO_PARKING;
                        } else {
                            autoState = FSM_RootAutoState.MOVING_TO_CYCLE;
                        }
                    }
                    break;
            }
        } else {
            switch (autoState) {
                // important note: this is the autonomous entrypoint for audience init
                case PLAY:
                    EnsureAttachmentNormalization();

                    autoTimer.reset();
                    switch (randomization) {
                        case LOCATION_1:
                            drive.followTrajectoryAsync(CalcKinematics(AUDIENCE_PURPLE_PIXEL_VARIANCE[0], DriveConstants.MAX_VEL)); AutoWait();
                            break;
                        default:
                        case LOCATION_2:
                            drive.followTrajectoryAsync(CalcKinematics(AUDIENCE_PURPLE_PIXEL_VARIANCE[1], DriveConstants.MAX_VEL)); AutoWait();
                            break;
                        case LOCATION_3:
                            drive.followTrajectoryAsync(CalcKinematics(AUDIENCE_PURPLE_PIXEL_VARIANCE[2], DriveConstants.MAX_VEL)); AutoWait();
                            break;
                    }

                    outtakeState = FSM_Outtake.IDLE;
                    autoState = FSM_RootAutoState.MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE;
                case MOVING_TO_SPIKEMARK_AND_DEPOSIT_PURPLE:
                    if (!drive.isBusy()) {
                        PrimePurple();

                        switch (randomization) {
                            case LOCATION_1:
                                drive.turn(-Math.toRadians(workingAudiencePurpleAlign[0]));
                                ExpelPurple();
                                drive.turn(Math.toRadians(workingAudiencePurpleAlign[0]));
                                break;
                            default:
                            case LOCATION_2:
                                ExpelPurple();
                                break;
                            case LOCATION_3:
                                drive.turn(Math.toRadians(workingAudiencePurpleAlign[2]));
                                ExpelPurple();
                                drive.turn(-Math.toRadians(workingAudiencePurpleAlign[2]));
                                break;
                        }

                        outtakeState = FSM_Outtake.ACTIVATED;
                        DropAndReset();
                        autoState = FSM_RootAutoState.MOVING_TO_BACKDROP;
                    }
            }
        }
    }

    private void HandleCycle() {
        // note: we assume we only do cycling if starting at backdrop. however, this is subject to change.
        switch (autoState) {
            case MOVING_TO_CYCLE:
                if (!drive.isBusy()) {
                    TrajectorySequence cycleTrajectory = alliance == RobotAlliance.RED ?
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    // note: this ensures robot doesn't crash into truss and goes through stage door on appropriate side
                                    .lineToSplineHeading(STAGE_DOOR_POSES[0])
                                    .waitSeconds(0.05)
                                    .splineTo(CYCLING_STACK_INNER_POSES[0].vec(), CYCLING_STACK_INNER_POSES[0].getHeading())
                                    .build()
                            :
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(STAGE_DOOR_POSES[1])
                                    .waitSeconds(0.05)
                                    .splineTo(CYCLING_STACK_INNER_POSES[1].vec(), CYCLING_STACK_INNER_POSES[1].getHeading())
                                    .build()
                            ;

                    drive.followTrajectorySequence(cycleTrajectory); // note: blocking
                    ExecuteRotation(180, true);
                    autoState = FSM_RootAutoState.INTAKE_PIXELS_FROM_STACK;
                }
                break;
            case INTAKE_PIXELS_FROM_STACK:
                if (!drive.isBusy()) {
                    intake.setPower(0.65);
                    drive.followTrajectory(CalcKinematics(0.135, CAUTION_SPEED));
                    ExecuteRotation(180, false);
                    Delay(1000);
                    intake.setPower(0);

                    TrajectorySequence toBackdropTrajectory = alliance == RobotAlliance.RED ?
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    // note: this ensures robot doesn't crash into truss and goes through stage door on appropriate side
                                    .lineToConstantHeading(STAGE_DOOR_POSES[0].vec())
                                    .waitSeconds(0.05)
                                    .lineToConstantHeading(BACKDROP_CENTER_POSES[0].vec())
                                    .build()
                            :
                            drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                    .lineToConstantHeading(STAGE_DOOR_POSES[1].vec())
                                    .waitSeconds(0.05)
                                    .lineToConstantHeading(BACKDROP_CENTER_POSES[1].vec())
                                    .build()
                            ;

                    outtakeState = FSM_Outtake.ACTIVATED;
                    Delay(500); // note: allow for some time for flap to open and claw to grab

                    drive.followTrajectorySequenceAsync(toBackdropTrajectory);
                    autoState = FSM_RootAutoState.MOVING_BACK_FROM_CYCLE;
                }
                break;
            case MOVING_BACK_FROM_CYCLE:
                // note: empty for now
                autoState = FSM_RootAutoState.DEPOSIT_WHITE;
                break;
            case DEPOSIT_WHITE:
                // note: if it has stopped at backboard AND is grabbed and ready
                if (!drive.isBusy() && outtakeState == FSM_Outtake.GRABBED_AND_READY) {
                    ExecuteRotation(180, false);
                    RaiseAndPrime(600);
                    Delay(300);
                    drive.followTrajectory(CalcKinematics(-0.25, DriveConstants.MAX_VEL));
                    Delay(400);
                    servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                    drive.followTrajectory(CalcKinematics(BACKDROP_DEPOSIT_PUSHBACK_AMOUNT, DriveConstants.MAX_VEL));
                    DropAndReset();
                    outtakeState = FSM_Outtake.IDLE;
                    autoState = FSM_RootAutoState.MOVING_TO_PARKING;
                }
        }
    }

    private void HandleFinish() {
        switch (autoState) {
            case MOVING_TO_PARKING:
                if (!drive.isBusy()) {
                    ParkRobotAtBackboard();
                    autoState = FSM_RootAutoState.PARKED;
                }
                break;
            case PARKED:
                // other custom logic if parked
                break;
        }
    }

    // note: mid level helper functions -------------------------------------------------------
    private void ParkRobotAtBackboard() {
        Trajectory parking = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .splineToLinearHeading(PARKING_POSE, PARKING_POSE.getHeading()).build();

        drive.followTrajectory(parking);
        ExecuteRotation(alliance == RobotAlliance.RED ? 90 : 270, true); // note: ensure field centric heading on finish
    }

    private void CenterRobotForSpikemark() {
        Trajectory center = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .lineToSplineHeading(SPIKEMARK_CENTER_POSES[allianceIndex])
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
                if (outtakeTimer.milliseconds() >= 450) {
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
        frontRM = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT);
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


        Delay(100);
    }

    private void StatusTelemetry() {
        telemetry.addData("Current State", autoState);
        telemetry.addData("Autonomous Clockspeed", autoTimer.seconds());
        telemetry.addData("Robot X", drive.getPoseEstimate().getX());
        telemetry.addData("Robot Y", drive.getPoseEstimate().getY());
        telemetry.addData("Robot Heading", Math.toDegrees(drive.getPoseEstimate().getHeading()));
        telemetry.addData("Target Location",
                randomization == VisionPropPipeline.Randomization.LOCATION_1 ? "LEFT (LOC_1)" :
                randomization == VisionPropPipeline.Randomization.LOCATION_2 ? "MIDDLE (LOC_2)" :
                randomization == VisionPropPipeline.Randomization.LOCATION_3 ? "RIGHT (LOC_3)" : "NONE"
        );
        telemetry.addLine("---------");
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Starting Position", startingPosition);
        telemetry.addData("Task Completion Behavior", taskFinishBehaviour);
        telemetry.addData("Parking Location", parkingLocation);
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

    private void EnsureAttachmentNormalization() {
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY);
    }

    private void PrimePurple() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
    }

    private void ExpelPurple() {
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(400);
    }

    public void RaiseAndPrime(int height) {
        intake.setPower(0); // make sure intake is not running

        targetOuttakePosition = height;
        UpdateOuttake(false, 0);

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

        MoveElbow(RobotConstants.ELBOW_ACTIVE);
        Delay(50); // debounce

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
    }

    public void DropAndReset() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(700); // wait for claw to open

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

    private double[] SortPurpleVariance() {
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            return alliance == RobotAlliance.RED ? new double[] {
                    BACKDROP_PURPLE_PIXEL_VARIANCE[0],
                    BACKDROP_PURPLE_PIXEL_VARIANCE[1],
                    BACKDROP_PURPLE_PIXEL_VARIANCE[2]
            } : new double[] {
                    BACKDROP_PURPLE_PIXEL_VARIANCE[2],
                    BACKDROP_PURPLE_PIXEL_VARIANCE[1],
                    BACKDROP_PURPLE_PIXEL_VARIANCE[0]
            };
        } else {
            return alliance == RobotAlliance.RED ? new double[] {
                    AUDIENCE_PURPLE_PIXEL_VARIANCE[0],
                    AUDIENCE_PURPLE_PIXEL_VARIANCE[1],
                    AUDIENCE_PURPLE_PIXEL_VARIANCE[2]
            } : new double[] {
                    AUDIENCE_PURPLE_PIXEL_VARIANCE[2],
                    AUDIENCE_PURPLE_PIXEL_VARIANCE[1],
                    AUDIENCE_PURPLE_PIXEL_VARIANCE[0]
            };
        }
    }

    private double[] SortPurpleAlignVariance() {
        return alliance == RobotAlliance.RED ? new double[] {
                AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE[0],
                AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE[1],
                AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE[2],
        } : new double[] {
                AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE[2],
                AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE[1],
                AUDIENCE_PURPLE_PIXEL_ALIGN_VARIANCE[0],
        };
    }

    private Pose2d[] SortYellowBackdropAlign() {
        return alliance == RobotAlliance.RED ?
                RED_YELLOW_PIXEL_BACKDROP_POSES : BLUE_YELLOW_PIXEL_BACKUP_POSES;
    }

    private Trajectory ConstructYellowBackdropAlign(Pose2d target) {
        return
                drive.trajectoryBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(target.vec())
                        .build();
    }

    public Trajectory CalcKinematics(double tiles, double speed) {
        return drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(tiles * INCHES_PER_TILE,
                        OdysseyMecanumDrive.getVelocityConstraint(speed, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        OdysseyMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    public void ExecuteRotation(double heading, boolean async) {
        double diff = heading - Math.toDegrees(drive.getPoseEstimate().getHeading());
        double amt = diff > 180 ? Math.toRadians(-(360 - diff)) : Math.toRadians(diff);
        if (async) {
            drive.turnAsync(amt);
        } else {
            drive.turn(amt);
        }

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
