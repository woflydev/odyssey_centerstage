package org.firstinspires.ftc.teamcode.drive.Robotv8;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Auto_State.*;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.rr.OdysseyMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.vision2.VisualLoc;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class RR_AutoBase2 extends FSM_TeleOp_Fullstack {
    private OdysseyMecanumDrive drive;
    private VisualLoc localizer;
    private VisionPropPipeline.Randomization randomization;
    private final ElapsedTime autoTimer = new ElapsedTime();
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

    // note: custom behaviour -----------------------------------------------------------
    public RR_AutoBase2(RobotAlliance alliance, RobotStartingPosition startPos, RobotParkingLocation parkLoc, Point r1, Point r2, Point r3) {
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

    public void MainInit() {
        localizer = new VisualLoc(hardwareMap, "Webcam 1", START_POSE, telemetry, drive, false);
        drive.setLocalizer(localizer);
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(2000);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);

        drive = new OdysseyMecanumDrive(hardwareMap, telemetry);
        drive.setPoseEstimate(START_POSE);

        VisionPropDetection(); // note: assert where the prop is

        telemetry.addData("TEAM_PROP_LOCATION", randomization);
        telemetry.addData("SELECTED_ALLIANCE", alliance);
        telemetry.update();
    }

    public void MainStart() {
        HandleYellowPixel(); AutoWait();
        HandlePurplePixel(); AutoWait();
        ParkRobotAtBackboard(); AutoWait();
    }

    private void VisionPropDetection() {
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
        while (autoTimer.seconds() < 3) {
            randomization = pipeline.getRandomization();
            telemetry.addData("TEAM_PROP_LOCATION", randomization);
            telemetry.update();
        }
        webcam.closeCameraDevice();
    }

    private void HandleYellowPixel() {
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);

        switch (randomization) {
            case LOCATION_1:
                drive.followTrajectory(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[0])); AutoWait();
                break;
            case LOCATION_2:
                drive.followTrajectory(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[1])); AutoWait();
                break;
            case LOCATION_3:
                drive.followTrajectory(CalcKinematics(BACKDROP_YELLOW_PIXEL_VARIANCE[2])); AutoWait();
                break;
        }

        RaiseAndPrime(100);
        ExecuteRotation(180); // note: robot has to be backwards to deposit
        drive.followTrajectory(CalcKinematics(-DEPOSIT_YELLOW_TO_BACKDROP_TRANSIT_RED)); AutoWait();
        DropAndReset();
        CenterRobotAtBackboard();
    }

    private void HandlePurplePixel() {
        GrabAndReady();
        ExecuteRotation(0);
        PrimePurple();
        Delay(500);

        switch (randomization) {
            case LOCATION_1:
                drive.followTrajectory(CalcKinematics(-BACKDROP_RED_PURPLE_PIXEL_VARIANCE[0])); // note: has to drive backwards
                ExpelPurple();
                break;
            case LOCATION_2:
                drive.followTrajectory(CalcKinematics(-BACKDROP_RED_PURPLE_PIXEL_VARIANCE[1]));
                drive.turn(Math.toRadians(-BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG * dir)); // note: always counterclockwise
                ExpelPurple();
                drive.turn(Math.toRadians(BACKDROP_CENTER_SPIKEMARK_ALIGN_TURN_DEG * dir));
                break;
            case LOCATION_3:
                drive.followTrajectory(CalcKinematics(-BACKDROP_RED_PURPLE_PIXEL_VARIANCE[2]));
                ExpelPurple();
                break;
        }
    }

    private void ParkRobotAtBackboard() {
        Trajectory parking = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(PARKING_POSE.vec(),PARKING_POSE.getHeading()).build();

        drive.followTrajectory(parking);
        ExecuteRotation(alliance == FSM_Auto_State.RobotAlliance.RED ? 90 : 270); // note: ensure field centric heading on finish
    }

    public void CenterRobotAtBackboard() {
        Trajectory center = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(SPIKEMARK_CENTER_POSES[allianceIndex].vec(), SPIKEMARK_CENTER_POSES[allianceIndex].getHeading())
                .build();

        drive.followTrajectory(center);
    }

    // note: helper functions -----------------------------------------------------------
    private void PrimePurple() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
    }

    private void ExpelPurple() {
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
        Delay(800);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(1000);
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
    }

    public void GrabAndReady() {
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        Delay(500);

        servoFlap.setPosition(RobotConstants.FLAP_OPEN);
        Delay(700);

        // transfer stage sequence
        servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_STANDBY); // moves it up a little to avoid tubes
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_PICKUP);

        Delay(200);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        Delay(500);

        // primes the elbow
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(100);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
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

    public void ExecuteRotation(double heading) {
        double diff = heading - Math.toDegrees(drive.getPoseEstimate().getHeading());
        drive.turn(diff > 180 ? Math.toRadians(-(360 - diff)) : Math.toRadians(diff));
    }
}
