package org.firstinspires.ftc.teamcode.drive.Robotv8;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAutoConstants.*;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class RR_DRIVE_ONLY_AutoBase extends FSM_Fullstack {
    private SampleMecanumDrive drive;
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
    public RR_DRIVE_ONLY_AutoBase(RobotAlliance alliance, RobotStartingPosition startPos, RobotParkingLocation parkLoc, Point r1, Point r2, Point r3) {
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
        drive = new SampleMecanumDrive(hardwareMap);
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

        ExecuteRotation(180); // note: robot has to be backwards to deposit
        drive.followTrajectory(CalcKinematics(-SPIKE_TO_BACKBOARD_TRANSIT));
        CenterRobotAtBackboard();
    }

    private void HandlePurplePixel() {
        ExecuteRotation(0);

        switch (randomization) {
            case LOCATION_1:
                drive.followTrajectory(CalcKinematics(-BACKDROP_PURPLE_PIXEL_VARIANCE[0])); // note: has to drive backwards
                ExpelPurple(); AutoWait();
                break;
            case LOCATION_2:
                drive.followTrajectory(CalcKinematics(-BACKDROP_PURPLE_PIXEL_VARIANCE[1]));
                drive.turn(Math.toRadians(-25 * dir)); AutoWait(); // note: always counterclockwise
                ExpelPurple(); AutoWait();
                drive.turn(Math.toRadians(25 * dir)); AutoWait();
                break;
            case LOCATION_3:
                drive.followTrajectory(CalcKinematics(-BACKDROP_PURPLE_PIXEL_VARIANCE[2]));
                ExpelPurple(); AutoWait();
                break;
        }
    }

    private void ParkRobotAtBackboard() {
        Trajectory parking = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(PARKING_POSE.vec(),PARKING_POSE.getHeading()).build();

        drive.followTrajectory(parking);
        ExecuteRotation(alliance == RobotAlliance.RED ? 90 : 270); // note: ensure field centric heading on finish
    }

    public void CenterRobotAtBackboard() {
        Trajectory center = drive
                .trajectoryBuilder(drive.getPoseEstimate())
                .splineToConstantHeading(SPIKEMARK_CENTER_POSES[allianceIndex].vec(), SPIKEMARK_CENTER_POSES[allianceIndex].getHeading())
                .build();

        drive.followTrajectory(center);
    }

    // note: helper functions -----------------------------------------------------------

    private void ExpelPurple() {
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
        Delay(800);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(1000);
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
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
