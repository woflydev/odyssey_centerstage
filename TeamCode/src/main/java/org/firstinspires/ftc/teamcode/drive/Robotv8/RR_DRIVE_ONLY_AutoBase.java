package org.firstinspires.ftc.teamcode.drive.Robotv8;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotParkingLocation;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.vision.CameraLocalizer;
import org.firstinspires.ftc.teamcode.drive.vision2.TFPropPipeline;
import org.firstinspires.ftc.teamcode.drive.vision2.VisionPropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.function.Function;

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

    public static final double INCHES_PER_TILE = 26;

    public static final Pose2d[] RED_STARTING_POSES = {
            new Pose2d(11.5, -60, Math.toRadians(90)),
            new Pose2d(-35, -60, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_STARTING_POSES = {
            new Pose2d(11.5, 60, Math.toRadians(270)),
            new Pose2d(-35, 60, Math.toRadians(270)),
    };
    public static final Pose2d[] RED_PARKING_POSES = {
            // note: inner is first
            new Pose2d(50, -8.5, Math.toRadians(90)),
            new Pose2d(50, -53.5, Math.toRadians(90)),
    };
    public static final Pose2d[] BLUE_PARKING_POSES = {
            new Pose2d(50, 8.5, Math.toRadians(270)),
            new Pose2d(50, 53.5, Math.toRadians(270))
    };
    public static final Pose2d[] BACKBOARD_CENTER_POSES = {
            // note: red first
            new Pose2d(53.5, -36.5, Math.toRadians(180)),
            new Pose2d(53.5, 36.5, Math.toRadians(180)),
    };
    public static final double[] YELLOW_PIXEL_VARIANCE = {
            1.6,
            1.4,
            1.2,
    };
    public static final double[] PURPLE_PIXEL_VARIANCE = {
            1.5,
            1,
            0.5,
    };

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
                drive.followTrajectory(CalcKinematics(YELLOW_PIXEL_VARIANCE[0])); AutoWait();
                break;
            case LOCATION_2:
                drive.followTrajectory(CalcKinematics(YELLOW_PIXEL_VARIANCE[1])); AutoWait();
                break;
            case LOCATION_3:
                drive.followTrajectory(CalcKinematics(YELLOW_PIXEL_VARIANCE[2])); AutoWait();
                break;
        }

        ExecuteRotation(180); // note: robot has to be backwards to deposit
        drive.followTrajectory(CalcKinematics(-1.45)); AutoWait();
        CenterRobotAtBackboard();
    }

    private void HandlePurplePixel() {
        ExecuteRotation(0);

        switch (randomization) {
            case LOCATION_1:
                drive.followTrajectory(CalcKinematics(-PURPLE_PIXEL_VARIANCE[0])); // note: has to drive backwards
                break;
            case LOCATION_2:
                drive.followTrajectory(CalcKinematics(-PURPLE_PIXEL_VARIANCE[1]));
                break;
            case LOCATION_3:
                drive.followTrajectory(CalcKinematics(-PURPLE_PIXEL_VARIANCE[2]));
                break;
        }

        ExpelPurple(); AutoWait();
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
                .splineToConstantHeading(BACKBOARD_CENTER_POSES[allianceIndex].vec(), BACKBOARD_CENTER_POSES[allianceIndex].getHeading()).build();

        drive.followTrajectory(center);
    }

    // note: helper functions -----------------------------------------------------------

    private void ExpelPurple() {
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(500);
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
        drive.turn(Math.toRadians(heading) - drive.getPoseEstimate().getHeading());
    }

    public Trajectory path(Pose2d start, Pose2d end) {
        // Same side of the truss
        if (start.getX() * end.getX() >= 0) {
            return drive.trajectoryBuilder(start)
                    .splineTo(end.vec(), end.getHeading())
                    .build();
        } else {
            double avgY = (start.getY() + end.getY()) / 2;
            Double[] diffY = RobotConstants.PATH_Y;
            for (int i = 0; i < diffY.length; i++) {
                diffY[i] = Math.abs(diffY[i] - avgY);
            }

            Function<Double, Double> cmpFn = (Double x) -> x;

            double pathValue = RobotConstants.PATH_Y[CameraLocalizer.maxOfArr(diffY, cmpFn, false)];
            return drive.trajectoryBuilder(start)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(end.vec(), end.getHeading())
                    .build();
        }
    }

    public Trajectory path(Pose2d[] points) {
        TrajectoryBuilder currentTrajectory = drive.trajectoryBuilder(points[0]);
        for (int i = 0; i < points.length - 1; i++) {
            Pose2d start = points[i];
            Pose2d end = points[i + 1];
            // Same side of the truss
            if (start.getX() * end.getX() >= 0) {
                currentTrajectory = currentTrajectory.splineTo(end.vec(), end.getHeading());
            } else {
                double avgY = (start.getY() + end.getY()) / 2;
                Double[] diffY = RobotConstants.PATH_Y;
                for (int j = 0; j < diffY.length; j++) {
                    diffY[j] = Math.abs(diffY[j] - avgY);
                }

                Function<Double, Double> cmpFn = (Double x) -> x;

                double pathValue = RobotConstants.PATH_Y[CameraLocalizer.maxOfArr(diffY, cmpFn, false)];
                currentTrajectory = currentTrajectory
                        .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                                ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                        .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                                ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                        .splineTo(end.vec(), end.getHeading());
            }

        }
        return currentTrajectory.build();
    }

    public TFPropPipeline.Randomisation convert(VisionPropPipeline.Randomization x) {
        switch (x) {
            case LOCATION_1:
                return TFPropPipeline.Randomisation.LOCATION_1;
            case LOCATION_2:
                return TFPropPipeline.Randomisation.LOCATION_2;
            case LOCATION_3:
                return TFPropPipeline.Randomisation.LOCATION_3;
            default:
                return TFPropPipeline.Randomisation.NONE;
        }
    }
}
