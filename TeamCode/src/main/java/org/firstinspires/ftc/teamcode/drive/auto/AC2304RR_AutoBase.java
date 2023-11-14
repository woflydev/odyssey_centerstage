package org.firstinspires.ftc.teamcode.drive.auto;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.ENCODER_TICKS_PER_TILE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Robotv8.FSM_Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.vision.CameraLocalizer;
import org.firstinspires.ftc.teamcode.drive.vision2.PropDetection;
import org.firstinspires.ftc.teamcode.drive.vision2.PropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.function.Function;

//@Autonomous(name="NationalsAutoBase", group="Final")
@Disabled
public class AC2304RR_AutoBase extends FSM_Fullstack {
    private PropDetection.Randomisation location;
    private final ElapsedTime autoTimer = new ElapsedTime();
    public RobotAlliance alliance;
    public RobotStartingPosition startingPosition;
    public int dir;
    private Point r1;
    private Point r2;
    private Point r3;

    private static final double MAX_STRAFE_SPEED = 0.5;
    private static final double MAX_TRAJECTORY_SPEED = 0.6;
    private static final double MAX_CAUTIOUS_SPEED = 0.4;
    private static final double BACKDROP_ALIGN_STRAFE = 0.3;

    public AutoMecanumDrive drive;

    public static Pose2d[] RED_STARTING_POSES = {new Pose2d(0.29, -1.565, Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE),
            new Pose2d(-0.9, -1.565, Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public static Pose2d[] BLUE_STARTING_POSES = {new Pose2d(0.29, 1.565, 3 * Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE),
            new Pose2d(-0.9, 1.565, 3* Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE)};



    // Index 0 is audience, index 1 is backdrop

    public static Pose2d BLUE_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, -0.88, -Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE);
    public static Pose2d RED_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, 1.08, -Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE);

    public static Pose2d[] BLUE_SPIKE_MARK_LOCATIONS = {new Pose2d(0.29, 0.85, 3 * Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(0.9, 0.85, 3 * Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public static Pose2d[] RED_SPIKE_MARK_LOCATIONS = {new Pose2d(0.29, -0.85, Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(-0.9, -0.85, Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE)};

    public static Pose2d[] BLUE_PARKING_LOCATIONS = {};
    public static Pose2d[] RED_PARKING_LOCATIONS = {};

    public Pose2d STARTING_POSE;
    public Pose2d SPIKE_POSE;

    public Pose2d BACKDROP_POSE;

    public Pose2d PARKING_POSE;
    public static Pose2d PIXEL_OFFSET = new Pose2d(0, -0.005, 0).times(RobotConstants.ROAD_RUNNER_SCALE);


    // note: custom behaviour -----------------------------------------------------------
    public AC2304RR_AutoBase(RobotAlliance alliance, RobotStartingPosition startPos, Point r1, Point r2, Point r3) {
        this.alliance = alliance;
        this.startingPosition = startPos;
        this.dir = alliance == RobotAlliance.RED ? 1 : -1;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
        int allianceIndex = this.startingPosition == RobotStartingPosition.AUDIENCE ? 0 : 1;
        this.STARTING_POSE = this.alliance == RobotAlliance.BLUE ? BLUE_STARTING_POSES[allianceIndex] : RED_STARTING_POSES[allianceIndex];
        this.SPIKE_POSE = this.alliance == RobotAlliance.BLUE ? BLUE_SPIKE_MARK_LOCATIONS[allianceIndex] : RED_SPIKE_MARK_LOCATIONS[allianceIndex];
        this.PARKING_POSE = this.alliance == RobotAlliance.BLUE ? BLUE_PARKING_LOCATIONS[allianceIndex] : RED_PARKING_LOCATIONS[allianceIndex];
        this.BACKDROP_POSE = this.alliance == RobotAlliance.BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION;
    }

    public void MainInit() {
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(1000);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        drive = new AutoMecanumDrive(hardwareMap, STARTING_POSE, telemetry);

        DetectProp();

        drive.setPoseEstimate(STARTING_POSE);
    }

    public void MainStart() {
        //randomization = pipeline.getRandomization();
        telemetry.addData("TEAM_PROP_LOCATION", location);
        telemetry.addData("SELECTED_ALLIANCE", alliance);
        telemetry.update();

        HandlePurplePixel(); AutoWait();
        HandleYellowPixel(); AutoWait();
        Park(); AutoWait();

    }

    private void DetectProp() {
        OpenCvWebcam webcam;
        PropPipeline pipeline = new PropPipeline( alliance, r1, r2, r3 );

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
            location = convert(pipeline.getRandomization());
            telemetry.addData("TEAM_PROP_LOCATION", location);
            telemetry.update();
        }
        webcam.closeCameraDevice();
    }

    private void NewDetectProp() {
        PropDetection detector = new PropDetection(hardwareMap.get(WebcamName.class, "Webcam 1"), alliance, telemetry);
        autoTimer.reset();
        while (autoTimer.seconds() < 3) {
            location = detector.getLocation();
            telemetry.addData("TEAM_PROP_LOCATION", location);
            telemetry.update();
        }
        detector.stop();
    }


    private void HandlePurplePixel() {
        PrimePurple();
        drive.followTrajectory(path(STARTING_POSE, SPIKE_POSE));
        drive.turn((2 - location.ordinal()) * Math.PI / 2);
        ExpelPurple(); AutoWait();
        drive.turn((location.ordinal() - 2) * Math.PI / 2);
    }

    private void HandleYellowPixel() {
        GrabAndReady(); AutoWait();

        RaiseAndPrime(100); Delay(600);
        drive.followTrajectory(path(SPIKE_POSE, BACKDROP_POSE.plus(PIXEL_OFFSET.times(2 * location.ordinal()))));
        DropAndReset();
    }

    private void Park() {
        drive.followTrajectory(path(BACKDROP_POSE.plus(PIXEL_OFFSET.times(2 * location.ordinal())), PARKING_POSE));
    }

    // note: helper functions -----------------------------------------------------------
    private void PrimePurple() {
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
    }

    private void ExpelPurple() {
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(500);
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
    }

    public void DropAndReset() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(800); // wait for claw to open

        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY);

        Delay(350); // elbow should come down after the slide is near done

        targetOuttakePosition = 10;
        UpdateOuttake(true, 0);
    }

    private void AutoWait() {
        Delay(400);
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

    public PropDetection.Randomisation convert(PropPipeline.Randomization x) {
        switch (x) {
            case LOCATION_1:
                return PropDetection.Randomisation.LOCATION_1;
            case LOCATION_2:
                return PropDetection.Randomisation.LOCATION_2;
            case LOCATION_3:
                return PropDetection.Randomisation.LOCATION_3;
            default:
                return PropDetection.Randomisation.NONE;
        }
    }
}
