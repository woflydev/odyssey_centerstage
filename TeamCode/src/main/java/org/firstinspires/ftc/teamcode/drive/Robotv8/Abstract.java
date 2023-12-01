package org.firstinspires.ftc.teamcode.drive.Robotv8;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.vision.CameraLocalizer;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.Function;

public class Abstract {
    // Z-angle
    private static double YAW_ANGLE = 0;
    private static int ACQUISITION_TIME = 10;
    private static int SLEEP_TIME = 20;

    public static boolean PLAYING_BLUE = true;

    public static Pose2d[] RED_STARTING_POSES = {new Pose2d(0.29, -1.565, 90).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(-0.9, -1.565, 90).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public static Pose2d[] BLUE_STARTING_POSES = {new Pose2d(0.29, 1.565, 270).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(-0.9, 1.565, 270).times(RobotConstants.ROAD_RUNNER_SCALE)};

    // 0 is backdrop, 1 is audience
    public static int ALLIANCE_INDEX = 0;

    public static Pose2d STARTING_POSE = PLAYING_BLUE ? BLUE_STARTING_POSES[ALLIANCE_INDEX] : RED_STARTING_POSES[ALLIANCE_INDEX];


    // Location of the robot when it is about to drop a pixel on the leftmost slot
    public static Pose2d BLUE_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, -0.88, -Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE);
    public static Pose2d RED_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, 1.08, -Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE);

    public static Pose2d[] BLUE_SPIKE_MARK_LOCATIONS = {new Pose2d(0.29, 0.85, 270).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(0.9, 0.85, 270).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public static Pose2d[] RED_SPIKE_MARK_LOCATIONS = {new Pose2d(0.29, -0.85, 90).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(-0.9, -0.85, 90).times(RobotConstants.ROAD_RUNNER_SCALE)};

    public static Pose2d PIXEL_OFFSET = new Pose2d(0, -0.005, 0).times(RobotConstants.ROAD_RUNNER_SCALE);

    private HardwareMap hardwareMap;

    private Telemetry telemetry;
    private boolean TELEMETRY_GIVEN;
    public Fullstack stack;
    public CameraLocalizer localizer;

    public Abstract(Fullstack parentStack, HardwareMap map, Telemetry t) {
        telemetry = t;
        hardwareMap = map;
        TELEMETRY_GIVEN = true;
        stack = parentStack;

        if (RobotConstants.STARTUP_USE_LOCALIZER) {
            localizer = new CameraLocalizer(hardwareMap, RobotConstants.FRONT_CAMERA, RobotConstants.BACK_CAMERA, new Pose2d(0, 0, 0), telemetry, stack);
        }


        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }


    public void update() {
        Update();
    }

    public void update(Gamepad gamepad1) {
        Update();
    }

    public void Update() {
        if (RobotConstants.STARTUP_USE_LOCALIZER) {
            localizer.update();
        }
        telemetry.addLine("Updating!");
        telemetry.update();
        //tagTelemetry(localizer.currentDetections);
    }

    public void stop() {
        if (RobotConstants.STARTUP_USE_LOCALIZER) {
            localizer.stop();
        }
    }

    @SuppressLint("DefaultLocale")
    public void tagTelemetry(List<AprilTagDetection> detections) {
        if (TELEMETRY_GIVEN) {
            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (meter)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.3f %6.3f %6.3f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                    telemetry.addLine(String.format("RBE %6.3f %6.3f %6.3f  (meter, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            telemetry.update();
        }
    }

    public Trajectory path(Pose2d start, Pose2d end) {
        // Same side of the truss
        if (start.getX() * end.getX() >= 0) {
            return stack.drive.trajectoryBuilder(start)
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
            return stack.drive.trajectoryBuilder(start)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(end.vec(), end.getHeading())
                    .build();
        }
    }

    public void initTask() {
        // Below code assumes bad FOV
        /*Trajectory forward = stack.drive.trajectoryBuilder(STARTING_POSE).forward(RobotConstants.INITIAL_FORWARD).build();
        stack.drive.followTrajectory(forward);

        int spikeMark = 1;
        // Turning left
        stack.drive.turn(TURN_AUTO_ANGLE);
        for (int i = 0; i < 2; i++) {
            if (localizer.detectedTfod(PLAYING_BLUE)) {
                spikeMark = i;
                break;
            }
            stack.drive.turn(- TURN_AUTO_ANGLE);
        }
        // Back to straight
        stack.drive.turn(spikeMark * TURN_AUTO_ANGLE);*/
        stack.Delay(2000);


        Pose2d spikePose = PLAYING_BLUE ?
                BLUE_SPIKE_MARK_LOCATIONS[1] : RED_SPIKE_MARK_LOCATIONS[1];

        stack.drive.followTrajectory(path(STARTING_POSE, spikePose));

        // Placing purple pixel

        stack.MoveElbow(RobotConstants.ELBOW_DROPOFF);
        stack.intake.setPower(RobotConstants.INTAKE_OUTPUT);
        stack.Delay(RobotConstants.INTAKE_OUTPUT_TIME);


        // Placing yellow pixel
        Pose2d pixelPose = (PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION)
                .plus(PIXEL_OFFSET.times(1));
        stack.drive.followTrajectory(path(spikePose, pixelPose));

        // FIXME: BREAKING API CHANGES IN FULLSTACK
        //stack.DepositSequence(RobotConstants.INITIAL_HEIGHT);
    }
}
