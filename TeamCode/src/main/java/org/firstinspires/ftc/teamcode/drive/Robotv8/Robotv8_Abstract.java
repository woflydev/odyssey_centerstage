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
import org.firstinspires.ftc.teamcode.drive.vision.FieldPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.function.Function;

public class Robotv8_Abstract {
    // Z-angle
    private static double YAW_ANGLE = 0;
    private static int ACQUISITION_TIME = 10;
    private static int SLEEP_TIME = 20;

    public static boolean PLAYING_BLUE = true;

    public static Pose2d STARTING_POSE = new Pose2d(0, 0, 0);

    public static Pose2d TILE_LOCATION = new Pose2d(-0.89,  -1.62 * (PLAYING_BLUE ? 1 : -1), 0).div(1 / RobotConstants.ROAD_RUNNER_SCALE);
    public static Pose2d[] PIXEL_LOCATIONS = {
            new Pose2d(- RobotConstants.FIELD_LENGTH / 2 + RobotConstants.PIXEL_SPACE, -0.2, - Math.PI / 2).div(1 / RobotConstants.ROAD_RUNNER_SCALE),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };

    public Trajectory[] TILE_TO_PIXEL = new Trajectory[PIXEL_LOCATIONS.length];


    // Location of the robot when it is about to drop a pixel on the leftmost slot
    public static Pose2d BLUE_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, -0.88, -Math.PI / 2).div(1 / RobotConstants.ROAD_RUNNER_SCALE);
    public static Pose2d RED_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, 1.08, -Math.PI / 2).div(1 / RobotConstants.ROAD_RUNNER_SCALE);

    public static Pose2d[] BLUE_SPIKE_MARK_LOCATIONS;
    public static Pose2d[] RED_SPIKE_MARK_LOCATIONS;

    public static Pose2d PIXEL_OFFSET = new Pose2d(0, -0.005, 0);

    public Trajectory[] PIXEL_TO_BACKDROP = new Trajectory[PIXEL_LOCATIONS.length];

    public Trajectory TILE_TO_BACKDROP;
    public Trajectory BACKDROP_TO_TILE;

    private HardwareMap hardwareMap;

    private Telemetry telemetry;
    private boolean TELEMETRY_GIVEN;
    public Robotv8_Fullstack stack;
    public CameraLocalizer localizer;

    public Robotv8_Abstract(Robotv8_Fullstack parentStack, HardwareMap map)  {
        TELEMETRY_GIVEN = false;
        //telemetry.addLine("Initialising...");
        hardwareMap = map;

        stack = parentStack;

        localizer = new CameraLocalizer(hardwareMap, RobotConstants.FRONT_CAMERA, RobotConstants.BACK_CAMERA, STARTING_POSE, telemetry, stack);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.clear();
    }

    public Robotv8_Abstract(Robotv8_Fullstack parentStack, HardwareMap map, Telemetry t) {
        telemetry = t;
        hardwareMap = map;
        TELEMETRY_GIVEN = true;
        stack = parentStack;

        if (RobotConstants.USE_LOCALISER) {
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
        if (RobotConstants.USE_LOCALISER) {
            localizer.update();
        }
        telemetry.addLine("Updating!");
        telemetry.update();
        //tagTelemetry(localizer.currentDetections);
    }

    public void stop() {
        if (RobotConstants.USE_LOCALISER) {
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

    public void initialisePaths() {
        for (int i = 0; i < TILE_TO_PIXEL.length; i++) {
            TILE_TO_PIXEL[i] = path(TILE_LOCATION, PIXEL_LOCATIONS[i]);
            PIXEL_TO_BACKDROP[i] = path(PIXEL_LOCATIONS[i], PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION);
        }

        TILE_TO_BACKDROP = path(TILE_LOCATION, PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION);

        BACKDROP_TO_TILE = path(PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION, TILE_LOCATION);
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
        int spikeMark = localizer.propTfod(PLAYING_BLUE);
        Pose2d spikePose = PLAYING_BLUE ?
                BLUE_SPIKE_MARK_LOCATIONS[spikeMark] : RED_SPIKE_MARK_LOCATIONS[spikeMark];
        stack.drive.followTrajectory(path(STARTING_POSE, spikePose));
        stack.MoveElbow(RobotConstants.ELBOW_DROPOFF);
        stack.intake.setPower(RobotConstants.INTAKE_OUTPUT);
        stack.Delay(RobotConstants.INTAKE_OUTPUT_TIME);
        Pose2d pixelPose = (PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION)
                .plus(PIXEL_OFFSET.times(spikeMark * 2 + 1));
        stack.drive.followTrajectory(path(spikePose, pixelPose));

        // FIXME: BREAKING API CHANGES IN FULLSTACK
        //stack.DepositSequence(RobotConstants.INITIAL_HEIGHT);
    }
    public void transferPixel(int pixelColour, int pixelSlot, int height, boolean fromTile) {
        // FIXME: BREAKING API CHANGES
        //stack.DropAndReset();

        stack.intake.setPower(RobotConstants.INTAKE_POWER);
        stack.Delay(RobotConstants.INTAKE_TIME);

        // FIXME: BREAKING API CHANGES
        //stack.GrabAndReady();

        if (fromTile) {
            stack.drive.followTrajectory(TILE_TO_BACKDROP);
        } else {
            stack.drive.followTrajectory(PIXEL_TO_BACKDROP[pixelColour]);
        }

        stack.drive.followTrajectory(
                path(PIXEL_TO_BACKDROP[pixelColour].end(), PIXEL_TO_BACKDROP[pixelColour].end().plus(PIXEL_OFFSET.times(pixelSlot)))
        );
        // FIXME: BREAKING API CHANGES
        //stack.DepositSequence(height);

    }
}
