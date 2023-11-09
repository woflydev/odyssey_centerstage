package org.firstinspires.ftc.teamcode.Deprecated.VeryDeprecated;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.vision.CameraLocalizer;
import org.firstinspires.ftc.teamcode.drive.vision.FieldPipeline;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

@Deprecated()
public class NewRobot_v8_AbstractTesting {
    // Z-angle
    private static double YAW_ANGLE = 0;
    private static int ACQUISITION_TIME = 10;
    private static int SLEEP_TIME = 20;

    public static boolean PLAYING_BLUE = true;

    public static Pose2d TILE_LOCATION = new Pose2d(-0.89,  -1.62 * (PLAYING_BLUE ? 1 : -1), 0).div(1 / RobotConstants.ROAD_RUNNER_SCALE);
    public static Pose2d[] PIXEL_LOCATIONS = {
            new Pose2d(- RobotConstants.FIELD_LENGTH / 2 + RobotConstants.PIXEL_SPACE, -0.2, - Math.PI / 2).div(1 / RobotConstants.ROAD_RUNNER_SCALE),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };


    public static Pose2d INTER_POINT = new Pose2d(-0.87, 0.89 * (PLAYING_BLUE ? 1 : -1), -Math.PI / 2).div(1 / RobotConstants.ROAD_RUNNER_SCALE);

    public Trajectory[] TILE_TO_PIXEL = new Trajectory[PIXEL_LOCATIONS.length];


    // Location of the robot when it is about to drop a pixel on the leftmost slot
    public static Pose2d BLUE_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, -0.88, -Math.PI / 2).div(1 / RobotConstants.ROAD_RUNNER_SCALE);
    public static Pose2d RED_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, 1.08, -Math.PI / 2).div(1 / RobotConstants.ROAD_RUNNER_SCALE);

    public Trajectory[] PIXEL_TO_BACKDROP = new Trajectory[PIXEL_LOCATIONS.length];

    public Trajectory TILE_TO_BACKDROP;
    public Trajectory BACKDROP_TO_TILE;

    private OpenCvCamera frontCamera;
    private OpenCvCamera backCamera;
    private FieldPipeline frontPipeline;
    private FieldPipeline backPipeline;

    private HardwareMap hardwareMap;

    private Telemetry telemetry;
    private boolean TELEMETRY_GIVEN;
    public Robotv8_FullstackTestingAgain stack;
    public CameraLocalizer localizer;

    public NewRobot_v8_AbstractTesting(Robotv8_FullstackTestingAgain parentStack, HardwareMap map) {
        TELEMETRY_GIVEN = false;
        //telemetry.addLine("Initialising...");
        hardwareMap = map;

        if (RobotConstants.USE_DRIVE) {
            stack = parentStack;
        }
        localizer = new CameraLocalizer(hardwareMap, RobotConstants.FRONT_CAMERA, RobotConstants.BACK_CAMERA, new Pose2d(0, 0, 0), telemetry);

        //telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        //telemetry.addData(">", "Touch Play to start OpMode");

        localizer.update();
        if (!localizer.isBlind && RobotConstants.USE_DRIVE) {
            //stack.drive.setPoseEstimate(localizer.poseEstimate);
        }
        frontPipeline = new FieldPipeline(0);
        if (RobotConstants.USE_BACK) {
            backPipeline = new FieldPipeline(1);
            initCameras(frontPipeline, backPipeline);
        } else {
            initCameras(frontPipeline);
        }

        frontPipeline = new FieldPipeline(0);

        /*if (RobotConstants.USE_DRIVE) {
            for (int i = 0; i < TILE_TO_PIXEL.length; i++) {
                TILE_TO_PIXEL[i] = path(TILE_LOCATION, PIXEL_LOCATIONS[i]);
                PIXEL_TO_BACKDROP[i] = path(PIXEL_LOCATIONS[i], PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION);
            }

            TILE_TO_BACKDROP = path(TILE_LOCATION, PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION);

            BACKDROP_TO_TILE = path(PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION, TILE_LOCATION);
        }*/
        //telemetry.clear();
    }

    public NewRobot_v8_AbstractTesting(Robotv8_FullstackTestingAgain parentStack, HardwareMap map, Telemetry t) {
        telemetry = t;
        hardwareMap = map;
        TELEMETRY_GIVEN = true;
        telemetry.addLine("Initialising...");

        if (RobotConstants.USE_DRIVE) {
            stack = parentStack;
        }
        localizer = new CameraLocalizer(hardwareMap, RobotConstants.FRONT_CAMERA, RobotConstants.BACK_CAMERA, new Pose2d(0, 0, 0), telemetry);
        //stack.drive.setLocalizer(localizer);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //localizer.update();
        /*if (!localizer.isBlind && RobotConstants.USE_DRIVE) {
            stack.drive.setPoseEstimate(localizer.poseEstimate);
        }
        frontPipeline = new FieldPipeline(0);
        if (RobotConstants.USE_BACK) {
            backPipeline = new FieldPipeline(1);
            initCameras(frontPipeline, backPipeline);
        } else {
            initCameras(frontPipeline);
        }

        frontPipeline = new FieldPipeline(0);*/

        /*if (RobotConstants.USE_DRIVE) {
            for (int i = 0; i < TILE_TO_PIXEL.length; i++) {
                TILE_TO_PIXEL[i] = stack.drive.trajectoryBuilder(TILE_LOCATION)
                        .splineTo(INTER_POINT.vec(), INTER_POINT.minus(TILE_LOCATION).getHeading())
                        .splineTo(PIXEL_LOCATIONS[i].vec(), PIXEL_LOCATIONS[i].minus(INTER_POINT).getHeading())
                        .build();

                // Note, the robot turns 180 degrees before moving to the backdrop

                PIXEL_TO_BACKDROP[i] = stack.drive.trajectoryBuilder(PIXEL_LOCATIONS[i])
                        .strafeTo(PLAYING_BLUE ? BLUE_BACKDROP_LOCATION.vec() : RED_BACKDROP_LOCATION.vec())
                        .build();
            }

            TILE_TO_BACKDROP = stack.drive.trajectoryBuilder(TILE_LOCATION)
                    .strafeTo(INTER_POINT.vec())
                    .splineTo(PLAYING_BLUE ? BLUE_BACKDROP_LOCATION.vec() : RED_BACKDROP_LOCATION.vec(), (PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION).minus(INTER_POINT).getHeading())
                    .build();

            BACKDROP_TO_TILE = stack.drive.trajectoryBuilder(TILE_LOCATION, true)
                    .strafeTo(INTER_POINT.vec())
                    .splineTo(PLAYING_BLUE ? BLUE_BACKDROP_LOCATION.vec() : RED_BACKDROP_LOCATION.vec(), (PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION).minus(INTER_POINT).getHeading())
                    .build();
        }*/
    }

    private void initCameras(OpenCvPipeline frontPipeline) {
        if (RobotConstants.USE_CAMERA_STREAM) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, RobotConstants.FRONT_CAMERA),
                    cameraMonitorViewId
            );
        } else {
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, RobotConstants.FRONT_CAMERA)
            );
        }

        frontCamera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        // Usually this is where you'll want to start streaming from the camera (see section 4)
                        frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        frontCamera.setPipeline(frontPipeline);
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                        if (TELEMETRY_GIVEN) {
                            telemetry.addLine("Front camera could not be opened.");
                        }
                    }
                }
        );
    }

    private void initCameras(OpenCvPipeline frontPipeline, OpenCvPipeline backPipeline) {
        if (RobotConstants.USE_CAMERA_STREAM) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, RobotConstants.FRONT_CAMERA),
                    cameraMonitorViewId
            );
            backCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, RobotConstants.BACK_CAMERA),
                    cameraMonitorViewId
            );
        } else {
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, RobotConstants.FRONT_CAMERA)
            );
            backCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, RobotConstants.BACK_CAMERA)
            );
        }

        frontCamera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        // Usually this is where you'll want to start streaming from the camera (see section 4)
                        frontCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        frontCamera.setPipeline(frontPipeline);
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                        if (TELEMETRY_GIVEN) {
                            telemetry.addLine("Front camera could not be opened.");
                        }
                    }
                }
        );
        backCamera.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        // Usually this is where you'll want to start streaming from the camera (see section 4)
                        backCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                        backCamera.setPipeline(backPipeline);
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                        if (TELEMETRY_GIVEN) {
                            telemetry.addLine("Back camera could not be opened.");
                        }
                    }
                }
        );
    }

    public void update() {
        localizer.update();
        tagTelemetry(localizer.currentDetections);
        if (TELEMETRY_GIVEN) {
            telemetry.addData("Pose: ", localizer.getPoseEstimate());
            telemetry.update();
        }
    }

    public void update(Gamepad gamepad1) {
        localizer.update();
        tagTelemetry(localizer.currentDetections);
        if (gamepad1.dpad_down) {
            localizer.visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            localizer.visionPortal.resumeStreaming();
        }
    }

    public void stop() {
        localizer.visionPortal.close();
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

    /*public Trajectory path(Pose2d start, Pose2d end) {
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

            double pathValue = RobotConstants.PATH_Y[FieldPipeline.maxOfArr(diffY, cmpFn, false)];
            return stack.drive.trajectoryBuilder(start)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(end.vec(), end.getHeading())
                    .build();
        }
    }*/

    public void initTask(int pixelColour) {
        transferPixel(pixelColour, frontPipeline.spikeMark, true);
    }
    public void transferPixel(int pixelColour, int pixelSlot, boolean fromTile) {
        stack.ArmStandby();

        stack.intake.setPower(RobotConstants.INTAKE_POWER);
        stack.Delay(RobotConstants.INTAKE_TIME);

        stack.Grab();

        /*if (fromTile) {
            stack.drive.followTrajectory(TILE_TO_BACKDROP);
        } else {
            stack.drive.followTrajectory(PIXEL_TO_BACKDROP[pixelColour]);
        }

        stack.drive.followTrajectory(
                stack.drive.trajectoryBuilder(PLAYING_BLUE ? BLUE_BACKDROP_LOCATION : RED_BACKDROP_LOCATION)
                        .strafeTo((PLAYING_BLUE ? BLUE_BACKDROP_LOCATION.vec() : RED_BACKDROP_LOCATION.vec()).plus(new Vector2d(pixelSlot * FieldPipeline.PIXEL_EDGE_TO_EDGE, 0)))
                        .build()
        );*/
        stack.Deposit((int) (frontPipeline.backdrop.rows * FieldPipeline.PIXEL_HEIGHT + FieldPipeline.BACKDROP_Z_OFFSET));

    }
}
