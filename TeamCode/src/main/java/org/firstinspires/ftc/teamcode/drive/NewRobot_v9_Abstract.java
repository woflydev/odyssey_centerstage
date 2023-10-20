package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.List;

public class NewRobot_v9_Abstract {
    private static final boolean USE_VIEWPORT = true;
    private static final boolean USE_DRIVE = false;
    private static final boolean USE_BACK = false;

    public static String FRONT_CAMERA = "Webcam 1";
    public static String BACK_CAMERA = "Webcam 2";

    private static double FIELD_LENGTH = 3.58;
    private static double CAMERA_HEIGHT = 0.313;
    private static double WALL_TAG_X = 1.005;
    private static double SMALL_WALL_TAG_X = 0.9;

    private static double BACKDROP_DEPTH = 1.55;
    private static double TAG_HEIGHT = 0.12;

    private static double PIXEL_SPACE = 0.05;

    // Z-angle
    private static double YAW_ANGLE = 0;

    private static int ACQUISITION_TIME = 10;

    private static int SLEEP_TIME = 20;

    public static Pose2d TILE_LOCATION = new Pose2d(1.62, 0.89, Math.PI);
    public static Pose2d[] PIXEL_LOCATIONS = {
            new Pose2d(0.2, - FIELD_LENGTH / 2 + PIXEL_SPACE, - Math.PI / 2),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
    };

    public static Pose2d INTER_POINT = new Pose2d(-0.87, 0.89, Math.PI / 2);

    public Trajectory[] STRIPE_TO_PIXEL = new Trajectory[PIXEL_LOCATIONS.length];


    // Location of the robot when it is about to drop a pixel on the leftmost slot
    public static Pose2d BACKDROP_LOCATION = new Pose2d(-0.88, BACKDROP_DEPTH, Math.PI / 2);

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

    private SampleMecanumDrive drive;
    private CameraLocalizer localizer;

    public NewRobot_v9_Abstract() {
        TELEMETRY_GIVEN = false;
        //telemetry.addLine("Initialising...");

        if (USE_DRIVE) {
            drive = new SampleMecanumDrive(hardwareMap);
        }
        localizer = new CameraLocalizer(hardwareMap, FRONT_CAMERA, BACK_CAMERA, new Pose2d(0, 0, 0), telemetry);

        //telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        //telemetry.addData(">", "Touch Play to start OpMode");

        localizer.update();
        if (!localizer.isBlind && USE_DRIVE) {
            drive.setPoseEstimate(localizer.poseEstimate);
        }
        frontPipeline = new FieldPipeline(0);
        if (USE_BACK) {
            backPipeline = new FieldPipeline(1);
            initCameras(frontPipeline, backPipeline);
        } else {
            initCameras(frontPipeline);
        }

        frontPipeline = new FieldPipeline(0);

        if (USE_DRIVE) {
            for (int i = 0; i < STRIPE_TO_PIXEL.length; i++) {
                STRIPE_TO_PIXEL[i] = drive.trajectoryBuilder(TILE_LOCATION)
                        .splineTo(INTER_POINT.vec(), INTER_POINT.minus(TILE_LOCATION).getHeading())
                        .splineTo(PIXEL_LOCATIONS[i].vec(), PIXEL_LOCATIONS[i].minus(INTER_POINT).getHeading())
                        .build();

                // Note, the robot turns 180 degrees before moving to the backdrop

                PIXEL_TO_BACKDROP[i] = drive.trajectoryBuilder(PIXEL_LOCATIONS[i])
                        .strafeTo(BACKDROP_LOCATION.vec())
                        .build();
            }

            TILE_TO_BACKDROP = drive.trajectoryBuilder(TILE_LOCATION)
                    .strafeTo(INTER_POINT.vec())
                    .splineTo(BACKDROP_LOCATION.vec(), BACKDROP_LOCATION.minus(INTER_POINT).getHeading())
                    .build();

            BACKDROP_TO_TILE = drive.trajectoryBuilder(TILE_LOCATION, true)
                    .strafeTo(INTER_POINT.vec())
                    .splineTo(BACKDROP_LOCATION.vec(), BACKDROP_LOCATION.minus(INTER_POINT).getHeading())
                    .build();
        }
        //telemetry.clear();
    }

    public NewRobot_v9_Abstract(Telemetry t) {
        telemetry = t;
        TELEMETRY_GIVEN = true;
        telemetry.addLine("Initialising...");

        if (USE_DRIVE) {
            drive = new SampleMecanumDrive(hardwareMap);
        }
        localizer = new CameraLocalizer(hardwareMap, FRONT_CAMERA, BACK_CAMERA, new Pose2d(0, 0, 0), telemetry);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");

        localizer.update();
        if (!localizer.isBlind && USE_DRIVE) {
            drive.setPoseEstimate(localizer.poseEstimate);
        }
        frontPipeline = new FieldPipeline(0);
        if (USE_BACK) {
            backPipeline = new FieldPipeline(1);
            initCameras(frontPipeline, backPipeline);
        } else {
            initCameras(frontPipeline);
        }

        frontPipeline = new FieldPipeline(0);

        if (USE_DRIVE) {
            for (int i = 0; i < STRIPE_TO_PIXEL.length; i++) {
                STRIPE_TO_PIXEL[i] = drive.trajectoryBuilder(TILE_LOCATION)
                        .splineTo(INTER_POINT.vec(), INTER_POINT.minus(TILE_LOCATION).getHeading())
                        .splineTo(PIXEL_LOCATIONS[i].vec(), PIXEL_LOCATIONS[i].minus(INTER_POINT).getHeading())
                        .build();

                // Note, the robot turns 180 degrees before moving to the backdrop

                PIXEL_TO_BACKDROP[i] = drive.trajectoryBuilder(PIXEL_LOCATIONS[i])
                        .strafeTo(BACKDROP_LOCATION.vec())
                        .build();
            }

            TILE_TO_BACKDROP = drive.trajectoryBuilder(TILE_LOCATION)
                    .strafeTo(INTER_POINT.vec())
                    .splineTo(BACKDROP_LOCATION.vec(), BACKDROP_LOCATION.minus(INTER_POINT).getHeading())
                    .build();

            BACKDROP_TO_TILE = drive.trajectoryBuilder(TILE_LOCATION, true)
                    .strafeTo(INTER_POINT.vec())
                    .splineTo(BACKDROP_LOCATION.vec(), BACKDROP_LOCATION.minus(INTER_POINT).getHeading())
                    .build();
        }
        telemetry.clear();
    }

    private void initCameras(OpenCvPipeline frontPipeline) {
        if (USE_VIEWPORT) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, FRONT_CAMERA),
                    cameraMonitorViewId
            );
        } else {
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, FRONT_CAMERA)
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
        if (USE_VIEWPORT) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, FRONT_CAMERA),
                    cameraMonitorViewId
            );
            backCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, BACK_CAMERA),
                    cameraMonitorViewId
            );
        } else {
            frontCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, FRONT_CAMERA)
            );
            backCamera = OpenCvCameraFactory.getInstance().createWebcam(
                    hardwareMap.get(WebcamName.class, BACK_CAMERA)
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

    public void initTask(int pixelColour) {
        transferPixel(pixelColour, frontPipeline.spikeMark, true);
    }
    public void transferPixel(int pixelColour, int pixelSlot, boolean fromTile) {
        //activateIntake()
        //wait until desired pixel is in the slot
        if (fromTile) {
            drive.followTrajectory(TILE_TO_BACKDROP);
        } else {
            drive.followTrajectory(PIXEL_TO_BACKDROP[pixelColour]);
        }

        drive.followTrajectory(
                drive.trajectoryBuilder(BACKDROP_LOCATION)
                        .strafeTo(BACKDROP_LOCATION.vec().plus(new Vector2d(pixelSlot * FieldPipeline.PIXEL_EDGE_TO_EDGE, 0)))
                        .build()
        );
        //dropOnBackdrop()
    }
}
