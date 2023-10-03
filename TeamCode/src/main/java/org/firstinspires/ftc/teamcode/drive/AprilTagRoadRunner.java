package org.firstinspires.ftc.teamcode.drive;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp(name = "Concept: AprilTagWithRoadRunner", group = "Concept")
public class AprilTagRoadRunner extends LinearOpMode {

    private static final boolean USE_VIEWPORT = true;

    public static String FRONT_CAMERA = "Webcam 1";
    public static String BACK_CAMERA = "Webcam 2";

    private static double FIELD_LENGTH = 3.58;
    private static double CAMERA_HEIGHT = 0.313;

    private static double YAW_ANGLE = Math.PI / 2;

    private static int ACQUISITION_TIME = 10;

    private static int SLEEP_TIME = 20;


    private ElapsedTime elapsedTime = new ElapsedTime();
    private TimeUnit timeUnit = TimeUnit.MILLISECONDS;

    private Trajectory pixelToBackdrop;


    // This assumes the april tag starts facing along the y-axis, may change later
    public static AprilTagMetadata[] tagArray = {
            new AprilTagMetadata(0, "Forward", 0.1,
                    new VectorF(0, (float) FIELD_LENGTH / 2, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(YAW_ANGLE), 0, 0,
                    (float) Math.sin(YAW_ANGLE), ACQUISITION_TIME)),
            new AprilTagMetadata(1, "Left", 0.1,
                    new VectorF((float) - FIELD_LENGTH / 2, 0, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(YAW_ANGLE / 2), 0, 0,
                    (float) Math.sin(YAW_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(3, "Back", 0.1,
                    new VectorF(0, (float) - FIELD_LENGTH / 2, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, Quaternion.identityQuaternion()
            ),
            new AprilTagMetadata(2, "Right", 0.1,
                    new VectorF((float) FIELD_LENGTH / 2, 0, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(-YAW_ANGLE / 2), 0, 0,
                    (float) Math.sin(-YAW_ANGLE / 2), ACQUISITION_TIME))
    };

    private List<AprilTagDetection> currentDetections;

    private OpenCvCamera frontCamera;
    private OpenCvCamera backCamera;

    @SuppressLint("DefaultLocale")
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        CameraLocalizer localizer = new CameraLocalizer(hardwareMap, FRONT_CAMERA, BACK_CAMERA, new Pose2d(0, 0, 0), tagArray);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        localizer.update();
        currentDetections = localizer.currentDetections;
        tagTelemetry(currentDetections);
        drive.setPoseEstimate(localizer.poseEstimate);

        // TODO: Add directions to go from pixels to backdrop
        pixelToBackdrop = drive.trajectoryBuilder(new Pose2d())
                        .build();

        initCameras(new FieldPipeline(), new FieldPipeline());

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                localizer.update();
                // Add trajectories with drive.followTrajectory

                tagTelemetry(currentDetections);

                telemetry.addData("Position: ", String.format("x: %.2f, y: %.2f, h: %.2f", localizer.poseEstimate.getX(),localizer.poseEstimate.getY(), localizer.poseEstimate.getHeading()));
                // Push telemetry to the Driver Station.
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    localizer.visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    localizer.visionPortal.resumeStreaming();
                }

                if (isStopRequested()) return;


                // Share the CPU.
                sleep(SLEEP_TIME);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        localizer.visionPortal.close();
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
                        telemetry.addLine("Front camera could not be opened.");
                        telemetry.update();
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
                        telemetry.addLine("Back camera could not be opened.");
                        telemetry.update();
                    }
                }
        );
    }

    @SuppressLint("DefaultLocale")
    public void tagTelemetry(List<AprilTagDetection> detections) {
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
    }
}
