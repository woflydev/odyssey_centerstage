package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.teamcode.drive.localizer.CameraLocalizer;
import org.firstinspires.ftc.teamcode.drive.localizer.FieldPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;


@TeleOp(name = "Concept: AutoTeleOp_V1", group = "Concept")
public class NewRobot_v8_AutoTeleOp_V1 extends Robotv8_Fullstack {

    private static final boolean USE_VIEWPORT = true;

    public static String FRONT_CAMERA = "Webcam 1";
    public static String BACK_CAMERA = "Webcam 2";

    private static double FIELD_LENGTH = 3.58;
    private static double CAMERA_HEIGHT = 0.313;

    private static double BACKDROP_DEPTH = 1.55;
    private static double TAG_HEIGHT = 0.12;

    private static double PIXEL_SPACE = 0.05;

    private static double YAW_ANGLE = Math.PI / 2;

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

    public Trajectory[] STRIPE_TO_PIXEL = new Trajectory[FieldPipeline.PIXEL_COLOURS.length];


    // Location of the robot when it is about to drop a pixel on the leftmost slot
    public static Pose2d BACKDROP_LOCATION = new Pose2d(-0.88, BACKDROP_DEPTH, Math.PI / 2);

    public Trajectory[] PIXEL_TO_BACKDROP = new Trajectory[FieldPipeline.PIXEL_COLOURS.length];

    public Trajectory TILE_TO_BACKDROP;
    public Trajectory BACKDROP_TO_TILE;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private TimeUnit timeUnit = TimeUnit.MILLISECONDS;

    private List<AprilTagDetection> currentDetections;

    private OpenCvCamera frontCamera;
    private OpenCvCamera backCamera;

    private FieldPipeline frontPipeline;
    private FieldPipeline backPipeline;

    private SampleMecanumDrive drive;
    private CameraLocalizer localizer;

    @SuppressLint("DefaultLocale")
    public void init() {
        drive = new SampleMecanumDrive(hardwareMap);
        localizer = new CameraLocalizer(hardwareMap, FRONT_CAMERA, BACK_CAMERA, new Pose2d(0, 0, 0));


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        localizer.update();
        currentDetections = localizer.currentDetections;
        tagTelemetry(currentDetections);
        drive.setPoseEstimate(localizer.poseEstimate);

        frontPipeline = new FieldPipeline(0);
        backPipeline = new FieldPipeline(1);

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


        initCameras(frontPipeline, backPipeline);


    }

    public void start() {
        int startPropPos = frontPipeline.spikeMark;
        transferPixel(drive, 1, startPropPos, true);
    }

    public void loop() {
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

        PassiveArmResetCheck();
        RuntimeConfig();
        Macros();

        // TELEMETRY
        telemetry.addData("Arm Left: ", armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", armR.getCurrentPosition());
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/
        telemetry.addData("Target Wrist Position: ", targetWristPosition);
        telemetry.addData("Target Elbow Position: ", targetElbowPosition);
        telemetry.addData("Target Outtake Position: ", targetOuttakePosition);
        telemetry.addData("Target Claw Position: ", targetClawPosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        //telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();

        // Share the CPU.
        //sleep(SLEEP_TIME);
    }

    public void stop() {
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

    public void transferPixel(SampleMecanumDrive drive, int pixelColour, int pixelSlot, boolean fromTile) {
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
    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (adjustmentAllowed) { // lining up arm for topmost cone
            if (gamepad1.right_trigger >= 0.6 && ((armL.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition += RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false);
                }
            } else if (gamepad1.left_trigger >= 0.6 && ((armL.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition -= RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false);
                }
            } else if (gamepad1.dpad_down) {
                targetOuttakePosition = 30;
                UpdateOuttake(true);
            } else if (gamepad1.dpad_up) {
                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                UpdateOuttake(false);
            } /*else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }*/

            if (gamepad1.square) {
                targetClawPosition -= 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad1.circle) {
                targetClawPosition += 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            if (gamepad1.right_bumper) {
                targetWristPosition += 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad1.left_bumper) {
                targetWristPosition -= 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }

            if (gamepad1.triangle) {
                targetElbowPosition += 0.02;
                MoveElbow(targetElbowPosition);
            } else if (gamepad1.cross) {
                targetElbowPosition -= 0.02;
                MoveElbow(targetElbowPosition);
            }
        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            fieldCentricRed = !fieldCentricRed;
            Delay(50);
        }

        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
        }
    }

    private void Macros() {
        // test transfer stage macro
        if (gamepad1.dpad_left) {
            if (!wristActive) {
                wristActive = true;
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                Delay(600);
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

            } else {
                wristActive = false;
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                Delay(200);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
            }
            Delay(200);
        }

        else if (gamepad1.dpad_right && gamepad1.left_bumper) {
            if (!transferStageDeployed) {
                transferStageDeployed = true;
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                Delay(500);
                // TODO: test if this reinforcement actually works
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                MoveElbow(RobotConstants.ELBOW_ACTIVE);

                Delay(100);

                UpdateOuttake(false);
            } else {
                transferStageDeployed = false;
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                Delay(300);

                targetOuttakePosition = RobotConstants.MIN_OUTTAKE_HEIGHT + 1;
                UpdateOuttake(false);
                MoveElbow(RobotConstants.ELBOW_STANDBY);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
            }
        } else if (gamepad1.right_stick_button) {
            drive.followTrajectory(TILE_TO_BACKDROP);
        } else if (gamepad1.left_stick_button) {
            drive.followTrajectory(BACKDROP_TO_TILE);
        }
    }

    private void UpdateOuttake(boolean reset) { // test new function
        armR.setTargetPosition(targetOuttakePosition);
        armL.setTargetPosition(targetOuttakePosition);

        if (reset) {
            armR.setTargetPosition(10);
            armL.setTargetPosition(10);
            targetOuttakePosition = 10;
            armRuntime.reset();

            /*while (armM.getCurrentPosition() >= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

                if (armM.getCurrentPosition() <= 50 || armRuntime.seconds() >= ARM_RESET_TIMEOUT) {
                    break;
                }
            }*/

            if ((armL.getCurrentPosition() <= 15 || armR.getCurrentPosition() <= 15) || armRuntime.seconds() >= RobotConstants.ARM_RESET_TIMEOUT) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }

            telemetry.update();
        }

        else {
            armRuntime.reset();
            armR.setVelocity(1700);
            armL.setVelocity(1700); // velocity used to be 1800, could be faster
        }
    }

    private void PassiveArmResetCheck() {
        if ((armL.getCurrentPosition() <= 15 && armR.getCurrentPosition() <= 15) && targetOuttakePosition <= 30) {
            armR.setVelocity(0);
            armL.setVelocity(0);
            resetTimer.reset();
        }
    }
}

