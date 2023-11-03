package org.firstinspires.ftc.teamcode.drive.vision;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

public class CameraLocalizer implements Localizer {
    public Pose2d poseEstimate;
    public Pose2d poseVelocity;

    private ArrayList<Double> lastWheelPositions = new ArrayList<>();
    private Double lastExtHeading = Double.NaN;

    public HardwareMap hardwareMap;
    private static double CAMERA_HEIGHT = 0.313;

    private static int SLEEP_TIME = 100;

    private static int STARTUP_TIME = 1000;

    private static TimeUnit TIME_UNIT = TimeUnit.MILLISECONDS;

    private static float CORRECTION_FACTOR = 1;

    private String FRONT_CAMERA;
    private String BACK_CAMERA;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private TimeUnit timeUnit = TimeUnit.MILLISECONDS;

    private boolean useExternalHeading = true;

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    private static int ACQUISITION_TIME = 10;

    private static double ANGLE_THRESHOLD = Math.toRadians(15);

    // This assumes the april tag starts facing along the y-axis, may change later
    public static AprilTagMetadata[] tagArray = {
            new AprilTagMetadata(7, "Back 1", 0.127,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float) -RobotConstants.WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(9, "Back 2", 0.127,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float) RobotConstants.WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(8, "Back 1a", 0.1,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float)-RobotConstants.SMALL_WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(10, "Back 2a", 0.1,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float)RobotConstants.SMALL_WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.TAG_WALL_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(1, "Backdrop 1", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, (float) 1.003F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(2, "Backdrop 2", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, 0.88F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(3, "Backdrop 3", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, 0.74F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(4, "Backdrop 4", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -0.75F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(5, "Backdrop 5", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -0.9F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(6, "Backdrop 6", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -1.05F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(RobotConstants.BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(RobotConstants.BACKDROP_ANGLE / 2), ACQUISITION_TIME))
    };

    private int AVERAGE_LENGTH = 3;

    private ArrayList<Pose2d> previousPoses = new ArrayList<>();

    public List<AprilTagDetection> currentDetections;

    private long blindTime = 0;
    public boolean isBlind = false;
    public boolean stopTrigger = false;

    public Robotv8_Fullstack stack;

    private Telemetry t;
    private boolean TELEMETRY_GIVEN;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "PropsRecognition.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "BLUE_PROP",
            "RED_PROP"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    @NonNull
    public Pose2d getPoseEstimate() {
        return this.poseEstimate;
    }

    public void setPoseEstimate(Pose2d newPose) {
        this.poseEstimate = newPose;
    }

    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    public CameraLocalizer(HardwareMap map, String front, String back, Pose2d startingPose, Telemetry t, Robotv8_Fullstack stack) {
        this.hardwareMap = map;
        this.poseEstimate = startingPose;
        this.poseVelocity = new Pose2d(0, 0, 0);
        this.t = t;
        this.TELEMETRY_GIVEN = true;
        this.FRONT_CAMERA = front;
        this.BACK_CAMERA = back;
        this.stack = stack;

        elapsedTime.reset();

        initPortal();
    }
    public CameraLocalizer(HardwareMap map, String front, String back, Pose2d startingPose, Telemetry t) {
        this.hardwareMap = map;
        this.poseEstimate = startingPose;
        this.poseVelocity = new Pose2d(0, 0, 0);
        this.t = t;
        this.TELEMETRY_GIVEN = true;
        this.FRONT_CAMERA = front;
        this.BACK_CAMERA = back;

        elapsedTime.reset();

        initPortal();
    }

    public void update() {
        if (elapsedTime.time(TIME_UNIT) > STARTUP_TIME && !stopTrigger) {
            analyseDetections();
            if (TELEMETRY_GIVEN) {
                t.addData("Pose", poseEstimate);
            }
            Delay(SLEEP_TIME);
        }
    }

    public void stop() {
        stopTrigger = true;
        analyseDetections(); // TODO: this is called here to stop while loops, hopefully
        visionPortal.close();
        t.addData("Vision portal closed!", "");
    }

    public void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

    private void initPortal() {

        AprilTagLibrary.Builder b = new AprilTagLibrary.Builder();
        for (AprilTagMetadata tag : tagArray) {
            b.addTag(tag);
        }

        AprilTagLibrary library = b.build();

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(library)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS)
                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                .setLensIntrinsics(1389.80870649, 1389.80870649, 663.268596171, 399.045042197)
                // ... these parameters are fx, fy, cx, cy.

                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, FRONT_CAMERA));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        if (RobotConstants.USE_LIVE_VIEW) {
            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            builder.setAutoStopLiveView(false);
        } else {
            builder.enableLiveView(false);
        }
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()
                //.setModelAssetName(TFOD_MODEL_ASSET)

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Default just in case
        //tfod = TfodProcessor.easyCreateWithDefaults();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.7f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

        // Set and enable the processor.
        builder.addProcessors(aprilTag, tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Tries until the stream has been set
        // Disable or re-enable the aprilTag processor at any time.
        // visionPortal.setProcessorEnabled(aprilTag, true);

    }
    @SuppressLint("DefaultLocale")
    public void analyseDetections() {
        if (stopTrigger) { // TODO: check if this works, might be while loop
            t.addData("STOP TRIGGER SET!", "TRUE");
        } else {
            currentDetections = aprilTag.getDetections();
            //telemetry.addData("# AprilTags Detected", currentDetections.size());

            double heading = 0;
            int notNullTags = 0;

            VectorF avgPos = new VectorF(0, 0, 0);
            //ArrayList<VectorF> positions = new ArrayList<VectorF>();

            //tagTelemetry(currentDetections, this.t);
            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    avgPos.add(vectorFromPose(detection, false));
                    //positions.add(detection.metadata.fieldPosition.multiplied(FEET_TO_METERS));
                    heading += yawFromPose(detection);
                    notNullTags++;
                }
            }

        /*VectorF[] normalArr = new VectorF[normals.size()];
        VectorF[] posArr = new VectorF[positions.size()];

        normals.toArray(normalArr);
        positions.toArray(posArr);*/

            if (notNullTags > 0) {
                avgPos.multiply(1 / (float) notNullTags);
                heading /= notNullTags;
                Pose2d roughPose = new Pose2d(avgPos.get(0), avgPos.get(1), heading);

                Pose2d previousAvg = previousPoses.size() > 0 ? new Pose2d(0, 0, 0) : roughPose;
                for (Pose2d pose : previousPoses) {
                    previousAvg = previousAvg.plus(pose.times(1 / (float) previousPoses.size()));
                }

                poseEstimate = roughPose.plus(previousAvg).times(0.5);
                previousPoses.add(poseEstimate);

                while (previousPoses.size() > AVERAGE_LENGTH) {
                    previousPoses.remove(0);
                }

                poseVelocity = poseEstimate.minus(previousPoses.get(previousPoses.size() - 1)).div(SLEEP_TIME);
                isBlind = false;
            } else {
                // Assumes constant velocity if no April tags can be seen
                if (!isBlind) {
                    blindTime = elapsedTime.time(timeUnit);
                    isBlind = true;
                }
                if (RobotConstants.USE_DRIVE) {
                    MecanumLocalization();
                }
            }
        }
    }

    public void MecanumLocalization() {
        List<Double> wheelPositions = stack.drive.getWheelPositions();
        Double extHeading = useExternalHeading ? stack.drive.getExternalHeading() : Double.NaN;
        if (lastWheelPositions.size() > 0) {
            ArrayList<Double> wheelDeltas = differences((ArrayList<Double>) wheelPositions, lastWheelPositions);
            Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    DriveConstants.TRACK_WIDTH,
                    DriveConstants.wheelBase,
                    Robotv8_Fullstack.AutoMecanumDrive.LATERAL_MULTIPLIER
            );
            Double finalHeadingDelta = useExternalHeading ?
                    Angle.normDelta(extHeading - lastExtHeading) :
                    robotPoseDelta.getHeading();
            poseEstimate = Kinematics.relativeOdometryUpdate(
                    poseEstimate,
                    new Pose2d(robotPoseDelta.getX(), robotPoseDelta.getY(), finalHeadingDelta)
            );
        }

        List<Double> wheelVelocities = stack.drive.getWheelVelocities();
        Double extHeadingVel = stack.drive.getExternalHeadingVelocity();
        if (wheelVelocities != null) {
            poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                    wheelVelocities,
                    DriveConstants.TRACK_WIDTH,
                    DriveConstants.wheelBase,
                    Robotv8_Fullstack.AutoMecanumDrive.LATERAL_MULTIPLIER
            );
            if (useExternalHeading && extHeadingVel != null) {
                if (poseVelocity == null) {
                    throw new NullPointerException();
                }
                poseVelocity = new Pose2d(poseVelocity.getX(), poseVelocity.getY(), extHeadingVel);
            }
        }

        lastWheelPositions = (ArrayList<Double>) wheelPositions;
        lastExtHeading = extHeading;
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        t.addData("# Props Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            t.addData(""," ");
            t.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            t.addData("Angle", "%.3f degrees", recognition.estimateAngleToObject(AngleUnit.DEGREES));
            double angle = recognition.estimateAngleToObject(AngleUnit.RADIANS);


        }   // end for() loop


        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
    }   // end method telemetryTfod()

    public int propTfod(boolean playingBlue) {
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        t.addData("# Props Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {

            double angle = recognition.estimateAngleToObject(AngleUnit.RADIANS);
            visionPortal.setProcessorEnabled(tfod, false);
            visionPortal.setProcessorEnabled(aprilTag, true);
            if (recognition.getLabel() == LABELS[playingBlue ? 0 : 1]) {
                return (angle > ANGLE_THRESHOLD) ? 2 : ((angle > -ANGLE_THRESHOLD) ? 1 : 0);
            }

        }   // end for() loop

        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
        return -1;
    }

    public ArrayList<Double> differences(ArrayList<Double> first, ArrayList<Double> second) {
        if (first.size() != second.size()) {
            throw new IllegalArgumentException();
        }

        ArrayList<Double> tmp = new ArrayList<>();
        for (int i = 0; i < first.size(); i++) {
            tmp.add(first.get(i) - second.get(i));
        }
        return tmp;
    }

    public VectorF vectorFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;

        VectorF u = new VectorF(0, -1, 0);
        VectorF v = new VectorF(1, 0, 0);

        u = detection.metadata.fieldOrientation.applyToVector(u);
        v = detection.metadata.fieldOrientation.applyToVector(v);
        VectorF w = cross(u, v);

        VectorF newNormal = u;
        newNormal = rotationAboutAxis(pose.bearing - pose.yaw, w).applyToVector(newNormal);
        newNormal = rotationAboutAxis(pose.elevation, v).applyToVector(newNormal);

        return newNormal.multiplied((float) pose.range * CORRECTION_FACTOR).added(detection.metadata.fieldPosition);
    }
    public VectorF vectorFromPose(AprilTagDetection detection, boolean normal) {
        AprilTagPoseFtc pose = detection.ftcPose;

        VectorF u = new VectorF(0, -1, 0);
        VectorF v = new VectorF(1, 0, 0);

        u = detection.metadata.fieldOrientation.applyToVector(u);
        v = detection.metadata.fieldOrientation.applyToVector(v);
        VectorF w = cross(u, v);

        VectorF newNormal = u;
        newNormal = rotationAboutAxis(pose.bearing - pose.yaw, w).applyToVector(newNormal);
        newNormal = rotationAboutAxis(pose.elevation, v).applyToVector(newNormal);
        if (normal) {
            return newNormal;
        } else {
            return newNormal.multiplied((float) pose.range * CORRECTION_FACTOR).added(detection.metadata.fieldPosition);
        }
    }

    // Assumes pitch and roll are negligible
    // Heading is clockwise
    public double yawFromPose(AprilTagDetection detection) {
        AprilTagPoseFtc pose = detection.ftcPose;
        return mod((float) (pose.bearing - pose.yaw - Math.acos(detection.metadata.fieldOrientation.w) * 2), (float) (2 * Math.PI));
    }

    public VectorF cross(VectorF a, VectorF b) {
        return new VectorF(a.get(1) * b.get(2) - a.get(2) * b.get(1),
                a.get(2) * b.get(0) - a.get(0) * b.get(2),
                a.get(0) * b.get(1) - a.get(1) * b.get(0));
    }

    public Quaternion rotationAboutAxis(double theta, VectorF axis) {
        VectorF normAxis = axis.multiplied(1 / axis.magnitude());
        return new Quaternion((float) Math.cos(theta / 2),
                (float) (Math.sin(theta / 2) * normAxis.get(0)),
                (float) (Math.sin(theta / 2) * normAxis.get(1)),
                (float) (Math.sin(theta / 2) * normAxis.get(2)),
                0);
    }

    // Return negatives as well, if only positive use Math.floor
    public float mod(float n, float m) {
        return (n - m * Math.round(n / m));
    }

    public static Scalar[][] PIXEL_BOUNDS = {
            {new Scalar(0, 0, 248), new Scalar(179, 17, 255)},
            {new Scalar(44, 64, 166), new Scalar(106, 252, 255)},
            {new Scalar(112, 55, 189), new Scalar(128, 252, 255)},
            {new Scalar(17, 91, 0), new Scalar(236, 30, 255)}
    };

    public static String[] PIXEL_COLOURS = {
            "white",
            "green",
            "purple",
            "yellow"
    };

    // Blue prop
    public static Scalar[] TEAM_PROP_BOUNDS = {new Scalar(97, 85, 107), new Scalar(110, 190, 206)};
    public static long PROP_THRESHOLD = 1000;

    public static int PIXEL_THRESHOLD = 1000;
    // Distance from opposite edges of the pixel in metres
    public static double PIXEL_EDGE_TO_EDGE = 0.0762;
    public static double PIXEL_CORNER_TO_CORNER = PIXEL_EDGE_TO_EDGE * 2 / Math.sqrt(3);
    public static double PIXEL_SIDE_LENGTH = PIXEL_CORNER_TO_CORNER / 2;

    public static int SCORE_PER_BACKDROP_PIXEL = 5;

    public static int ROWS_PER_LINE = 3;
    public static int ROWS_FOR_FIRST = 3;
    public static int SCORE_PER_LINE = 10;
    public static int MAXIMUM_LINE_SCORE = 30;

    public static int SCORE_PER_MOSAIC = 10;

    public static int HEXAGON = 6;

    public static int SPIKE_BACKDROP_COLOUR = 2;
    // This assumes team prop, use 10 if using white pixel
    public static int SCORE_SPIKE_BACKDROP = 20;

    public static int APRIL_TAG_BACKDROP_SPACING = 2;
    public static int FIRST_APRIL_TAG_LOCATION = 0;

    public static int SCREEN_WIDTH = 1080;
    public static int SCREEN_HEIGHT = 720;

    public static float BACKDROP_Z_OFFSET = 0.18f;
    public static float PIXEL_HEIGHT = 0.1f;

    public FieldPipeline.Pixel.Backdrop backdrop = null;

    public int spikeMark;

    public int mode;

    public Mat processFrame (Mat input) {
        spikeMark = propLocation(input);
        backdrop = new FieldPipeline.Pixel.Backdrop(recognisePixels(input));
        return input;
    }

    public static int propLocation(Mat input) {
        Mat masked = new Mat();
        Core.inRange(input, TEAM_PROP_BOUNDS[0], TEAM_PROP_BOUNDS[1], masked);
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Function<MatOfPoint, Double> sort = Imgproc::contourArea;
        MatOfPoint[] contourArr = new MatOfPoint[contours.size()];
        contours.toArray(contourArr);

        int maxIndex = maxOfArr(contourArr, sort);

        if (Imgproc.contourArea(contourArr[maxIndex]) > PROP_THRESHOLD) {
            Moments M = Imgproc.moments(contourArr[maxIndex]);

            long cX = Math.round(M.m10 / M.m00);
            long cY = Math.round(M.m01 / M.m00);

            double angle = Math.atan2(cY, cX - SCREEN_WIDTH / 2) - Math.PI / 2;
            if (Math.abs(angle) > ANGLE_THRESHOLD) {
                return angle > 0 ? 5 : 1;
            }
            return 3;
        }
        // Prop not found
        return -1;
    }

    // Given a backdrop image, this function approximates the score that
    public static int scoreOnBackdrop(Mat input) {
        FieldPipeline.Pixel[] pixels = recognisePixels(input);
        FieldPipeline.Pixel.Backdrop backdrop = new FieldPipeline.Pixel.Backdrop(pixels);
        return backdrop.score();
    }

    public static FieldPipeline.Pixel[] recognisePixels(Mat input) {
        List<FieldPipeline.Pixel> currentPixels = new ArrayList<FieldPipeline.Pixel>();

        for (int i = 0; i < PIXEL_BOUNDS.length; i++) {
            Scalar[] pixelBound = PIXEL_BOUNDS[i];
            Mat masked = new Mat();
            Core.inRange(input, pixelBound[0], pixelBound[1], masked);

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Mat hierarchy = new Mat();
            // Chain approx simple because we are only searching for hexagons
            Imgproc.findContours(masked, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                double contourArea = Imgproc.contourArea(contour);
                // If the shape is a hexagon and is big enough
                if (contourArea > PIXEL_THRESHOLD && contour.size(0) == HEXAGON) {
                    Moments M = Imgproc.moments(contour);
                    long cX = Math.round(M.m10 / M.m00);
                    long cY = Math.round(M.m01 / M.m00);

                    // The hexagon is considered to have a heading of 0 if it has an apex / \
                    //                                                                    | |
                    //                                                                    \ /

                    // How we calculate this is by finding the highest point in the contour,
                    // and the point 3 indices away and calculating its angle away from the vertical
                    Point[] points = contour.toArray();
                    Function<Point, Double> f = (Point p) -> {return p.y;};

                    int highest = maxOfArr(points, f);
                    int lowest = (highest + points.length / 2) % (points.length);

                    double heading = Math.atan2(points[highest].y - points[lowest].y,
                            points[highest].x - points[lowest].x) - Math.PI / 2;

                    double estimatedSideLength = Math.sqrt(2 * contourArea / HEXAGON / Math.tan(2 * Math.PI / HEXAGON));

                    currentPixels.add(new FieldPipeline.Pixel(new Pose2d(cX, cY, heading), i, estimatedSideLength));
                }
            }
        }

        FieldPipeline.Pixel[] tmpArray = new FieldPipeline.Pixel[currentPixels.size()];
        return currentPixels.toArray(tmpArray);
    }

    // Finds the index of the element with the highest sort value
    public static <T> int maxOfArr(T[] points, Function<T, Double> sort) {
        int maxIndex = -1;
        double maxValue = 0;
        for (int i = 0; i < points.length; i++) {
            double newValue = sort.apply(points[i]);
            if (maxIndex == -1 || newValue > maxValue) {
                maxValue = newValue;
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    // Finds the index of the element with the highest sort value, unless reverse is true, in which case
    // it does the opposite
    public static <T> int maxOfArr(T[] points, Function<T, Double> sort, boolean reverse) {
        int maxIndex = -1;
        double maxValue = 0;
        for (int i = 0; i < points.length; i++) {
            double newValue = sort.apply(points[i]);
            if (maxIndex == -1 || ((newValue > maxValue && !reverse) ||(newValue < maxValue && reverse))) {
                maxValue = newValue;
                maxIndex = i;
            }
        }
        return maxIndex;
    }

    public static class Pixel {
        public Pose2d pose;
        public double sideLength;
        // Refers to the index of PIXEL_BOUNDS or PIXEL_COLOURS
        public int colour;

        public Pixel(Pose2d p, int i, double s) {
            this.pose = p;
            this.colour = i;
            this.sideLength = s;
        }

        public static class Backdrop {
            public ArrayList<FieldPipeline.Pixel>[] pixels;
            public int pixelNum;
            public int rows;
            // This is in pixels
            public double rowHeight;
            // Creates a backdrop object from a list of pixels
            // Note, the height difference in each of the rows of pixels is 1.5 * sidelength
            // The sidelength can be estimated from the average contour area of the pixels and
            // assuming it is a perfect hexagon
            public Backdrop(FieldPipeline.Pixel[] p) {

                // The number of rows must be given because the pixels' positions on the camera changes with distance
                Function<FieldPipeline.Pixel, Double> height = (FieldPipeline.Pixel a) -> {return a.pose.getY();};

                int highestIndex = maxOfArr(p, height, false);
                int lowestIndex = maxOfArr(p, height, false);

                double highY = p[highestIndex].pose.getY();
                double lowY = p[lowestIndex].pose.getY();

                double averageSideLength = 0;
                for (FieldPipeline.Pixel pixel : p) {
                    averageSideLength += pixel.sideLength / p.length;
                }

                this.rows = (int) Math.floor((highY - lowY) / averageSideLength);
                this.rowHeight = (highY - lowY) / this.rows;
                this.pixelNum = p.length;

                pixels = new ArrayList[rows];

                for (FieldPipeline.Pixel pixel : p) {
                    int row = (int) Math.floor((pixel.pose.getY() - lowY) / this.rowHeight);
                    this.pixels[row].add(pixel);
                }

                for (int i = 0; i < rows; i++) {
                    this.pixels[i].sort((FieldPipeline.Pixel a, FieldPipeline.Pixel b) -> (int) Math.round(a.pose.getX() - b.pose.getX()));
                }
            }
            public int score() {
                int baseScore = SCORE_PER_BACKDROP_PIXEL * this.pixelNum;
                // Change this later because we need more information
                int bonusFromRandomisation = 0;

                return baseScore + bonusFromRandomisation;
            }
        }
    }
}
