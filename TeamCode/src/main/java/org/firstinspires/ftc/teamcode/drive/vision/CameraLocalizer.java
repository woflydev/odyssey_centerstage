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
import org.firstinspires.ftc.teamcode.drive.Robotv8.Fullstack;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
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

    private static int SLEEP_TIME = 20;

    private static int STARTUP_TIME = 1000;

    private static TimeUnit TIME_UNIT = TimeUnit.MILLISECONDS;

    private static float CORRECTION_FACTOR = 1;

    private String FRONT_CAMERA;
    private String BACK_CAMERA;

    private double WIDTH_LIMIT = 150;

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

    private static double ANGLE_THRESHOLD = Math.toRadians(30);

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

    public static int SCREEN_WIDTH = 640;
    public static int SCREEN_HEIGHT = 480;

    public double MIDDLE_X = (double) SCREEN_WIDTH / 2;

    private ArrayList<Pose2d> previousPoses = new ArrayList<>();

    public List<AprilTagDetection> currentDetections;

    private long blindTime = 0;
    public boolean isBlind = false;
    public boolean stopTrigger = false;

    //public Fullstack stack;

    private Telemetry t;
    private boolean TELEMETRY_GIVEN;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "RedPropNewCamera.tflite";

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            //"BlueProp",
            "RedProp"
    };

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    private TfodProcessor tfod;

    //fx, fy, cx, cy
    public static double[] OLD_CALIBRATION = {1389.80870649, 1389.80870649, 663.268596171, 399.045042197};
    public static double[] NEW_CALIBRATION = {1805.11209646, 1805.11209646, 1020.05252149, 743.423990613};

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

    public CameraLocalizer(HardwareMap map, String front, String back, Pose2d startingPose, Telemetry t, Fullstack stack) {
        this.hardwareMap = map;
        this.poseEstimate = startingPose;
        this.poseVelocity = new Pose2d(0, 0, 0);
        this.t = t;
        this.TELEMETRY_GIVEN = true;
        this.FRONT_CAMERA = front;
        this.BACK_CAMERA = back;
        //this.stack = stack;

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
            /*if (TELEMETRY_GIVEN) {
                t.addData("Pose", poseEstimate);
            }*/
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
                //.setLensIntrinsics(OLD_CALIBRATION[0], OLD_CALIBRATION[1], OLD_CALIBRATION[2], OLD_CALIBRATION[3])
                // New camera
                .setLensIntrinsics(NEW_CALIBRATION[0], NEW_CALIBRATION[1], NEW_CALIBRATION[2], NEW_CALIBRATION[3])
                // ... these parameters are fx, fy, cx, cy.

                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, FRONT_CAMERA));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 720));

        builder.enableLiveView(false);

        /*if (RobotConstants.USE_LIVE_VIEW) {
            // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
            builder.enableLiveView(true);

            // Set the stream format; MJPEG uses less bandwidth than default YUY2.
            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

            // Choose whether or not LiveView stops if no processors are enabled.
            // If set "true", monitor shows solid orange screen if no processors enabled.
            // If set "false", monitor shows camera view without annotations.
            builder.setAutoStopLiveView(false);
        } else {

        }*/
        // Create the TensorFlow processor by using a builder.

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Default just in case
        //tfod = TfodProcessor.easyCreateWithDefaults();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.75f);

        // Set and enable the processor.
        builder.addProcessors(aprilTag, tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

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
                /*if (RobotConstants.USE_DRIVE) {
                    MecanumLocalization();
                }*/
            }
        }
    }

    /*public void MecanumLocalization() {
        List<Double> wheelPositions = stack.drive.getWheelPositions();
        Double extHeading = useExternalHeading ? stack.drive.getExternalHeading() : Double.NaN;
        if (lastWheelPositions.size() > 0) {
            ArrayList<Double> wheelDeltas = differences((ArrayList<Double>) wheelPositions, lastWheelPositions);
            Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    DriveConstants.TRACK_WIDTH,
                    DriveConstants.wheelBase,
                    Fullstack.AutoMecanumDrive.LATERAL_MULTIPLIER
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
                    Fullstack.AutoMecanumDrive.LATERAL_MULTIPLIER
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
    }*/

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
            double angle = Math.atan2((recognition.getBottom() + recognition.getTop()) / 2, (recognition.getLeft() + recognition.getRight()) / 2 - MIDDLE_X);
            t.addData(""," ");
            t.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            t.addData("Angle", "%.3f degrees", angle);
            t.update();

        }   // end for() loop


        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);
    }   // end method telemetryTfod()

    public boolean detectedTfod(boolean playingBlue) {
        visionPortal.setProcessorEnabled(aprilTag, false);
        visionPortal.setProcessorEnabled(tfod, true);

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        visionPortal.setProcessorEnabled(tfod, false);
        visionPortal.setProcessorEnabled(aprilTag, true);

        return currentRecognitions.size() > 0 && (Objects.equals(currentRecognitions.get(0).getLabel(), LABELS[playingBlue ? 0 : 1]));
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
        return mod((float) Math.toDegrees(pose.bearing - pose.yaw - Math.acos(detection.metadata.fieldOrientation.w) * 2), 360);
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
}
