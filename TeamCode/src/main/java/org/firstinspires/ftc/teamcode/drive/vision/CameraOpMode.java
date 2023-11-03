package org.firstinspires.ftc.teamcode.drive.vision;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name="April Tag", group="Testing")
public class CameraOpMode extends OpMode {

    private static double CAMERA_HEIGHT = 0.313;

    private static int SLEEP_TIME = 20;

    private static int STARTUP_TIME = 1000;

    private static TimeUnit TIME_UNIT = TimeUnit.MILLISECONDS;

    private static float CORRECTION_FACTOR = 1;

    private String FRONT_CAMERA = "Webcam 1";
    private String BACK_CAMERA;

    public Pose2d poseEstimate;
    public Pose2d poseVelocity;

    private ArrayList<Double> lastWheelPositions = new ArrayList<>();
    private Double lastExtHeading = Double.NaN;

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

    public static final double FIELD_LENGTH = 3.58;
    public static final double WALL_TAG_X = 1.005;
    public static final double SMALL_WALL_TAG_X = 0.9;

    public static final double BACKDROP_DEPTH = 1.55;
    public static final double TAG_HEIGHT = 0.12;

    public static final double BACKDROP_ANGLE = - Math.PI / 2;

    public static final double TAG_WALL_ANGLE = Math.PI / 2;

    // This assumes the april tag starts facing along the y-axis, may change later
    public static AprilTagMetadata[] tagArray = {
            new AprilTagMetadata(7, "Back 1", 0.127,
                    new VectorF((float) - FIELD_LENGTH / 2, (float) -WALL_TAG_X, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(TAG_WALL_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(9, "Back 2", 0.127,
                    new VectorF((float) - FIELD_LENGTH / 2, (float) WALL_TAG_X, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(TAG_WALL_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(8, "Back 1a", 0.1,
                    new VectorF((float) - FIELD_LENGTH / 2, (float)-SMALL_WALL_TAG_X, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(TAG_WALL_ANGLE / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(10, "Back 2a", 0.1,
                    new VectorF((float) - FIELD_LENGTH / 2, (float)SMALL_WALL_TAG_X, (float) CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(TAG_WALL_ANGLE / 2), 0, 0,
                    (float) Math.sin(TAG_WALL_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(1, "Backdrop 1", 0.05,
                    new VectorF((float) BACKDROP_DEPTH, (float) 1.003F, (float) TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(2, "Backdrop 2", 0.05,
                    new VectorF((float) BACKDROP_DEPTH, 0.88F, (float) TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(3, "Backdrop 3", 0.05,
                    new VectorF((float) BACKDROP_DEPTH, 0.74F, (float) TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(4, "Backdrop 4", 0.05,
                    new VectorF((float) BACKDROP_DEPTH, -0.75F, (float) TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(5, "Backdrop 5", 0.05,
                    new VectorF((float) BACKDROP_DEPTH, -0.9F, (float) TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(BACKDROP_ANGLE / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(6, "Backdrop 6", 0.05,
                    new VectorF((float) BACKDROP_DEPTH, -1.05F, (float) TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(BACKDROP_ANGLE / 2), 0, 0,
                    (float) Math.sin(BACKDROP_ANGLE / 2), ACQUISITION_TIME))
    };

    private int AVERAGE_LENGTH = 3;

    private ArrayList<Pose2d> previousPoses = new ArrayList<>();

    public List<AprilTagDetection> currentDetections;

    public boolean stopTrigger = false;
    public boolean isBlind = false;

    public long blindTime;

    public Pose2d blindPose;
    public void init() {
        initAprilTag();
        elapsedTime.reset();
        telemetry.addLine("Initialised camera and Vision Portal.");
        telemetry.update();

    }
    public void start() {

    }
    public void loop() {
        analyseDetections();
        tagTelemetry(currentDetections);
        telemetry.addData("Pose: ", poseEstimate);
        telemetry.update();
    }
    public void stop() {
        visionPortal.close();
        telemetry.addData("Vision portal closed!", "");
    }

    public void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

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

    private void initAprilTag() {

        AprilTagLibrary.Builder b = new AprilTagLibrary.Builder();
        for (AprilTagMetadata tag : tagArray) {
            b.addTag(tag);
        }

        AprilTagLibrary library = b.build();

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
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

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true); // TODO: deprecated API
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }

    @SuppressLint("DefaultLocale")
    public void analyseDetections() {
        if (stopTrigger) { // TODO: check if this works, might be while loop
            telemetry.addData("STOP TRIGGER SET!", "TRUE");
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
                    blindPose = poseEstimate;
                    isBlind = true;
                }
                telemetry.addLine("Blind! Cannot see any April Tags.");
                poseEstimate = poseVelocity.times(elapsedTime.time(timeUnit) - blindTime).plus(blindPose);
            }
        }
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
}
