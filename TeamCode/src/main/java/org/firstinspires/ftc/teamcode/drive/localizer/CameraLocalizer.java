package org.firstinspires.ftc.teamcode.drive.localizer;

import static java.lang.Thread.sleep;

import android.annotation.SuppressLint;
import android.util.Size;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
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
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotConstants;

public class CameraLocalizer implements Localizer {
    public Pose2d poseEstimate;
    public Pose2d lastEstimate;
    public Pose2d poseVelocity;

    public HardwareMap hardwareMap;
    private static double CAMERA_HEIGHT = 0.313;

    private static int SLEEP_TIME = 20;

    private static int STARTUP_TIME = 1000;

    private static TimeUnit TIME_UNIT = TimeUnit.MILLISECONDS;

    private static float CORRECTION_FACTOR = 1;

    private String FRONT_CAMERA;
    private String BACK_CAMERA;

    private ElapsedTime elapsedTime = new ElapsedTime();
    private TimeUnit timeUnit = TimeUnit.MILLISECONDS;

    /**
     * {@link #aprilTag} is the variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * {@link #visionPortal} is the variable to store our instance of the vision portal.
     */
    public VisionPortal visionPortal;

    private static int ACQUISITION_TIME = 10;

    // This assumes the april tag starts facing along the y-axis, may change later
    public static AprilTagMetadata[] tagArray = {
            new AprilTagMetadata(7, "Back 1", 0.127,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float) -RobotConstants.WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(Math.PI / 2), 0, 0,
                    (float) Math.sin(Math.PI / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(9, "Back 2", 0.127,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float) RobotConstants.WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(Math.PI / 2 / 2), 0, 0,
                    (float) Math.sin(Math.PI / 2 / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(8, "Back 1a", 0.1,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float)-RobotConstants.SMALL_WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(Math.PI / 2 / 2), 0, 0,
                    (float) Math.sin(Math.PI / 2 / 2), ACQUISITION_TIME)
            ),
            new AprilTagMetadata(10, "Back 2a", 0.1,
                    new VectorF((float) - RobotConstants.FIELD_LENGTH / 2, (float)RobotConstants.SMALL_WALL_TAG_X, (float) RobotConstants.CAMERA_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(Math.PI / 2 / 2), 0, 0,
                    (float) Math.sin(Math.PI / 2 / 2), ACQUISITION_TIME)),
            new AprilTagMetadata(1, "Backdrop 1", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, (float) 1.003F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(- Math.PI / 4), 0, 0,
                    (float) Math.sin(- Math.PI / 4), ACQUISITION_TIME)),
            new AprilTagMetadata(2, "Backdrop 2", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, 0.88F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(- Math.PI / 4), 0, 0,
                    (float) Math.sin(- Math.PI / 4), ACQUISITION_TIME)),
            new AprilTagMetadata(3, "Backdrop 3", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, 0.74F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(- Math.PI / 4), 0, 0,
                    (float) Math.sin(- Math.PI / 4), ACQUISITION_TIME)),
            new AprilTagMetadata(4, "Backdrop 4", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -0.75F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(- Math.PI / 4), 0, 0,
                    (float) Math.sin(- Math.PI / 4), ACQUISITION_TIME)),
            new AprilTagMetadata(5, "Backdrop 5", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -0.9F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(- Math.PI / 4), 0, 0,
                    (float) Math.sin(- Math.PI / 4), ACQUISITION_TIME)),
            new AprilTagMetadata(6, "Backdrop 6", 0.05,
                    new VectorF((float) RobotConstants.BACKDROP_DEPTH, -1.05F, (float) RobotConstants.TAG_HEIGHT),
                    DistanceUnit.METER, new Quaternion(
                    (float) Math.cos(- Math.PI / 4), 0, 0,
                    (float) Math.sin(- Math.PI / 4), ACQUISITION_TIME))
    };

    private int AVERAGE_LENGTH = 3;

    private VectorF previousPosition;

    private ArrayList<VectorF> previousPositions;

    private VectorF currentPosition;

    private VectorF currentVelocity;

    private double currentHeading;

    private ArrayList<Double> previousHeadings;

    public List<AprilTagDetection> currentDetections;

    private long blindTime = 0;
    public boolean isBlind = false;

    private Telemetry t;
    private boolean TELEMETRY_GIVEN;

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

    public CameraLocalizer(HardwareMap map, String front, String back, Pose2d startingPose) {
        this.hardwareMap = map;
        this.poseEstimate = startingPose;
        this.lastEstimate = startingPose;
        this.poseVelocity = new Pose2d(0, 0, 0);
        this.TELEMETRY_GIVEN = false;

        this.currentPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);
        this.previousPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);;
        this.previousPositions = new ArrayList<>();
        this.currentVelocity = new VectorF(0, 0, 0);
        this.currentHeading = startingPose.getHeading();
        this.previousHeadings = new ArrayList<>();

        this.FRONT_CAMERA = front;
        this.BACK_CAMERA = back;

        initAprilTag();
    }

    public CameraLocalizer(HardwareMap map, String front, String back, Pose2d startingPose, Telemetry t) {
        this.hardwareMap = map;
        this.poseEstimate = startingPose;
        this.lastEstimate = startingPose;
        this.poseVelocity = new Pose2d(0, 0, 0);
        this.t = t;
        this.TELEMETRY_GIVEN = true;

        this.currentPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);
        this.previousPosition = new VectorF((float) startingPose.getX(), (float) startingPose.getY(), (float)CAMERA_HEIGHT);;
        this.previousPositions = new ArrayList<>();
        this.currentVelocity = new VectorF(0, 0, 0);
        this.currentHeading = startingPose.getHeading();
        this.previousHeadings = new ArrayList<>();

        this.FRONT_CAMERA = front;
        this.BACK_CAMERA = back;

        initAprilTag();

        elapsedTime.reset();
    }

    public void update() {
        if (elapsedTime.time(TIME_UNIT) > STARTUP_TIME) {
            lastEstimate = poseEstimate;
            poseEstimate = analyseDetections();
            poseVelocity = poseEstimate.minus(lastEstimate).div(SLEEP_TIME);
            if (TELEMETRY_GIVEN) {
                t.addData("Pose", poseEstimate);
            }
            Delay(SLEEP_TIME);
        }
    }

    public void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
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
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    @SuppressLint("DefaultLocale")
    public Pose2d analyseDetections() {
        currentDetections = aprilTag.getDetections();
        //telemetry.addData("# AprilTags Detected", currentDetections.size());

        double heading = 0;
        int notNullTags = 0;

        ArrayList<VectorF> normals = new ArrayList<VectorF>();
        //ArrayList<VectorF> positions = new ArrayList<VectorF>();

        //tagTelemetry(currentDetections, this.t);
        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {

                normals.add(vectorFromPose(detection, false));
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
            heading /= notNullTags;
            previousHeadings.add(heading);

            while (previousHeadings.size() > AVERAGE_LENGTH) {
                previousHeadings.remove(0);
            }

            currentHeading = 0;

            for (int i = 0; i < previousHeadings.size(); i++) {
                currentHeading += previousHeadings.get(i) / previousHeadings.size();
            }

            /*if (notNullTags > 1) {
                currentPosition = intersectionEstimate(normalArr, posArr);
            } else {
                currentPosition = normalArr[0].multiplied((float) firstDetection.ftcPose.range * FEET_TO_METERS);
            }*/
            VectorF tmpPosition = new VectorF(0, 0, 0);

            for (int i = 0; i < normals.size(); i++) {
                tmpPosition.add(normals.get(i));
            }
            tmpPosition.multiply(1 / (float) normals.size());
            previousPositions.add(tmpPosition);

            while (previousPositions.size() > AVERAGE_LENGTH) {
                previousPositions.remove(0);
            }

            VectorF avgPos = new VectorF(0, 0, 0);

            for (VectorF pos : previousPositions) {
                avgPos.add(pos);
            }

            avgPos.multiply( 1 / (float)previousPositions.size());

            previousPosition = currentPosition;
            /*if (TELEMETRY_GIVEN) {
                for (int j = 0; j < previousPositions.size(); j++) {
                    this.t.addData("Previous position: ", String.format("X: %6.3f, Y: %6.3f", previousPositions.get(j).get(0), previousPositions.get(j).get(1)));
                }
                this.t.addData("Smoothed position: ", avgPos);
            }*/

            currentPosition = avgPos;

            currentVelocity = currentPosition.subtracted(previousPosition).multiplied(1 / (float) SLEEP_TIME);
            isBlind = false;
        } else {
            // Assumes constant velocity if no April tags can be seen
            if (!isBlind) {
                blindTime = elapsedTime.time(timeUnit);
                isBlind = true;
            }
            previousPositions = new ArrayList<>();
            currentPosition = previousPosition.added(currentVelocity.multiplied(elapsedTime.time(timeUnit) - blindTime));
        }
        //int index = 0;
        /*if (this.TELEMETRY_GIVEN) {
            this.t.addData("Position: ",  currentPose);
            this.t.update();
        }*/
        return new Pose2d(currentPosition.get(0), currentPosition.get(1), currentHeading);
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

    @SuppressLint("DefaultLocale")
    public void tagTelemetry(List<AprilTagDetection> detections, Telemetry telemetry) {
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("ID %d", detection.id));
                telemetry.addLine(String.format("XYZ %6.3f %6.3f %6.3f  (meter)", detection.ftcPose.x * CORRECTION_FACTOR, detection.ftcPose.y * CORRECTION_FACTOR, detection.ftcPose.z * CORRECTION_FACTOR));
                telemetry.addLine(String.format("PRY %6.3f %6.3f %6.3f  (rad)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.3f %6.3f %6.3f  (meter, rad, rad)", detection.ftcPose.range * CORRECTION_FACTOR, detection.ftcPose.bearing, detection.ftcPose.elevation));
            }
            telemetry.addLine();
        }
        //telemetry.update();
    }

    // Return negatives as well, if only positive use Math.floor
    public float mod(float n, float m) {
        return (n - m * Math.round(n / m));
    }
}
