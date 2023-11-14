package org.firstinspires.ftc.teamcode.drive.auto;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.ENCODER_TICKS_PER_TILE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.Robotv8.FSM_Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.vision.CameraLocalizer;
import org.firstinspires.ftc.teamcode.drive.vision2.PropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.function.Function;

//@Autonomous(name="NationalsAutoBase", group="Final")
@Disabled
public class AC2304RR_AutoBase extends FSM_Fullstack {
    private PropPipeline.Randomization randomization;
    private final ElapsedTime autoTimer = new ElapsedTime();
    public RobotAlliance alliance;
    public RobotStartingPosition startingPosition;
    public int dir;
    private Point r1;
    private Point r2;
    private Point r3;

    private static final double MAX_STRAFE_SPEED = 0.5;
    private static final double MAX_TRAJECTORY_SPEED = 0.6;
    private static final double MAX_CAUTIOUS_SPEED = 0.4;
    private static final double BACKDROP_ALIGN_STRAFE = 0.3;

    public AutoMecanumDrive drive;

    public static Pose2d[] RED_STARTING_POSES = {new Pose2d(0.29, -1.565, 90).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(-0.9, -1.565, 90).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public static Pose2d[] BLUE_STARTING_POSES = {new Pose2d(0.29, 1.565, 270).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(-0.9, 1.565, 270).times(RobotConstants.ROAD_RUNNER_SCALE)};



    // Location of the robot when it is about to drop a pixel on the leftmost slot
    public static Pose2d BLUE_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, -0.88, -Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE);
    public static Pose2d RED_BACKDROP_LOCATION = new Pose2d(RobotConstants.BACKDROP_DEPTH, 1.08, -Math.PI / 2).times(RobotConstants.ROAD_RUNNER_SCALE);

    public static Pose2d[] BLUE_SPIKE_MARK_LOCATIONS = {new Pose2d(0.29, 0.85, 270).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(0.9, 0.85, 270).times(RobotConstants.ROAD_RUNNER_SCALE)};
    public static Pose2d[] RED_SPIKE_MARK_LOCATIONS = {new Pose2d(0.29, -0.85, 90).times(RobotConstants.ROAD_RUNNER_SCALE), new Pose2d(-0.9, -0.85, 90).times(RobotConstants.ROAD_RUNNER_SCALE)};

    public Pose2d STARTING_POSE;
    public Pose2d SPIKE_POSE;
    public static Pose2d PIXEL_OFFSET = new Pose2d(0, -0.005, 0).times(RobotConstants.ROAD_RUNNER_SCALE);


    // note: custom behaviour -----------------------------------------------------------
    public AC2304RR_AutoBase(RobotAlliance alliance, RobotStartingPosition startPos, Point r1, Point r2, Point r3) {
        this.alliance = alliance;
        this.startingPosition = startPos;
        this.dir = alliance == RobotAlliance.RED ? 1 : -1;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
        int allianceIndex = this.startingPosition == RobotStartingPosition.AUDIENCE ? 0 : 1;
        this.STARTING_POSE = this.alliance == RobotAlliance.BLUE ? BLUE_STARTING_POSES[allianceIndex] : RED_STARTING_POSES[allianceIndex];
        this.SPIKE_POSE = this.alliance == RobotAlliance.BLUE ? BLUE_SPIKE_MARK_LOCATIONS[allianceIndex] : RED_SPIKE_MARK_LOCATIONS[allianceIndex];
    }

    public void MainInit() {
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(1000);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);

        OpenCvWebcam webcam;
        drive = new AutoMecanumDrive(hardwareMap, STARTING_POSE, telemetry);
        PropPipeline pipeline = new PropPipeline( alliance, r1, r2, r3 );

        @SuppressLint("DiscouragedApi") int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 20);
            }

            @Override
            public void onError(int errorCode) {
                // intentional noop
                telemetry.addLine("error, you stupid idiot");
            }
        });

        autoTimer.reset();
        while (autoTimer.seconds() < 3) {
            randomization = pipeline.getRandomization();
            telemetry.addData("TEAM_PROP_LOCATION", randomization);
            telemetry.update();
        }
        webcam.closeCameraDevice();



        /*Pose2d startPose = new Pose2d(10.59, -60.49, Math.toRadians(90.00));
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(10.59, -60.49, Math.toRadians(90.00)))
                .splineTo(new Vector2d(35.75, -11.73), Math.toRadians(90.00))
                .build();
        drive.followTrajectorySequence(trajectory);*/
    }

    public void MainStart() {
        PrimePurple();

        //randomization = pipeline.getRandomization();
        telemetry.addData("TEAM_PROP_LOCATION", randomization);
        telemetry.addData("SELECTED_ALLIANCE", alliance);
        telemetry.update();

        drive.followTrajectory(path(STARTING_POSE, SPIKE_POSE));
        switch(startingPosition) {
            case BACKDROP:
                HandlePurplePixel(); AutoWait();
                GrabAndReady(); AutoWait();

                RaiseAndPrime(100); Delay(600);
                VisualMove(MAX_TRAJECTORY_SPEED, -1.6, -1.6, false, false, 3); AutoWait();

                DropAndReset();

                VisualMove(MAX_TRAJECTORY_SPEED, 0.1, 0.1, false, false, 3); AutoWait(); // note: move a little away from the backdrop
                HandleBackdropLocalize(); AutoWait();
                BackdropToParking(); AutoWait();
                break;
            case AUDIENCE:
                HandlePurplePixel(); AutoWait();
                GrabAndReady(); AutoWait();

                RaiseAndPrime(100); Delay(600);
                VisualMove(MAX_CAUTIOUS_SPEED, -3.6, -3.6, false, false, 10); AutoWait();

                DropAndReset();

                VisualMove(MAX_TRAJECTORY_SPEED, 0.1, 0.1, false, false, 3); AutoWait();
                HandleBackdropLocalize(); AutoWait();
                BackdropToParking(); AutoWait();
                break;
        }
    }
    private void HandlePurplePixel() {
        // note: drop off at correct spikemark
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            switch(randomization) {
                case LOCATION_1: // note: left
                    VisualMove(MAX_TRAJECTORY_SPEED, -1.08, -1.08, false, false, 4); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, -dir, dir, false, false, 5); AutoWait();
                    ExpelPurple(); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, -2 * dir, 2 * dir, false, false, 5); AutoWait();
                    VisualMove(MAX_STRAFE_SPEED, BACKDROP_ALIGN_STRAFE, BACKDROP_ALIGN_STRAFE, true, true, 3); // note: strafe right to align with backdrop objective
                    break;
                case LOCATION_2: // note： forward
                    VisualMove(MAX_TRAJECTORY_SPEED, -1.08, -1.08, false, false, 4); AutoWait();
                    ExpelPurple(); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, dir, -dir, false, false, 5); // note: turn 90 deg on same tile.
                    break;
                case LOCATION_3: // note: right
                    VisualMove(MAX_TRAJECTORY_SPEED, -1.08, -1.08, false, false, 4); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, dir, -dir, false, false, 5); AutoWait();
                    ExpelPurple(); AutoWait();
                    VisualMove(MAX_STRAFE_SPEED, BACKDROP_ALIGN_STRAFE, BACKDROP_ALIGN_STRAFE, true, false, 3);
                    break;
            }
        } else {
            switch (randomization) {
                // TODO: pathing for audience side
                case LOCATION_1:
                    VisualMove(0.6, -1, -1, false, false, 3); AutoWait();
                    VisualMove(0.6, -dir, dir, false, false, 5);
                    ExpelPurple(); AutoWait();
                    VisualMove(0.5, -2 * dir, 2 * dir, false, false, 5); AutoWait();
                    break;
                case LOCATION_2:
                    VisualMove(0.6, -1.08, -1.08, false, false, 3); AutoWait();
                    ExpelPurple(); AutoWait();
                    VisualMove(0.6, -dir, dir, false, false, 5);
                    break;
                case LOCATION_3:
                    VisualMove(0.6, -1, -1, false, false, 3); AutoWait();
                    VisualMove(0.5, dir, -dir, false, false, 5); AutoWait();
                    ExpelPurple(); AutoWait();
                    break;
            }
        }
    }

    private void HandleBackdropLocalize() {
        switch(randomization) {
            case LOCATION_1:
                VisualMove(MAX_STRAFE_SPEED, BACKDROP_ALIGN_STRAFE, BACKDROP_ALIGN_STRAFE, true, false, 3); // note: strafe right to align with backdrop objective
                break;
            case LOCATION_2:
                break;
            case LOCATION_3:
                VisualMove(MAX_STRAFE_SPEED, BACKDROP_ALIGN_STRAFE, BACKDROP_ALIGN_STRAFE, true, true, 3);
                break;
        }
    }

    private void BackdropToParking() {
        VisualMove(MAX_STRAFE_SPEED, 1, 1, true, true, 5); // note: strafe
        VisualMove(0.8, dir, -dir, false, false, 5);
        AutoWait();
        VisualMove(MAX_STRAFE_SPEED, 0.2, 0.2, true, true, 4);
    }

    private void BackdropToPixels() {
        VisualMove(0.6, 2, 2, false, false, 3);
        AutoWait();
        VisualMove(0.6, 0.1, 2, false, false, 3);
        VisualMove(0.6, 2, 2, false, false, 3);
        AutoWait();
    }

    // note: helper functions -----------------------------------------------------------
    private void PrimePurple() {
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
    }

    private void ExpelPurple() {
        MoveElbow(RobotConstants.ELBOW_STANDBY_BACK);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY_BACK);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(500);
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
    }

    // note: sequenced movement  --------------------------------------------------------
    private void BackboardToParking() {
        VisualMove(0.7, 1, 1, true, false, 5); // note: strafe
        VisualMove(0.5, dir, -dir, false, false, 3);
        AutoWait();
        VisualMove(0.5, 0.2, 0.2, true, true, 3);
    }

    private void BackboardToPixels() {
        VisualMove(0.6, 2, 2, false, false, 3);
        AutoWait();
        VisualMove(0.6, 0.1, 2, false, false, 3);
        VisualMove(0.6, 2, 2, false, false, 3);
        AutoWait();
    }

    // note: helper functions -----------------------------------------------------------
    private void ExpelPixel() {
        intake.setPower(-0.4);
        Delay(800);
        intake.setPower(0);
    }

    public void CloseClaw() {

    }
    public void GrabAndReady() {
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        Delay(500);

        servoFlap.setPosition(RobotConstants.FLAP_OPEN);
        Delay(700);

        // transfer stage sequence
        servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_STANDBY); // moves it up a little to avoid tubes
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_PICKUP);

        Delay(200);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        Delay(500);

        // primes the elbow
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(100);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
    }

    public void RaiseAndPrime(int height) {
        intake.setPower(0); // make sure intake is not running

        targetOuttakePosition = height;
        UpdateOuttake(false, 0);

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

        MoveElbow(RobotConstants.ELBOW_ACTIVE);

        outtakeState = FSM_Outtake.PRIMED_FOR_DEPOSIT;
        Delay(50); // debounce
    }

    public void DropAndReset() {
        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(800); // wait for claw to open

        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY);

        Delay(350); // elbow should come down after the slide is near done

        targetOuttakePosition = 10;
        UpdateOuttake(true, 0);
    }

    private double TilesToTicks(double input) {
        return ENCODER_TICKS_PER_TILE * input;
    }

    private void VisualMove(double power, double left, double right, boolean strafe, boolean strafeRight, double safetyTimeout) {

        int backLMTarget;
        int frontLMTarget;
        int backRMTarget;
        int frontRMTarget;

        if (!strafe) {
            backLMTarget = backLM.getCurrentPosition() - (int)(TilesToTicks(left));
            frontLMTarget = frontLM.getCurrentPosition() - (int)(TilesToTicks(left));
            backRMTarget = backRM.getCurrentPosition() - (int)(TilesToTicks(right));
            frontRMTarget = frontRM.getCurrentPosition() - (int)(TilesToTicks(right));
        }

        else {
            int dir = strafeRight ? 1 : -1;
            dir *= alliance == RobotAlliance.RED ? 1 : -1;
            backLMTarget = backLM.getCurrentPosition() + (int)(TilesToTicks(left) * dir);
            frontLMTarget = frontLM.getCurrentPosition() - (int)(TilesToTicks(left) * dir);
            backRMTarget = backRM.getCurrentPosition() - (int)(TilesToTicks(right) * dir);
            frontRMTarget = frontRM.getCurrentPosition() + (int)(TilesToTicks(right) * dir);
        }

        backLM.setTargetPosition(backLMTarget);
        frontLM.setTargetPosition(frontLMTarget);
        backRM.setTargetPosition(backRMTarget);
        frontRM.setTargetPosition(frontRMTarget);

        backLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoderRuntime.reset();
        backLM.setPower(Math.abs(power));
        frontLM.setPower(Math.abs(power));
        backRM.setPower(Math.abs(power));
        frontRM.setPower(Math.abs(power));

        while ((encoderRuntime.seconds() <= safetyTimeout) && (backRM.isBusy() && backLM.isBusy())) {
            telemetry.clear();
            telemetry.addData("CURRENT COORDINATE: ",  "%7d :%7d", backLM.getCurrentPosition(), backRM.getCurrentPosition());
            telemetry.update();
        }

        backLM.setPower(0);
        frontLM.setPower(0);
        backRM.setPower(0);
        frontRM.setPower(0);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*backLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    private void AutoWait() {
        Delay(400);
    }

    public Trajectory path(Pose2d start, Pose2d end) {
        // Same side of the truss
        if (start.getX() * end.getX() >= 0) {
            return drive.trajectoryBuilder(start)
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
            return drive.trajectoryBuilder(start)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                            ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                    .splineTo(end.vec(), end.getHeading())
                    .build();
        }
    }

    public Trajectory path(Pose2d[] points) {
        TrajectoryBuilder currentTrajectory = drive.trajectoryBuilder(points[0]);
        for (int i = 0; i < points.length - 1; i++) {
            Pose2d start = points[i];
            Pose2d end = points[i + 1];
            // Same side of the truss
            if (start.getX() * end.getX() >= 0) {
                currentTrajectory = currentTrajectory.splineTo(end.vec(), end.getHeading());
            } else {
                double avgY = (start.getY() + end.getY()) / 2;
                Double[] diffY = RobotConstants.PATH_Y;
                for (int j = 0; j < diffY.length; j++) {
                    diffY[j] = Math.abs(diffY[j] - avgY);
                }

                Function<Double, Double> cmpFn = (Double x) -> x;

                double pathValue = RobotConstants.PATH_Y[CameraLocalizer.maxOfArr(diffY, cmpFn, false)];
                currentTrajectory = currentTrajectory
                        .splineTo(new Vector2d(((start.getX() >= 0) ? 1 : -1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                                ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                        .splineTo(new Vector2d(((start.getX() >= 0) ? -1 : 1) * RobotConstants.TRUSS_WIDTH / 2, pathValue),
                                ((start.getX() >= 0) ? 1 : -1) * RobotConstants.HEADING)
                        .splineTo(end.vec(), end.getHeading());
            }

        }
        return currentTrajectory.build();
    }
}
