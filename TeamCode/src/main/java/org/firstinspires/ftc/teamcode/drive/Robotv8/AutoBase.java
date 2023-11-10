package org.firstinspires.ftc.teamcode.drive.Robotv8;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.ENCODER_TICKS_PER_TILE;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.FSM_Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotStartingPosition;
import org.firstinspires.ftc.teamcode.drive.vision2.PropPipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//@Autonomous(name="NationalsAutoBase", group="Final")
public class AutoBase extends FSM_Fullstack {
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
    private static final double BACKDROP_ALIGN_STRAFE = 0.35;

    // note: custom behaviour -----------------------------------------------------------
    public AutoBase(RobotAlliance alliance, RobotStartingPosition startPos, Point r1, Point r2, Point r3) {
        this.startingPosition = startPos;
        this.alliance = alliance;
        this.dir = alliance == RobotAlliance.RED ? 1 : -1;
        this.r1 = r1;
        this.r2 = r2;
        this.r3 = r3;
    }

    public void MainInit() {
        //BackboardToPixels(); // note: testing smooth spline
        //Delay(5000);

        OpenCvWebcam webcam;
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
    }

    public void MainStart() {
        // note: should grab yellow pixel
        GrabAndReady();

        //randomization = pipeline.getRandomization();
        telemetry.addData("TEAM_PROP_LOCATION", randomization);
        telemetry.addData("SELECTED_ALLIANCE", alliance);
        telemetry.update();

        switch(startingPosition) {
            case BACKDROP:
                HandlePurplePixel(); AutoWait();
                VisualMove(MAX_TRAJECTORY_SPEED, -1.59, -1.59, false, false, 3); AutoWait();

                RaiseAndPrime(150); Delay(600);
                DropAndReset();

                VisualMove(MAX_TRAJECTORY_SPEED, 0.1, 0.1, false, false, 3); // note: move a little away from the backdrop
                BackdropToParking(); AutoWait();
                HandleBackdropLocalize();
                break;
            case AUDIENCE:
                HandlePurplePixel(); AutoWait();
                VisualMove(MAX_CAUTIOUS_SPEED, -3.59, -3.59, false, false, 10); AutoWait();

                RaiseAndPrime(150); Delay(600);
                DropAndReset();

                VisualMove(MAX_TRAJECTORY_SPEED, 0.1, 0.1, false, false, 3);
                BackdropToParking(); AutoWait();
                HandleBackdropLocalize();
                break;
        }

        /*// note: drop off at correct april tag
        switch(randomization) {
            case LOCATION_1:
                break;
            case LOCATION_2:
                break;
            case LOCATION_3:
                break;
            default:
                break;
        }*/
    }

    // note: sequenced movement  --------------------------------------------------------
    private void HandlePurplePixel() {
        // note: drop off at correct spikemark
        if (startingPosition == RobotStartingPosition.BACKDROP) {
            switch(randomization) {
                case LOCATION_1: // note: left
                    VisualMove(MAX_TRAJECTORY_SPEED, 1.08, 1.08, false, false, 4); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, -dir, dir, false, false, 5); AutoWait();
                    ExpelPixel(); AutoWait();
                    VisualMove(MAX_STRAFE_SPEED, BACKDROP_ALIGN_STRAFE, BACKDROP_ALIGN_STRAFE, true, true, 3); // note: strafe right to align with backdrop objective
                    break;
                case LOCATION_2: // note： forward
                    VisualMove(MAX_TRAJECTORY_SPEED, 1.08, 1.08, false, false, 4); AutoWait();
                    ExpelPixel(); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, -dir, dir, false, false, 5); // note: turn 90 deg on same tile.
                    break;
                case LOCATION_3: // note: right
                    VisualMove(MAX_TRAJECTORY_SPEED, 1.08, 1.08, false, false, 4); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, dir, -dir, false, false, 5); AutoWait();
                    ExpelPixel(); AutoWait();
                    VisualMove(MAX_TRAJECTORY_SPEED, -2 * dir, 2 * dir, false, false, 5); AutoWait();
                    VisualMove(MAX_STRAFE_SPEED, BACKDROP_ALIGN_STRAFE, BACKDROP_ALIGN_STRAFE, true, false, 3);
                    break;
            }
        } else {
            switch (randomization) {
                // TODO: pathing for audience side
                case LOCATION_1:
                    VisualMove(0.6, 1, 1, false, false, 3); AutoWait();
                    VisualMove(0.5, -dir, dir, false, false, 5); AutoWait();
                    ExpelPixel(); AutoWait();
                    break;
                case LOCATION_2:
                    VisualMove(0.6, 1.08, 1.08, false, false, 3); AutoWait();
                    ExpelPixel(); AutoWait();
                    VisualMove(0.6, -dir, dir, false, false, 5);
                    break;
                case LOCATION_3:
                    VisualMove(0.6, 1, 1, false, false, 3); AutoWait();
                    VisualMove(0.5, dir, -dir, false, false, 5); AutoWait();
                    ExpelPixel(); AutoWait();
                    VisualMove(0.5, -2 * dir, 2 * dir, false, false, 5); AutoWait();
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
    private void ExpelPixel() {
        intake.setPower(-0.35);
        Delay(1500);
        intake.setPower(0);
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
}
