package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.ENCODER_TICKS_PER_TILE;
import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.JUNCTION_LOW;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.FSM_Outtake;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Abstract;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

@Autonomous(name="Final RoadRunner Autonomous", group="Final")
public class NewRobot_v9_Autonomous extends Robotv8_Fullstack {
    public AutonomousRobotState robotState = AutonomousRobotState.IDLE;
    public ElapsedTime autoTimer = new ElapsedTime();

    public boolean detected = false;
    public boolean searchLeft = true;

    public enum AutonomousRobotState {
        IDLE,
        SEARCHING,
        YELLOW_OUT,
        PARKING,
    }

    public void AutonomousSubsystem() {
        switch (robotState) {
            case IDLE:
                EncoderMove(0.5, 0.7, 0.7, false, false, 3);
                Delay(200);
                robotState = AutonomousRobotState.SEARCHING;
                break;
            case SEARCHING: // covers getting purple out
                if (!detected) {
                    if (handler.localizer.spikeMark == 1) {
                        EncoderMove(0.5, -0.3, 0.3, false, false, 3);
                        Delay(200);

                        ExpelPixel();

                        EncoderMove(0.5, -0.7, 0.7, false, false, 3);
                        Delay(200);
                        EncoderMove(0.5, -1, -1, false, false, 3);
                        Delay(200);

                        detected = true;
                        robotState = AutonomousRobotState.YELLOW_OUT;
                    }
                    else if (handler.localizer.spikeMark == 3) {
                        EncoderMove(0.5, 0.3, 0.3, false, false, 3);
                        Delay(200);

                        ExpelPixel();

                        EncoderMove(0.5, 0.3, 0.3, false, false, 3);
                        EncoderMove(0.5, -1, 1, false, false, 3);
                        Delay(200);
                        EncoderMove(0.5, -1, -1, false, false, 3);
                        Delay(200);

                        detected = true;
                        robotState = AutonomousRobotState.YELLOW_OUT;
                    }
                    else if (handler.localizer.spikeMark == 5) {
                        EncoderMove(0.5, 0.3, -0.3, false, false, 3);
                        Delay(200);

                        ExpelPixel();

                        EncoderMove(0.5, -1.3, 1.3, false, false, 3);
                        Delay(200);
                        EncoderMove(0.5, -1, -1, false, false, 3);
                        Delay(200);

                        detected = true;
                        robotState = AutonomousRobotState.YELLOW_OUT;
                    }
                }
                else if (searchLeft) {
                    EncoderMove(0.5, -0.3, 0.3, false, false, 3);
                    Delay(2000);
                    if (handler.localizer.spikeMark == 1 || handler.localizer.spikeMark == 3 || handler.localizer.spikeMark == 5) {
                        EncoderMove(0.5, 0.3, -0.3, false, false, 3);
                    }
                } else if (!searchLeft) {
                    EncoderMove(0.5, 0.3, -0.3, false, false, 3);
                    Delay(2000);
                    if (handler.localizer.spikeMark == 1 || handler.localizer.spikeMark == 3 || handler.localizer.spikeMark == 5) {
                        EncoderMove(0.5, -0.3, 0.3, false, false, 3);
                    }
                }
                break;
            case YELLOW_OUT:
                GrabAndReady();
                RaiseAndPrime(500);
                Delay(800);
                DropAndReset();
                break;
            case PARKING:
                // TODO: park
                break;
        }
    }

    private void ExpelPixel() {
        intake.setPower(-0.7);
        Delay(2000);
        intake.setPower(0);
    }

    public void GrabAndReady() {
        servoFlap.setPosition(RobotConstants.FLAP_OPEN);
        Delay(700);

        // transfer stage sequence
        servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
        MoveElbow(RobotConstants.ELBOW_STANDBY); // moves it up a little to avoid tubes
        Delay(200);
        MoveElbow(RobotConstants.ELBOW_PICKUP);

        Delay(200);
        servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
        Delay(250);

        // primes the elbow
        MoveElbow(RobotConstants.ELBOW_STANDBY);
        Delay(100);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
    }

    @Override
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
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        Delay(300); // wait for claw to open

        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        MoveElbow(RobotConstants.ELBOW_STANDBY);

        Delay(350); // elbow should come down after the slide is near done

        servoFlap.setPosition(RobotConstants.FLAP_CLOSE);
        targetOuttakePosition = 10;
        UpdateOuttake(true, 0);
    }

    private double TilesToTicks(double input) {
        return ENCODER_TICKS_PER_TILE * input;
    }

    private void EncoderMove(double power, double left, double right, boolean strafe, boolean strafeRight, double safetyTimeout) {

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

        backLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void MainStart() {
        //handler.initTask();
    }

    public void MainLoop() {
        handler.localizer.telemetryTfod();
        AutonomousSubsystem();
    }
}
