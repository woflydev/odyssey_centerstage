package org.firstinspires.ftc.teamcode.Deprecated.VeryDeprecated;

import static org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.RC2301_FSM;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;

@Deprecated()
public class _Auto_v1RC2301 extends RC2301_FSM {

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

    // this function is removed from the FSM
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

    // runs on start press, only once
    public void MainStart() {
        //handler.initTask(2);

        EncoderMove(0.6, 2.3, 2.3, false, false, 4);
        Delay(100);
        EncoderMove(0.8, -1, 1, false, false, 2);
        Delay(100);
        EncoderMove(0.8, 3, 3, false, false, 5);
        Delay(100);

        intake.setPower(0.6);
        Delay(2000);
        intake.setPower(0);

        EncoderMove(0.8, -4.8, -4.8, false, false, 5);

        /*GrabAndReady();
        EncoderMove(0.8, 1, 1, false, false, 5);
        Delay(50);
        EncoderMove(0.8, -1, 1, false, false, 4);
        Delay(100);
        EncoderMove(1, -1.65, -1.65, false, false, 4);
        Delay(100);

        RaiseAndPrime(100);

        //RaiseAndPrime(JUNCTION_LOW);
        Delay(500);
        DropAndReset();
        Delay(500);

        //EncoderMove(1, 1, 1, true, false, 5);
        EncoderMove(0.7, -0.3, -0.3, false, false, 2);
        //EncoderMove(1, 1.7, 1.7, false, false, 5); //NOTE: added onto next one
        EncoderMove(0.75, 4.4, 4.4, false, false, 5);

        intake.setPower(0.5);
        Delay(2000);
        intake.setPower(0);*/


    }

    public void MainLoop() {
        // Get Pixel
        // Go to
    }
}
