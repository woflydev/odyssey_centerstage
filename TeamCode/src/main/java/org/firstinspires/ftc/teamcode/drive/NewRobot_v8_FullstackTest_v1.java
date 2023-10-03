package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.Robotv7.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv7.Robotv7;

@TeleOp()
public class NewRobot_v8_FullstackTest_v1 extends Robotv7 {
    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (adjustmentAllowed) { // lining up arm for topmost cone
            if (gamepad1.right_trigger >= 0.6 && ((armL.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition += RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    NewUpdateOuttake(false);
                }
            } else if (gamepad1.left_trigger >= 0.6 && ((armL.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition -= RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    NewUpdateOuttake(false);
                }
            } else if (gamepad1.dpad_down) {
                targetOuttakePosition = 30;
                NewUpdateOuttake(true);
            } else if (gamepad1.dpad_up) {
                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                NewUpdateOuttake(false);
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }

            if (gamepad1.square) {
                targetClawPosition -= 0.05;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad1.circle) {
                targetClawPosition += 0.05;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            if (gamepad1.right_bumper) {
                targetWristPosition += 0.05;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad1.left_bumper) {
                targetWristPosition -= 0.05;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }

            if (gamepad1.triangle) {
                targetElbowPosition += 0.05;
                servoElbowR.setPosition(targetElbowPosition);
                servoElbowL.setPosition(targetElbowPosition);
                Delay(50);
            } else if (gamepad1.cross) {
                targetElbowPosition -= 0.05;
                servoElbowR.setPosition(targetElbowPosition);
                servoElbowL.setPosition(targetElbowPosition);
                Delay(50);
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

        else if ((gamepad1.left_trigger >= 0.25 && gamepad1.right_trigger >= 0.25) ||
                (gamepad2.left_trigger >= 0.25 && gamepad2.right_trigger >= 0.25)) {
            driveSpeedModifier = (driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER) ? RobotConstants.PRECISION_DRIVE_SPEED_MODIFIER : RobotConstants.BASE_DRIVE_SPEED_MODIFIER;
        }

        else {
            driveSpeedModifier = RobotConstants.BASE_DRIVE_SPEED_MODIFIER;
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
    }

    private void NewUpdateOuttake(boolean reset) { // test new function
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
            armR.setVelocity(3800);
            armL.setVelocity(3800); // velocity used to be 1800, could be faster
        }
    }

    public void Delay(double time) {
        try { sleep((long)time); } catch (Exception e) { System.out.println("interrupted"); }
    }

    private void InitializeBlock() {
        driveSpeedModifier = RobotConstants.BASE_DRIVE_SPEED_MODIFIER;

        /*backLM = hardwareMap.get(DcMotorEx.class, BACK_LEFT);
        backRM = hardwareMap.get(DcMotorEx.class, BACK_RIGHT);

        frontLM = hardwareMap.get(DcMotorEx.class, FRONT_LEFT);
        frontRM = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT); //frontRM.setDirection(DcMotorSimple.Direction.REVERSE); // weird workaround Stanley put in

        backLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRM.setDirection(DcMotorSimple.Direction.REVERSE);
        backRM.setDirection(DcMotorSimple.Direction.REVERSE);*/

        armR = hardwareMap.get(DcMotorEx.class, "armR");
        armL = hardwareMap.get(DcMotorEx.class, "armL");

        servoElbowR = hardwareMap.get(Servo.class, "elbowR");
        servoElbowL = hardwareMap.get(Servo.class, "elbowL");
        servoClaw = hardwareMap.get(Servo.class, "claw");
        servoWrist = hardwareMap.get(Servo.class, "wrist");

        clawOpen = true;
        wristActive = false;
        elbowActive = false;
        servoClaw.setPosition(RobotConstants.CLAW_OPEN);
        servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
        servoElbowR.setPosition(RobotConstants.ELBOW_STANDBY);
        servoElbowL.setPosition(RobotConstants.ELBOW_ACTIVE);

        // -------------------------------------------------------------- IMU INIT

        telemetry.addData("Status", "CALIBRATING IMU...");
        telemetry.addData("Important Information", "PLACE ROBOT FACING AWAY FROM ALLIANCE BOX!");
        telemetry.update();

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        imu = hardwareMap.get(IMU.class, RobotConstants.HUB_IMU);
        imu.initialize(parameters);
        imu.resetYaw();

        Delay(500);
    }

    public void init() {
        telemetry.addData("Status", "INITIALIZING ROBOT...");    //
        telemetry.update();

        InitializeBlock();

        telemetry.setAutoClear(false);
        telemetry.addData("Status", "INITIALIZATION COMPLETE!");
        telemetry.update();
    }

    public void loop() {
        RuntimeConfig();
        Macros();

        // TELEMETRY
        telemetry.addData("Arm Left: ", armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", armR.getCurrentPosition());
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/

        telemetry.addData("Target Outtake Position: ", targetOuttakePosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        //telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();
    }
}
