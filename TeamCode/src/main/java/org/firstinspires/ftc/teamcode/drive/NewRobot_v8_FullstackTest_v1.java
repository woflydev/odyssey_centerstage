package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.Robotv7.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv7.Robotv7;

public class NewRobot_v8_FullstackTest_v1 extends OpMode {
    private static final Robotv7 r = new Robotv7();

    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (r.adjustmentAllowed) { // lining up arm for topmost cone
            if (gamepad1.right_trigger >= 0.6 && ((r.armL.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (r.armR.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (r.targetOuttakePosition < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    r.targetOuttakePosition += RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    NewUpdateOuttake(false);
                }
            } else if (gamepad1.left_trigger >= 0.6 && ((r.armL.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (r.armR.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (r.targetOuttakePosition > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    r.targetOuttakePosition -= RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    NewUpdateOuttake(false);
                }
            } else if (gamepad1.dpad_down) {
                r.targetOuttakePosition = 30;
                NewUpdateOuttake(true);
            } else if (gamepad1.dpad_up) {
                r.targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                NewUpdateOuttake(false);
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                r.armR.setVelocity(0);
                r.armL.setVelocity(0);
            }

            if (gamepad1.square) {
                r.targetClawPosition -= 0.05;
                r.servoClaw.setPosition(r.targetClawPosition);
                r.Delay(50);
            } else if (gamepad1.circle) {
                r.targetClawPosition += 0.05;
                r.servoClaw.setPosition(r.targetClawPosition);
                r.Delay(50);
            }

            if (gamepad1.right_bumper) {
                r.targetWristPosition += 0.05;
                r.servoWrist.setPosition(r.targetWristPosition);
                r.Delay(50);
            } else if (gamepad1.left_bumper) {
                r.targetWristPosition -= 0.05;
                r.servoWrist.setPosition(r.targetWristPosition);
                r.Delay(50);
            }
        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            r.fieldCentricRed = !r.fieldCentricRed;
            r.Delay(50);
        }

        if (gamepad1.start) { // re-calibrate field centric drive
            r.imu.resetYaw();
        }

        else if ((gamepad1.left_trigger >= 0.25 && gamepad1.right_trigger >= 0.25) ||
                (gamepad2.left_trigger >= 0.25 && gamepad2.right_trigger >= 0.25)) {
            r.driveSpeedModifier = (r.driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER) ? RobotConstants.PRECISION_DRIVE_SPEED_MODIFIER : RobotConstants.BASE_DRIVE_SPEED_MODIFIER;
        }

        else {
            r.driveSpeedModifier = RobotConstants.BASE_DRIVE_SPEED_MODIFIER;
        }
    }

    private void NewUpdateOuttake(boolean reset) { // test new function
        r.armR.setTargetPosition(r.targetOuttakePosition);
        r.armL.setTargetPosition(r.targetOuttakePosition);

        if (reset) {
            r.armR.setTargetPosition(10);
            r.armL.setTargetPosition(10);
            r.targetOuttakePosition = 10;
            r.armRuntime.reset();

            /*while (armM.getCurrentPosition() >= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

                if (armM.getCurrentPosition() <= 50 || armRuntime.seconds() >= ARM_RESET_TIMEOUT) {
                    break;
                }
            }*/

            if ((r.armL.getCurrentPosition() <= 15 || r.armR.getCurrentPosition() <= 15) || r.armRuntime.seconds() >= RobotConstants.ARM_RESET_TIMEOUT) {
                r.armR.setVelocity(0);
                r.armL.setVelocity(0);
            }

            telemetry.update();
        }

        else {
            r.armRuntime.reset();
            r.armR.setVelocity(3800);
            r.armL.setVelocity(3800); // velocity used to be 1800, could be faster
        }
    }

    public void init() {
        r.init();
    }

    public void loop() {
        RuntimeConfig();


        // TELEMETRY
        telemetry.addData("Arm Left: ", r.armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", r.armR.getCurrentPosition());
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/

        telemetry.addData("Target Arm Position: ", r.targetOuttakePosition);
        telemetry.addData("Adjustment Allowed: ", r.adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", r.fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", r.driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        telemetry.addData("IMU Yaw: ", r.GetHeading());

        telemetry.update();

    }
}
