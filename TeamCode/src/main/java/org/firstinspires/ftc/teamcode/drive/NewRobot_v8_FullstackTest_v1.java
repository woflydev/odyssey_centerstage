package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv7.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv7.Robotv7_Fullstack;

@TeleOp()
public class NewRobot_v8_FullstackTest_v1 extends Robotv7_Fullstack {
    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (adjustmentAllowed) { // lining up arm for topmost cone
            if (gamepad2.right_trigger >= 0.6 && ((armL.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition += RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false, 0);
                }
            } else if (gamepad2.left_trigger >= 0.6 && ((armL.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition -= RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false, 0);
                }
            } else if (gamepad2.dpad_down) {
                targetOuttakePosition = 30;
                UpdateOuttake(true, 0);
            } else if (gamepad2.dpad_up) {
                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                UpdateOuttake(false, 0);
            } /*else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }*/

            if (gamepad2.square) {
                targetClawPosition -= 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad2.circle) {
                targetClawPosition += 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            if (gamepad2.right_bumper) {
                targetWristPosition += 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad2.left_bumper) {
                targetWristPosition -= 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            }

            if (gamepad1.triangle) {
                targetElbowPosition += 0.02;
                MoveElbow(targetElbowPosition);
            } else if (gamepad1.cross) {
                targetElbowPosition -= 0.02;
                MoveElbow(targetElbowPosition);
            }

            if (gamepad2.cross) {
                if (!planeTriggered) {
                    planeTriggered = true;
                    servoPlane.setPosition(RobotConstants.PLANE_ACTIVE);
                } else {
                    planeTriggered = false;
                    servoPlane.setPosition(RobotConstants.PLANE_STANDBY);
                }
                Delay(250);
            }
        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        /*if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            fieldCentricRed = !fieldCentricRed;
            Delay(50);
        }*/

        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
        }
    }

    private void Macros() {
        // test transfer stage macro
        if (gamepad1.dpad_left) {
            if (!wristActive) {
                wristActive = true;
                servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
                Delay(500);
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                Delay(100);
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

            } else {
                wristActive = false;
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                Delay(200);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
            }
            Delay(200);
        }

        else if (gamepad1.dpad_right && gamepad1.left_bumper) {
            if (!transferStageDeployed) {
                transferStageDeployed = true;
                servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
                Delay(300);
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                Delay(300);
                // TODO: test if this reinforcement actually works
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

                MoveElbow(RobotConstants.ELBOW_ACTIVE);

                Delay(100);

                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                UpdateOuttake(false, 0);
            } else {
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                targetOuttakePosition = RobotConstants.MIN_OUTTAKE_HEIGHT;
                UpdateOuttake(false, 800);
                Delay(1300);
                MoveElbow(RobotConstants.ELBOW_STANDBY);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
                transferStageDeployed = false;
            }
        }
    }

    private void UpdateOuttake(boolean reset, double delay) { // test new function
        if (reset) {
            Delay(delay);
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
            Delay(delay);
            armR.setTargetPosition(targetOuttakePosition);
            armL.setTargetPosition(targetOuttakePosition);
            armRuntime.reset();
            armR.setVelocity(2400);
            armL.setVelocity(2400); // velocity used to be 1800, could be faster
        }
    }

    private void PassiveArmResetCheck() {
        if ((armL.getCurrentPosition() <= 15 && armR.getCurrentPosition() <= 15) && targetOuttakePosition <= 30) {
            armR.setVelocity(0);
            armL.setVelocity(0);
            resetTimer.reset();
        }
    }

    public void loop() {
        PassiveArmResetCheck();
        RuntimeConfig();
        Macros();

        // TELEMETRY
        telemetry.addData("Arm Left: ", armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", armR.getCurrentPosition());
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/
        telemetry.addData("Target Wrist Position: ", targetWristPosition);
        telemetry.addData("Target Elbow Position: ", targetElbowPosition);
        telemetry.addData("Target Outtake Position: ", targetOuttakePosition);
        telemetry.addData("Target Claw Position: ", targetClawPosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        //telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();
    }
}
