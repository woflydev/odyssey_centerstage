package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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
        if (gamepad1.dpad_right && gamepad1.left_bumper) {
            GrabAndDeposit(RobotConstants.MAX_OUTTAKE_HEIGHT); // thi function handles both grab and deposit, but requires two presses for each process
        } else if (gamepad1.dpad_down && gamepad1.left_bumper) {
            GrabAndDeposit(RobotConstants.JUNCTION_LOW);
        } else if (gamepad1.dpad_left && gamepad1.left_bumper) {
            GrabAndDeposit(RobotConstants.JUNCTION_MID);
        } else if (gamepad1.dpad_up && gamepad1.left_bumper) {
            GrabAndDeposit(RobotConstants.JUNCTION_HIGH);
        }
    }

    private void GrabAndDeposit(int height) {
        if (!transferStageDeployed) {
            transferStageDeployed = true;
            servoWrist.setPosition(RobotConstants.WRIST_PICKUP);
            MoveElbow(RobotConstants.ELBOW_PICKUP);
            Delay(200);
            MoveElbow(RobotConstants.ELBOW_STANDBY);
            Delay(200);
            servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
            Delay(300);
            // TODO: test if this reinforcement actually works
            servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
            servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

            MoveElbow(RobotConstants.ELBOW_ACTIVE);

            Delay(100);

            targetOuttakePosition = height;
            UpdateOuttake(false, 0);
        } else {
            servoClaw.setPosition(RobotConstants.CLAW_OPEN);
            targetOuttakePosition = RobotConstants.MIN_OUTTAKE_HEIGHT;
            UpdateOuttake(true, 300);
            Delay(350); // elbow should come down after the slide is near done
            MoveElbow(RobotConstants.ELBOW_STANDBY);
            servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
            transferStageDeployed = false;
        }
    }

    private void UpdateOuttake(boolean reset, double delay) { // test new function
        if (reset) {
            Delay(delay);
            armR.setTargetPosition(10);
            armL.setTargetPosition(10);
            targetOuttakePosition = 10;
            armRuntime.reset();
            armR.setVelocity(2800);
            armL.setVelocity(2800);
            /*while (armM.getCurrentPosition() >= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

                if (armM.getCurrentPosition() <= 50 || armRuntime.seconds() >= ARM_RESET_TIMEOUT) {
                    break;
                }
            }*/

            if ((armL.getCurrentPosition() <= 20 || armR.getCurrentPosition() <= 20) || armRuntime.seconds() >= RobotConstants.ARM_RESET_TIMEOUT) {
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
            armR.setVelocity(2800);
            armL.setVelocity(2800); // velocity used to be 1800, could be faster
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
