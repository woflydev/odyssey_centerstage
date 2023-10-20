package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.drive.NewRobot_v9_Abstract;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

@TeleOp(name="Finalised RoadRunner TeleOp", group="Final")
public class NewRobot_v9_TeleOp extends Robotv8_Fullstack {
    NewRobot_v9_Abstract handler;
    public void init() {
        telemetry.addLine("Initialising...");
        telemetry.update();
        handler = new NewRobot_v9_Abstract(hardwareMap, telemetry);
        telemetry.addLine("Press Play to begin TeleOp: ");
        telemetry.update();
    }

    public void update() {
        // Optional, to get position feedback
        handler.update();

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

    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (adjustmentAllowed) { // lining up arm for topmost cone
            if (gamepad1.right_trigger >= 0.6 && ((armL.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition += RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false);
                }
            } else if (gamepad1.left_trigger >= 0.6 && ((armL.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (armR.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (targetOuttakePosition > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    targetOuttakePosition -= RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    UpdateOuttake(false);
                }
            } else if (gamepad1.dpad_down) {
                targetOuttakePosition = 30;
                UpdateOuttake(true);
            } else if (gamepad1.dpad_up) {
                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                UpdateOuttake(false);
            } /*else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                armR.setVelocity(0);
                armL.setVelocity(0);
            }*/

            if (gamepad1.square) {
                targetClawPosition -= 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            } else if (gamepad1.circle) {
                targetClawPosition += 0.02;
                servoClaw.setPosition(targetClawPosition);
                Delay(50);
            }

            if (gamepad1.right_bumper) {
                targetWristPosition += 0.02;
                servoWrist.setPosition(targetWristPosition);
                Delay(50);
            } else if (gamepad1.left_bumper) {
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
        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            fieldCentricRed = !fieldCentricRed;
            Delay(50);
        }

        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
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

        else if (gamepad1.dpad_right && gamepad1.left_bumper) {
            if (!transferStageDeployed) {
                transferStageDeployed = true;
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                Delay(500);
                // TODO: test if this reinforcement actually works
                servoClaw.setPosition(RobotConstants.CLAW_CLOSE);
                servoWrist.setPosition(RobotConstants.WRIST_ACTIVE);

                targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                MoveElbow(RobotConstants.ELBOW_ACTIVE);

                Delay(100);

                UpdateOuttake(false);
            } else {
                transferStageDeployed = false;
                servoClaw.setPosition(RobotConstants.CLAW_OPEN);
                Delay(300);

                targetOuttakePosition = RobotConstants.MIN_OUTTAKE_HEIGHT + 1;
                UpdateOuttake(false);
                MoveElbow(RobotConstants.ELBOW_STANDBY);
                servoWrist.setPosition(RobotConstants.WRIST_STANDBY);
            }
        } else if (gamepad1.right_stick_button) {
            handler.drive.followTrajectory(handler.TILE_TO_BACKDROP);
        } else if (gamepad1.left_stick_button) {
            handler.drive.followTrajectory(handler.BACKDROP_TO_TILE);
        }
    }

    private void UpdateOuttake(boolean reset) { // test new function
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
            armR.setVelocity(1700);
            armL.setVelocity(1700); // velocity used to be 1800, could be faster
        }
    }

    private void PassiveArmResetCheck() {
        if ((armL.getCurrentPosition() <= 15 && armR.getCurrentPosition() <= 15) && targetOuttakePosition <= 30) {
            armR.setVelocity(0);
            armL.setVelocity(0);
            resetTimer.reset();
        }
    }
}
