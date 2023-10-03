package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.drive.Robotv7.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv7.Robotv7;

public class NewRobot_v8_FullstackTest_v1 extends OpMode {
    private static final Robotv7 robot = new Robotv7();

    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (robot.adjustmentAllowed) { // lining up arm for topmost cone
            if (gamepad1.right_trigger >= 0.6 && ((robot.armL.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (robot.armR.getCurrentPosition() < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (robot.targetOuttakePosition < RobotConstants.MAX_OUTTAKE_HEIGHT - RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    robot.targetOuttakePosition += RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    NewUpdateOuttake(false);
                }
            } else if (gamepad1.left_trigger >= 0.6 && ((robot.armL.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) && (robot.armR.getCurrentPosition() > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT))) {
                if (robot.targetOuttakePosition > RobotConstants.MIN_OUTTAKE_HEIGHT + RobotConstants.ARM_ADJUSTMENT_INCREMENT) {
                    robot.targetOuttakePosition -= RobotConstants.ARM_ADJUSTMENT_INCREMENT;
                    NewUpdateOuttake(false);
                }
            } else if (gamepad1.dpad_down) {
                robot.targetOuttakePosition = 30;
                NewUpdateOuttake(true);
            } else if (gamepad1.dpad_up) {
                robot.targetOuttakePosition = RobotConstants.MAX_OUTTAKE_HEIGHT;
                NewUpdateOuttake(false);
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                robot.armR.setVelocity(0);
                robot.armL.setVelocity(0);
            }
        }

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            robot.fieldCentricRed = !robot.fieldCentricRed;
            robot.Delay(50);
        }

        if (gamepad1.start) { // re-calibrate field centric drive
            robot.imu.resetYaw();
        }

        else if ((gamepad1.left_trigger >= 0.25 && gamepad1.right_trigger >= 0.25) ||
                (gamepad2.left_trigger >= 0.25 && gamepad2.right_trigger >= 0.25)) {
            robot.driveSpeedModifier = (robot.driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER) ? RobotConstants.PRECISION_DRIVE_SPEED_MODIFIER : RobotConstants.BASE_DRIVE_SPEED_MODIFIER;
        }

        else {
            robot.driveSpeedModifier = RobotConstants.BASE_DRIVE_SPEED_MODIFIER;
        }
    }

    private void NewUpdateOuttake(boolean reset) { // test new function
        robot.armR.setTargetPosition(robot.targetOuttakePosition);
        robot.armL.setTargetPosition(robot.targetOuttakePosition);

        if (reset) {
            robot.armR.setTargetPosition(10);
            robot.armL.setTargetPosition(10);
            robot.targetOuttakePosition = 10;
            robot.armRuntime.reset();

            /*while (armM.getCurrentPosition() >= 50 || armRuntime.seconds() <= ARM_RESET_TIMEOUT) {
                armM.setVelocity((double)2100 / ARM_BOOST_MODIFIER);

                if (armM.getCurrentPosition() <= 50 || armRuntime.seconds() >= ARM_RESET_TIMEOUT) {
                    break;
                }
            }*/

            if ((robot.armL.getCurrentPosition() <= 15 || robot.armR.getCurrentPosition() <= 15) || robot.armRuntime.seconds() >= RobotConstants.ARM_RESET_TIMEOUT) {
                robot.armR.setVelocity(0);
                robot.armL.setVelocity(0);
            }

            telemetry.update();
        }

        else {
            robot.armRuntime.reset();
            robot.armR.setVelocity(3800);
            robot.armL.setVelocity(3800); // velocity used to be 1800, could be faster
        }
    }

    public void init() {
        robot.init();
    }

    public void loop() {
        RuntimeConfig();


        // TELEMETRY
        telemetry.addData("Arm Left: ", robot.armL.getCurrentPosition());
        telemetry.addData("Arm Right: ", robot.armR.getCurrentPosition());
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/

        telemetry.addData("Target Arm Position: ", robot.targetOuttakePosition);
        telemetry.addData("Adjustment Allowed: ", robot.adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", robot.fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", robot.driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        telemetry.addData("IMU Yaw: ", robot.GetHeading());

        telemetry.update();

    }
}
