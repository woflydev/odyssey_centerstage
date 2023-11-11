package org.firstinspires.ftc.teamcode.Deprecated.VeryDeprecated;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.testing.Robotv8_TestingOpMode;

@Deprecated()
// note: this class won't show up on the driver station, unless @TeleOp is added
public class NewRobot_v8_HangIntakeNoDrive_v1 extends Robotv8_TestingOpMode {
    private void RuntimeConfig() {
        // -------------------------------------------------------------- MANUAL ARM CONTROL (directly effects bot)
        if (adjustmentAllowed) { // lining up arm for topmost cone
            if (gamepad1.right_bumper) {
                servoHangR.setPower(1);
                servoHangL.setPower(1);
            } else if (gamepad1.left_bumper) {
                servoHangR.setPower(-1);
                servoHangL.setPower(-1);
            } else {
                servoHangR.setPower(0);
                servoHangL.setPower(0);
            }
        }

        double targetIntakePower = 0;
        if (gamepad1.left_trigger > 0.2) {
            targetIntakePower = 0.4;
            if (gamepad1.left_bumper) {
                targetIntakePower = 0.6;
            }
        }

        intake.setPower(targetIntakePower);

        // -------------------------------------------------------------- CONFIGURATION (don't directly move the bot)

        if (gamepad1.x && gamepad1.back) { // toggle red / blue alliance for FCD
            fieldCentricRed = !fieldCentricRed;
            Delay(50);
        }

        if (gamepad1.start) { // re-calibrate field centric drive
            imu.resetYaw();
        }
    }

    
    public void loop() {
        RuntimeConfig();

        // TELEMETRY
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/
        //telemetry.addData("Intake Speed: ", gamepad1.left_trigger);
        telemetry.addData("Target Outtake Position: ", targetOuttakePosition);
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        //telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();
    }
}
