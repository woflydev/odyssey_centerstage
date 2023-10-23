package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.testing.Robotv8_FullstackTesting;

@TeleOp()
public class NewRobot_v8_FullRobot_v2 extends Robotv8_FullstackTesting {
    public void loop() {
        double targetIntakePower = 0;
        if (gamepad1.left_trigger > 0.2) {
            targetIntakePower = 0.4;
            if (gamepad1.left_bumper) {
                targetIntakePower = 0.6;
            }
        }
        intake.setPower(targetIntakePower);

        PassiveArmResetCheck();
        RuntimeConfig();
        Mecanum();
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
        telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();
    }
}
