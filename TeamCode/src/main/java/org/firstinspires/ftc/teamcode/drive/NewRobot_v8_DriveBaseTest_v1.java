package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_FullstackTesting;

@TeleOp()
public class NewRobot_v8_DriveBaseTest_v1 extends Robotv8_FullstackTesting {
    public void MainLoop() {
        Mecanum();
        RuntimeConfig();
        //Add drive.followTrajectory stuff here

        if (gamepad1.start) {
            imu.resetYaw();
        }

        // TELEMETRY
        /*telemetry.addData("FrontRM Encoder Value: ", frontRM.getCurrentPosition());
        telemetry.addData("FrontLM Encoder Value: ", frontLM.getCurrentPosition());
        telemetry.addData("BackRM Encoder Value: ", backRM.getCurrentPosition());
        telemetry.addData("BackLM Encoder Value: ", backLM.getCurrentPosition());*/
        telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();
    }
}
