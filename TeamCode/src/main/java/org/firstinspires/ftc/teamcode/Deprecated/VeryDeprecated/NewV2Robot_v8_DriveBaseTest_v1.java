package org.firstinspires.ftc.teamcode.Deprecated.VeryDeprecated;

@Deprecated()
public class NewV2Robot_v8_DriveBaseTest_v1 extends Robotv8_FullstackTesting_v2 {
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
        /*telemetry.addData("Adjustment Allowed: ", adjustmentAllowed);
        telemetry.addData("Field Centric Mode : ", fieldCentricRed ? "RED" : "BLUE");
        telemetry.addData("Current Speed Mode: ", driveSpeedModifier == RobotConstants.BASE_DRIVE_SPEED_MODIFIER ? "BASE SPEED" : "PRECISION MODE");
        telemetry.addData("IMU Yaw: ", GetHeading());

        telemetry.update();*/
    }
}
