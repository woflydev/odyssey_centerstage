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
        Macros(handler);

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
