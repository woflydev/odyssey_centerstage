package org.firstinspires.ftc.teamcode.drive;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.NewRobot_v8_Abstract;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

import android.annotation.SuppressLint;


@TeleOp(name = "Concept: AutoTeleOp_V1", group = "Concept")
public class NewRobot_v8_AutoTeleOp_V1 extends Robotv8_Fullstack {

    NewRobot_v8_Abstract handler;

    @SuppressLint("DefaultLocale")
    public void init() {
        telemetry.addLine("Initialising...");
        telemetry.update();
        handler = new NewRobot_v8_Abstract(this, hardwareMap, telemetry);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
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

        // Share the CPU.
        //sleep(SLEEP_TIME);
    }

    public void stop() {
        handler.stop();
    }
}

