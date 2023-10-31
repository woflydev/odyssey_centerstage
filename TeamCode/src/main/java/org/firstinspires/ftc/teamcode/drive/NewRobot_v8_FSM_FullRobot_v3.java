package org.firstinspires.ftc.teamcode.drive;

import android.media.Image;

import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_FSM_FullstackTesting;

@TeleOp(name="TeleOp FSM v3", group="RC-1.0.0")
public class NewRobot_v8_FSM_FullRobot_v3 extends Robotv8_FSM_FullstackTesting {
    public void MainLoop() {
        // note: intake, manual controls
        RuntimeConfig();

        // note: FSMs
        DrivetrainSubsystem();
        OuttakeSubsystem();
        PlaneLauncherSubsystem();

        // note: redundancies
        PassiveArmResetCheck();

        // note: telemetry is handled in FSM_FullStackTesting
    }
}
