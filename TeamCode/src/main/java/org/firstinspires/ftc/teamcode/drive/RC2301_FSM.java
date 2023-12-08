package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.FSM_TeleOp_Fullstack;

@TeleOp(name="FINAL TeleOp FSM v3", group="!RC-1.0.0")
public class RC2301_FSM extends FSM_TeleOp_Fullstack {
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
