package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_FSM_FullstackTesting;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_FullstackTesting;

@TeleOp()
public class NewRobot_v8_FSM_FullRobot_v3 extends Robotv8_FSM_FullstackTesting {
    public void MainLoop() {
        Mecanum();
        RuntimeConfig();
        OuttakeSubsystem();
        PassiveArmResetCheck();

        // telemetry is handled in FSM_FullStackTesting
    }
}
