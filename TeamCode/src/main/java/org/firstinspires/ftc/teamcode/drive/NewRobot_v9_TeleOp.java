package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

@TeleOp(name="Finalised RoadRunner TeleOp", group="Final")
public class NewRobot_v9_TeleOp extends Robotv8_Fullstack {
    public void MainStart() {
        telemetry.addLine("Started!!!");
        telemetry.update();
        Delay(5000);
    }
    public void MainLoop() {
        PassiveArmResetCheck();
        RuntimeConfig();
        Mecanum();
    }
}
