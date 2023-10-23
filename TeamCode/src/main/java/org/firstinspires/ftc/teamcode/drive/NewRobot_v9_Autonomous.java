package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

@Autonomous(name="Final RoadRunner Autonomous", group="Final")
public class NewRobot_v9_Autonomous extends Robotv8_Fullstack {

    public void start() {
        handler.initTask(2);
    }

    public void MainLoop() {
        PassiveArmResetCheck();
        RuntimeConfig();
        Macros(handler);
    }
}
