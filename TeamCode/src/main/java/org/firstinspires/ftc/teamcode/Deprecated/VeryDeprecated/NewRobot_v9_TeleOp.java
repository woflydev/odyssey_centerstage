package org.firstinspires.ftc.teamcode.Deprecated.VeryDeprecated;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.Robotv8.Fullstack;

//@TeleOp(name="Finalised RoadRunner TeleOp", group="Final")
@Disabled()
@Deprecated()
public class NewRobot_v9_TeleOp extends Fullstack {
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
