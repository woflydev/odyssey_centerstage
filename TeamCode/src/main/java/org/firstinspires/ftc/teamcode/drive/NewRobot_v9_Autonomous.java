package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Abstract;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

@Autonomous(name="Final RoadRunner Autonomous", group="Final")
public class NewRobot_v9_Autonomous extends Robotv8_Fullstack {

    public void MainInit() {

    }

    public void MainStart() {
        //handler.initTask(2);
        // Go back to starting position
        drive.followTrajectory(handler.path(Robotv8_Abstract.STARTING_POSE, Robotv8_Abstract.TILE_LOCATION));
    }

    public void MainLoop() {
        // Get Pixel
        // Go to
    }
}
