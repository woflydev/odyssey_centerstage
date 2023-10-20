package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.NewRobot_v9_Abstract;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotConstants;
import org.firstinspires.ftc.teamcode.drive.Robotv8.Robotv8_Fullstack;

@Autonomous(name="Final RoadRunner Autonomous", group="Final")
public class NewRobot_v9_Autonomous extends LinearOpMode {
    NewRobot_v9_Abstract handler;
    public void runOpMode() {
        telemetry.addLine("Initialising...");
        telemetry.update();
        handler = new NewRobot_v9_Abstract(hardwareMap, telemetry);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                // Optional, to see position output
                handler.update();
            }
        }
    }
}
