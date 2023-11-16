package org.firstinspires.ftc.teamcode.drive.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.Robotv8.RobotInfo.RobotAlliance;
import org.firstinspires.ftc.teamcode.drive.vision2.PropDetection;

@Autonomous(name="Vision Portal Test", group="Test")

public class VisionPortalTest extends LinearOpMode {
    public void runOpMode() {
        PropDetection detector = new PropDetection(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                RobotAlliance.RED,
                telemetry
        );
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetry.addData("Prop location: ", detector.getLocation());
            }
        }
    }
}
