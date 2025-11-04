package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;


// Non-RR imports
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;




@Config
@Autonomous(name = "RoadRunnerFirstFile", group = "Autonomous")


public class FirstRoadRunnerFile extends LinearOpMode {
    public double squareSize = 20;

    @Override
    public void runOpMode() {
        // instantiate your MecanumDrive at a particular pose.
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Action trajectoryAction = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(0, squareSize))
                .strafeTo(new Vector2d(squareSize, squareSize))
                .waitSeconds(.5)
                .strafeTo(new Vector2d(squareSize, 0))
                .strafeTo(new Vector2d(0, 0))
                .waitSeconds(1)
                .turn(Math.toRadians(90))

                //.splineToLinearHeading(new Pose2d(squareSize, squareSize, Math.toRadians(-40)), 1)
                .build(); // <-- Thi

        waitForStart();

        // This is how you run the action you just built.
        if (opModeIsActive()) {
            Actions.runBlocking(trajectoryAction);
        }







    }

}

