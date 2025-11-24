package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "PerfectTurning", group = "Autonomous")
public class PerfectTurning extends LinearOpMode {
    public static double squareSize = 48;

    private IMU imu;




    @Override
    public void runOpMode() {

        //initialization

        imu = hardwareMap.get(IMU.class, "imu");

        // Configure the IMU. This is critical for field-centric drive.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        imu.resetYaw(); // Zero the IMU heading at the start of the match



        Pose2d initialPose = new Pose2d(0, 0, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        // Your full action sequence
        Action trajectoryAction = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(0, 24))

                .waitSeconds(2)


                .build();

        waitForStart();

        if (isStopRequested()) return;

        // The issue was in this part of the code.
        // Instead of Actions.run(trajectoryAction), you should call run on the action itself.
        while (opModeIsActive() && trajectoryAction.run(new TelemetryPacket())) {
            // This updates the robot's current pose using the localizer/IMU
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            Pose2d pose = drive.localizer.getPose();
            drive.updatePoseEstimate();



            // Reset Yaw TO Right one whcih is imu
            //drive.updatePoseEstimate(new Pose2d(correctedPose));

            // Telemetry Updates (Real-Time Data)
            telemetry.addData("STATUS", "EXECUTING ACTION");
            telemetry.addData("X", pose.position.x);
            telemetry.addData("Y", pose.position.y);
            // Convert heading to degrees for easy reading
            telemetry.addData("Heading (Deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("True Heading (Deg)", Math.toDegrees(robotHeading));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        // 3. Optional: Display final pose after action is complete
        telemetry.addData("STATUS", "Action Complete!");
        //telemetry.addData("Final Heading (Deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();
        sleep(5000); // Pause to view final result
    }
}
