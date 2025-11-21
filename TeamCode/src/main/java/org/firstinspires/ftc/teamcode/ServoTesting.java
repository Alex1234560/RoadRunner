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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous

public class ServoTesting extends LinearOpMode {

    // Setting up drivetrain file

    //setting up motors and time

    private Servo ServoShooter1;
    private Servo ServoShooter2;
    private DcMotorEx ShooterMotor1 = null;
    private DcMotorEx ShooterMotor2 = null;




    //public static double DirectionMultiplierForServo2=  0;
    public static double ShooterMotorSpeed = .05;
    public static double startPoint = 0;
    public static double endPoint = 1;


    @Override
    public void runOpMode() {
        //Start a drive class to be able to use wheels

        ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
        ServoShooter2 = hardwareMap.get(Servo.class, "ServoShooter2");
        ShooterMotor1 = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        ShooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.x){
                ShooterMotor1.setPower(ShooterMotorSpeed);
                ShooterMotor2.setPower(ShooterMotorSpeed*-1);
            } else {
                ShooterMotor1.setPower(0);
                ShooterMotor2.setPower(0);
            }




            double ShooterAngle = ((endPoint-startPoint)*gamepad1.right_trigger) + startPoint;


            ServoShooter1.setPosition(1-ShooterAngle);
            ServoShooter2.setPosition(ShooterAngle);


            // Show the elapsed game time and wheel power.
            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Angle Servo 1= ", 1-ShooterAngle );
            //telemetry.addData("Angle Servo 2= ", (ShooterAngle  ));

            //telemetry.addData("Intake speed =  ", intakeVelocity);

            telemetry.update();
        }
    }
}

