/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Collections;

@Config
@TeleOp

public class CleanTeleop extends LinearOpMode {

    // Setting up drivetrain file



    //setting up shooter servo stuff
    private Servo ServoShooter1;
    private Servo ServoShooter2;
    public static double startPoint = .2;
    public static double endPoint = .70;
    private static double shooterAngle = startPoint;
    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx IntakeMotor = null;
    private DcMotor StopIntakeMotor = null;
    private DcMotorEx ShooterMotor = null;
    private DcMotorEx ShooterMotor2 = null;
    private CRServo BallFeederServo = null;
    private Servo ShooterRotatorServo = null;
    private IMU imu;

    //starting angle
    private double startingAngleRad = Math.toRadians(0);

    //april tag stuff
    private double AprilTagBearing = 0;
    private double AprilTagIdTeam = 20;//20 blue | 24 red
    //private boolean SelfAimToggle = true;
    public static double currentAngle = 0;
    public static double AmountOfMovement = 0.005;

    //Variables for statement printing
    public static double ShooterMotorSpeed = .8;
    double shooterVelocity = 0; // Ticks per second


    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off
    ArrayList<Double> LastValuesList = new ArrayList<>();
    //private boolean wasXButtonPressed = false;   // Tracks the button's state from the last loop
    // ------------

    // --- D-pad tracking ---
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

// ---------------------------------------

    @Override
    public void runOpMode() {
        InitializeIMU();
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
        ShooterRotatorServo = hardwareMap.get(Servo.class, "ShooterRotatorServo");

        ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
        ServoShooter2 = hardwareMap.get(Servo.class, "ServoShooter2");


        // run shooter with encoder

        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //whats supposed to be in initialize hardware

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, startingAngleRad));

        //vission !!!
        AprilTagVision vision = new AprilTagVision(hardwareMap, "Webcam"); // <-- Use your webcam name!

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        imu.resetYaw();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            vision.update();
            telemetry.addData("Tag Visible", vision.isTagVisible());

            if (vision.isTagVisible()) {
                //telemetry.addData("ID", vision.getID());
                //telemetry.addData("Range", "%.2f in", vision.getRange());
                //telemetry.addData("Yaw", "%.2f deg", vision.getYaw());
                //telemetry.addData("X Offset", "%.2f in", vision.getX());
                AprilTagBearing = vision.getBearing();
            } else {
                //telemetry.addData("ID", "None");
                AprilTagBearing = 0;
            }

            //handleDriving();
            double speed = .5;
            //if (gamepad1.right_trigger ==1){speed = 1;}
            speed += gamepad1.right_trigger/2;

            double axial = -gamepad1.left_stick_y *speed;
            double lateral = gamepad1.left_stick_x*speed; // Note: pushing stick forward gives negative value
            double yaw = gamepad1.right_stick_x*speed;

            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            /*double fieldCentricAxial = axial * Math.cos(robotHeading) - lateral * Math.sin(robotHeading);
            double fieldCentricLateral = axial * Math.sin(robotHeading) + lateral * Math.cos(robotHeading);*/ // this is for field centric

            //robotDrive.move(axial, lateral, yaw, speed);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(axial,-lateral),-yaw));
            /*drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(fieldCentricAxial,fieldCentricLateral),-yaw)); */ // this is for field centric

            // whats supposed to be in handle driving ^^

            handleIntake();
            handleShooter();
            handleShooterServos();
            handleShooterRotation(AprilTagBearing,vision.isTagVisible());

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            Pose2d pose = drive.localizer.getPose();
            //telemetry.addData("Heading ( Encoders prob )= ", Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Heading ( IMU )= ", Math.toDegrees(robotHeading));
            telemetry.addData("ShooterPower= ", ShooterMotorSpeed);
            telemetry.addData("ShooterMotorTickPerSecond= ", shooterVelocity);
            //telemetry.addData("Intake speed =  ", intakeVelocity);

            telemetry.update();
        }
    }

    private void InitializeIMU() {
        // Drivetrain & IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure the IMU. This is critical for field-centric drive.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

    }

    private void handleShooter(){
        shooterVelocity = ShooterMotor.getVelocity(); // Ticks per second

        //boolean isXPressed = gamepad2.x;
        double shooterSpeedRange = 100000;

        // 2. Check if the button was JUST pressed (it was up, but is now down)
        if (gamepad2.xWasPressed()){//(isXPressed && !wasXButtonPressed) {
            // Flip the state: if it was on, turn it off; if it was off, turn it on.
            shooterMotorOn = !shooterMotorOn;
        }
        // 3. Update the tracking variable for the next loop
        //wasXButtonPressed = isXPressed;

        // SHOOTING VARIABLE MECHANICS START
        boolean isDpadUpPressed = gamepad2.dpad_up;
        boolean isDpadDownPressed = gamepad2.dpad_down;

// Check if D-pad down was JUST pressed (rising edge detection)
        if (isDpadDownPressed && !wasDpadDownPressed && ShooterMotorSpeed > 0) {
            ShooterMotorSpeed -= 0.05;
        }
// Check if D-pad up was JUST pressed
        else if (isDpadUpPressed && !wasDpadUpPressed && ShooterMotorSpeed < 1) {
            ShooterMotorSpeed += 0.05;
        }

// VERY IMPORTANT: Update the tracking variables for the next loop cycle
        wasDpadUpPressed = isDpadUpPressed;
        wasDpadDownPressed = isDpadDownPressed;

        if (shooterMotorOn) {

            ShooterMotor.setPower(-1 * ShooterMotorSpeed);
            ShooterMotor2.setPower(ShooterMotorSpeed);

        }else{
            ShooterMotor.setPower(0);
            ShooterMotor2.setPower(0);
        }
        //stabilizing values to know when to shoot
        LastValuesList.add(shooterVelocity);
        if (LastValuesList.size() > 4) {
            LastValuesList.remove(0);
        }
        if (!LastValuesList.isEmpty()) {
            // Use the Collections utility to find the min and max values directly
            double minVal = Collections.min(LastValuesList);
            double maxVal = Collections.max(LastValuesList);

            // Calculate the range and store it
            shooterSpeedRange = maxVal - minVal;
        } else {
            shooterSpeedRange = 9999999; // If the list is empty, the range is 0
        }

        telemetry.addData("ShooterSpeedRangeForStabilizing", shooterSpeedRange );

    }

    //note to self error occuring in handleShooterRotation idk why.
    private void handleShooterRotation(double bearing, boolean tag){

        if (gamepad2.dpad_left && currentAngle < 1){
            currentAngle += .005;
        }
        if (gamepad2.dpad_right && currentAngle > 0){
            currentAngle -= .005;
        }

        if (Math.abs(bearing)>2 && Math.abs(bearing)<30 && gamepad2.b){

            if (bearing < -2 && currentAngle < 1){
                currentAngle +=AmountOfMovement;
            }
            if (bearing > 2 && currentAngle > 0){
                currentAngle -=AmountOfMovement;
            }
        }

        ShooterRotatorServo.setPosition(currentAngle);

        telemetry.addData("current angle", currentAngle );
        telemetry.addData("bearing", bearing );


       /* if (gamepad1.a && gamepad2.dpad_left || gamepad2.dpad_right){
            SelfAimToggle = !SelfAimToggle;
        }
        if (!SelfAimToggle) {


            if (gamepad2.dpad_right ) {
               currentAngle = -90;
            }
            if (gamepad2.dpad_left) {
                currentAngle = 90;
            }
        }



        ShooterRotatorServo.setPosition((90+currentAngle)*1/180);
        //ShooterRotatorServo.setPosition((90)*1/180);
        */

    }

    private void handleIntake(){
        //double intakeVelocity = IntakeMotor.getVelocity(); // Ticks per second
        double TriggerValue =0;

        if (gamepad2.left_trigger > gamepad1.left_trigger){TriggerValue = gamepad2.left_trigger;}
        if (gamepad1.left_trigger > gamepad2.left_trigger){TriggerValue = gamepad1.left_trigger;}
        if (gamepad2.left_trigger == gamepad1.left_trigger){TriggerValue = gamepad2.left_trigger;}

        double intake=0;
        if (gamepad2.right_bumper){ // to spit out balls
            intake = .5;
        }
        else { // to intake balls
            intake = -TriggerValue;
        }
        //stuff so u cant reverse suddently used to use encoder, but now its gone. can only go forwards
        /*if ( intakeVelocity >= 0 && gamepad2.left_bumper){
            intake = TriggerValue;
        }
        else if (intakeVelocity <= 0){
            intake = -TriggerValue;
        }*/

        double StopIntake = gamepad2.right_trigger;

        IntakeMotor.setPower(intake);
        StopIntakeMotor.setPower(StopIntake);

        //handle feeder to launcher
        if (gamepad2.right_trigger > 0){BallFeederServo.setPower(gamepad2.right_trigger);}
        else if (gamepad2.right_bumper){BallFeederServo.setPower(-1);}
        else{BallFeederServo.setPower(0);}

    }

    private void handleShooterServos(){

        // Initialize shooterAngle with the servo's current position to start.
        // This ensures it always has a value.


        if (gamepad1.a) {
            shooterAngle = startPoint;
        } else if (gamepad1.b) {
            shooterAngle = endPoint;
        }

        // Now, shooterAngle is guaranteed to have a value,
        // so the compiler will be happy.
        ServoShooter1.setPosition(1 - shooterAngle);
        ServoShooter2.setPosition(shooterAngle);
    }
    }


