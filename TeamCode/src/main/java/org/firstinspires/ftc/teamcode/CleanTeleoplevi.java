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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.HashMap;

@Config
@TeleOp
@Disabled

public class CleanTeleoplevi extends LinearOpMode {

    // Setting up drivetrain file



    //setting up shooter servo stuff
    private Servo ServoShooter1;
    private Servo ServoShooter2;
    public static double startPoint = .30;
    public static double endPoint = .75;//.7
    private static double ShooterAngle = startPoint;
    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx IntakeMotor = null;
    private DcMotor StopIntakeMotor = null;
    private DcMotorEx ShooterMotor = null;
    private DcMotorEx ShooterMotor2 = null;
    private CRServo BallFeederServo = null;
    private CRServo ServoHelper = null;
    private Servo ShooterRotatorServo = null;
    private IMU imu;

    // angle for hooded shooter
    double HoodAngle = 0;// value from 0 to 1.0

    //shooter rotatr
    /*public static double SlowAmountOfMovement = 0.0025;
    public static double FastAmountOfMovement = 0.007;
    public static double AmountOfMovement = 0.0025;*/

    //starting angle
    private double startingAngleRad = Math.toRadians(0);

    //april tag stuff
    private double AprilTagBearing = 0;
    public static double AprilTagIdTeam = 20;//20 blue | 24 red
    private double rotationTolerance = 1.5;
    public static double rotationCompensation = .09;
    //private boolean SelfAimToggle = true;
    public static double currentAngle = 90;


    //Variables for statement printing
    private static double ShooterMotorSpeed = 0;
    public static double GoalShooterMotorTPS = 1200;// rotation ticks per seond
    //variables belof for testing
    public static double SpeedFast = 0.01;
    public static double SpeedNormal = 0.005;
    public static double SpeedPrecise = 0.003;
    public static double BigRange = 400;
    public static double MediumRange = 200;
    public static double SmallRange = 80;
    //variables avobe for testing
    double shooterTPS = 0; // Ticks per second



    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off
    ArrayList<Double> LastValuesList = new ArrayList<>();



// ---------------------------------------

    @Override
    public void runOpMode() {
        InitializeIMU();
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
        ServoHelper = hardwareMap.get(CRServo.class, "ServoHelper");
        ShooterRotatorServo = hardwareMap.get(Servo.class, "ShooterRotatorServo");
        ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
        ServoShooter2 = hardwareMap.get(Servo.class, "ServoShooter2");

        ServoShooter1.setDirection(Servo.Direction.REVERSE);


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
                telemetry.addData("ID", vision.getID());
                if (vision.getID()==AprilTagIdTeam) {
                    telemetry.addData("Range", "%.2f in", vision.getRange());
                    //telemetry.addData("Yaw", "%.2f deg", vision.getYaw());
                    //telemetry.addData("X Offset", "%.2f in", vision.getX());
                    AprilTagBearing = vision.getBearing();
                }
            } else {
                telemetry.addData("ID", "None");

                //AprilTagBearing = 0;
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
            HandleCloseButton();

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            Pose2d pose = drive.localizer.getPose();
            //telemetry.addData("Heading ( Encoders prob )= ", Math.toDegrees(pose.heading.toDouble()));
            //telemetry.addData("Heading ( IMU )= ", Math.toDegrees(robotHeading));
            telemetry.addData("ShooterPower= ", ShooterMotorSpeed);
            telemetry.addData("GoalShooterPower= ", GoalShooterMotorTPS);
            telemetry.addData("ShooterMotorTickPerSecond= ", shooterTPS);
            telemetry.addData("x,y,heading ", pose.position.x + ", " + pose.position.y + ", heading: " + pose.heading.toDouble());


            telemetry.update();
        }
    }

    private void HandleCloseButton(){
        if (gamepad2.left_bumper) {
            currentAngle = 90;
            ShooterMotorSpeed = .6;
            ShooterAngle=.3;
        }
    }


/*
    private int[] getGoals(int curRange){

        int[] turretGoals = new int[2];

        //Formatted in angle(degrees), speed
        int[][] dataCollection = {
            //Data point 1
            {1, 2},
            //Data point 2
            {1, 2},
            //Data point 3
            {1, 2},
        };
        int [] ranges = {
            //Data point 1 range
            1,
            //Data point 2 range
            2,
            //Data point 3 range
            3,
        };


        if (curRange > ranges[ranges.length - 1]) {
            turretGoals[0] = dataCollection[dataCollection.length -1][0];
            turretGoals[1] = dataCollection[dataCollection.length - 1][1];
            return turretGoals;
        } else {
            for (int i = 0; i < ranges.length; i++) {
                if (curRange == ranges[i]) {
                turretGoals[0] = dataCollection[i][0];
                turretGoals[1] = dataCollection[i][1];
                return turretGoals;
                } else if (curRange < ranges[i]) {
                    if (i == 0) {
                        //Set to minimum
                        turretGoals[0] = dataCollection[0][0];
                        turretGoals[1] = dataCollection[0][1];
                        return turretGoals;
                    } else {
                        //get weighted averages for the points
                        int firstRange = ranges[i - 1];
                        int secondRange = ranges[i];

                        double secondWeight = (double)(curRange - firstRange) / (secondRange - firstRange);
                        double firstWeight = 1.0 - secondWeight;

                        double firstAW = dataCollection[i -1][0] * firstWeight;
                        double secondAW = dataCollection[i][0] * secondWeight;
                        double targAngle = (firstAW + secondAW);

                        double firstSW = dataCollection[i -1][1] * firstWeight;
                        double secondSW = dataCollection[i][1] * secondWeight;
                        double targSpeed = (firstSW + secondSW);

                        turretGoals[0] = (int)targAngle;
                        turretGoals[1] = (int)targSpeed;
                        return turretGoals;
                        }
                    }
                }
            }
        return turretGoals;
    }

 */

    private void HandleShootingRanges(int curRange){

        //int[] Goals = getGoals(curRange);

        //if (gamepad2.left_bumper) {
        //            currentAngle = 90;
        //            ShooterMotorSpeed = .6;
        //            ShooterAngle=.3;
        //        }





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

    //note to self need to change this to speed based, not power based.
    private void handleShooter(){
        shooterTPS = -ShooterMotor.getVelocity(); // Ticks per second

        // 2. Check if the button was JUST pressed (it was up, but is now down)
        if (gamepad2.xWasPressed()){//(isXPressed && !wasXButtonPressed) {
            // Flip the state: if it was on, turn it off; if it was off, turn it on.
            shooterMotorOn = !shooterMotorOn;
        }

// Check if D-pad down was JUST pressed (rising edge detection)
        if (gamepad2.dpadDownWasPressed() && ShooterMotorSpeed > 0) {
            GoalShooterMotorTPS -= 50;
        }
// Check if D-pad up was JUST pressed
        else if (gamepad2.dpadUpWasPressed()) {
            GoalShooterMotorTPS += 50;

        }

        double incrementAmount = SpeedFast;
        if (Math.abs(shooterTPS - GoalShooterMotorTPS) <= BigRange){
            incrementAmount = SpeedNormal;
        }
        if (Math.abs(shooterTPS - GoalShooterMotorTPS) <= MediumRange){
            incrementAmount = SpeedPrecise;
        }
        if (Math.abs(shooterTPS - GoalShooterMotorTPS) <= SmallRange){
            incrementAmount = 0;
        }



        if (shooterMotorOn) {

            if (shooterTPS < GoalShooterMotorTPS){
                ShooterMotorSpeed +=incrementAmount;
            }
            if (shooterTPS > GoalShooterMotorTPS){
                ShooterMotorSpeed -=incrementAmount;
            }

            ShooterMotor.setPower(-1 * ShooterMotorSpeed);
            ShooterMotor2.setPower(ShooterMotorSpeed);

        }else{
            ShooterMotor.setPower(0);
            ShooterMotor2.setPower(0);
        }

        telemetry.addData("incrementAMount ", incrementAmount );
        //stabilizing values to know when to shoot
        /*
        LastValuesList.add(shooterTPS);
        if (LastValuesList.size() > 6) {
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

        //telemetry.addData("ShooterSpeedRangeForStabilizing", shooterSpeedRange );
        */


    }

    //note to self error occuring in handleShooterRotation idk why. ( i think its solved now )
    private void handleShooterRotation(double bearing, boolean tagBool){


        if (gamepad2.dpad_right && currentAngle < 180){
            currentAngle += 2;
        }
        if (gamepad2.dpad_left && currentAngle > 0){
            currentAngle -= 2;
        }

        if (Math.abs(bearing)>rotationTolerance && Math.abs(bearing)<30 && gamepad2.y) {
            //currentAngle = 90;

            currentAngle -= bearing * rotationCompensation;

            currentAngle = Math.min(180, Math.max(0, currentAngle));
        }

        double ShooterRotationAngle = Range.scale(
                currentAngle,   // value you want to map
                0, 180,        // input range
                0,  // output start
                1     // output end
        );

        ShooterRotatorServo.setPosition(ShooterRotationAngle);

        telemetry.addData("current angle", currentAngle );
        telemetry.addData("bearing", bearing );


    }

    private void handleIntake(){
        //double intakeVelocity = IntakeMotor.getVelocity(); // Ticks per second
        double TriggerValue =0;

        if (gamepad2.left_trigger > gamepad1.left_trigger){TriggerValue = gamepad2.left_trigger;}
        if (gamepad1.left_trigger > gamepad2.left_trigger){TriggerValue = gamepad1.left_trigger;}
        if (gamepad2.left_trigger == gamepad1.left_trigger){TriggerValue = gamepad2.left_trigger;}

        double intake=0;
        if (gamepad2.right_bumper){ // to spit out balls
            intake = TriggerValue;
        }
        else { // to intake balls
            intake = -TriggerValue;
        }

        double StopIntake = 0;

        //handle feeder to launcher
        if (gamepad2.right_trigger > 0 && !gamepad2.right_bumper){
            BallFeederServo.setPower(-gamepad2.right_trigger);
            StopIntake = gamepad2.right_trigger;
            ServoHelper.setPower(-1);

        }
        else if (gamepad2.right_bumper){
            BallFeederServo.setPower(gamepad2.right_trigger);
            StopIntake = -gamepad2.right_trigger;

            if (gamepad2.right_trigger > 0){ServoHelper.setPower(1);}
        }
        else{
            BallFeederServo.setPower(0);
            ServoHelper.setPower(0);
        }

        IntakeMotor.setPower(intake);
        StopIntakeMotor.setPower(StopIntake);

    }

    private void handleShooterServos(){

        // Initialize shooterAngle with the servo's current position to start.
        // This ensures it always has a value.

        if (gamepad2.a || gamepad2.b) {
            if (gamepad2.a && HoodAngle < 1) {
                HoodAngle += .02;
            } else if (gamepad2.b && HoodAngle > 0) {
                HoodAngle -= .02;
            }
            ShooterAngle = Range.scale(
                    HoodAngle,   // value you want to map
                    0, 1,        // input range
                    startPoint,  // output start
                    endPoint     // output end
            );
        }

        ServoShooter1.setPosition(ShooterAngle);
        ServoShooter2.setPosition(ShooterAngle);
        //telemetry.addData("ActualVAlueHood ", HoodAngle );
        //telemetry.addData("Angle Servo 1= ", ShooterAngle );
        //telemetry.addData("Angle Servo 2= ", (ShooterAngle  ));
    }
    }


