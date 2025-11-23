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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Config
@Autonomous
public class CleanAuto extends LinearOpMode {

    //setting up shooter servo stuff
    private Servo ServoShooter1;
    private Servo ServoShooter2;
    public static double startPoint = .30;
    public static double endPoint = .75;//.7
    private static double ShooterAngle = startPoint;

    public static long TimeForBallToGetShotMS = 200;//.7
    private static long TimeBetweenBallsMS = 500;
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

    public static boolean fieldCentricDrive = true;

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
    public static double rotationCompensation = -.09;
    //private boolean SelfAimToggle = true;
    public static double currentAngle = 90;


    //Variables for statement printing
    private static double ShooterMotorSpeed = 0;
    public static double GoalShooterMotorTPS = 1500;// rotation ticks per seond
    //variables belof for testing
//    public static double SpeedFast = 0.01;
//    public static double SpeedNormal = 0.001;
    public static double SpeedPrecise = 0.0005;
//    public static double BigRange = 300;
//    public static double MediumRange = 100;
    public static double SmallRange = 40;

    //(1100,.47),(1600,.675), (2100, .877), (2360,1)

    public static double ToleranceForShooting = 40;
    //variables avobe for testing
    double shooterTPS = 0; // Ticks per second


    public double range = 0;

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
            if (vision.isTagVisible()) {
                //telemetry.addData("ID", vision.getID());
                if (vision.getID() == 20 || vision.getID() == 24) {
                    telemetry.addData("GOAL TAG?: ", vision.isTagVisible());
                    telemetry.addData("Range", "%.2f in", vision.getRange());
                    //telemetry.addData("Yaw", "%.2f deg", vision.getYaw());
                    //telemetry.addData("X Offset", "%.2f in", vision.getX());
                    AprilTagBearing = vision.getBearing();
                }
            }
//

            //handleDriving();
            double speed = 1;
            //if (gamepad1.right_trigger ==1){speed = 1;}

            double axial = 0 * speed;
            double lateral =0 * speed; // Note: pushing stick forward gives negative value
            double yaw = 0 * speed;



            if (runtime.seconds()<30) {
                handleShooter(1550, true);
                ShooterAngle = .35;
                handleShooterServos();


            }else{handleShooter(1400, false);}


            ShootCycle();

            axial = -1;



            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(axial, -lateral), -yaw));

            if (vision.isTagVisible()) {
                range = vision.getRange();
            }
            //handleShootingRanges(range,vision.isTagVisible());

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            Pose2d pose = drive.localizer.getPose();
            //telemetry.addData("Heading ( Encoders prob )= ", Math.toDegrees(pose.heading.toDouble()));
            //telemetry.addData("Heading ( IMU )= ", Math.toDegrees(robotHeading));

            telemetry.addData("GoalShooterPower= ", GoalShooterMotorTPS);
            telemetry.addData("ShooterMotorTickPerSecond= ", shooterTPS);
            //telemetry.addData("x,y,heading ", pose.position.x + ", " + pose.position.y + ", heading: " + pose.heading.toDouble());


            telemetry.update();
            break;

///
            for (i = 0; i<=2; i++) {
//            ShootCycle();
//            //shootCycle
//            //Replace this with the time it takes to move to the first ball segment
//            double mainDist = 1.0;
//
//            //Replace this with the amount of time it takes to strafe between ball lines
//            double ballDist = 0.7;
//
//            //This calculates the total amount of time the robot must strafe for to be in the correct position
//            double timeTotal = mainDist + (ballDist * i);
//
//            //Move up
//            /*
//            Replace with code that turns on motor for timeTotal
//             */
//
//
//            //collect (same each time) (Move forward with intake on)
//
//
//            //MoveDown
//            /*
//            Replace with code to move down timetotal
//             */
//            if (i == 2) {
//                Shoot
//            }
//
//        }
//


        }
    }

    private void ShootSingle() {
        handleIntake(1,.5,.5);
        sleep(TimeForBallToGetShotMS); //time it takes for ball to get shot
        handleIntake(0,0,0);
    }

    private void ShootCycle() {
        for (i = 0; i<=2; i++) {
            ShootSingle();
            if (i == 2) {
                break;
            }
            sleep(TimeBetweenBallsMS);
        }
    }


    private void handleShootingRanges(double range,boolean tagBool) {

        if (gamepad2.y) {

            //int[] turretGoals = new int[2]; // angle then ticks per second

            double targAngle = (0.00458093 * range) + 0.340541;
            double targSpeed = (3.90841 * range) + 1409.28811;

            ShooterAngle = targAngle;
            GoalShooterMotorTPS = targSpeed;

            telemetry.addData("ThingsBeingAdjusted ", ShooterAngle );

        }

        if (gamepad2.left_bumper) {
            currentAngle = 90;
            GoalShooterMotorTPS = 1450;
            ShooterAngle = .35;
        }

    }

    private void InitializeIMU() {
        // Drivetrain & IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure the IMU. This is critical for field-centric drive.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

    }

    //note to self need to change this to speed based, not power based.
    private void handleShooter(double speed,boolean on) {
        shooterTPS = -ShooterMotor.getVelocity(); // Ticks per second
        GoalShooterMotorTPS=speed;


        // 2. Check if the button was JUST pressed (it was up, but is now down)

        shooterMotorOn = on;




// Check if D-pad down was JUST pressed (rising edge detection)
        if (gamepad2.dpadDownWasPressed() && GoalShooterMotorTPS > 1100) {
            GoalShooterMotorTPS -= 50;
            ShooterMotorSpeed = calculateSpeedForShooter(GoalShooterMotorTPS);
        }
// Check if D-pad up was JUST pressed
        else if (gamepad2.dpadUpWasPressed() && GoalShooterMotorTPS < 2400) {
            GoalShooterMotorTPS += 50;
            ShooterMotorSpeed = calculateSpeedForShooter(GoalShooterMotorTPS);

        }
        double incrementAmount = SpeedPrecise;//test
//        double incrementAmount = SpeedFast;
//        if (Math.abs(shooterTPS - GoalShooterMotorTPS) <= BigRange){
//            incrementAmount = SpeedNormal;
//        }
//        if (Math.abs(shooterTPS - GoalShooterMotorTPS) <= MediumRange){
//            incrementAmount = SpeedPrecise;
//        }
        if (Math.abs(shooterTPS - GoalShooterMotorTPS) <= SmallRange){
            incrementAmount = 0;
        }


        if (shooterMotorOn) {

            if (shooterTPS < GoalShooterMotorTPS) {
                ShooterMotorSpeed += incrementAmount;
            }
            if (shooterTPS > GoalShooterMotorTPS) {
                ShooterMotorSpeed -= incrementAmount;
            }

            ShooterMotor.setPower(-1 * ShooterMotorSpeed);
            ShooterMotor2.setPower(ShooterMotorSpeed);

        } else {
            ShooterMotor.setPower(0);
            ShooterMotor2.setPower(0);
            ShooterMotorSpeed = 0;
        }

        //telemetry.addData("incrementAMount ", incrementAmount);
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
    private void handleShooterRotation(double bearing, boolean tagBool) {


        if (gamepad2.dpad_left && currentAngle < 180) {
            currentAngle += 2;
        }
        if (gamepad2.dpad_right && currentAngle > 0) {
            currentAngle -= 2;
        }

        if (Math.abs(bearing) > rotationTolerance && Math.abs(bearing) < 30 && gamepad2.y) {
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

        //telemetry.addData("current angle", currentAngle);
        //telemetry.addData("bearing", bearing);


    }

    private void handleIntake(double roller, double StopIntake, double intake) {
        //double intakeVelocity = IntakeMotor.getVelocity(); // Ticks per second


        IntakeMotor.setPower(intake);
        StopIntakeMotor.setPower(StopIntake);
        BallFeederServo.setPower(roller);

    }

    private void handleShooterServos() {

        // Initialize shooterAngle with the servo's current position to start.
        // This ensures it always has a value.

        ServoShooter1.setPosition(ShooterAngle);
        ServoShooter2.setPosition(ShooterAngle);
        //telemetry.addData("ActualVAlueHood ", HoodAngle );
        telemetry.addData("ServoAngle ", ShooterAngle );

    }


    // --- NEW helper: maps GoalTPS → motor power ---
    private double calculateSpeedForShooter(double GoalTPS) {
        // tweak these numbers if you need to recalibrate
        double MotorSpeed = 0.000417188 * GoalTPS + 0.00873318;
        return MotorSpeed;
    }
}

