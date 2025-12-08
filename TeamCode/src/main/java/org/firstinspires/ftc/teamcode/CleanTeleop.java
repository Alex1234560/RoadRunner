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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp

public class CleanTeleop extends LinearOpMode {
    // Hardware Setup Variables
    private Servo ServoShooter1;
    private Servo ReadyToShootServo;

    //private Servo ServoShooter2;

    private static double ShooterAngle = FunctionsAndValues.startPoint;
    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx IntakeMotor = null;
    private DcMotor StopIntakeMotor = null;
    private DcMotorEx ShooterMotor = null;
    private DcMotorEx ShooterMotor2 = null;
    private CRServo BallFeederServo = null;
    private CRServo BallFeederServo2 = null;

    private Servo ShooterRotatorServo = null;
    private IMU imu;

    //classes
    //Vision !!!
    private double startingAngleRad = Math.toRadians(0);
    private AprilTagVision vision;
    private MecanumDrive drive;
    private FunctionsAndValues FAndV;

    public static boolean fieldCentricDrive = false;

    // angle for hooded shooter
    double HoodAngle = 0;// value from 0 to 1.0

    //april tag stuff
    private double AprilTagBearing = 0;

    private boolean SelfAimToggle = true;
    public static double currentAngle = 90;

    //Variables for statement printing
    private static double ShooterMotorPower = 0;
    public static double GoalShooterMotorTPS = 1200;// rotation ticks per seond

    //variables avobe for testing
    public double shooterTPS = 0; // Ticks per second
    public double range = 0;

    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off

    //declaring button globally
    //private boolean autoAimButton = false;
    public static boolean AutoAim = false;

    @Override
    public void runOpMode() {
        currentAngle=90;//center current angle for shooter
        InitializeIMU();
        SetupHardware();

        vision = new AprilTagVision(hardwareMap, "Webcam");
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, startingAngleRad));
        FAndV = new FunctionsAndValues();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();
        vision.setManualExposure(AprilTagVision.myExposure, AprilTagVision.myGain);
        imu.resetYaw();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad2.aWasPressed()) {AutoAim = true;}
            if (gamepad2.bWasPressed()) {AutoAim = false;}

            vision.update();
            updateBearing();
            handleDriving();
            handleIntake();
            handleShooterServos();
            handleFlywheel();
            handleShooterRotation();
            SpeedAndAngleAutoAimUpdate();
            TelemetryStatements();
            handleUserShootingRanges();
            updateReadyToShoot();
            //autoLock();
        }
    }

    private void updateReadyToShoot(){
        //(Math.abs(GoalShooterMotorTPS - shooterTPS) <= FunctionsAndValues.SpeedToleranceToStartShooting){
        if (vision.isTagVisible()) {
            if (Math.abs(AprilTagBearing) < FunctionsAndValues.AngleToleranceToStartShooting && shooterTPS > FunctionsAndValues.MinimumSpeed) {
                ReadyToShootServo.setPosition(1);
            } else {
                ReadyToShootServo.setPosition(.66666666);
            }
        }
        else{
            ReadyToShootServo.setPosition(.33333);
        }
}

    private void TelemetryStatements(){
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        //telemetry.addData("Heading ( Encoders prob )= ", Math.toDegrees(pose.heading.toDouble()));
        //telemetry.addData("Heading ( IMU )= ", Math.toDegrees(robotHeading));

        telemetry.addData("GoalShooterPower= ", GoalShooterMotorTPS);
        boolean stabilized;
        if (Math.abs(shooterTPS - GoalShooterMotorTPS) <= FunctionsAndValues.SpeedToleranceToStartShooting) {
            stabilized = true;
        } else{stabilized=false;}
        telemetry.addData("Stablized?: ", stabilized);
        telemetry.addData("ShooterMotorTickPerSecond= ", shooterTPS);

        telemetry.addData("X= ", pose.position.x);
        telemetry.addData("Y= ", pose.position.y);





        telemetry.update();
    }

    private void handleShooterRotation(){
        //this function will return current value unless able to adjust it, with autoaim and autoaim activated
        double[] ShooterRotatorServoAngle = FAndV.calculateShooterRotation(AprilTagBearing, AutoAim,currentAngle,false);
        ShooterRotatorServo.setPosition(ShooterRotatorServoAngle[0]);
        currentAngle = ShooterRotatorServoAngle[1];
        AprilTagBearing = ShooterRotatorServoAngle[2];

        //if (gamepad2.dpad_left && currentAngle < 180) {currentAngle += 2;}
        //if (gamepad2.dpad_right && currentAngle > 0) {currentAngle -= 2;}
        currentAngle -= gamepad2.right_stick_x*3.5;
        if (currentAngle > 180) {currentAngle = 180;}
        if (currentAngle < 0) {currentAngle = 0;}

        if (gamepad2.right_stick_button){currentAngle = 90;}

    }

    private void SpeedAndAngleAutoAimUpdate(){

        if (vision.getRange()!=-1) {
            range = vision.getRange();
        }


        if (AutoAim){
            double[] shooterGoals = FAndV.handleShootingRanges(range);
            ShooterAngle = shooterGoals[0];
            GoalShooterMotorTPS = shooterGoals[1];

        }
    }

    private void handleUserShootingRanges(){
        if (gamepad2.left_bumper){
            AutoAim = false;
            //ShooterRotatorServo.setPosition(.5);
            currentAngle = 90;
            GoalShooterMotorTPS = 1025;
            ShooterAngle = .15;
        }

        if (gamepad2.dpad_left){
            //code for left side far shooting
        }
        if (gamepad2.dpad_right){
            //code for right side far shooting
        }
    }

    private void handleFlywheel(){
        if (Math.abs(ShooterMotor.getVelocity())< Math.abs(ShooterMotor2.getVelocity())) {
            shooterTPS = Math.abs( ShooterMotor2.getVelocity());
        }
        else {
            shooterTPS = Math.abs(ShooterMotor.getVelocity()); // Ticks per second
        }

    //flywheel stuff.

    if (gamepad2.xWasPressed()) {//(isXPressed && !wasXButtonPressed) {
        shooterMotorOn = true;
    }
        if (gamepad2.yWasPressed()) {//(isXPressed && !wasXButtonPressed) {
            shooterMotorOn = false;
        }

    if (gamepad2.dpadDownWasPressed() && GoalShooterMotorTPS > FunctionsAndValues.MinimumSpeed) {
        GoalShooterMotorTPS -= 50;
    }
    else if (gamepad2.dpadUpWasPressed() && GoalShooterMotorTPS < 2400) {
        GoalShooterMotorTPS += 50;

    }

    ShooterMotorPower = FAndV.handleShooter(shooterTPS,shooterMotorOn,GoalShooterMotorTPS, ShooterMotorPower);
    ShooterMotor.setPower(ShooterMotorPower);
    ShooterMotor2.setPower(ShooterMotorPower);

    telemetry.addData("ShooterMotorSpeed= ", ShooterMotorPower);
    }

    private void handleIntake() {
        // handle feeding to shooter
        double ShootMechanismPower = 0; //Positive value = shooting, negative value = retract balls and spit them out

        if (!gamepad2.right_bumper && gamepad2.right_trigger > 0 && Math.abs(GoalShooterMotorTPS - shooterTPS) <= FunctionsAndValues.SpeedToleranceToStartShooting) {//(Math.abs(GoalShooterMotorTPS - shooterTPS) <= ToleranceForShooting)
            ShootMechanismPower=1;
//            if (vision.isTagVisible() && Math.abs(AprilTagBearing) > FunctionsAndValues.AngleToleranceToStartShooting){
//                // should make it so that if ur scanning the code and bearing is more than blah blah, it doesnt let u shoot
//                ShootMechanismPower=0;
//            }
        }

        else if (gamepad2.right_bumper&& gamepad2.right_trigger > 0) {
            ShootMechanismPower=1;
        }

        else if(gamepad2.back) {
            ShootMechanismPower=-1;
        }


        //IntakeMotor.setPower(-gamepad2.right_trigger * ShootMechanismPower);
        //StopIntakeMotor.setPower(gamepad2.right_trigger *ShootMechanismPower);
        BallFeederServo.setPower(gamepad2.right_trigger * ShootMechanismPower);
        BallFeederServo2.setPower(BallFeederServo.getPower());


        //handle normal intake stuff

        double IntakePowerValue = -Math.abs(gamepad2.left_trigger);
        if (Math.abs(gamepad1.left_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = -Math.abs(gamepad1.left_trigger);
        }
        if (Math.abs(gamepad2.right_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = -Math.abs(gamepad2.right_trigger);
        }
        if (ShootMechanismPower==-1){
            IntakePowerValue = 1;
        }

        IntakeMotor.setPower(IntakePowerValue);
        StopIntakeMotor.setPower(-IntakePowerValue);


        /*if (!gamepad2.right_bumper && gamepad2.right_trigger > 0 && Math.abs(GoalShooterMotorTPS - shooterTPS) <= FunctionsAndValues.SpeedToleranceToStartShooting) {//(Math.abs(GoalShooterMotorTPS - shooterTPS) <= ToleranceForShooting)

            if (vision.isTagVisible() && Math.abs(AprilTagBearing) < FunctionsAndValues.AngleToleranceToStartShooting){}

            IntakeMotor.setPower(-gamepad2.right_trigger /1.2);
            StopIntakeMotor.setPower(gamepad2.right_trigger / 1.2);
            BallFeederServo.setPower(gamepad2.right_trigger);


        }
        // so u can force it to shoot if it isnt ready
        else if (gamepad2.right_bumper&& gamepad2.right_trigger > 0) {
            BallFeederServo.setPower(gamepad2.right_trigger);
            IntakeMotor.setPower(-gamepad2.right_trigger /1.2);
            StopIntakeMotor.setPower(gamepad2.right_trigger / 1.2);
        }
        // SPIT BALL BACK
        else if (gamepad2.back) {
            BallFeederServo.setPower(-1);
            IntakeMotor.setPower(.8);
            StopIntakeMotor.setPower(-.8);
        }



        else{
            BallFeederServo.setPower(0);
        }

        BallFeederServo2.setPower(BallFeederServo.getPower());*/
    }

    private void handleDriving() {
        double speed = .5;
        //if (gamepad1.right_trigger ==1){speed = 1;}
        speed += gamepad1.right_trigger / 2;

        double axial = -gamepad1.left_stick_y * speed;
        double lateral = gamepad1.left_stick_x * speed; // Note: pushing stick forward gives negative value
        double yaw = gamepad1.right_stick_x * speed;

        double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double fieldCentricAxial = axial * Math.cos(robotHeading) - lateral * Math.sin(robotHeading);
        double fieldCentricLateral = axial * Math.sin(robotHeading) + lateral * Math.cos(robotHeading);

        if (gamepad1.backWasPressed()) {
            imu.resetYaw();
        }

        if (gamepad1.xWasPressed()) {
            fieldCentricDrive = !fieldCentricDrive;
        }

        if (fieldCentricDrive) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(fieldCentricAxial, -fieldCentricLateral), -yaw));
        }

        /**/ // this is for field centric

        //robotDrive.move(axial, lateral, yaw, speed);
        if (!fieldCentricDrive) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(axial, -lateral), -yaw));
        }
        telemetry.addData("FieldCentricDrive?: ", fieldCentricDrive);

    }

    private void handleShooterServos() {

        // Initialize shooterAngle with the servo's current position to start.
        // This ensures it always has a value.

        /*if (gamepad2.dpad_right || gamepad2.dpad_left) {
            if (gamepad2.dpad_right && HoodAngle < 1) {
                HoodAngle += .02;
            } else if (gamepad2.dpad_left && HoodAngle > 0) {
                HoodAngle -= .02;
            }
            ShooterAngle = Range.scale(
                    HoodAngle,   // value you want to map
                    0, 1,        // input range
                    FunctionsAndValues.startPoint,  // output start
                    FunctionsAndValues.endPoint     // output end
            );
        }*/

        ShooterAngle-=gamepad2.left_stick_y/15;
        //normalize
        if (ShooterAngle>FunctionsAndValues.endPoint){ShooterAngle=FunctionsAndValues.endPoint;}
        if (ShooterAngle<FunctionsAndValues.startPoint){ShooterAngle=FunctionsAndValues.startPoint;}

        ServoShooter1.setPosition(ShooterAngle);
        telemetry.addData("ServoAngle ", ShooterAngle );

    }

    private void updateBearing(){
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
        telemetry.addData("Bearing", "%.2f Deg", AprilTagBearing);
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

    private void SetupHardware(){
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
        BallFeederServo2 = hardwareMap.get(CRServo.class, "BallFeederServo2");
        ShooterRotatorServo = hardwareMap.get(Servo.class, "ShooterRotatorServo");
        ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
        ReadyToShootServo = hardwareMap.get(Servo.class, "IndicatorServo");


        //directions
        //BallFeederServo2.setDirection(CRServo.Direction.REVERSE);
        //ServoShooter1.setDirection(Servo.Direction.REVERSE);
        ShooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        // run shooter with encoder
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //private double[] lastRangeBearing = {0.1, 0.1};
    //private double lastRangeInertOff = 0.0;
//private double lastRotInertOff = 0.0;
    //private double lastCheckedTime = 0.0;

    /*private void autoLock(){
        if (gamepad2.dpadLeftWasPressed()) {
            SelfAimToggle = !SelfAimToggle;
        };

        if (SelfAimToggle & vision.isTagVisible()) {
            double timeScaleVal = (100*getRuntime()) - lastCheckedTime;
            double rangeScaleVal = timeScaleVal * 0.5;
            double bearingScaleVal = timeScaleVal * 1;


            lastCheckedTime = (100*getRuntime());
            range = vision.getRange();

            double rangeChange = Math.abs(range - lastRangeBearing[0]);
            double bearingChange = Math.abs(AprilTagBearing - lastRangeBearing[1]);

            double tempRange = range;
            double tempBearing = AprilTagBearing;
            if (rangeChange > rangeScaleVal) {
                tempRange += (range - lastRangeBearing[0]);

            }

            if (bearingChange > bearingScaleVal){
                tempBearing += AprilTagBearing - lastRangeBearing[1];

            }


            double[] ShooterRotatorServoAngle = FAndV.calculateShooterRotation(tempBearing,true, currentAngle, false);
            ShooterRotatorServo.setPosition(ShooterRotatorServoAngle[0]);
            currentAngle = ShooterRotatorServoAngle[1];


            double[] shooterGoals = FAndV.handleShootingRanges(tempRange);
            ShooterAngle = shooterGoals[0];
            GoalShooterMotorTPS = shooterGoals[1];

            lastRangeBearing[0] = range;
            lastRangeBearing[1] = AprilTagBearing;
        };
    }*/

  }
