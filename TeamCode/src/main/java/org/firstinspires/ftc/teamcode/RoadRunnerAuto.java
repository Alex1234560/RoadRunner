package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Function;

@Config
@Autonomous(name = "RoadRunnerAuto", group = "Autonomous")
public class RoadRunnerAuto extends LinearOpMode {
    public static double turnAmount = -50;
    public static int RotatorAngleTolerance = 2;
    public static boolean ShootingNow = false;
    public static double GoalTPS = 1200;
    public static double shootingDuration = 1.6;   // How long to run in seconds
    public static double range = 0;
    public static boolean SpinShootingBallFeeder = false;

    private FunctionsAndValues FAndV;
    private AprilTagVision vision;


    public class Intake {
        private DcMotorEx intakeMotor;
        private DcMotor StopIntakeMotor;
        private CRServo BallFeederServo;
        private CRServo BallFeederServo2;

        public Intake(HardwareMap hardwareMap) {
            BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
            BallFeederServo2 = hardwareMap.get(CRServo.class, "BallFeederServo2");
            BallFeederServo2.setDirection(CRServo.Direction.REVERSE);
            BallFeederServo.setPower(0);
            BallFeederServo2.setPower(0);

            intakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
            StopIntakeMotor.setDirection(DcMotor.Direction.FORWARD);


        }

        public class IntakeOn implements Action {
            private boolean initialized = false;
            private ElapsedTime timer; // To track time
            private double duration = .001;   // How long to run in seconds

            public IntakeOn() {
                this.timer = new ElapsedTime(); // This creates the timer object.
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    intakeMotor.setPower(1);
                    StopIntakeMotor.setPower(1);
                    initialized = true;
                }
                packet.put("time", timer.seconds());
                if (timer.seconds() < duration) {
                    return true;
                } else {
//                    intakeMotor.setPower(0);
//                    StopIntakeMotor.setPower(0);
//                    ServoHelper.setPower(0);
                    return false;
                }
            }
        }

        public class IntakeOff implements Action {
            private boolean initialized = false;
            private ElapsedTime timer; // To track time
            private double duration = .001;   // How long to run in seconds

            public IntakeOff() {
                this.timer = new ElapsedTime(); // This creates the timer object.
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                }
                packet.put("time", timer.seconds());
                if (timer.seconds() < duration) {
                    return true;
                } else {
                    intakeMotor.setPower(0);
                    StopIntakeMotor.setPower(0);

                    return false;
                }
            }
        }

        public class PassABallToShooter implements Action {
            private boolean initialized = false;
            private ElapsedTime timer; // To track time
            private double duration = shootingDuration;   // How long to run in seconds

            public PassABallToShooter() {
                this.timer = new ElapsedTime(); // This creates the timer object.
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();



                    intakeMotor.setPower(.7);
                    StopIntakeMotor.setPower(.7);
                    initialized = true;
                }

                packet.put("time", timer.seconds());
                if (timer.seconds() < duration) {
                    if (SpinShootingBallFeeder){
                        BallFeederServo.setPower(1);
                        BallFeederServo2.setPower(1);
                    }
                    else{BallFeederServo.setPower(0);}
                    return true;
                } else {
                    ShootingNow = false;
                    BallFeederServo.setPower(0);
                    BallFeederServo2.setPower(0);
                    intakeMotor.setPower(0);
                    StopIntakeMotor.setPower(0);


                    return false;
                }
            }
        }


        public Action intakeOff() {return new IntakeOff();}
        public Action intakeOn() {return new IntakeOn();}
        public Action passABallToShooter() {return new PassABallToShooter();}
        //public Action initializeBallFeeder() {return new InitializeBallFeeder();}

    }

    public class Shooter {
        private Servo ServoShooter1;
        private Servo ShooterRotatorServo;
        private DcMotorEx ShooterMotor = null;
        private DcMotorEx ShooterMotor2 = null;

        public Shooter(HardwareMap hardwareMap) {
            ShooterRotatorServo = hardwareMap.get(Servo.class, "ShooterRotatorServo");

            ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");

            ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
            ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
            ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            //ServoShooter1.setDirection(Servo.Direction.REVERSE);
            ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

        public class RunShooter implements Action {

            private double AprilTagBearing = 0;
            private double CurrentAngle = 90;
            private FunctionsAndValues FAndV;

            private boolean initialized = false;
            private ElapsedTime timer; // To track time



            private double ServoAngle;

            private double ShooterMotorPower=0;
            private double shooterTPS=0;
            private boolean far=false;

            public RunShooter(boolean far) {
                this.timer = new ElapsedTime(); // This creates the timer object.
                this.far = far;

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    ShootingNow=true;
                    initialized = true;
                    FAndV = new FunctionsAndValues();
                    CurrentAngle = ShooterRotatorServo.getPosition()*180;

                    if (!this.far){
                        //settings for far position
                        GoalTPS=1200;
                        ServoAngle = .3;
                    }
                    ServoShooter1.setPosition(ServoAngle);
                }


                packet.put("time", timer.seconds());
                if(ShootingNow){//(timer.seconds() < duration) {
                    //handling aiming below

                    if (this.far) {
                        vision.update();
                        telemetry.addData("AprilTag: ", vision.isTagVisible());

                        if (vision.isTagVisible()) {
                            if (vision.getID() == 20 || vision.getID() == 24) {
                                AprilTagBearing = vision.getBearing();
                                range = vision.getRange();

                            }
                        }
                        double[] ShooterRotatorServoAngle = FAndV.calculateShooterRotation(AprilTagBearing, true, CurrentAngle, true);
                        ShooterRotatorServo.setPosition(ShooterRotatorServoAngle[0]);
                        CurrentAngle = ShooterRotatorServoAngle[1];
                        AprilTagBearing = ShooterRotatorServoAngle[2];

                        // handling aiming avobe

                        // handling shooter angle below

                        if (range != 0) {
                            double[] shooterGoals = FAndV.handleShootingRanges(range);
                            GoalTPS = shooterGoals[1];
                            ServoAngle = shooterGoals[0];

                        }
                        ServoShooter1.setPosition(ServoAngle);
                    }

                    // handling shooter angle avobe




//                    telemetry.addData("shooterTPS: ", shooterTPS);
//                    telemetry.addData("GoalTPS: ", GoalTPS);
//                    telemetry.addData("SpinShootingBallFeeder : ", SpinShootingBallFeeder);
//                    telemetry.addData("ShooterMotorPower : ", ShooterMotorPower);

                    if (Math.abs(shooterTPS-GoalTPS)<=FunctionsAndValues.SpeedToleranceToStartShooting && Math.abs(AprilTagBearing)<=RotatorAngleTolerance) {
                            SpinShootingBallFeeder=true;
                        }

                    else {
                        SpinShootingBallFeeder=false;
                        telemetry.addData("No SPinning Ball Feeder : ", "");
                    }


                    shooterTPS = ShooterMotor.getVelocity();

                    ShooterMotorPower = FAndV.handleShooter(shooterTPS,true, GoalTPS, ShooterMotorPower);
                    ShooterMotor.setPower(ShooterMotorPower);
                    ShooterMotor2.setPower(ShooterMotorPower);

                    telemetry.update();

                    return true;
                } else {
                    ShooterMotor.setPower(0);
                    ShooterMotor2.setPower(0);
                    return false;
                }
            }
        }
        public Action runShooter(boolean far) {return new RunShooter(far);}

        public class Aim implements Action {
            private double TimeToAutoAimBeforeSpinningUp = .2;

            private boolean initialized = false;
            private ElapsedTime timer; // To track time
            private double AprilTagBearing = 0;
            private double CurrentAngle = 90;
            private FunctionsAndValues FAndV;




            public Aim() {
                this.timer = new ElapsedTime(); // This creates the timer object.

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                    this.FAndV = new FunctionsAndValues();
                    CurrentAngle = ShooterRotatorServo.getPosition()*180;

                }


                packet.put("time", timer.seconds());
                if(timer.seconds() < TimeToAutoAimBeforeSpinningUp || ShootingNow) {
                    vision.update();
                    telemetry.addData("AprilTag: ", vision.isTagVisible());

                    if (vision.isTagVisible()) {
                        if (vision.getID() == 20 || vision.getID() == 24) {
                            AprilTagBearing = vision.getBearing();
                            range=vision.getRange();

                        }


                    }
                    double[] ShooterRotatorServoAngle = this.FAndV.calculateShooterRotation(AprilTagBearing, true, CurrentAngle, true);
                    ShooterRotatorServo.setPosition(ShooterRotatorServoAngle[0]);
                    CurrentAngle = ShooterRotatorServoAngle[1];
                    AprilTagBearing = ShooterRotatorServoAngle[2];

                    telemetry.addData("PowerTo Servo: ", ShooterRotatorServoAngle[0]);
                    telemetry.addData("AprilBearing: ", AprilTagBearing);
                    telemetry.addData("CurrentANgle: ", CurrentAngle);

                    telemetry.update();

                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action aim() {return new Aim();}

        public class CenterShooter implements Action {

            private boolean initialized = false;
            private ElapsedTime timer; // To track time

            public CenterShooter() {
                this.timer = new ElapsedTime(); // This creates the timer object.

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;
                    ShooterRotatorServo.setPosition(0.5);

                }

                packet.put("time", timer.seconds());
                if(timer.seconds() < .01) {


                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action centerShooter() {return new CenterShooter();}

    }

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new AprilTagVision(hardwareMap, "Webcam");
        vision.setManualExposure(AprilTagVision.myExposure, AprilTagVision.myGain);
        double side = -1; // -1 is blue 1 is red
        double StartingAngle = (90*side); // =128
        double ShootAngle = Math.toRadians( (50+180)*-side ); // not sure if it works on other side ( 1 )
        double StartingX = -60.5;
        double StartingY = 36.5*side;

        double distanceBetweenBalls = 23;

        //poses
        Pose2d initialPos = new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle));//(0, 0, Math.toRadians(0));
        Pose2d IntakePosition = new Pose2d(-11.5, 18.5*side , Math.toRadians(StartingAngle));
        Pose2d BallsIntakenPos = new Pose2d(-11.5, 50.5*side , Math.toRadians(StartingAngle));
        Pose2d ShootPos = new Pose2d(-35, 35*side , ShootAngle); // not sure if work on both sides



        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPos);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        //poses



        // actionBuilder builds from the drive steps passed to it
        Pose2d pose = drive.localizer.getPose();
        TrajectoryActionBuilder GoToShootPos = drive.actionBuilder(BallsIntakenPos)
                //.turn(Math.toRadians(-50)) for some reason this requires minus 50 but in strafeToLinearHeading i need +50?, oh well
                .strafeToLinearHeading(new Vector2d( ShootPos.position.x,ShootPos.position.y), ShootPos.heading.log())


                ;


        waitForStart();


        if (isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                shooter.runShooter(false),//shoot at a predetermined speed
                                intake.passABallToShooter()
                        )
                )
        );

        for (int i = 0; i < 3; i++) {

            TrajectoryActionBuilder GoForwardsToIntakeBalls = drive.actionBuilder(IntakePosition)
                    //grab balls
                    .strafeTo(new Vector2d(BallsIntakenPos.position.x, BallsIntakenPos.position.y))
                    ;

            TrajectoryActionBuilder MoveTowardsIntakePosition = drive.actionBuilder(initialPos)
                    //go to balls
                    .strafeTo(new Vector2d(IntakePosition.position.x, IntakePosition.position.y))
                    //.strafeToLinearHeading(new Vector2d(IntakePosition.position.x, IntakePosition.position.y), IntakePosition.heading.log())//Math.toRadians(-50+180))
                    ;

            TrajectoryActionBuilder TurnFromShootPosToNormal = drive.actionBuilder(ShootPos)
                    .turn(Math.toRadians(StartingAngle) - ShootPos.heading.log())


                    ;


            Actions.runBlocking(
                    new SequentialAction(

                            MoveTowardsIntakePosition.build(),
                            intake.intakeOn(),
                            GoForwardsToIntakeBalls.build(),
                            intake.intakeOff(),
                            GoToShootPos.build(),
                            shooter.centerShooter(),
                            //shooter.aim(),// give a little time to aim
                            new ParallelAction(
                                    //shooter.aim(), // aim while shooting
                                    shooter.runShooter(true),
                                    intake.passABallToShooter()
                            )
                            //TurnFromShootPosToNormal.build()

                    )
            );

            BallsIntakenPos = new Pose2d(BallsIntakenPos.position.x + distanceBetweenBalls, BallsIntakenPos.position.y, BallsIntakenPos.heading.log());
            IntakePosition = new Pose2d(IntakePosition.position.x + distanceBetweenBalls, IntakePosition.position.y, IntakePosition.heading.log());
            initialPos = new Pose2d(ShootPos.position.x, ShootPos.position.y, StartingAngle);



        }




        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("running telemetry", ":)");
            telemetry.update();
        }
    }


}