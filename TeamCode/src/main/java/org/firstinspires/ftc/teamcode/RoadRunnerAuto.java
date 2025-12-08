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
    public static double shootingDuration = 2.4;   // How long to run in seconds
    public static double ShootingDurationMax=6; // if conditions arent met for some reason for shooting duration, it will shoot until 6s
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
            //BallFeederServo2.setDirection(CRServo.Direction.REVERSE);
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
                if (ShootingNow) {
                    if (SpinShootingBallFeeder){
                        BallFeederServo.setPower(1);
                        BallFeederServo2.setPower(1);
                    }
                    else{BallFeederServo.setPower(0);}
                    return true;
                } else {
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
            ServoShooter1.setDirection(Servo.Direction.REVERSE);

            ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
            ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
            ShooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

            ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        public class RunShooter implements Action {

            private double AprilTagBearing = 0;
            private double CurrentAngle = 90;
            private FunctionsAndValues FAndV;

            private boolean initialized = false;
            private ElapsedTime GlobalTimer; // To track time



            private double ServoAngle;

            private double ShooterMotorPower=0;
            private double accumulatedShootingTime=0;
            private double shooterTPS=0;
            private boolean far=false;

            private double lastLoopTime;

            public RunShooter(boolean far) {
                this.GlobalTimer = new ElapsedTime(); // This creates the timer object.

                this.far = far;

            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    ShootingNow=true;
                    GlobalTimer.reset();
                    lastLoopTime = GlobalTimer.seconds();
                    accumulatedShootingTime = 0.0;

                    initialized = true;
                    FAndV = new FunctionsAndValues();
                    CurrentAngle = ShooterRotatorServo.getPosition()*180;


                    if (!this.far){
                        //settings for far position
                        GoalTPS=1050;
                        ServoAngle = .195;
                    }
                    ServoShooter1.setPosition(ServoAngle);
                }

                double currentTime = GlobalTimer.seconds();
                double deltaTime = currentTime - lastLoopTime;
                lastLoopTime = currentTime; // Update for the next loop

                telemetry.addData("AccumulatedTimeShooting: ", accumulatedShootingTime);
                telemetry.addData("GlobalTime: ", GlobalTimer.seconds());



                packet.put("time", GlobalTimer.seconds());
                if(GlobalTimer.seconds() < ShootingDurationMax && accumulatedShootingTime < shootingDuration ){//(timer.seconds() < duration) {
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

                    if (Math.abs(shooterTPS-GoalTPS)<=FunctionsAndValues.SpeedToleranceToStartShooting && Math.abs(AprilTagBearing)<=RotatorAngleTolerance) {
                            SpinShootingBallFeeder=true;

                        }

                    else {
                        SpinShootingBallFeeder=false;
                        telemetry.addData("No SPinning Ball Feeder : ", "");
                    }

                    if (SpinShootingBallFeeder) {
                        accumulatedShootingTime += deltaTime;
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
                    ShootingNow = false;
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
        double side = -1; // -1 is blue 1 is red
        boolean FrontAuto = true; // -1 is blue 1 is red

        vision = new AprilTagVision(hardwareMap, "Webcam");
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Alliance Selection", "X for BLUE, B for RED, Y for FRONT, A for BACK");
            if (side == -1) {
                telemetry.addData("Color: BLUE ", "");
            }
            if (side == 1) {
                telemetry.addData("Color: RED ", "");
            }
            if (FrontAuto) {
                telemetry.addData("pos: FRONT ", "");
            }
            if (!FrontAuto) {
                telemetry.addData("pos: BACK ", "");
            }

            if (gamepad1.x) {side = -1;}
            if (gamepad1.b) {side = 1;}
            if (gamepad1.y) {FrontAuto=true;}
            if (gamepad1.a) {FrontAuto=false;}
            telemetry.update();
        }

        //vision.setManualExposure(AprilTagVision.myExposure, AprilTagVision.myGain);





        //side selection
        double StartingAngle;
        double ShootAngle;
        double StartingX;
        double StartingY;
        double distanceBetweenBalls = 23;


        //poses
        Pose2d initialPos;
        Pose2d IntakePosition;
        Pose2d BallsIntakenPos;
        Pose2d ShootPos;
        Pose2d ParkPosition;

        if (FrontAuto) {
            StartingAngle = (90 * side); // =128
            ShootAngle = Math.toRadians(-130 * -side);//(50+180)*-side ); // not sure if it works on other side ( 1 )
            StartingX = -60.5;
            StartingY = 36.5 * side;


            initialPos = new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle));//(0, 0, Math.toRadians(0));
            IntakePosition = new Pose2d(-11.5, 18.5 * side, Math.toRadians(StartingAngle));
            BallsIntakenPos = new Pose2d(-11.5, 46.5 * side, Math.toRadians(StartingAngle));
            ShootPos = new Pose2d(-35, 35 * side, ShootAngle); // not sure if work on both sides

            ParkPosition = new Pose2d(5.7, 28 * side, Math.toRadians(StartingAngle)); // not sure if work on both sides
        }
        else{
            StartingAngle = Math.toRadians(0);
            ShootAngle = Math.toRadians(0);
            StartingX = 61.3;
            StartingY = 14 * side;

            initialPos = new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle));//(0, 0, Math.toRadians(0));
            IntakePosition = new Pose2d(36.1, 30 * side, Math.toRadians(StartingAngle));
            BallsIntakenPos = new Pose2d(36.1, 50 * side, Math.toRadians(StartingAngle));
            ShootPos = new Pose2d(StartingX, StartingY, ShootAngle); // not sure if work on both sides

            ParkPosition = new Pose2d(41, 26 * side, Math.toRadians(StartingAngle)); // not sure if work on both sides

        }


        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPos);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        ElapsedTime autoTimer = new ElapsedTime();
        //poses
        waitForStart();


        if (isStopRequested()) return;
        autoTimer.reset();

        if (FrontAuto) {

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    shooter.runShooter(false),//shoot at a predetermined speed
                                    intake.passABallToShooter()
                            )
                    )
            );

            for (int i = 0; i < 3; i++) {

                TrajectoryActionBuilder GoToShootPos = drive.actionBuilder(BallsIntakenPos)
                        .strafeTo(new Vector2d(BallsIntakenPos.position.x, 40 * side))
                        .strafeToLinearHeading(new Vector2d(ShootPos.position.x, ShootPos.position.y), ShootPos.heading.log());

                TrajectoryActionBuilder GoForwardsToIntakeBalls = drive.actionBuilder(IntakePosition) // switch to drive.localizer.getPose() to make s
                        //grab balls
                        .strafeTo(new Vector2d(BallsIntakenPos.position.x, BallsIntakenPos.position.y));

                TrajectoryActionBuilder MoveTowardsIntakePosition;

                MoveTowardsIntakePosition = drive.actionBuilder(initialPos)
                        //go to balls
                        .strafeTo(new Vector2d(IntakePosition.position.x, IntakePosition.position.y))

                ;

                TrajectoryActionBuilder TurnFromShootPosToNormal = drive.actionBuilder(ShootPos)
                        .turn(Math.toRadians(StartingAngle) - ShootPos.heading.log());


                Actions.runBlocking(
                        new SequentialAction(

                                MoveTowardsIntakePosition.build(),
                                intake.intakeOn(),
                                GoForwardsToIntakeBalls.build(),
                                intake.intakeOff(),
                                GoToShootPos.build(),
                                shooter.centerShooter(),

                                new ParallelAction(
                                        shooter.runShooter(true),
                                        intake.passABallToShooter()
                                ),
                                TurnFromShootPosToNormal.build()

                        )
                );

                BallsIntakenPos = new Pose2d(BallsIntakenPos.position.x + distanceBetweenBalls, BallsIntakenPos.position.y, BallsIntakenPos.heading.log());
                IntakePosition = new Pose2d(IntakePosition.position.x + distanceBetweenBalls, IntakePosition.position.y, IntakePosition.heading.log());
                initialPos = new Pose2d(ShootPos.position.x, ShootPos.position.y, Math.toRadians(StartingAngle));//ShootPos.heading.log());
            }


        }
        if (!FrontAuto){

            TrajectoryActionBuilder MoveToParkPosition = drive.actionBuilder(initialPos)
                    .strafeTo(new Vector2d(IntakePosition.position.x, IntakePosition.position.y))
                    ;

            Actions.runBlocking(
                    new SequentialAction(

                            new ParallelAction(
                                    shooter.runShooter(true),
                                    intake.passABallToShooter()
                            ),

                            MoveToParkPosition.build()
                    )
            );

        }
    }


}