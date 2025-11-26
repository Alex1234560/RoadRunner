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

@Config
@Autonomous(name = "RoadRunnerAuto", group = "Autonomous")
public class RoadRunnerAuto extends LinearOpMode {
    public static boolean ShootingNow = false;
    public static boolean SpinShootingBallFeeder = false;
    private FunctionsAndValues FAndV;


    public class Intake {
        private DcMotorEx intakeMotor;
        private DcMotor StopIntakeMotor;
        private CRServo BallFeederServo;

        public Intake(HardwareMap hardwareMap) {
            BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
            BallFeederServo.setDirection(CRServo.Direction.REVERSE);
            BallFeederServo.setPower(0);

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
            private double duration = 6;   // How long to run in seconds

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
                    }
                    else{BallFeederServo.setPower(0);}
                    return true;
                } else {
                    ShootingNow = false;
                    BallFeederServo.setPower(0);
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
        private Servo ServoShooter2;
        private Servo ShooterRotatorServo;;
        private DcMotorEx ShooterMotor = null;
        private DcMotorEx ShooterMotor2 = null;


        public Shooter(HardwareMap hardwareMap) {
            ShooterRotatorServo = hardwareMap.get(Servo.class, "ShooterRotatorServo");
            ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
            ServoShooter2 = hardwareMap.get(Servo.class, "ServoShooter2");
            ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
            ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
            ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            ServoShooter1.setDirection(Servo.Direction.REVERSE);
            ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }



        public class RunShooter implements Action {

            private boolean initialized = false;
            private ElapsedTime timer; // To track time

            private double duration = 5;   // How long to run in seconds

            private double GoalTPS;
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

                    if (this.far){
                        //settings for far position
                        GoalTPS=1550;
                        ServoAngle = .6;
                    }else{
                        //settings for close position
                        GoalTPS=1400;
                        ServoAngle = .6;
                    }

                    ServoShooter1.setPosition(ServoAngle);
                    ServoShooter2.setPosition(ServoAngle);
                    ShooterRotatorServo.setPosition(.5);
                    //values for right in front.
//                    ShooterMotor.setPower(.6);
//                    ShooterMotor2.setPower(.6);
//                    ServoShooter1.setPosition(.32);
//                    ServoShooter2.setPosition(.32);


                }




                packet.put("time", timer.seconds());
                if(ShootingNow){//(timer.seconds() < duration) {

                    telemetry.addData("shooterTPS: ", shooterTPS);
                    telemetry.addData("GoalTPS: ", GoalTPS);
                    telemetry.addData("SpinShootingBallFeeder : ", SpinShootingBallFeeder);
                    telemetry.addData("ShooterMotorPower : ", ShooterMotorPower);

                    if (Math.abs(shooterTPS-GoalTPS)<=FunctionsAndValues.SpeedToleranceToStartShooting){SpinShootingBallFeeder=true;}
                    else{SpinShootingBallFeeder=false;
                        telemetry.addData("No SPinning Ball Feeder : ", "");
                    }


                    shooterTPS = ShooterMotor.getVelocity();

                    ShooterMotorPower = FAndV.handleShooter(shooterTPS,true, GoalTPS, ShooterMotorPower);
                    ShooterMotor.setPower(ShooterMotorPower);
                    ShooterMotor2.setPower(ShooterMotorPower);

                    telemetry.update();

                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action runShooter(boolean far) {return new RunShooter(far);}

    }





    @Override
    public void runOpMode() throws InterruptedException {
        double side = -1; // -1 is blue 1 is red
        double StartingAngle = (90*side); // =128
        double StartingX = -60.5;
        double StartingY = 36.5*side;

        double turn90Left = -90;//StartingAngle-90;
        double turn90Right = 90;//StartingAngle+90;

        Pose2d initialPose = new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle));//(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        Pose2d pose = drive.localizer.getPose();
        TrajectoryActionBuilder GoToShootPos = drive.actionBuilder(new Pose2d(StartingX+49,StartingY+(14*side) , Math.toRadians(StartingAngle)))
                .strafeTo(new Vector2d(-35, -35))
                .turn(Math.toRadians(-50))
                ;
        TrajectoryActionBuilder GoForwardsToIntakeBalls = drive.actionBuilder(new Pose2d(StartingX+49,StartingY+(-18*side) , Math.toRadians(StartingAngle)))
                //grab balls
                //.strafeTo(new Vector2d(StartingX+49, ))
                .lineToY(StartingY+(14*side))
                ;
        TrajectoryActionBuilder MoveTowardsIntakePosition = drive.actionBuilder(initialPose)
                //go to balls
                .strafeTo(new Vector2d(StartingX, StartingY+(-18*side)))
                .strafeTo(new Vector2d(StartingX+49, StartingY+(-18*side)))

                ;

        waitForStart();


        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(


                        //GetToBalls.build(),
                        //shooting stuff

//                        shooter.runShooter(),
//                        intake.passABallToShooter(),
//                        shooter.turnOffShooter()

                        MoveTowardsIntakePosition.build(),
                        intake.intakeOn(),
                        GoForwardsToIntakeBalls.build(),
                        intake.intakeOff(),
                        GoToShootPos.build(),
                        new ParallelAction(
                                shooter.runShooter(false),
                                intake.passABallToShooter()


                        )
                        /*MoveTowardsIntakePosition.build(),
                        intake.intakeOn(),
                        GoForwardsToIntakeBalls.build(),
                        intake.intakeOff(),
                        GoToShootPos.build()*/





                        )
                        /*

                        )*/


        );
    }
}