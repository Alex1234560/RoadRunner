package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
            private double duration = 5;   // How long to run in seconds

            public PassABallToShooter() {
                this.timer = new ElapsedTime(); // This creates the timer object.
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();

                    BallFeederServo.setPower(.5);
                    telemetry.addData("Ball Feeder Servo Activated  ", 1 );
                    intakeMotor.setPower(.5);
                   telemetry.addData("IntakeMotor  ", 1 );
                    StopIntakeMotor.setPower(.5);
                    telemetry.addData("StopIntakeMotor  ", 1 );
                    telemetry.update();


                    initialized = true;
                }


                packet.put("time", timer.seconds());
                if (timer.seconds() < duration) {
//                    if (timer.seconds()<1.25){
//                        BallFeederServo.setPower(.5);
//                        telemetry.addData("Ball Feeder Servo Activated  ", 1 );
//                        intakeMotor.setPower(.5);
//                        telemetry.addData("IntakeMotor  ", 1 );
//                        StopIntakeMotor.setPower(.5);
//                    }




                    return true;
                } else {

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


        }

        public class TurnOffShooter implements Action {
            private boolean initialized = false;
            private ElapsedTime timer; // To track time
            private double duration = .1;   // How long to run in seconds

            public TurnOffShooter() {
                this.timer = new ElapsedTime(); // This creates the timer object.
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;

                    ShooterMotor.setPower(.6);
                    ShooterMotor2.setPower(.6);
                    ServoShooter1.setPosition(.32);
                    ServoShooter2.setPosition(.32);
                    ShooterRotatorServo.setPosition(.5);
                }
                packet.put("time", timer.seconds());
                if (timer.seconds() < duration) {
                    return true;
                } else {
                    ShooterMotor.setPower(0);
                    ShooterMotor2.setPower(0);
                    return false;
                }
            }
        }

        public Action turnOffShooter() {return new TurnOffShooter();}

        public class RunShooter implements Action {
            private boolean initialized = false;
            private ElapsedTime timer; // To track time
            private double duration = .01;   // How long to run in seconds

            public RunShooter() {
                this.timer = new ElapsedTime(); // This creates the timer object.
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    timer.reset();
                    initialized = true;

                    ShooterMotor.setPower(.62);
                    ShooterMotor2.setPower(.62);
                    ServoShooter1.setPosition(.33);
                    ServoShooter2.setPosition(.33);
                    ShooterRotatorServo.setPosition(.5);


                }


                packet.put("time", timer.seconds());
                if (timer.seconds() < duration) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action runShooter() {return new RunShooter();}

    }

    @Override
    public void runOpMode(){
        double side = -1; // -1 is blue 1 is red
        double StartingAngle = (90*side); // =128
        double turn90Left = -90;//StartingAngle-90;
        double turn90Right = 90;//StartingAngle+90;

        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));//(-45, 53, Math.toRadians(StartingAngle));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);
        Shooter shooter = new Shooter(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        Pose2d pose = drive.localizer.getPose();
        TrajectoryActionBuilder GoBackwards = drive.actionBuilder(initialPose)
                //.strafeTo(new Vector2d(-11, 20*side))
                //.turn(Math.toRadians((-90-38)*side)) // turns 90 degrees to right
                //.strafeTo(new Vector2d(-11, 33*side))
                //.strafeTo(new Vector2d(10, 20*side))
                ;
        TrajectoryActionBuilder DriveRightAwayFromShootingArea = drive.actionBuilder(initialPose)
                //.strafeTo(new Vector2d(-11, 20*side))
                //.turn(Math.toRadians((-90-38)*side)) // turns 90 degrees to right
               //.strafeTo(new Vector2d(10, 53))
                //.strafeTo(new Vector2d(10, 20*side))
                ;
        TrajectoryActionBuilder MoveForwards = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(0, 24))

                ;
        TrajectoryActionBuilder GoToShootPos = drive.actionBuilder(initialPose)

                //.strafeTo(new Vector2d(-11, 33*side))
                //turn to shooting place
                //  go to shooting place
                //.strafeTo(new Vector2d(-62, 33*side))


                ;
        waitForStart();


        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(


                        //GetToBalls.build(),
                        //shooting stuff
                        shooter.runShooter(),
                        intake.passABallToShooter(),
                        shooter.turnOffShooter()

                        //MoveForwards.build()




                        )
                        /*new ParallelAction(
                                intake.passABallToShooter(),
                                shooter.runShooter()

                        )*/


        );
    }
}