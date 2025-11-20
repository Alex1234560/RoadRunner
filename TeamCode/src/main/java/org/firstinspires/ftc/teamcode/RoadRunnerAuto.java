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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "RoadRunnerAuto", group = "Autonomous")
public class RoadRunnerAuto extends LinearOpMode {
    public class Intake {
        private DcMotorEx intakeMotor;
        private DcMotor StopIntakeMotor;

        public Intake(HardwareMap hardwareMap) {
            intakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
            StopIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        public class IntakeBalls implements Action {
            private boolean initialized = false;
            private ElapsedTime timer; // To track time
            private double duration = 2;   // How long to run in seconds

            public IntakeBalls() {
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
                    intakeMotor.setPower(0);
                    StopIntakeMotor.setPower(0);
                    return false;
                }
            }
        }

        public Action intakeBalls() {return new IntakeBalls();}

    }

    @Override
    public void runOpMode(){
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Intake intake = new Intake(hardwareMap);

        // actionBuilder builds from the drive steps passed to it
        Pose2d pose = drive.localizer.getPose();
        TrajectoryActionBuilder GetToBalls = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-11, 25))
                .turn(Math.toRadians(90))

                ;
        TrajectoryActionBuilder MoveForwards = drive.actionBuilder(initialPose)

                .strafeTo(new Vector2d(-11, 50))


                ;
        waitForStart();


        if (isStopRequested()) return;

        Actions.runBlocking(

                new SequentialAction(
                        //GetToBalls.build(),
                        new ParallelAction(
                        MoveForwards.build(),
                        intake.intakeBalls()

                        )
                )
        );
    }
}