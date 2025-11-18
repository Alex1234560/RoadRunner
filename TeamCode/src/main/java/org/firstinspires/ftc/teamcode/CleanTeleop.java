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

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp

public class CleanTeleop extends LinearOpMode {

    // Setting up drivetrain file


    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx IntakeMotor = null;
    private DcMotor StopIntakeMotor = null;
    private DcMotorEx ShooterMotor = null;
    private DcMotorEx ShooterMotor2 = null;
    private CRServo BallFeederServo = null;
    private CRServo ShooterRotatorServo = null;



    //Variables for statement printing
    double ShooterMotorSpeed = .8;
    double shooterVelocity = 0; // Ticks per second

    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off
    private boolean wasXButtonPressed = false;   // Tracks the button's state from the last loop
    // ------------

    // --- D-pad tracking ---
    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

// ----------------------------------------




    @Override
    public void runOpMode() {
        //initializeHardware()
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
        ShooterRotatorServo = hardwareMap.get(CRServo.class, "ShooterRotatorServo");


        // run shooter with encoder

        ShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //whats supposed to be in initialize hardware

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //handleDriving();
            double speed = .5;
            //if (gamepad1.right_trigger ==1){speed = 1;}
            speed += gamepad1.right_trigger/2;

            double axial = -gamepad1.left_stick_y *speed;
            double lateral = gamepad1.left_stick_x*speed; // Note: pushing stick forward gives negative value
            double yaw = gamepad1.right_stick_x*speed;

            //robotDrive.move(axial, lateral, yaw, speed);

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            axial,
                            -lateral
                    ),
                    -yaw
            ));
            // whats supposed to be in handle driving ^^

            handleIntake();
            handleShooter();

            //telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("ShooterPower= ", ShooterMotorSpeed);
            telemetry.addData("ShooterMotorTickPerSecond= ", shooterVelocity);
            //telemetry.addData("Intake speed =  ", intakeVelocity);

            telemetry.update();
        }
    }

    private void handleShooter(){
        shooterVelocity = ShooterMotor.getVelocity(); // Ticks per second

        boolean isXPressed = gamepad2.x;

        // 2. Check if the button was JUST pressed (it was up, but is now down)
        if (isXPressed && !wasXButtonPressed) {
            // Flip the state: if it was on, turn it off; if it was off, turn it on.
            shooterMotorOn = !shooterMotorOn;
        }
        // 3. Update the tracking variable for the next loop
        wasXButtonPressed = isXPressed;

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

        ShooterMotor.setPower(1*ShooterMotorSpeed);
        ShooterMotor2.setPower(-1*ShooterMotorSpeed);

        //handle rotation of shooter HEHEEEHHEHE

        if (gamepad2.dpad_left){ShooterRotatorServo.setPower(1);}
        else if (gamepad2.dpad_right){ShooterRotatorServo.setPower(-1);}
        else{ShooterRotatorServo.setPower(0);}



    }

    private void handleIntake(){
        double intakeVelocity = IntakeMotor.getVelocity(); // Ticks per second
        double TriggerValue =0;

        if (gamepad2.left_trigger > gamepad1.left_trigger){TriggerValue = gamepad2.left_trigger;}
        if (gamepad1.left_trigger > gamepad2.left_trigger){TriggerValue = gamepad1.left_trigger;}
        if (gamepad2.left_trigger == gamepad1.left_trigger){TriggerValue = gamepad2.left_trigger;}

        double intake=0;
        if ( intakeVelocity >= 0 && gamepad2.left_bumper){
            intake = TriggerValue;
        }
        else if (intakeVelocity <= 0){
            intake = -TriggerValue;
        }

        double StopIntake = gamepad2.right_trigger;

        IntakeMotor.setPower(intake);
        StopIntakeMotor.setPower(StopIntake);

        //handle feeder to launcher
        if (gamepad2.right_bumper){BallFeederServo.setPower(1);}
        else if (gamepad2.left_bumper){BallFeederServo.setPower(-1);}
        else{BallFeederServo.setPower(0);}

    }

    //private void InitializeHardware(){}

    //private void handleDriving(){}

    private void handleShooterServos(){
        //fill with shooter servo stuff
    }

}

