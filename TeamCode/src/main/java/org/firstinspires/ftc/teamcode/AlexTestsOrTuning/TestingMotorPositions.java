package org.firstinspires.ftc.teamcode.AlexTestsOrTuning;

// RR-specific imports

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//@Config
@Autonomous
@Disabled

public class TestingMotorPositions extends LinearOpMode {

    // Setting up drivetrain file

    //setting up motors and time

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {
        //Start a drive class to be able to use wheels

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBack");

        // 3. Set Motor Directions
        // This configuration is for a standard mecanum drive.
        // You may need to adjust this based on your robot's build.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

           if (gamepad1.x){
               leftFrontDrive.setPower(1);

           }else{
               leftFrontDrive.setPower(0);
           }
            if (gamepad1.y){
                rightFrontDrive.setPower(1);

            }else{
                rightFrontDrive.setPower(0);
            }
            if (gamepad1.a){
                leftBackDrive.setPower(1);

            }else{
                leftBackDrive.setPower(0);
            }
            if (gamepad1.b){
                rightBackDrive.setPower(1);

            }else{
                rightBackDrive.setPower(0);
            }


            telemetry.update();
        }
    }
}

