package org.firstinspires.ftc.teamcode; // Make sure this matches your team's package name

import com.qualcomm.robotcore.util.Range;

/**
 * This class encapsulates all the logic for initializing and using the AprilTag processor.
 * It simplifies OpModes by hiding the complex setup of the VisionPortal and providing
 * easy-to-use methods for accessing detection data.
 */
public class FunctionsAndValues {
    public static double ToleranceForShooting = 40;

    public static double rotationCompensation = .09;
    public static double rotationTolerance = 1.5;
    public static double SpeedPrecise = 0.0005;
    public static double SmallRange = 40;

    public FunctionsAndValues() {

    }

    public double ReCalibrateShooterSpeed(double GoalTPS) {
        return calculateSpeedForShooter(GoalTPS);
    }
    public double handleShooter(double shooterTPS, double ShooterMotorSpeed, boolean isMotorOn, double GoalSpeedTPS) {


        double NewShooterMotorSpeed = ShooterMotorSpeed;

        double incrementAmount = SpeedPrecise;//test
        if (Math.abs(shooterTPS - GoalSpeedTPS) <= SmallRange){
            incrementAmount = 0;
        }


        if (isMotorOn) {

            if (shooterTPS < GoalSpeedTPS) {
                ShooterMotorSpeed += incrementAmount;
            }
            else if (shooterTPS > GoalSpeedTPS) {
                ShooterMotorSpeed -= incrementAmount;
            }

        } else {
            NewShooterMotorSpeed = 0;
        }

        return NewShooterMotorSpeed;

    }
    public double[] calculateShooterRotation(double bearing, boolean autorotate, double currentAngle) {

        double[] ValuesForAngleAndCurrentAngle = new double[2];

        double newCurrentAngle = currentAngle;

        if (Math.abs(bearing) > rotationTolerance && Math.abs(bearing) < 30 && autorotate) {

            newCurrentAngle += bearing * rotationCompensation;

            newCurrentAngle = Math.min(180, Math.max(0, newCurrentAngle));
        }


        //returning value that is mapped from degrees to 0-1.
        double valueForShooterServo = Range.scale(
                newCurrentAngle,   // value you want to map
                0, 180,        // input range
                0,  // output start
                1     // output end
        );

        ValuesForAngleAndCurrentAngle[0] = valueForShooterServo;
        ValuesForAngleAndCurrentAngle[1] = newCurrentAngle;

        return ValuesForAngleAndCurrentAngle;


        //telemetry.addData("current angle", currentAngle);
        //telemetry.addData("bearing", bearing);


    }
    //range to rpm and angle
    public double[] handleShootingRanges(double range) {

        double[] turretGoals = new double[2];

        double targAngle = (0.00458093 * range) + 0.340541;
        double targSpeed = (3.90841 * range) + 1409.28811;

        turretGoals[0] = targAngle;
        turretGoals[1] = targSpeed;
        return turretGoals;
    }
    //rpm to power
    public double calculateSpeedForShooter(double GoalTPS) {
        // tweak these numbers if you need to recalibrate
        double MotorSpeed = 0.000417188 * GoalTPS + 0.00873318;
        return MotorSpeed;
    }

    }


