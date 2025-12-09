package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;
import javax.imageio.ImageIO;


public class MeepMeepTesting {



    public static void main(String[] args) {
        int side = 1; // - equal blue,+ equal red
        boolean FrontAuto=false;
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
            StartingAngle = (180);
            ShootAngle = (180);
            StartingX = 61.3;
            StartingY = 14 * side;

            initialPos = new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle));//(0, 0, Math.toRadians(0));
            IntakePosition = new Pose2d(36.1, 30 * side, Math.toRadians(StartingAngle));
            BallsIntakenPos = new Pose2d(36.1, 50 * side, Math.toRadians(StartingAngle));
            ShootPos = new Pose2d(StartingX, StartingY, ShootAngle); // not sure if work on both sides

            ParkPosition = new Pose2d(41, 26 * side, Math.toRadians(StartingAngle)); // not sure if work on both sides

        }

        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)//track width 15

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle))) // for next to wall on shooter thing
                        .strafeTo(new Vector2d(ParkPosition.getX(), ParkPosition.getY()))



                        .build());



        Image img = null;
        // New way with relative path
        try { img = ImageIO.read(new File("MeepMeepTesting/assets/Decode_Dark.jpeg")); }

        catch(IOException e) {}

        meepMeep.setBackground(img)
                //meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}