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
    public static int side = -1; // - equal blue,+ equal red
    public static double StartingAngle = (90*side); // =128
    public static double StartingX = -60.5;
    public static double StartingY = 36.5*side;


    static double ShootAngle = Math.toRadians( (50+180)*side ); // not sure if it works on other side ( 1 )



    static Pose2d initialPos = new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle));//(0, 0, Math.toRadians(0));
    static Pose2d IntakePosition = new Pose2d(-11.5, 18.5*side , Math.toRadians(StartingAngle));
    static Pose2d BallsIntakenPos = new Pose2d(-11.5, 46.5*side , Math.toRadians(StartingAngle));
    static Pose2d ShootPos = new Pose2d(-35, 35*side , ShootAngle); // not sure if work on both sides


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)//track width 15

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle))) // for next to wall on shooter thing
                        .strafeTo(new Vector2d(IntakePosition.getX(), IntakePosition.getY()))




                        .strafeTo(new Vector2d(BallsIntakenPos.getX(), BallsIntakenPos.getY()))
                        .strafeTo(new Vector2d(ShootPos.getX(), ShootPos.getY()))



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