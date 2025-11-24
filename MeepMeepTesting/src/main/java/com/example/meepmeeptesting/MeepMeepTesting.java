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

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)//track width 15

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(StartingX, StartingY, Math.toRadians(StartingAngle))) // for next to wall on shooter thing

                        //.strafeTo(new Vector2d(0, squareSize))
                        //.waitSeconds(1)
                        //.turn(Math.toRadians(90))

                                //.strafeTo(new Vector2d(40, 0))
                        // Strafe to (50, 50) while turning to a 90-degree heading.
                        // The last argument is the end tangent, which affects the path shape.
                        //.strafeTo(new Vector2d(-11.1,39.1))

                        //.strafeTo(new Vector2d(-11,30))
                        // FROM:



                        //.strafeTo(new Vector2d(-11, 25))
                        //.turn(Math.toRadians(90))
                        //.strafeTo(new Vector2d(10, 53))
                        //go to balls
                        .strafeTo(new Vector2d(StartingX, StartingY+(-18*side)))
                        .strafeTo(new Vector2d(StartingX+49, StartingY+(-18*side)))
                        //grab balls
                        .strafeTo(new Vector2d(StartingX+49, StartingY+(14*side)))
                        //goback

                        .strafeTo(new Vector2d(-35, -35))
                        .turn(Math.toRadians(-50))
                        //.strafeTo(new Vector2d(20, 40*side))


                        //grab balls
                        //.strafeTo(new Vector2d(-11, 53*side))

//
                        //  go back to before grabbing balls
                        //.strafeTo(new Vector2d(-11, 33*side))
                        //turn to shooting place

                        //  go to shooting place
                       // .strafeTo(new Vector2d(-62, 33*side))









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