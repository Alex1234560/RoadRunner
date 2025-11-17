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
    public static int squareSize = 48;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, Math.toRadians(0)))

                        //.strafeTo(new Vector2d(0, squareSize))
                        //.waitSeconds(1)
                        //.turn(Math.toRadians(90))

                                //.strafeTo(new Vector2d(40, 0))
                        // Strafe to (50, 50) while turning to a 90-degree heading.
                        // The last argument is the end tangent, which affects the path shape.
                        .strafeTo(new Vector2d(-11.1,39.1))




                        //.(new Vector2d(0, 0),Math.toRadians(180))


                        .waitSeconds(5)
                        //.strafeTo(new Vector2d(0, 0))


                        //.splineToLinearHeading(new Pose2d(12, 12, Math.toRadians(-40)), 1)


                        /*.forward(30)

                            .lineToX(DISTANCE)
                            .lineToX(0)
                            .build());

                        .forward(30)

                        .forward(30)
                        .turn(Math.toRadians(90))
                        .forward(30)
                        .turn(Math.toRadians(90))
                        .*/
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