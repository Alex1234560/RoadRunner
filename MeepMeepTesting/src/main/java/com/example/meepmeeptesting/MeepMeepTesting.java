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
        MeepMeep meepMeep = new MeepMeep(800);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)

                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                        // Strafe to the destination while simultaneously turning to a 90-degree heading
                        .lineToLinearHeading(new Pose2d(-30, -30, Math.toRadians(-40)))


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