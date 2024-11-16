package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-34.09, -63.19, Math.toRadians(-90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .lineToConstantHeading(new Vector2d(-10.04, -34.01))
                        .setReversed(false)
                        .splineTo(new Vector2d(-47.87, -39.89), Math.toRadians(90.00))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                        .back(2)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-57.75, -38.37, Math.toRadians(88.83)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                        .back(2)
                        .setReversed(false)

                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}