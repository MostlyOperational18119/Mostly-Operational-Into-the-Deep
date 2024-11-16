package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.geometry.Vector2d;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);
        Pose2d startPose = new Pose2d(-34.09, -63.19, Math.toRadians(90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .splineTo(new Vector2d(-39.64, -47.68), Math.toRadians(144.20))
                        .splineTo(new Vector2d(-58.26, -57.0), Math.toRadians(225.00))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-20.0,-45.0, Math.toRadians(135.0)))
                        .setReversed(false)
                        .splineTo(new Vector2d(-31.64, -37.09), Math.toRadians(135.20))
                        .lineTo(new Vector2d(-39.5, -37.09))
                        .back(10.0)
                        .setReversed(false)
                        .splineTo(new Vector2d(-58.26, -57.0), Math.toRadians(225.00))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-37.83, -13.00, Math.toRadians(180.0)))
                        .lineTo(new Vector2d(-56.00, -13.00))
                        .lineToConstantHeading(new Vector2d(-60.03, -57.00))
                        .setReversed(false)
                        .waitSeconds(0.25)
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK )
                .setDarkMode(true)
                .setBackgroundAlpha(1.0f)
                .addEntity(myBot)
                .start();
    }
}