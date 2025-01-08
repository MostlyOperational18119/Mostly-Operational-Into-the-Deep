package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(-34.09, -63.19, Math.toRadians(-90.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(startPose)
                        .setReversed(true)
                        .lineToConstantHeading(new Vector2d(-10.04, -33.0))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(-48.25, -32.5, Math.toRadians(90.0)), Math.toRadians(90.00))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-57.0, -32.5, Math.toRadians(90.00))) // Get thing
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(-58.0, -32.5, Math.toRadians(122.00)))
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
                        .setReversed(false)
                        .lineTo(new Vector2d(-44.0, -44.0))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}