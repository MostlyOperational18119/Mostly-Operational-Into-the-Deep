package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        Auto targetAuto = new BAR5Meep();

        MeepMeep meepMeep = new MeepMeep(600);

        Pose2d startPose = new Pose2d(14.5, -63.19, Math.toRadians(-90.00));
        //Pose2d startPose = new Pose2d(-38.0, -63.19, Math.toRadians(0.00));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(70, 70, 3.5, 3.5, 16.5)
                .followTrajectorySequence(drive -> targetAuto.buildTrajectorySequence(drive, startPose));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.90f)
                .addEntity(myBot)
                .start();
    }
}