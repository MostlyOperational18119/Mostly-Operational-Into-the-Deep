package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BASKET3 implements Auto {
    public TrajectorySequence buildTrajectorySequence(DriveShim drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
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
                .build();
    }
}
