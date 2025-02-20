package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class BASKET3Meep implements Auto {
    public TrajectorySequence buildTrajectorySequence(DriveShim drive, Pose2d startPose) {
        return drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(4.4, -32.0))
                .waitSeconds(0.01)
                .splineTo(new Vector2d(36.05, -29.0), Math.toRadians(40.00))
                .lineToLinearHeading(new Pose2d(37.03, -46.07, Math.toRadians(-30.00)))
                .lineToLinearHeading(new Pose2d(46.26, -29.37, Math.toRadians(35.00)))
                .lineToLinearHeading(new Pose2d(43.71, -45.68, Math.toRadians(-35.00)))
                .lineToLinearHeading(new Pose2d(56.87, -29.76, Math.toRadians(35.00)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(47.44, -60.20, Math.toRadians(-90.00)), Math.toRadians(-90.0))
                .splineToConstantHeading(new Vector2d(0.3, -33.5), Math.toRadians(90.0))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(47.44, -60.20), Math.toRadians(-90.00))
                .build();
    }
}
