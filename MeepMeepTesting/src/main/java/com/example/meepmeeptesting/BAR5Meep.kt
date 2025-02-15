package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.rowlandhall.meepmeep.roadrunner.DriveShim
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence

class BAR5Meep : Auto {

    override fun buildTrajectorySequence(drive: DriveShim, startPose: Pose2d): TrajectorySequence {
        return drive.trajectorySequenceBuilder(startPose)
            .lineToConstantHeading(Vector2d(4.4,-32.0))

            .waitSeconds(0.01)

            //SAMPLE1
            .splineTo(Vector2d(36.05, -29.0), Math.toRadians(40.00))
            .lineToLinearHeading(Pose2d(37.03, -46.07, Math.toRadians(-30.00)))

            //SAMPLE2
            .lineToLinearHeading(Pose2d(46.26, -29.37, Math.toRadians(35.00)))
            .lineToLinearHeading(Pose2d(43.71, -45.68, Math.toRadians(-35.00)))

            //SAMPLE 3
            .lineToLinearHeading(Pose2d(57.87, -29.76, Math.toRadians(35.00)))
            .setReversed(true)
            .splineToLinearHeading(Pose2d(47.44, -60.20, Math.toRadians(-90.00)), Math.toRadians(-90.0))
            .setReversed(false)

            //PICK1
            //BAR 1
            .setReversed(true)
            .splineToConstantHeading(Vector2d(3.0, -33.5), Math.toRadians(90.0))
            .setReversed(false)
            //PICK2
            .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
            //BAR 2
            .setReversed(true)
            .splineToConstantHeading(Vector2d(1.5, -33.5), Math.toRadians(90.0))
            .setReversed(false)

            //PICK3
            .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
            //BAR 3
            .setReversed(true)
            .splineToConstantHeading(Vector2d(0.0, -33.5), Math.toRadians(90.0))
            .setReversed(false)

            //PICK4
            .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
            //BAR 4
            .setReversed(true)
            .splineToConstantHeading(Vector2d(0.0, -33.5), Math.toRadians(90.0))
            .setReversed(false)

            //PICK5
            .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
            //BAR 5
            .setReversed(true)
            .splineToConstantHeading(Vector2d(-1.5, -33.5), Math.toRadians(90.0))
            .setReversed(false)

            .lineToConstantHeading(Vector2d(43.25, -55.55))
            .build()
    }
}
