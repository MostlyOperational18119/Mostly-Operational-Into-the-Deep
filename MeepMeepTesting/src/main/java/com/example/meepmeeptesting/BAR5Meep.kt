package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import org.rowlandhall.meepmeep.roadrunner.DriveShim
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import java.util.Arrays

class BAR5Meep : Auto {

    val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
        Arrays.asList(
            TranslationalVelocityConstraint(20.0),
            AngularVelocityConstraint(1.0)
        )
    )

    override fun buildTrajectorySequence(drive: DriveShim, startPose: Pose2d): TrajectorySequence {
        return drive.trajectorySequenceBuilder(startPose)
            .lineToConstantHeading(Vector2d(-6.0,-33.5))

            .waitSeconds(0.01)

            //SAMPLE1
            .splineTo(Vector2d(36.05, -29.0), Math.toRadians(40.00))
            .lineToLinearHeading(Pose2d(37.03, -46.07, Math.toRadians(-30.00)))

            //SAMPLE2
            .lineToLinearHeading(Pose2d(46.26, -29.37, Math.toRadians(35.00)))
            .lineToLinearHeading(Pose2d(43.71, -45.68, Math.toRadians(-35.00)))

            //SAMPLE 3
            .lineToLinearHeading(Pose2d(56.87, -28.76, Math.toRadians(35.00)))
            .setVelConstraint(slowConstraint)
            .setReversed(true)
            .splineToLinearHeading(Pose2d(47.44, -60.20, Math.toRadians(-90.00)), Math.toRadians(-90.0))
            .setReversed(false)
            .resetConstraints()

            //PICK1
            //BAR 1
            .setReversed(true)
            .splineToConstantHeading(Vector2d(-4.0, -33.5), Math.toRadians(90.0))
            .setReversed(false)

            //PICK2
            .splineToLinearHeading(Pose2d(47.44, -60.20,Math.toRadians(90.0)), Math.toRadians(-90.00))
            //BAR 2
            .splineToConstantHeading(Vector2d(-2.0, -33.5), Math.toRadians(90.0))

            //PICK3
            .setReversed(true)
            .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
            .setReversed(false)
            //BAR 3
            .splineToConstantHeading(Vector2d(0.0, -33.5), Math.toRadians(90.0))

            //PICK4
            .setReversed(true)
            .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
            .setReversed(false)
            //BAR 4
            .splineToConstantHeading(Vector2d(2.0, -33.5), Math.toRadians(90.0))

            //PICK5
            .setReversed(true)
            .splineToConstantHeading(Vector2d(47.44, -60.20), Math.toRadians(-90.00))
            .setReversed(false)
            //BAR 5
            .splineToConstantHeading(Vector2d(4.0, -33.5), Math.toRadians(90.0))

            .lineToConstantHeading(Vector2d(43.25, -55.55))
            .build()
    }
}
