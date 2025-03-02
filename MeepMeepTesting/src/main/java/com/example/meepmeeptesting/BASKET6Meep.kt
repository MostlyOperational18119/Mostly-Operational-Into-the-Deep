package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint
import org.rowlandhall.meepmeep.roadrunner.DriveShim
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence

class BASKET6Meep : Auto {
    val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(13.0),
        AngularVelocityConstraint(1.25)))

    val basket1SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(44.0),
        AngularVelocityConstraint(3.0)))

    val basket2SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(3.9),
        AngularVelocityConstraint(1.0)))

    val basket3SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(3.4),
        AngularVelocityConstraint(1.0)))

    val basket4SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(3.9),
        AngularVelocityConstraint(1.0)))

    val basket5SlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(37.0),
        AngularVelocityConstraint(2.5)))

    override fun buildTrajectorySequence(drive: DriveShim, startPose: Pose2d): TrajectorySequence {
        return drive.trajectorySequenceBuilder(startPose)
            .setVelConstraint(slowConstraint)
            .setReversed(true)
            .splineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE 1
            .splineToLinearHeading(Pose2d(19.5, -63.75, Math.toRadians(-6.0)), Math.toRadians(-6.0))
            .waitSeconds(1.2)

            //BASKET 1
            .setVelConstraint(basket1SlowConstraint)
            .setReversed(true)
            .splineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE2
            .lineToLinearHeading(Pose2d(-49.0, -50.3, Math.toRadians(90.0)))
            .waitSeconds(1.2)

            //BASKET2
            .setVelConstraint(basket2SlowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE3
            .lineToLinearHeading(Pose2d(-58.5, -53.3, Math.toRadians(90.00)))
            .waitSeconds(1.2)

            //BASKET3
            .setVelConstraint(basket3SlowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE4
            .lineToLinearHeading(Pose2d(-57.5, -46.5, Math.toRadians(130.00)))
            .waitSeconds(1.2)

            //BASKET4
            .setVelConstraint(basket4SlowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE5
            //.turn(Math.toRadians(-30.0))
            .splineTo(Vector2d(-22.0, -11.3), Math.toRadians(0.0))
            .waitSeconds(1.5)

            //BASKET5
            .setVelConstraint(basket5SlowConstraint)
            .setReversed(true)
            .splineTo(Vector2d(-60.0, -60.0), Math.toRadians(225.0))
            .setReversed(false)
            .resetConstraints()

            .build()
    }
}
