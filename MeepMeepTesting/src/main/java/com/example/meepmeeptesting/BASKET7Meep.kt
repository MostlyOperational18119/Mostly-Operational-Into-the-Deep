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

class BASKET7Meep : Auto {
    val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(12.0),
        AngularVelocityConstraint(1.2)))

    val secondSlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(55.0),
        AngularVelocityConstraint(3.0)))

    val thirdSlowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(listOf(
        TranslationalVelocityConstraint(8.0),
        AngularVelocityConstraint(1.0)))

    override fun buildTrajectorySequence(drive: DriveShim, startPose: Pose2d): TrajectorySequence {
        return drive.trajectorySequenceBuilder(startPose)
            .setVelConstraint(slowConstraint)
            .setReversed(true)
            .splineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE 1
            .splineToLinearHeading(Pose2d(16.3, -61.78, Math.toRadians(0.00)),Math.toRadians(0.0))
            .waitSeconds(1.2)

            //BASKET 1
            .setVelConstraint(secondSlowConstraint)
            .setReversed(true)
            .splineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)), Math.toRadians(225.00))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE2
            .lineToLinearHeading(Pose2d(-49.0, -43.3, Math.toRadians(90.0)))
            .waitSeconds(1.2)

            //BASKET2
            .setVelConstraint(thirdSlowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE3
            .lineToLinearHeading(Pose2d(-58.5, -43.3, Math.toRadians(90.00)))
            .waitSeconds(1.2)

            //BASKET3
            .setVelConstraint(thirdSlowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE4
            .lineToLinearHeading(Pose2d(-60.5, -46.5, Math.toRadians(118.00)))
            .waitSeconds(1.2)

            //BASKET4
            .setVelConstraint(thirdSlowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-60.0, -60.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE5
            .splineTo(Vector2d(-24.5, -11.3), Math.toRadians(0.0))
            .waitSeconds(1.5)

            //BASKET5
            .setVelConstraint(secondSlowConstraint)
            .setReversed(true)
            .splineTo(Vector2d(-60.0, -60.0), Math.toRadians(225.0))
            .setReversed(false)
            .resetConstraints()

            .build()
    }
}
