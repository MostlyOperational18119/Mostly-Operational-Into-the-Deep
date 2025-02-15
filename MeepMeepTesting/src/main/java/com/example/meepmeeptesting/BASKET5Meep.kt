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

class BASKET5Meep : Auto {
    val slowConstraint: TrajectoryVelocityConstraint = MinVelocityConstraint(
        Arrays.asList(
            TranslationalVelocityConstraint(20.0),
            AngularVelocityConstraint(1.0)
        )
    )

    override fun buildTrajectorySequence(drive: DriveShim, startPose: Pose2d): TrajectorySequence {
        return drive.trajectorySequenceBuilder(startPose)
            .waitSeconds(0.3)
            .setVelConstraint(slowConstraint)
            .setReversed(true)
            .splineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)), Math.toRadians(225.00))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE 1
            .splineToLinearHeading(Pose2d(22.30, -61.78, Math.toRadians(0.00)),Math.toRadians(0.0))
            .waitSeconds(1.5)

            //BASKET 1
            .setReversed(true)
            .splineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)), Math.toRadians(225.00))
            .setReversed(false)

            //SAMPLE2
            .lineToLinearHeading(Pose2d(-48.75, -43.3, Math.toRadians(90.0)))
            .waitSeconds(1.5)

            //BASKET2
            .setVelConstraint(slowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE3
            .lineToLinearHeading(Pose2d(-59.0, -43.3, Math.toRadians(90.00)))
            .waitSeconds(1.5)

            //BASKET3
            .setVelConstraint(slowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //SAMPLE4
            .lineToLinearHeading(Pose2d(-58.5, -46.5, Math.toRadians(118.00)))
            .waitSeconds(1.5)

            //BASKET4
            .setVelConstraint(slowConstraint)
            .setReversed(true)
            .lineToLinearHeading(Pose2d(-57.0, -57.0, Math.toRadians(45.00)))
            .setReversed(false)
            .resetConstraints()

            //END
            .splineTo(Vector2d(-24.5, -11.3), Math.toRadians(0.0))
            .build()
    }
}
