package com.example.meepmeeptesting

import com.acmerobotics.roadrunner.geometry.Pose2d
import org.rowlandhall.meepmeep.MeepMeep
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder
import org.rowlandhall.meepmeep.roadrunner.DriveShim
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence

abstract class AutoBoilerplateSingle {
    fun run() {
        val meepMeep = MeepMeep(800)

        val bot = DefaultBotBuilder(meepMeep)
            .setStartPose(startPose)
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .followTrajectorySequence {getAutoTrajectorySequence(it, startPose)}

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(1.0f)
            .addEntity(bot)
            .start()
    }

    abstract val startPose: Pose2d
    abstract fun getAutoTrajectorySequence(drive: DriveShim, startPose2d: Pose2d): TrajectorySequence
}