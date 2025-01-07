package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.Methods


@Autonomous(name = "SIMPLEBASKET3")
class SIMPLEBASKET3 : Methods() {
    override fun runOpMode() {
        initMotors()
        initOdometry()

        waitForStart()

        val toBasket = drive!!.trajectorySequenceBuilder(Pose2d(-36.49, -61.17, Math.toRadians(90.00)))
            .splineTo(Vector2d(-55.49, -54.60), Math.toRadians(-45.0))
            .build()

        val toRest = drive!!.trajectorySequenceBuilder(toBasket.end())
            .splineTo(Vector2d(-59.57, -26.90), Math.toRadians(90.0))
            .build()

        drive!!.followTrajectorySequence(toBasket)


    }
}