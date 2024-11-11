package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale


// Autonomous

@Autonomous(name = "Straight", group = "B")
@Disabled
class Stest : DriveMethods() {
    override fun runOpMode() {
        // Setup Odometry :)
        val drive = SampleMecanumDrive(hardwareMap)

        // Setup up the trajectory sequence (drive path)
        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(0.0, 0.0, Math.toRadians(90.0)))
                .splineTo(Vector2d(10.0, 0.0), Math.toRadians(90.0))
                .build()

        // Tell the User the Robot has been initialized
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        drive.followTrajectorySequenceAsync(traj1)
        while (opModeIsActive() && !isStopRequested) {
            drive.update()
        }
    }
}