package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.autonomous.goBuilda.SampleGoBildaPinpointMecanumDrive

@Autonomous(name = "GoBilda Test Autonomous")
class TestGoBildaAuto : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleGoBildaPinpointMecanumDrive(hardwareMap)

        telemetry.addLine("Init done")
        telemetry.update()

        // Start Auto
        val trajectorySequence = drive.trajectorySequenceBuilder(Pose2d())
            .forward(10.0)
            .build()

        drive.followTrajectorySequence(trajectorySequence)

        // End Auto
    }
}
