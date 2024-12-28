package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleGoBildaPinpointMecanumDriveCancelable

@Autonomous(name = "GoBilda Test Autonomous")
class TestGoBildaAuto : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleGoBildaPinpointMecanumDriveCancelable(hardwareMap)
        drive.poseEstimate = Pose2d()

        telemetry.addLine("Init done")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addLine("Pose estimate: ${drive.poseEstimate}")
            telemetry.addLine("Pose velocity: ${drive.poseVelocity}")
            telemetry.addLine("Press A to continue")
            telemetry.update()

            if (gamepad1.a) break
        }

        // Start Auto
        val trajectorySequence = drive.trajectorySequenceBuilder(Pose2d())
            .forward(10.0)
            .build()

        drive.followTrajectorySequenceAsync(trajectorySequence)

        while (drive.isBusy) {
            drive.update()
            telemetry.addLine("Pose estimate: ${drive.poseEstimate}")
            telemetry.addLine("Pose velocity: ${drive.poseVelocity}")
            telemetry.update()
        }

        // End Auto
    }
}
