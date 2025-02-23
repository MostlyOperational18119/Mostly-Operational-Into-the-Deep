package org.firstinspires.ftc.teamcode.autonomous.goBuilda

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.drive.SampleGoBildaPinpointMecanumDriveCancelable

@Autonomous(name = "GoBilda Test Autonomous")
@Disabled
class TestGoBildaAuto : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleGoBildaPinpointMecanumDriveCancelable(hardwareMap)

        sleep(250)

        telemetry.addLine("Init done")
        telemetry.update()

        waitForStart()

        // Create trajectory sequence
        val trajectorySequence = drive.trajectorySequenceBuilder(Pose2d())
            .forward(42.0)
            .strafeRight(84.0)
            .back(84.0)
            .strafeLeft(84.0)
            .forward(42.0)
            .build()

        while (opModeIsActive()) {
            telemetry.addLine("Pose estimate: ${drive.poseEstimate}")
            telemetry.addLine("Pose velocity: ${drive.poseVelocity}")
            telemetry.addLine("Press A to continue")
            telemetry.update()

            if (gamepad1.a) break
        }

        if (!opModeIsActive()) return

        while (opModeIsActive()) {
            // Start Auto movement
            if (!drive.isBusy) drive.followTrajectorySequenceAsync(trajectorySequence)

            drive.update()

            telemetry.addLine("Pose estimate: ${drive.poseEstimate}")
            telemetry.addLine("Pose velocity: ${drive.poseVelocity}")
            telemetry.update()

            sleep(50)
        }

        // End Auto
    }
}
