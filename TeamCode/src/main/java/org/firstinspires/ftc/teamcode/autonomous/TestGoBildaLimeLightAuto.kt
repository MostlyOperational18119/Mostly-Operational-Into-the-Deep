package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drive.SampleGoBildaPinpointMecanumDriveCancelable

@Autonomous(name = "GoBilda + LimeLight Test Autonomous")
class TestGoBildaLimeLightAuto : LinearOpMode() {
    override fun runOpMode() {
        val drive = SampleGoBildaPinpointMecanumDriveCancelable(hardwareMap)
        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

        telemetry.addLine("Init done")
        telemetry.update()

        waitForStart()

        telemetry.addLine("Finding clip")
        telemetry.update()

        var choice = -1

        while (choice == -1 && !isStopRequested) {
            telemetry.addLine("A for RED \nB for BLUE \nX for ORANGE")
            telemetry.update()
            choice = if (gamepad1.a) 0 else if (gamepad1.b) 1 else if (gamepad1.x) 2 else -1

            sleep(50)
        }

        if (isStopRequested) return

        limelight.pipelineSwitch(choice)
        limelight.start()

        sleep(1000) // Just to be safe

        val llResults = limelight.latestResult
        var lMove = 0.0 // Add offsets from camera space to centered robot here
        var fMove = 0.0

        if (llResults.colorResults.isNotEmpty()) {
            val colorResult = llResults.colorResults[0]

            // Center to sample
            val inchSamplePose = colorResult.targetPoseRobotSpace.position.toUnit(DistanceUnit.INCH)
            lMove += inchSamplePose.x
            fMove += inchSamplePose.z
        }

        // Start Auto Movement
        val trajectorySequence = drive.trajectorySequenceBuilder(Pose2d())
            .strafeLeft(lMove)
            .forward(fMove)
            .build()

        drive.followTrajectorySequence(trajectorySequence)

        // End Auto Movement
    }
}
