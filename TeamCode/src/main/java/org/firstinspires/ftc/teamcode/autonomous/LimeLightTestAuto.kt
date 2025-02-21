package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Methods

@Autonomous(name = "LimeLight Test Autonomous")
class LimeLightTestAuto : Methods() {
    override fun runOpMode() {
//        val drive = SampleMecanumDriveCancelable(hardwareMap)
        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

        telemetry.addLine("Init done")
        telemetry.update()

        waitForStart()

        telemetry.addLine("Finding clip")
        telemetry.update()

        var choice: PipelineType? = null

        while (choice == null && !isStopRequested) {
            telemetry.addLine("A for RED \nB for BLUE \nX for ORANGE")
            telemetry.update()
            choice = if (gamepad1.a) PipelineType.Red else if (gamepad1.b) PipelineType.Blue else if (gamepad1.x) PipelineType.Orange else null

            sleep(50)
        }

        if (isStopRequested) return

        switchPipelineEnum(limelight, choice!!)
        limelight.start()

        sleep(1000) // Just to be safe

        /*
        val llResults = limelight.latestResult
        var lMove = 0.0 // Add offsets from camera space to centered robot here
        var fMove = 0.0


        if (llResults.colorResults.isNotEmpty()) {
            val colorResult = llResults.colorResults[0]

            // Center to sample
            val inchSamplePose = colorResult.targetPoseRobotSpace.position.toUnit(DistanceUnit.INCH)
            lMove += inchSamplePose.x
            fMove += inchSamplePose.z

            telemetry.addLine("Result: (${inchSamplePose.x}, ${inchSamplePose.y}, ${inchSamplePose.z})")
        }

        // Start Auto Movement
        val trajectorySequence = drive.trajectorySequenceBuilder(Pose2d())
            .strafeLeft(lMove)
            .forward(fMove)
            .build()
         */

        while(opModeIsActive()) {
            val llResults = limelight.latestResult

            if (llResults.colorResults.isNotEmpty()) {
                llResults.colorResults.forEach {
                    val inchSamplePose = it.targetPoseCameraSpace.position.toUnit(DistanceUnit.INCH)

                    telemetry.addLine("Result: (${inchSamplePose.x}, ${inchSamplePose.y}, ${inchSamplePose.z})")
                    telemetry.update()
                }
            } else {
                telemetry.addLine("No Result")
                telemetry.update()
            }

            sleep(50)

        }

//        drive.followTrajectorySequence(trajectorySequence)

        // End Auto Movement
    }
}
