package org.firstinspires.ftc.teamcode.autonomous

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Methods
import org.opencv.calib3d.Calib3d
import org.opencv.core.Core

@Autonomous(name = "LimeLight Test Autonomous2")
class LimeLightTestAuto2 : Methods() {
    override fun runOpMode() {
        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")

        System.loadLibrary(Core.NATIVE_LIBRARY_NAME)



        telemetry.addLine("Init done")
        telemetry.update()

        waitForStart()

        telemetry.addLine("Finding clip")
        telemetry.update()

        if (isStopRequested) return

        switchPipelineEnum(limelight, PipelineType.Red)
        limelight.start()

        sleep(1000) // Just to be safe

        while(opModeIsActive()) {
            val llResults = limelight.latestResult

            if (llResults.colorResults.isNotEmpty()) {
                llResults.colorResults.forEach {
                    val inchSamplePose = it.targetPoseCameraSpace.position.toUnit(DistanceUnit.INCH)
                    it.targetCorners.forEach { it2 ->
                        telemetry.addLine("Corner: (${it2[0]}, ${it2[1]})")
                    }

//                    telemetry.addLine("Result Pose: (${inchSamplePose.x}, ${inchSamplePose.y}, ${inchSamplePose.z})")
//                    telemetry.addLine("Result Orientation: (${it.targetPoseCameraSpace.orientation.pitch}, ${it.targetPoseCameraSpace.orientation.yaw}, ${it.targetPoseCameraSpace.orientation.roll})")
                    telemetry.addLine("Result Degrees: (${it.targetXDegreesNoCrosshair}, ${it.targetYDegreesNoCrosshair})")
                    telemetry.update()
                }
            } else {
                telemetry.addLine("No Result")
                telemetry.update()
            }

            sleep(50)

        }

    }
}
