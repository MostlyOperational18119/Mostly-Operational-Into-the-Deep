package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.VisionProcessors.SampleProcessor
import org.firstinspires.ftc.vision.VisionPortal
import android.util.Size

@TeleOp(name = "TestCVOpMode")
class TestCVOpMode : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addLine("Initialized")
        telemetry.update()

        waitForStart()

        val sampleProcessor = SampleProcessor()
        val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .addProcessors(sampleProcessor)
            .setCameraResolution(Size(1280, 720))
            .build()

        while (opModeIsActive()) {
            val detections = sampleProcessor.getResults().detections
            for (detection in detections) {
                telemetry.addLine(
                    String.format(
                        "%s Detection at (%i, %i), with points: %s",
                        detection.colour,
                        detection.x,
                        detection.y,
                        detection.pointsList
                    )
                )
            }

            telemetry.update()
        }
    }
}