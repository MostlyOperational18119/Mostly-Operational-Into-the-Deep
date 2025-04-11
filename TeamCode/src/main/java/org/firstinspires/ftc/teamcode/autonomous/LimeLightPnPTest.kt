package org.firstinspires.ftc.teamcode.autonomous

import android.util.Log
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.LimeLightPipelines
import org.firstinspires.ftc.teamcode.Methods
import java.util.Locale

@Autonomous
class LimeLightPnPTest: Methods() {
    override fun runOpMode() {
        val limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
//        switchPipelineEnum(limelight, PipelineType.SnapScript)
//
//        val script = formatSnapScript(LimeLightPipelines.RedLower, LimeLightPipelines.RedUpper)
//        Log.i("A Script Thing", "It: $script")
//
//        limelight.uploadPython(script, 0)

        telemetry.addLine("Init done")
        telemetry.update()

        waitForStart()

        limelight.start()

        var lastKnownTvec = arrayOf(0.0, 0.0, 0.0)

        while (opModeIsActive()) {
            telemetry.addLine("Status: ${limelight.status}")

            val latestResult = limelight.latestResult
            if (latestResult != null) {
                val pythonResult = latestResult.pythonOutput

                if (pythonResult[4] == -1.0) {
                    val tvec = arrayOf(pythonResult[5], pythonResult[6], pythonResult[7])
                    lastKnownTvec = tvec

                    telemetry.addLine(
                        String.format(Locale.US, "tvec: [%.2f, %.2f, %.2f]", tvec[0], tvec[1], tvec[2])
                    )
                } else {
                    telemetry.addLine(
                        String.format(
                            Locale.US,
                            "No tvec, last one was [%.2f, %.2f, %.2f]",
                            lastKnownTvec[0],
                            lastKnownTvec[1],
                            lastKnownTvec[2]
                        )
                    )
                }
            } else {
                telemetry.addLine("Latest results are null :(")
            }

            telemetry.update()
        }
    }
}