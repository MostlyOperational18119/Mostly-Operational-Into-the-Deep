package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "TestLimelightOpMode")
@Disabled
class TestLimelightOpMode : LinearOpMode() {
    override fun runOpMode() {
        val limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight")

        telemetry.addLine("Initialized")
        telemetry.update()

        waitForStart()

        limelight3A.start()

        while (opModeIsActive()) {
            telemetry.addData("Limelight status: ", limelight3A.status)

            val latestResult = limelight3A.latestResult
            if (latestResult != null) {
                telemetry.addData("Current robot pose: ", latestResult.botpose)

            }

            telemetry.update()
            sleep(100)
        }
    }
}