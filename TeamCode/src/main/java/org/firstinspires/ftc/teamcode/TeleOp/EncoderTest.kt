package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.util.Encoder
import java.util.Locale

@TeleOp(name = "EncoderTest")
@Disabled
class EncoderTest : LinearOpMode() {
    override fun runOpMode() {
        val leftEncoder = Encoder(
            hardwareMap.get(
                DcMotorEx::class.java, "motorFR"
            )
        )
        val rightEncoder = Encoder(
            hardwareMap.get(
                DcMotorEx::class.java, "motorBR"
            )
        )
        val frontEncoder = Encoder(
            hardwareMap.get(
                DcMotorEx::class.java, "motorBL"
            )
        )

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData(
                "Encoder positions",
                String.format(
                    Locale.ENGLISH,
                    "Left: %d, Right: %d, Front: %d",
                    leftEncoder.currentPosition,
                    rightEncoder.currentPosition,
                    frontEncoder.currentPosition
                )
            )
            telemetry.update()
        }
    }

}