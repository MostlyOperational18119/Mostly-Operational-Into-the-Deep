package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.DriveMethods

@TeleOp(name = "TeleopFromHell", group = "TeleopFinal")
@Disabled
class FirstTeleop: DriveMethods() {
    override fun runOpMode() {

        val mFL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val mFR = hardwareMap.get(DcMotor::class.java, "motorFR")
        val mBL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val mBR = hardwareMap.get(DcMotor::class.java, "motorBR")
        mFR.direction = DcMotorSimple.Direction.REVERSE
        mBR.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        while (opModeIsActive()){
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()

            mFL.power = y + x + rx
            mBL.power = y - x + rx
            mFR.power = y - x - rx
            mBR.power = y + x - rx

        }
    }
}