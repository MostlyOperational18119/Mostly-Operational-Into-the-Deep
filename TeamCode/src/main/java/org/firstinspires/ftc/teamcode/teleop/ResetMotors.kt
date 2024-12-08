package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Methods

@TeleOp(name = "RESETMOTORS", group = "Aardvark")
class ResetMotors : Methods() {
    override fun runOpMode() {
        // MOTORS
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]
        val slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        val slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        val hangerMotor = hardwareMap.dcMotor["hanger"]
        val tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        //MOTORS MODES
        motorBR.direction = DcMotorSimple.Direction.REVERSE
        motorFR.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideVerticalMotor)
        setMotorModePosition(hangerMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor)
        slideHorizontalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoder(tapeMeasureRotateMotor)
        setMotorModeEncoder(hangerMotor)
        tapeMeasureRotateMotor.targetPosition = 0
//        setMotorModeEncoder(grabberExtensionMotor)
//        setMotorModeEncoder(tapeMeasureRotateMotor)
    }
}