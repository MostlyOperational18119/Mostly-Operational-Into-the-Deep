package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad


abstract class DriveMethods : LinearOpMode() {
    val clawRotateRest = 0.64
    val clawRotateUpRight = 0.48
    val clawRotateOut = 0.0
    val clawRotateStraight = 0.08
    val clawRotateWall = 0.16
    val transferDownPos = 0.57
    val servoHangActive = 0.44
    val servoHangPassive = 0.3
    val transferMidPos = 0.4
    val transferUpPos = 0.20
    val clawServoOpen = 0.1
    val clawServoClosed = 0.22

    var motorFL: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBR: DcMotor? = null
    var slideVerticalMotor: DcMotor? = null
    var slideHorizontalMotor: DcMotor? = null
    var hangerMotor: DcMotor? = null
    var tapeMeasureRotateMotor: DcMotor? = null

    val controller1 = Gamepad()
    val controller2 = Gamepad()
    val previousController1 = Gamepad()
    val previousController2 = Gamepad()

    fun setMotorModeEncoder(motor: DcMotor) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = 0.0
    }

    fun setMotorModePosition(motor: DcMotor) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
}