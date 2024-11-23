package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor

abstract class DriveMethods : LinearOpMode() {
    var motorFL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorBR: DcMotor? = null

    fun initMotors() {
        motorFL = hardwareMap.get(DcMotor::class.java, "motorFL")
        motorFR = hardwareMap.get(DcMotor::class.java, "motorFR")
        motorBL = hardwareMap.get(DcMotor::class.java, "motorBL")
        motorBR = hardwareMap.get(DcMotor::class.java, "motorBR")
    }
}