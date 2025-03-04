package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable

@TeleOp
class AutoLocalizer : Methods() {
    override fun runOpMode() {
        initOdometry()
        initMotors()

        waitForStart()

        while (opModeIsActive()) {
            motorFL!!.power = (leftY1!! + leftX1!! + rightX1!!)
            motorBL!!.power = (leftY1!! - leftX1!! + rightX1!!)
            motorFR!!.power = (leftY1!! - leftX1!! - rightX1!!)
            motorBR!!.power = (leftY1!! + leftX1!! - rightX1!!)

            telemetry.addData("x: ", drive!!.poseEstimate.x)
            telemetry.addData("y: ", drive!!.poseEstimate.y)
            telemetry.addData("heading: ", drive!!.poseEstimate.heading)
        }
    }
}
