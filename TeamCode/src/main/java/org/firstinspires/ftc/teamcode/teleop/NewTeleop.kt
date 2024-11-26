package org.firstinspires.ftc.teamcode.teleop

import android.transition.Slide
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods

class NewTeleop : DriveMethods() {
    override fun runOpMode() {
        val FL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val BL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val FR = hardwareMap.get(DcMotor::class.java, "motorFR")
        val BR = hardwareMap.get(DcMotor::class.java, "motorBR")

        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()
        val SlideHorizontal = hardwareMap.get(DcMotor::class.java, "SlideHorizontal")
        val SlideVertical = hardwareMap.get(DcMotor::class.java, "SlideVertical")
        val TapeMeasure = hardwareMap.get(DcMotor::class.java, "TapeMeasure")
        val ClawRotate = hardwareMap.get(DcMotor::class.java, "ClawRotate")
        val HangPusher = hardwareMap.get(Servo::class.java, "HangPusher")
        val IntakeFlip = hardwareMap.get(Servo::class.java, "IntakeFlip")
        val ClawServo = hardwareMap.get(Servo::class.java, "ClawServo")
        val Hanger = hardwareMap.get(DcMotor::class.java, "Hanger")
        SlideHorizontal.mode = DcMotor.RunMode.RUN_TO_POSITION
        SlideHorizontal.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        SlideVertical.mode = DcMotor.RunMode.RUN_TO_POSITION
        SlideVertical.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        var IsAlreadyMid = false
        var Goto0 = false

        var x = 0.00
        var y = 0.00
        var rx = 0.00
        var g2y = 0.00


        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            y = -gamepad1.left_stick_y.toDouble()
            x = gamepad1.left_stick_x.toDouble()
            rx = gamepad1.right_stick_x.toDouble()
            g2y = -gamepad2.right_stick_y.toDouble()

            //---ACCOUNT FOR INVERTED MOTORS BELOW LATER---
            FL.power = y + x + rx
            BL.power = y - x + rx
            FR.power = y - x - rx
            BR.power = y + x - rx
            //----------------------------------------------

            /*if (SlideHorizontal.currentPosition >= (Lower value) && g2y < 0 && !Goto0) {
                SlideHorizontal.targetPosition = (Max encoder)
                SlideHorizontal.power = -g2y/3 //might need speed/direction change later
            } else if (SlideHorizontal.currentPosition <= (Higher value) && g2y > 0 && !Goto0) {
                SlideHorizontal.targetPosition = (Min encoder)
                SlideHorizontal.power = g2y/3 //might need speed/direction change later
            } else if (Goto0) {
                SlideHorizontal.targetPosition = (Lower value)
                SlideHorizontal.power = -g2y/3
                IntakeFlip.position = (Transfer point)
                if (SlideHorizontal.currentPosition <= 0) {
                    Goto0 = false
                }
            } else {
                SlideHorizontal.targetPosition = SlideHorizontal.currentPosition
                SlideHorizontal.power = 0.00
            }*

            if (SlideHorizontal.currentPosition >= (Flip into transfer point)) {
                IntakeFlip.position = (Transfer point)
                IsAlreadyMid = false
            } else if (!IsAlreadyMid) {
                IsAlreadyMid = true
                IntakeFlip.position = (mid point)
            }

            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                IntakeFlip.position = (Intake position)
            }

            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                Goto0 = true
            }*/
        }
    }
}