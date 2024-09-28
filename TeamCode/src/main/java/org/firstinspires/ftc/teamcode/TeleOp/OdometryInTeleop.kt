package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@TeleOp(name = "OdometryInTeleOp")
class OdometryInTeleOp : LinearOpMode() {

    enum class DRIVE_STATE{
        MANUAL,
        AUTONOMOUS
    };
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        val processor = AprilTagProcessor.easyCreateWithDefaults()!!
        val visionPortal = VisionPortal.Builder()
            .setCamera(hardwareMap.get(CameraName::class.java, "Webcam 1"))
            .addProcessor(processor)
            .build()

        //GAMEPADS
        val currentGamepad1: Gamepad = Gamepad()
        val currentGamepad2: Gamepad = Gamepad()
        val previousGamepad1: Gamepad = Gamepad()
        val previousGamepad2: Gamepad = Gamepad()

        waitForStart()

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);
            if (!drive.isBusy) {
                val leftY = -gamepad1.left_stick_y.toDouble()
                val leftX = -gamepad1.left_stick_x.toDouble()
                val rightX = gamepad1.right_stick_x.toDouble()

                drive.setMotorPowers(
                    leftY - leftX + rightX,
                    leftY + leftX + rightX,
                    leftY - leftX - rightX,
                    leftY + leftX - rightX
                )

                if (currentGamepad1.a && !previousGamepad1.a) {
                    var detection: AprilTagDetection? = null

                    processor.detections.forEach {
                        if (it.id == 1) detection = it
                    }

                    if (detection != null) {
                        val trajectorySequence =
                            calculateTrajectorySequenceApriltags(detection!!, drive)

                        drive.followTrajectorySequenceAsync(trajectorySequence)
                    } else {
                        telemetry.addLine("Unable to find an Apriltag with an ID of 1")
                        telemetry.update()
                    }
                }
            }

            else {
                drive.update()
                drive.updatePoseEstimate()

                telemetry.addData(
                    "Odometry info",
                    String.format(
                        "Pose: %s, Velocity: %s",
                        drive.poseEstimate.toString(),
                        drive.getWheelVelocities().toString()
                    )
                )
                telemetry.update()
            }
        }
    }

    private fun calculateTrajectorySequenceApriltags(
        detection: AprilTagDetection,
        drive: SampleMecanumDrive
    ): TrajectorySequence {
        val ftcPose = detection.ftcPose

        drive.update()
        drive.updatePoseEstimate()

        return drive.trajectorySequenceBuilder(drive.poseEstimate)
            .forward(ftcPose.x)
            .strafeRight(ftcPose.z)
            .build()
    }
}