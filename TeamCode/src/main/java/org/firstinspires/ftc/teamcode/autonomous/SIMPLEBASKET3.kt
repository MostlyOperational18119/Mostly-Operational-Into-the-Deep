//package org.firstinspires.ftc.teamcode.autonomous
//
//import com.acmerobotics.roadrunner.geometry.Pose2d
//import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous
//import com.qualcomm.robotcore.eventloop.opmode.Disabled
//import org.firstinspires.ftc.teamcode.Methods
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
//
//@Autonomous(name = "Simple_Basket_meet3", group = "AAAA")
//@Disabled
//
//class SIMPLEBASKET3 : Methods() {
//    override fun runOpMode() {
//        initOdometry()
//        initMotors()
//        initServosAndTouchWithoutSet()
//
//        drive!!.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))
//
//        val basket1: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-48.25, -40.5, Math.toRadians(90.00)))
//                .setReversed(true)
//                .lineToLinearHeading(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
//                .setReversed(false)
//                .build()
//
//        val end: TrajectorySequence =
//            drive!!.trajectorySequenceBuilder(Pose2d(-52.5, -52.5, Math.toRadians(45.00)))
//                .lineTo(Vector2d(-44.0, -44.0))
//                .build()
//
//        waitForStart()
//
//        //START
//        verticalSlideTo(1600, 1.0)
//        outRotationServo!!.position = outRotationBack
//
//        //BASKET1
//        drive!!.followTrajectorySequence(basket1)
//        placeSample()
//        verticalSlideTo(0, 0.5)
//
//        //END
//        drive!!.followTrajectorySequence(end)
//        verticalSlideTo(0, 1.0)
//        //clawRotateServo!!.position = clawRotateUpRight
//
//        while (opModeIsActive() && !isStopRequested) { drive!!.update() }
//        PoseStorage.currentPose = drive!!.poseEstimate
//    }
//}