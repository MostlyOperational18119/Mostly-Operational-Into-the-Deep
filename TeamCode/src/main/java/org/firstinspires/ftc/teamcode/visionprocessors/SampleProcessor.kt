package org.firstinspires.ftc.teamcode.visionprocessors

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc


data class OpenCVResults(val detections: ArrayList<OpenCVDetections>)

enum class SampleColour {
    RED, BLUE, YELLOW
}

data class OpenCVDetections(
    val x: Int,
    val y: Int,
    val rotation: Double,
    val colour: SampleColour,
    val pointsList: List<Point>
)

class SampleProcessor : VisionProcessor {
    private var enabledColours: List<Boolean> = listOf(true, true, true) // Red, blue, yellow
    private var recentResults = OpenCVResults(ArrayList())

    override fun init(width: Int, height: Int, calibration: CameraCalibration?) {
        // Not applicable, but it's necessary for a VisionProcessor :(
    }

    override fun processFrame(frame: Mat?, captureTimeNanos: Long): Any {
        // Convert to HSV :)
        val srcHSV = Mat()
        val matRed = Mat()
        val matBlue = Mat()
        val matYellow = Mat()
        val results = OpenCVResults(ArrayList())
        Imgproc.GaussianBlur(frame, frame, Size(5.0, 5.0), 0.0)
        Imgproc.cvtColor(frame, srcHSV, Imgproc.COLOR_BGR2HSV)

        if (enabledColours[0]) {
            Core.inRange(srcHSV, Scalar(0.0, 100.0, 100.0), Scalar(10.0, 255.0, 255.0), matRed)

            val redResults = findSamples(matRed, SampleColour.RED)
            results.detections.addAll(redResults)
        }
        if (enabledColours[1]) {
            Core.inRange(srcHSV, Scalar(10.0, 100.0, 100.0), Scalar(40.0, 255.0, 255.0), matBlue)

            val blueResults = findSamples(matBlue, SampleColour.BLUE)
            results.detections.addAll(blueResults)
        }
        if (enabledColours[2]) {
            Core.inRange(srcHSV, Scalar(90.0, 100.0, 160.0), Scalar(100.0, 255.0, 255.0), matYellow)

            val yellowResults = findSamples(matYellow, SampleColour.YELLOW)
            results.detections.addAll(yellowResults)
        }

        recentResults = results

        return results
    }

    private fun makeGraphicsRect(rect: Rect, scaleBmpPxToCanvasPx: Float): Rect {
        val left = Math.round(rect.x * scaleBmpPxToCanvasPx).toInt()
        val top = Math.round(rect.y * scaleBmpPxToCanvasPx).toInt()
        val right = (left + Math.round(rect.width * scaleBmpPxToCanvasPx)).toInt()
        val bottom = (top + Math.round(rect.height * scaleBmpPxToCanvasPx)).toInt()

        return Rect(left, top, right, bottom)
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        val circPaint = Paint()
        circPaint.color = Color.RED
        circPaint.style = Paint.Style.FILL_AND_STROKE
        circPaint.strokeWidth = scaleCanvasDensity * 4

        (userContext as OpenCVResults).detections.forEach {
            val cx = it.x * scaleBmpPxToCanvasPx
            val cy = it.y * scaleBmpPxToCanvasPx
            canvas!!.drawCircle(cx, cy, scaleCanvasDensity * 16, circPaint)
        }
    }

    fun setEnabledColours(red: Boolean, blue: Boolean, yellow: Boolean) {
        enabledColours = listOf(red, blue, yellow)
    }

    fun getEnabledColours(): List<Boolean> {
        return enabledColours
    }

    private fun findSamples(mat: Mat, colour: SampleColour): ArrayList<OpenCVDetections> {
        val results = ArrayList<OpenCVDetections>()
        val contours: List<MatOfPoint> = ArrayList()
        val hierarchy = Mat()
        Imgproc.findContours(
            mat,
            contours,
            hierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        contours.forEach {
            val cont2f = MatOfPoint2f()
            it.convertTo(cont2f, CvType.CV_32F)

            val perimeter = Imgproc.arcLength(cont2f, true)
            val approx = MatOfPoint2f()
            Imgproc.approxPolyDP(cont2f, approx, 0.04 * perimeter, true)

            val pointsList = approx.toList()

            if (pointsList.size == 4) {
                val x =
                    ((pointsList[0].x + pointsList[1].x + pointsList[2].x + pointsList[3].x) / 4).toInt()
                val y =
                    ((pointsList[0].y + pointsList[1].y + pointsList[2].y + pointsList[3].y) / 4).toInt()
                val rotation = 0.0 // TODO: FIND ROTATION USING POINTS
                val detection = OpenCVDetections(x, y, rotation, colour, pointsList)

                results.add(detection)
            }
        }

        hierarchy.release()

        return results
    }

    fun getResults(): OpenCVResults {
        return recentResults
    }

}
