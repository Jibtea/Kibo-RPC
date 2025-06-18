package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.opencv.core.Scalar;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfPoint;

/**
 * Utility class for efficient AR marker detection and drawing using OpenCV ArUco.
 * Optimized for real-time robotics on limited hardware.
 */
public class ArMarkerDetector {
    // Cache for ArUco dictionaries to avoid redundant creation
    private static final Map<Integer, Dictionary> dictionaryCache = new HashMap<>();

    /**
     * Get (and cache) the ArUco dictionary for the given ID.
     */
    private static Dictionary getDictionary(int dictionaryId) {
        if (!dictionaryCache.containsKey(dictionaryId)) {
            dictionaryCache.put(dictionaryId, Aruco.getPredefinedDictionary(dictionaryId));
        }
        return dictionaryCache.get(dictionaryId);
    }

    /**
     * Result object for marker detection.
     */
    public static class DetectionResult {
        public final List<Mat> corners;
        public final Mat markerIds;
        public DetectionResult(List<Mat> corners, Mat markerIds) {
            this.corners = corners;
            this.markerIds = markerIds;
        }
        public boolean hasMarkers() {
            return corners != null && !corners.isEmpty();
        }
    }

    /**
     * Detect ArUco markers in the image using the specified dictionary.
     * @param image Input image (will not be modified)
     * @param dictionaryId ArUco dictionary ID
     * @return DetectionResult containing corners and marker IDs
     */
    public static DetectionResult detectMarkers(Mat image, int dictionaryId) {
        Dictionary dictionary = getDictionary(dictionaryId);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);
        return new DetectionResult(corners, markerIds);
    }
    
    /**
     * Draw bounding boxes for detected markers on the image.
     * @param image Image to draw on (modified in place)
     * @param detectionResult Result from detectMarkers
     * @param color Color for bounding boxes
     */
    public static void drawBoundingBoxes(Mat image, DetectionResult detectionResult, Scalar color) {
        if (detectionResult != null && detectionResult.hasMarkers()) {
            Aruco.drawDetectedMarkers(image, detectionResult.corners, detectionResult.markerIds, color);
        }
    }

    /**
     * Detect markers and draw bounding boxes in one step (for convenience).
     * @param image Image to process (modified in place)
     * @param dictionaryId ArUco dictionary ID
     * @return DetectionResult containing marker data
     */
    public static DetectionResult detectAndDrawMarkers(Mat image, int dictionaryId) {
        DetectionResult result = detectMarkers(image, dictionaryId);
        //drawBoundingBoxes(image, result, new Scalar(0, 255, 0));
        return result;
    }

    /**
     * Extract camera matrix and distortion coefficients from flat arrays.
     * @param cameraMatrixArr double[9] camera matrix (row-major)
     * @param distCoeffsArr double[5] distortion coefficients
     * @return array: [cameraMatrix, distCoeffs]
     */
    public static Mat[] extractCameraIntrinsics(double[] cameraMatrixArr, double[] distCoeffsArr) {
        if (cameraMatrixArr == null || cameraMatrixArr.length != 9)
            throw new IllegalArgumentException("Camera matrix array must have 9 elements (3x3)");
        if (distCoeffsArr == null || distCoeffsArr.length != 5)
            throw new IllegalArgumentException("Distortion coefficients array must have 5 elements");
        Mat cameraMatrix = new Mat(3, 3, org.opencv.core.CvType.CV_64F);
        Mat distCoeffs = new Mat(1, 5, org.opencv.core.CvType.CV_64F);
        for (int i = 0; i < 9; i++) {
            cameraMatrix.put(i / 3, i % 3, cameraMatrixArr[i]);
        }
        for (int k = 0; k < 5; k++) {
            distCoeffs.put(0, k, distCoeffsArr[k]);
        }
        return new Mat[] { cameraMatrix, distCoeffs };
    }



    /**
     * Estimate pose and log tvec and angle for each detected marker.
     */
    public static void logMarkerPose(DetectionResult result, float markerLength, Mat cameraMatrix, Mat distCoeffs, Quaternion orientation) {
        Mat rvecsMat = new Mat();
        Mat tvecsMat = new Mat();
        org.opencv.aruco.Aruco.estimatePoseSingleMarkers(result.corners, markerLength, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
        for (int j = 0; j < result.markerIds.rows(); j++) {
            double[] tvec = tvecsMat.get(j, 0);
            double dx = tvec[0];
            double dy = tvec[1];
            double dz = tvec[2];
            double distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
            double angle = Math.toDegrees(Math.acos(dz / distance));
            android.util.Log.i("ArMarkerDetector", String.format(
                "AR marker ID: %d, tvec: [%.3f, %.3f, %.3f], distance: %.3f m, angle: %.2f deg, orientation: %s",
                (int)result.markerIds.get(j,0)[0], dx, dy, dz, distance, angle, orientation.toString()
            ));
        }
    }
    
    /**
     * Draw the boundary of the paper using the AR marker as reference with custom dimensions.
     * Additionally, blur everything outside the detected paper boundary.
     */
    public static void drawPaperBoundary(Mat image, Mat rvec, Mat tvec, Mat cameraMatrix, Mat distCoeffs) {
        double paperWidth = 0.35; // meters (28.35 cm)
        double paperHeight = 0.27; // meters (18 cm)
        double markerInsetTop = 0.0675; // meters (6.75 cm from top)
        double markerInsetRight = 0.071; // meters (5.1 cm from right)
        // Paper corners relative to marker center (0,0,0):
        // Top-left: left by (paperWidth - markerInsetRight), up by markerInsetTop
        // Top-right: right by markerInsetRight, up by markerInsetTop  
        // Bottom-left: left by (paperWidth - markerInsetRight), down by (paperHeight - markerInsetTop)
        // Bottom-right: right by markerInsetRight, down by (paperHeight - markerInsetTop)
        MatOfPoint3f paperWorldCorners = new MatOfPoint3f(
            new Point3(-(paperWidth - markerInsetRight), markerInsetTop, 0), // top-left
            new Point3(markerInsetRight, markerInsetTop, 0), // top-right
            new Point3(markerInsetRight, - (paperHeight - markerInsetTop), 0), // bottom-right
            new Point3(-(paperWidth - markerInsetRight), - (paperHeight - markerInsetTop), 0) // bottom-left
        );

        // Convert distCoeffs to MatOfDouble for projectPoints
        org.opencv.core.MatOfDouble distCoeffsDouble = new org.opencv.core.MatOfDouble();
        if (distCoeffs != null && distCoeffs.total() > 0) {
            double[] distVals = new double[(int)distCoeffs.total()];
            distCoeffs.get(0, 0, distVals);
            distCoeffsDouble.fromArray(distVals);
        }
        // Project paper corners to image
        MatOfPoint2f paperImageCorners = new MatOfPoint2f();
        Calib3d.projectPoints(paperWorldCorners, rvec, tvec, cameraMatrix, distCoeffsDouble, paperImageCorners);
        // Draw the boundary
        Point[] pts = paperImageCorners.toArray();
        for (int i = 0; i < 4; i++) {
            Imgproc.line(image, pts[i], pts[(i+1)%4], new Scalar(0,0,255), 3);
        }

        // --- Blur everything outside the paper boundary ---
        // 1. Create mask for the paper area
        Mat mask = Mat.zeros(image.size(), org.opencv.core.CvType.CV_8UC1);
        MatOfPoint paperContour = new MatOfPoint(
            new Point(pts[0].x, pts[0].y),
            new Point(pts[1].x, pts[1].y),
            new Point(pts[2].x, pts[2].y),
            new Point(pts[3].x, pts[3].y)
        );
        List<MatOfPoint> contours = new ArrayList<>();
        contours.add(paperContour);
        Imgproc.fillPoly(mask, contours, new Scalar(255));

        // 2. Blur the whole image
        Mat blurred = new Mat();
        Imgproc.blur(image, blurred, new org.opencv.core.Size(35, 35));

        // 3. Copy the unblurred paper area back onto the blurred image
        image.copyTo(blurred, mask);

        // 4. Copy the result back to the original image
        blurred.copyTo(image);
    }
    

}
