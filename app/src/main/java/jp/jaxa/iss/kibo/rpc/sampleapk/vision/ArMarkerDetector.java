package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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
        drawBoundingBoxes(image, result, new Scalar(0, 255, 0));
        return result;
    }

    /**
     * Extract camera matrix and distortion coefficients from NavCam intrinsics.
     * @param intrinsics double[4][5] or double[4][N] as returned by api.getNavCamIntrinsics()
     * @return array: [cameraMatrix, distCoeffs]
     */
    public static Mat[] extractCameraIntrinsics(double[][] intrinsics) {
        Mat cameraMatrix = new Mat(3, 3, org.opencv.core.CvType.CV_64F);
        Mat distCoeffs = new Mat(1, 5, org.opencv.core.CvType.CV_64F);
        for (int row = 0; row < 3; row++)
            for (int col = 0; col < 3; col++)
                cameraMatrix.put(row, col, intrinsics[row][col]);
        for (int k = 0; k < 5; k++)
            distCoeffs.put(0, k, intrinsics[3][k]);
        return new Mat[] { cameraMatrix, distCoeffs };
    }

    /**
     * Estimate pose and log tvec and angle for each detected marker.
     */
    public static void logMarkerPose(DetectionResult result, float markerLength, Mat cameraMatrix, Mat distCoeffs, Quaternion orientation) {
        List<Mat> rvecs = new ArrayList<>();
        List<Mat> tvecs = new ArrayList<>();
        org.opencv.aruco.Aruco.estimatePoseSingleMarkers(result.corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs);
        for (int j = 0; j < result.markerIds.rows(); j++) {
            Mat tvec = tvecs.get(j);
            double dx = tvec.get(0,0)[0];
            double dy = tvec.get(0,0)[1];
            double dz = tvec.get(0,0)[2];
            double distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
            double angle = Math.toDegrees(Math.acos(dz / distance));
            android.util.Log.i("ArMarkerDetector", String.format(
                "AR marker ID: %d, tvec: [%.3f, %.3f, %.3f], distance: %.3f m, angle: %.2f deg, orientation: %s",
                (int)result.markerIds.get(j,0)[0], dx, dy, dz, distance, angle, orientation.toString()
            ));
        }
    }
}
