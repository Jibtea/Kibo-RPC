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
}
