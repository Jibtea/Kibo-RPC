package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Scalar;

public class ArMarkerDetector {
    public static List<Mat> detectMarkers(Mat image, int dictionaryId) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(dictionaryId);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);
        return corners;
    }

    // New method: check if any markers are present
    public static boolean hasMarkers(Mat image, int dictionaryId) {
        List<Mat> corners = detectMarkers(image, dictionaryId);
        return !corners.isEmpty();
    }

    // Optional: Draw bounding boxes for detected markers
    public static void drawBoundingBoxes(Mat image, List<Mat> corners, Mat ids) {
        if (corners != null && !corners.isEmpty()) {
            Aruco.drawDetectedMarkers(image, corners, ids, new Scalar(0, 255, 0));
        }
    }
}
