package jp.jaxa.iss.kibo.rpc.sampleapk.ar;

import org.opencv.aruco.Aruco;
import org.opencv.core.Mat;
import java.util.List;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.ArMarkerDetector;

/**
 * Utility class for AR marker detection.
 */
public class ArDetectionUtils {
    public static boolean detectARMarker(Mat image) {
        List<Mat> corners = ArMarkerDetector.detectMarkers(image, Aruco.DICT_5X5_250);
        return !corners.isEmpty();
    }
}
