package jp.jaxa.iss.kibo.rpc.sampleapk.vision;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import java.util.ArrayList;
import java.util.List;

public class ArMarkerDetector {
    public static List<Mat> detectMarkers(Mat image, int dictionaryId) {
        Dictionary dictionary = Aruco.getPredefinedDictionary(dictionaryId);
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image, dictionary, corners, markerIds);
        return corners;
    }
}
