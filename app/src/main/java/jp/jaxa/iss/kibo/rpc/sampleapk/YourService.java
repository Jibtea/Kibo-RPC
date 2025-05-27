package jp.jaxa.iss.kibo.rpc.sampleapk;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.sampleapk.utils.ImageUtils;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.ArMarkerDetector;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.TemplateMatcher;
import jp.jaxa.iss.kibo.rpc.sampleapk.oasis.OasisUtils;
import jp.jaxa.iss.kibo.rpc.sampleapk.ar.ArDetectionUtils;
import jp.jaxa.iss.kibo.rpc.sampleapk.astronaut.AstronautUtils;
import jp.jaxa.iss.kibo.rpc.sampleapk.util.AreaRecognitionUtils;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.Utils;

/**
 * Main service class for handling Kibo RPC mission logic.
 * Refactored for modularity, maintainability, and clarity.
 */
public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();
    private static final int OASIS_AREA_COUNT = 4;
    private static final int OASIS_ORIENTATION_COUNT = 4;
    private final String[] TEMPLATE_FILE_NAME = {
        "coin.png", "compass.png", "coral.png", "crystal.png", "emerald.png",
        "fossil.png", "key.png", "letter.png", "shell.png", "treasure_box.png"
    };
    private final String[] TEMPLATE_NAME = {
        "coin", "compass", "coral", "crystal", "emerald",
        "fossil", "key", "letter", "shell", "treasure_box"
    };

    /**
     * Entry point for Plan 1 mission logic.
     * Follows SRP and delegates to modular methods.
     */
    @Override
    protected void runPlan1() {
        Log.i(TAG, "startMissionTest");
        api.startMission();
        List<Point> oasisPoints = OasisUtils.getOasisPoints();
        List<Quaternion> oasisQuaternions = OasisUtils.getOasisQuaternions();
        int arCounter = 0;
        for (int areaIdx = 0; areaIdx < OASIS_AREA_COUNT; areaIdx++) {
            arCounter += scanOasisArea(areaIdx, oasisPoints, oasisQuaternions);
            processAreaRecognition(areaIdx);
        }
        AstronautUtils.moveToAstronautAndReport(api);
        AstronautUtils.recognizeAndReportTargetItem(api);
        AstronautUtils.moveToTargetItemAndSnapshot(api);
    }

    /**
     * Scans an oasis area by moving to each orientation and detecting AR markers.
     * @param areaIdx Index of the area
     * @param oasisPoints List of oasis points
     * @param oasisQuaternions List of oasis quaternions
     * @return Number of AR markers found
     */
    private int scanOasisArea(int areaIdx, List<Point> oasisPoints, List<Quaternion> oasisQuaternions) {
        int arFound = 0;
        for (int i = 0; i < OASIS_ORIENTATION_COUNT; i++) {
            Mat image = captureImageAt(oasisPoints.get(areaIdx), oasisQuaternions.get(i));
            //More robust null check for image
            if (image == null) {
                Log.w(TAG, "Captured image is empty; skipping orientation " + i);
                continue;
            }
            saveOasisImage(areaIdx, i, image);
            if (ArDetectionUtils.detectARMarker(image)) {
                arFound++;
            }
        }
        return arFound;
    }

    /**
     * Moves to a given point and orientation, then captures an image.
     * @param point Target point
     * @param quaternion Target orientation
     * @return Captured image or null
     */
    private Mat captureImageAt(Point point, Quaternion quaternion) {
        api.moveTo(point, quaternion, false);
        Mat image = api.getMatNavCam();
        if (image == null) {
            Log.i(TAG, "image was null; cannot connect camera");
        }
        return image;
    }

    /**
     * Saves the captured image for a specific oasis area and orientation.
     */
    private void saveOasisImage(int areaIdx, int orientationIdx, Mat image) {
        String fileName = String.format("OasisArea%d_%d.png", areaIdx, orientationIdx);
        api.saveMatImage(image, fileName);
    }

    /**
     * Recognizes and reports items in the current area using template matching.
     * @param areaIdx Index of the area
     */
    private void processAreaRecognition(int areaIdx) {
        Mat image = api.getMatNavCam();
        if (image == null) {
            Log.w(TAG, "No image for area recognition");
            return;
        }
        Mat undistortImg = ImageUtils.undistortImage(image, api.getNavCamIntrinsics());
        Mat[] templates = AreaRecognitionUtils.loadTemplates(getAssets(), TEMPLATE_FILE_NAME, TAG);
        int[] templateMatchCnt = TemplateMatcher.matchTemplates(undistortImg, templates);
        int mostMatchTemplateNum = ImageUtils.getMaxIndex(templateMatchCnt);
        api.setAreaInfo(areaIdx, TEMPLATE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);
    }

    @Override
    protected void runPlan2() {
        // write your plan 2 here.
    }

    @Override
    protected void runPlan3() {
        // write your plan 3 here.
    }

    // You can add your method.
    private String yourMethod() {
        return "your method";
    }

    // The following utility methods have been moved to ImageUtils:
    // - resizeImg
    // - rotImg
    // - removeDuplicates
    // - calculateDistance
    // - getMaxIndex
}


/*
Key changes and reasoning:
- Broke down large methods into focused, single-responsibility methods (SRP).
- Extracted repeated logic (image capture, AR detection, template loading, undistortion, image saving) into their own methods.
- Improved error handling and logging for null images and IO exceptions.
- Added clear JavaDoc comments for all methods and the class.
- Used constants for magic numbers (area/orientation count).
- Improved method and variable naming for clarity.
- Removed duplicate code and ensured separation of concerns.
- Utility and vision logic is assumed to be in separate classes (ImageUtils, TemplateMatcher, ArMarkerDetector).
*/