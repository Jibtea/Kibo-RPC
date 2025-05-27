package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.io.InputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import jp.jaxa.iss.kibo.rpc.sampleapk.utils.ImageUtils;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.ArMarkerDetector;
import jp.jaxa.iss.kibo.rpc.sampleapk.vision.TemplateMatcher;

import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.core.Core;
import org.opencv.imgproc.Imgproc;
import org.opencv.android.Utils;


/**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee.
 */

public class YourService extends KiboRpcService {

    private final String TAG = this.getClass().getSimpleName();
    //template file name
    private final String[] TEMPLATE_FILE_NAME = {
            "coin.png",
            "compass.png",
            "coral.png",
            "crystal.png",
            "emerald.png",
            "fossil.png",
            "key.png",
            "letter.png",
            "shell.png",
            "treasure_box.png"
    };
    //template name
    private final String[] TEMPLATE_NAME = {
            "coin",
            "compass",
            "coral",
            "crystal",
            "emerald",
            "fossil",
            "key",
            "letter",
            "shell",
            "treasure_box"
    };

    @Override
    protected void runPlan1() {
        Log.i(TAG, "startMissionTest");
        api.startMission();
        List<Point> oasisPoints = getOasisPoints();
        List<Quaternion> oasisQuaternions = getOasisQuaternions();
        int arCounter = 0;
        for (int areaIdx = 0; areaIdx < 4; areaIdx++) {
            arCounter += scanOasisArea(areaIdx, oasisPoints, oasisQuaternions);
            processAreaRecognition(areaIdx);
        }
        moveToAstronautAndReport();
        recognizeAndReportTargetItem();
        moveToTargetItemAndSnapshot();
    }

    private List<Point> getOasisPoints() {
        List<Point> points = new ArrayList<>();
        points.add(new Point(10.925d, -9.85d, 4.695d));
        points.add(new Point(11.175d, -8.975d, 5.195d));
        points.add(new Point(10.7d, -7.925d, 5.195d));
        points.add(new Point(11.175d, -6.875d, 4.685d));
        return points;
    }

    private List<Quaternion> getOasisQuaternions() {
        List<Quaternion> quaternions = new ArrayList<>();
        quaternions.add(new Quaternion(0f, 0f, -0.707f, 0.707f));
        quaternions.add(new Quaternion(0.707f, 0f, 0.707f, 0.707f));
        quaternions.add(new Quaternion(0f, -0.707f, 0.707f, -0.707f));
        quaternions.add(new Quaternion(0.707f, 0.707f, -0.707f, -0.707f));
        return quaternions;
    }

    private int scanOasisArea(int areaIdx, List<Point> oasisPoints, List<Quaternion> oasisQuaternions) {
        int arFound = 0;
        for (int i = 0; i < 4; i++) {
            api.moveTo(oasisPoints.get(areaIdx), oasisQuaternions.get(i), false);
            Mat image = api.getMatNavCam();
            if (image == null) {
                Log.i(TAG, "image was null cannot connect camera maybe? not sure");
                return arFound;
            }
            String fileName = String.format("OasisArea%d_%d.png", areaIdx, i);
            api.saveMatImage(image, fileName);
            List<Mat> corners = ArMarkerDetector.detectMarkers(image, Aruco.DICT_5X5_250);
            if (corners.isEmpty()) {
                Log.w(TAG, "Cannot detect AR");
            } else {
                Log.i(TAG, "Here AR");
                arFound++;
            }
        }
        return arFound;
    }

    private void processAreaRecognition(int areaIdx) {
        Mat image = api.getMatNavCam();
        if (image == null) return;
        Mat undistortImg = undistortImage(image);
        Mat[] templates = loadTemplates();
        int[] templateMatchCnt = TemplateMatcher.matchTemplates(undistortImg, templates);
        int mostMatchTemplateNum = ImageUtils.getMaxIndex(templateMatchCnt);
        api.setAreaInfo(areaIdx, TEMPLATE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);
        // Optionally: api.setAreaInfo(areaIdx, "item_name", areaIdx);
    }

    private Mat undistortImage(Mat image) {
        Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
        cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
        Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
        cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
        cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);
        Mat undistortImg = new Mat();
        Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);
        return undistortImg;
    }

    private Mat[] loadTemplates() {
        Mat[] templates = new Mat[TEMPLATE_FILE_NAME.length];
        for (int i = 0; i < TEMPLATE_FILE_NAME.length; i++) {
            try {
                InputStream inputStream = getAssets().open(TEMPLATE_FILE_NAME[i]);
                Bitmap bitmap = BitmapFactory.decodeStream(inputStream);
                Mat mat = new Mat();
                Utils.bitmapToMat(bitmap, mat);
                Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);
                templates[i] = mat;
                inputStream.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        return templates;
    }

    private void moveToAstronautAndReport() {
        Point point = new Point(11.143d, -6.7607d, 4.9654d);
        Quaternion quaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
        api.moveTo(point, quaternion, false);
        api.reportRoundingCompletion();
    }

    private void recognizeAndReportTargetItem() {
        // TODO: Implement target item recognition logic here
        api.notifyRecognitionItem();
    }

    private void moveToTargetItemAndSnapshot() {
        // TODO: Implement logic to move to the target item
        api.takeTargetItemSnapshot();
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


// //=====move test ======
// for(int i=0;i<4;i++){
//     api.moveTo(oasispoint.get(i), oasisQuaternion.get(i), false);
// }
// //=====stop test====