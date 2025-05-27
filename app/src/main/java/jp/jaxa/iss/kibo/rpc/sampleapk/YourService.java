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

        //plz use Exception Handling ( try catch )for safetycode
        Log.i(TAG, "startMissionTest");

        // The mission starts.
        api.startMission();

        //========Move to a point program========
//        Point point = new Point(10.9d, -9.92284d, 5.195d);
//        Quaternion quaternion = new Quaternion(0f, 0f, -0.707f, 0.707f);
//        api.moveTo(point, quaternion, false);

        //oasis info put into List
        List<Point> oasispoint = new ArrayList<Point>();
        List<Quaternion> oasisQuaternion = new ArrayList<Quaternion>();
        //oasis1
        oasispoint.add(new Point(10.925d, -9.85d, 4.695d));
        oasisQuaternion.add(new Quaternion(0f, 0f, -0.707f, 0.707f));
        //oasis2
        oasispoint.add(new Point(11.175d, -8.975d, 5.195d));
        oasisQuaternion.add(new Quaternion(0.707f, 0f, 0.707f, 0.707f));
        //oasis3
        oasispoint.add(new Point(10.7d, -7.925d, 5.195d));
        oasisQuaternion.add(new Quaternion(0f, -0.707f, 0.707f, -0.707f));
        //oasis4
        oasispoint.add(new Point(11.175d, -6.875d, 4.685d));
        oasisQuaternion.add(new Quaternion(0.707f, 0.707f, -0.707f, -0.707f));


        //=========== i will create it to some func?============
        int num = 0;
        Mat image = new Mat();
        int arCounter = 0;

        while (num < 4) {
            //move to every oasis until find every area
            // api.moveTo(oasispoint.get(num), oasisQuaternion.get(num), false);

            String numCount = String.format("Num_%d.png", num);
            Log.i(TAG,numCount);
            //============================360 camera checkkkk==================
            for (int i = 0; i < 4; i++) {
                // Get a camera image.
                api.moveTo(oasispoint.get(num), oasisQuaternion.get(i), false);
                image = api.getMatNavCam();
                if (image == null) {
                    Log.i(TAG, "image was null cannot connect camera maybe? not sure");
                    return;
                }

                //Save image ถ่ายรูแฮั่นล่ะแชะๆ
                String fileName = String.format("OasisArea%d_%d.png", num, i);
                api.saveMatImage(image, fileName);

                //===========AR section=========
                //detect AR
                List<Mat> corners = ArMarkerDetector.detectMarkers(image, Aruco.DICT_5X5_250);

                //check AR
                if (corners.isEmpty()) {
                    Log.w(TAG, "Cannot detect AR");
                    continue;
                } else {
                    Log.i(TAG, "Here AR");
                    arCounter++;
                }
            }

            /* ******************************************************************************** */
            /* Write your code to recognize the type and number of landmark items in each area! */
            /* If there is a treasure item, remember it.                                        */
            /* ******************************************************************************** */


            ///=======================correcting image distortion===================
            //Get camera matrix
            Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
            cameraMatrix.put(0, 0, api.getNavCamIntrinsics()[0]);
            //get lens distance parameter
            Mat cameraCoefficients = new Mat(1, 5, CvType.CV_64F);
            cameraCoefficients.put(0, 0, api.getNavCamIntrinsics()[1]);
            cameraCoefficients.convertTo(cameraCoefficients, CvType.CV_64F);

            //undistort image
            Mat undistortImg = new Mat();
            Calib3d.undistort(image, undistortImg, cameraMatrix, cameraCoefficients);


            //======pattern matching=======
            //load template image
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

            //Number of matches for each template
            int[] templateMatchCnt = TemplateMatcher.matchTemplates(undistortImg, templates);

            //When u recognize landmark items let's type num
            int mostMatchTemplateNum = ImageUtils.getMaxIndex(templateMatchCnt);
            api.setAreaInfo(num, TEMPLATE_NAME[mostMatchTemplateNum], templateMatchCnt[mostMatchTemplateNum]);

            // When you recognize landmark items, let’s set the type and number.
            api.setAreaInfo(num, "item_name", num);

            num++;
        }




        /* **************************************************** */
        /* Let's move to each area and recognize the items. */
        /* **************************************************** */


        //===========move to astronaut==================
        // When you move to the front of the astronaut, report the rounding completion.
        Point point = new Point(11.143d, -6.7607d, 4.9654d);
        Quaternion quaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
//        point = new Point(11.143d, -6.7607d, 4.9654d);
//        quaternion = new Quaternion(0f, 0f, 0.707f, 0.707f);
        api.moveTo(point, quaternion, false);
        api.reportRoundingCompletion();

        /* ********************************************************** */
        /* Write your code to recognize which target item the astronaut has. */
        /* ********************************************************** */

        // Let's notify the astronaut when you recognize it.
        api.notifyRecognitionItem();

        /* ******************************************************************************************************* */
        /* Write your code to move Astrobee to the location of the target item (what the astronaut is looking for) */
        /* ******************************************************************************************************* */

        // Take a snapshot of the target item.
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