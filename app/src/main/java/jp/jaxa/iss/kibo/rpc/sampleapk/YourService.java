package jp.jaxa.iss.kibo.rpc.sampleapk;

import java.io.InputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.util.Log;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;

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
                Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
                List<Mat> corners = new ArrayList<>();
                Mat markerIds = new Mat();
                Aruco.detectMarkers(image, dictionary, corners, markerIds);

                // //Get corner information
                // for (Mat corner : corners) {
                //     //Process the corner to determine the rotaion vector and translation vector
//                ค่อยมาใส่โค้ดแปลงcornnersทีหลัง
                // }

                //check AR
                if (corners.isEmpty()) {
                    Log.w(TAG, "Cannot detect AR");
                    continue;
                    //corner is list of Ar tag information naja jubjub
                } else {
                    Log.i(TAG, "Here AR");
                    arCounter++;
//                    break;
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

                    //convert to gray scale
                    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_BGR2GRAY);

                    //assign to an array of template
                    templates[i] = mat;
                    inputStream.close();


                } catch (IOException e) {
                    e.printStackTrace();
                }
            }

            //Number of matches for each template
            int templateMatchCnt[] = new int[templates.length];
            //get num of template matches
            for (int tempNum = 0; tempNum < templates.length; tempNum++) {
                int matchCnt = 0;

                List<org.opencv.core.Point> matches = new ArrayList<>();

                //loding template image target
                Mat template = templates[tempNum].clone();
                Mat targetImg = undistortImg.clone();

                //pattern matching =>px
                int widthMin = 20;
                int widthMax = 100;
                int changeWidth = 5;
                int changeAngle = 45;

                for (int size = widthMin; size < widthMax; size += changeWidth) {
                    for (int angle = 0; angle < 360; angle += changeAngle) {
                        Mat resizedTemp = resizeImg(template, size);
                        Mat rotResizedTemp = rotImg(resizedTemp, angle);

                        Mat result = new Mat();
                        Imgproc.matchTemplate(targetImg, rotResizedTemp, result, Imgproc.TM_CCOEFF_NORMED);

                        //get similar by threshold
                        double threshold = 0.7;
                        Core.MinMaxLocResult mmlr = Core.minMaxLoc(result);
                        double maxVal = mmlr.maxVal;
                        if (maxVal >= threshold) {
                            Mat thresholdedResult = new Mat();
                            Imgproc.threshold(result, thresholdedResult, threshold, 1.0, Imgproc.THRESH_TOZERO);

                            //Get location
                            for (int y = 0; y < thresholdedResult.rows(); y++) {
                                for (int x = 0; x < thresholdedResult.cols(); x++) {
                                    if (thresholdedResult.get(y, x)[0] > 0) {
//                                    matchCnt++;
                                        matches.add(new org.opencv.core.Point(x, y));
                                        Log.i(TAG, "matches somthing IDK");
//                                    Log.i(TAG, String.valueOf(filteredMatches.size()));

                                    }
                                }
                            }
                        }
                    }
                }

                List<org.opencv.core.Point> filteredMatches = removeDuplicates(matches);
                matchCnt += filteredMatches.size();

                templateMatchCnt[tempNum] = matchCnt;

            }

            //When u recognize landmark items let's type num
            int mostMatchTemplateNum = getMaxIndex(templateMatchCnt);
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

    //REsize image
    private Mat resizeImg(Mat img, int width) {
        int height = (int) (img.rows() * ((double) width / img.cols()));
        Mat resizedImg = new Mat();
        Imgproc.resize(img, resizedImg, new Size(width, height));

        return resizedImg;
    }

    //rotate image
    private Mat rotImg(Mat img, int angle) {
        org.opencv.core.Point center = new org.opencv.core.Point(img.cols() / 2.0, img.rows() / 2.0);
        Mat rotatedMat = Imgproc.getRotationMatrix2D(center, angle, 1.0);
        Mat rotatedImg = new Mat();
        Imgproc.warpAffine(img, rotatedImg, rotatedMat, img.size());
        return rotatedImg;
    }

    //remove multiple direction
    private List<org.opencv.core.Point> removeDuplicates(List<org.opencv.core.Point> points) {
        double lenght = 10;
        List<org.opencv.core.Point> filteredList = new ArrayList<>();

        for (org.opencv.core.Point point : points) {
            boolean isInclude = false;
            for (org.opencv.core.Point checkPoint : filteredList) {
                double distance = calculateDistance(point, checkPoint);

                if (distance <= lenght) {
                    isInclude = true;
                    break;
                }
            }
            if (!isInclude) {
                filteredList.add(point);
            }
        }

        return filteredList;

    }

    //calculate between two point
    private double calculateDistance(org.opencv.core.Point p1, org.opencv.core.Point p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;

        return Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));
    }


    //maximum value of array
    private int getMaxIndex(int[] array) {
        int max = 0;
        int maxIndex = 0;

        //find index of element with largest value
        for (int i = 0; i < array.length; i++) {
            if (array[i] > max) {
                max = array[i];
                maxIndex = i;
            }
        }
        return maxIndex;
    }
}


// //=====move test ======
// for(int i=0;i<4;i++){
//     api.moveTo(oasispoint.get(i), oasisQuaternion.get(i), false);
// }
// //=====stop test====