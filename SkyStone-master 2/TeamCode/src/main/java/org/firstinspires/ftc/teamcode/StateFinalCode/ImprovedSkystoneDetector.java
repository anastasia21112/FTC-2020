/* Team 16748
 * Lucas Erickson
 */

package org.firstinspires.ftc.teamcode.StateFinalCode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ImprovedSkystoneDetector extends OpenCvPipeline {


    /* Stone one bounds */
    int stoneOneLeft = 50;
    int stoneOneRight = 100;
    int stoneOneBottom = 190;
    int stoneOneTop = 160;

    /* Stone two bounds */
    int stoneTwoLeft = 101;
    int stoneTwoRight = 150;
    int stoneTwoBottom = 190;
    int stoneTwoTop = 160;

    /* Stone three bounds */
    int stoneThreeLeft = 151;
    int stoneThreeRight = 200;
    int stoneThreeBottom = 190;
    int stoneThreeTop = 160;

    /* Show average color of each stone */
    boolean debug_averageRGB;



    private boolean stoneOneisSkystone = false;
    private boolean stoneTwoisSkystone = false;
    private boolean stoneThreeisSkystone = false;

    public ImprovedSkystoneDetector()
    {
        debug_averageRGB = true;
    }

    public ImprovedSkystoneDetector(boolean debug_averageRGB)
    {
        this.debug_averageRGB = debug_averageRGB;
    }



    @Override
    public Mat processFrame(Mat input)
    {
        /* RGB values of each block */
        int stoneOneRed = 0;
        int stoneOneGreen = 0;
        int stoneOneBlue = 0;

        int stoneTwoRed = 0;
        int stoneTwoGreen = 0;
        int stoneTwoBlue = 0;

        int stoneThreeRed = 0;
        int stoneThreeGreen = 0;
        int stoneThreeBlue = 0;

        /* Number of pixels tested on each block (for average) */
        int stoneOnePixels = 0;
        int stoneTwoPixels = 0;
        int stoneThreePixels = 0;

        /* Draw stone bounds */
        Imgproc.rectangle(input, new Point(stoneOneLeft, stoneOneBottom), new Point(stoneOneRight, stoneOneTop), new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, new Point(stoneTwoLeft, stoneTwoBottom), new Point(stoneTwoRight, stoneTwoTop), new Scalar(0, 0, 255), 1);
        Imgproc.rectangle(input, new Point(stoneThreeLeft, stoneThreeBottom), new Point(stoneThreeRight, stoneThreeTop), new Scalar(0, 0, 255), 1);



        /* Iterate through stone one bounds */
        for(int x = stoneOneLeft; x < stoneOneRight; x++)
        {
            for(int y = stoneOneTop; y < stoneOneBottom; y++)
            {
                double[] values = input.get(y, x);
                if(values != null)
                {
                    stoneOnePixels++;
                    stoneOneRed += values[0];
                    stoneOneGreen += values[1];
                    stoneOneBlue += values[2];
                }
            }
        }


        /* Iterate through stone two bounds */
        for(int x = stoneTwoLeft; x < stoneTwoRight; x++)
        {
            for(int y = stoneTwoTop; y < stoneTwoBottom; y++)
            {
                double[] values = input.get(y, x);
                if(values != null)
                {
                    stoneTwoPixels++;
                    stoneTwoRed += values[0];
                    stoneTwoGreen += values[1];
                    stoneTwoBlue += values[2];
                }
            }
        }


        /* Iterate through stone three bounds */
        for(int x = stoneThreeLeft; x < stoneThreeRight; x++)
        {
            for(int y = stoneThreeTop; y < stoneThreeBottom; y++)
            {
                double[] values = input.get(y, x);
                if(values != null)
                {
                    stoneThreePixels++;
                    stoneThreeRed += values[0];
                    stoneThreeGreen += values[1];
                    stoneThreeBlue += values[2];
                }
            }
        }

        stoneOneRed = stoneOneRed/stoneOnePixels;
        stoneOneGreen = stoneOneGreen/stoneOnePixels;
        stoneOneBlue = stoneOneBlue/stoneOnePixels;

        stoneTwoRed = stoneTwoRed/stoneTwoPixels;
        stoneTwoGreen = stoneTwoGreen/stoneTwoPixels;
        stoneTwoBlue = stoneTwoBlue/stoneTwoPixels;

        stoneThreeRed = stoneThreeRed/stoneThreePixels;
        stoneThreeGreen = stoneThreeGreen/stoneThreePixels;
        stoneThreeBlue = stoneThreeBlue/stoneThreePixels;

        int stoneOneAvg = (stoneOneRed + stoneOneGreen + stoneOneBlue) / 3;
        int stoneTwoAvg = (stoneTwoRed + stoneTwoGreen + stoneTwoBlue) / 3;
        int stoneThreeAvg = (stoneThreeRed + stoneThreeGreen + stoneThreeBlue) / 3;

        stoneOneisSkystone = stoneOneAvg < stoneTwoAvg && stoneOneAvg < stoneThreeAvg;
        stoneTwoisSkystone = stoneTwoAvg < stoneOneAvg && stoneTwoAvg < stoneThreeAvg;
        stoneThreeisSkystone = stoneThreeAvg < stoneOneAvg && stoneThreeAvg < stoneTwoAvg;

        if(debug_averageRGB)
        {
            Imgproc.circle(input, new Point((stoneOneLeft + stoneOneRight) / 2, (stoneOneTop + stoneOneBottom) / 2 - 100), 15, new Scalar(stoneOneRed, stoneOneGreen, stoneOneBlue), -1);
            Imgproc.circle(input, new Point((stoneTwoLeft + stoneTwoRight) / 2, (stoneTwoTop + stoneTwoBottom) / 2 - 100), 15, new Scalar(stoneTwoRed, stoneTwoGreen, stoneTwoBlue), -1);
            Imgproc.circle(input, new Point((stoneThreeLeft + stoneThreeRight) / 2, (stoneThreeTop + stoneThreeBottom) / 2 - 100), 15, new Scalar(stoneThreeRed, stoneThreeGreen, stoneThreeBlue), -1);
        }

        Imgproc.circle(input, new Point((stoneOneLeft + stoneOneRight) / 2, (stoneOneTop + stoneOneBottom) / 2 - 100), 17, stoneOneisSkystone?new Scalar(0, 255, 0):new Scalar(255, 0, 0), 3);
        Imgproc.circle(input, new Point((stoneTwoLeft + stoneTwoRight) / 2, (stoneTwoTop + stoneTwoBottom) / 2 - 100), 17, stoneTwoisSkystone?new Scalar(0, 255, 0):new Scalar(255, 0, 0), 3);
        Imgproc.circle(input, new Point((stoneThreeLeft + stoneThreeRight) / 2, (stoneThreeTop + stoneThreeBottom) / 2 - 100), 17, stoneThreeisSkystone?new Scalar(0, 255, 0):new Scalar(255, 0, 0), 3);



        return input;
    }

    public boolean stoneFourisSkystone()
    {
        return stoneOneisSkystone;
    }
    public boolean stoneFiveisSkystone()
    {
        return stoneTwoisSkystone;
    }
    public boolean stoneSixisSkystone()
    {
        return stoneThreeisSkystone;
    }

    public void setStoneOneBounds(int top, int bottom, int left, int right)
    {
        stoneOneTop = top;
        stoneOneBottom = bottom;
        stoneOneLeft = left;
        stoneOneRight = right;
    }

    public void setStoneTwoBounds(int top, int bottom, int left, int right)
    {
        stoneTwoTop = top;
        stoneTwoBottom = bottom;
        stoneTwoLeft = left;
        stoneTwoRight = right;
    }

    public void setStoneThreeBounds(int top, int bottom, int left, int right)
    {
        stoneThreeTop = top;
        stoneThreeBottom = bottom;
        stoneThreeLeft = left;
        stoneThreeRight = right;
    }

}