package transforma.qb2_user_app.quicabotPointcloud;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.view.MotionEvent;

/**
 * An implementation of SurfaceView that uses the dedicated surface for
 * displaying OpenGL PointCloud rendering.
 * <p>
 * A GLSurfaceView provides the following features:
 * <p>
 * <ul>
 * <li>Manages a surface, which is a special piece of memory that can be
 * composited into the Android view system.
 * <li>Manages an EGL display, which enables OpenGL to render into a surface.
 * <li>Accepts a user-provided Renderer object that does the actual rendering.
 * <li>Renders on a dedicated thread to decouple rendering performance from the
 * UI thread.
 * <li>Supports both on-demand and continuous rendering.
 * <li>Optionally wraps, traces, and/or error-checks the renderer's OpenGL calls.
 * </ul>
 *
 */

public class PointCloudView extends GLSurfaceView {

    private float mode = 0;
    /**
     * Mark the index of the first onTouch finger
     */
    private int primaryPointIndex = -1;
    /**
     * Mark the index of the second onTouch finger
     */
    private int secondaryPointIndex = -1;

    /**
     * Previous x axis touching position of the single finger before moves
     */
    private float previousXSingle;

    /**
     * Previous y axis touching position of the single finger before moves
     */
    private float previousYSingle;
    /**
     * Previous x axis touching position in the middle of the two fingers before moves
     */
    private float previousXDouble;
    /**
     * Previous y axis touching position in the middle of the two fingers before moves
     */
    private float previousYDouble;
    /**
     * Previous rotation position of the two fingers before moves
     */
    private float previousRotationAngle;
    /**
     * Previous zoom position (distance) of the two fingers before moves
     */
    private double previousZoomScale;
    /**
     * The pixel density of the current screen for adjusting moving sensitivity
     */
    private float screenDensity;

    PointCloudRenderer renderer;


    /**
     * SHould be called when firstly initiate the view
     * @param renderer THe renderer to be passed in for the view
     * @param screenDensity Screendensity of the mobile device
     */

    public void init(PointCloudRenderer renderer, float screenDensity){
        setEGLContextClientVersion(3);
        this.renderer = renderer;
        this.screenDensity = screenDensity;
        setRenderer(renderer);
    }

    public PointCloudView(Context context, AttributeSet attrs){
        super(context, attrs);
    }

    /**
     * This is a method that detects touch event so as to manipulate pointCloud respectively
     *
     * 1. One finger move for X, Y rotation
     * 2. Two finger drag for X, Y translate
     * 3. Two finger rotate for Z rotation
     * 4, Two finger zoom for camera zoom
     * @param event
     * @return
     */

    @Override
    public boolean onTouchEvent(MotionEvent event)
    {
        float touchedXSingle; //x coordinate for one touch point
        float touchedYSingle; //...
        float touchedXDouble; //Average x coordinate for two touch points
        float touchedYDouble; //...
        float rotationAngle;  //Current totation angle of two touch points
        double zoomscalde;

        switch (event.getAction() & MotionEvent.ACTION_MASK){
            case MotionEvent.ACTION_DOWN: //1st finger touches down
                primaryPointIndex = event.getActionIndex();
                touchedXSingle = event.getX();
                touchedYSingle = event.getY();
                previousXSingle = touchedXSingle;
                previousYSingle = touchedYSingle;
                break;
            case MotionEvent.ACTION_POINTER_DOWN: // 2nd finger or more fingers touch down
                secondaryPointIndex = event.getActionIndex();
                if (event.getPointerCount() == 2){
                    rotationAngle = getRotation(event);
                    previousRotationAngle = rotationAngle;
                    zoomscalde = getZoomscale(event);
                    previousZoomScale = zoomscalde;

                    touchedXDouble = getXdouble(event);
                    touchedYDouble = getYdouble(event);
                    previousXDouble = touchedXDouble;
                    previousYDouble = touchedYDouble;

                }

                break;
            case MotionEvent.ACTION_POINTER_UP:// 2nd finger or other fingers up (still have one finger down)
                if (event.getPointerCount() == 2) {
                    if(event.getActionIndex() == primaryPointIndex) primaryPointIndex = secondaryPointIndex;
                    touchedXSingle = event.getX(primaryPointIndex);
                    touchedYSingle = event.getY(primaryPointIndex);
                    previousXSingle = touchedXSingle;
                    previousYSingle = touchedYSingle;
                    secondaryPointIndex = -1;
                }
                break;
            case MotionEvent.ACTION_UP: // All finger up
                mode = 0;
                break;
            case MotionEvent.ACTION_MOVE: // Finger move
                if (event.getPointerCount() == 2){
                    //Z Axis Rotation Control
                    rotationAngle = getRotation(event);
                    renderer.fdZRotation += (previousRotationAngle - rotationAngle);
                    previousRotationAngle = rotationAngle;

                    //Zoom control
                    zoomscalde = getZoomscale(event);
                    renderer.fscale = renderer.fscale * (float) Math.sqrt(zoomscalde / previousZoomScale);
                    previousZoomScale = zoomscalde;

                    //Translate control
                    touchedXDouble = getXdouble(event);
                    touchedYDouble = getYdouble(event);
                    renderer.fdXTranslation += (touchedXDouble - previousXDouble) / screenDensity / 150f / renderer.fscale;
                    renderer.fdYTranslation += (touchedYDouble - previousYDouble) / screenDensity / 150f / renderer.fscale;
                    previousXDouble = touchedXDouble;
                    previousYDouble = touchedYDouble;


                } else if (event.getPointerCount() == 1){
                    //X, Y Axis Rotation Control
                    touchedXSingle = event.getX();
                    touchedYSingle = event.getY();
                    renderer.fdXRotation += (touchedXSingle - previousXSingle) / screenDensity / 2f;
                    renderer.fdYRotation += (touchedYSingle - previousYSingle) / screenDensity / 2f;
                    previousXSingle = touchedXSingle;
                    previousYSingle = touchedYSingle;
                }

        }


        return true;
    }

    /**
     * Get the current angle in degree of two touched points
     * @param event
     * @return
     */

    private float getRotation(MotionEvent event) {
        double x = (event.getX(0) - event.getX(1));
        double y = (event.getY(0) - event.getY(1));
        double radians = Math.atan2(y, x);
        return (float) Math.toDegrees(radians);
    }

    /**
     * Get the distance between two touched points
     * @param event
     * @return
     */
    private double getZoomscale(MotionEvent event) {
        float x = event.getX(0) - event.getX(1);
        float y = event.getY(0) - event.getY(1);
        return  Math.sqrt(x * x + y * y);
    }

    /**
     * Get an average X coordinate between two touch points
     * @param event
     * @return
     */

    private float getXdouble(MotionEvent event){
        return  (event.getX(0) + event.getX(1))/2;
    }

    /**
     * Get an average Y coordinate between two touch points
     * @param event
     * @return
     */

    private float getYdouble(MotionEvent event){
        return  (event.getY(0) + event.getY(1))/2;
    }




}
