package transforma.qb2_user_app.quicabotMap;

import android.content.Context;
import android.opengl.GLSurfaceView;
import android.util.AttributeSet;
import android.view.MotionEvent;

import transforma.qb2_user_app.ConfigurationParameters;


public class MapView extends GLSurfaceView {

    static final float INITIAL_X = ConfigurationParameters.INITIAL_X;
    static final float INITIAL_Y = ConfigurationParameters.INITIAL_Y;
    static final float INITIAL_Z = 0.0f;
    static final float RESOLUTION = ConfigurationParameters.PIXELRESOLUTION;
    static final int DEFINITION_X = ConfigurationParameters.DEFINITION_X;
    static final int DEFINITION_Y = ConfigurationParameters.DEFINITION_Y;


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



    private MapRenderer renderer;
    private float screenDensity;

    public void init(MapRenderer renderer, float screenDensity){
        setEGLContextClientVersion(3);
        this.renderer = renderer;
        this.screenDensity = screenDensity;
        setMinimumWidth(this.getHeight());
        setRenderer(renderer);
    }


    /**
     * Standard View constructor. In order to render something, you
     * must call {@link #setRenderer} to register a renderer.
     *
     * @param context
     * @param attrs
     */
    public MapView(Context context, AttributeSet attrs) {
        super(context, attrs);
    }



    @Override
    public boolean onTouchEvent(MotionEvent event)
    {
        float touchedXSingle; //x coordinate for one touch point
        float touchedYSingle; //...
        float touchedXDouble; //Average x coordinate for two touch points
        float touchedYDouble; //...
        float rotationAngle;  //Current totation angle of two touch points
        double zoomscale;

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
                    zoomscale = getZoomscale(event);
                    previousZoomScale = zoomscale;

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
                    zoomscale = getZoomscale(event);
                    renderer.fscale = renderer.fscale * (float) Math.sqrt(zoomscale / previousZoomScale);
                    previousZoomScale = zoomscale;
                    //Translate control
                    touchedXDouble = getXdouble(event);
                    touchedYDouble = getYdouble(event);
                    renderer.fdYTranslation += (touchedXDouble - previousXDouble) / screenDensity / 40f / renderer.fscale;
                    renderer.fdXTranslation -= (touchedYDouble - previousYDouble) / screenDensity / 40f / renderer.fscale;
                    previousXDouble = touchedXDouble;
                    previousYDouble = touchedYDouble;


                }

        }
        return true;
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

}
