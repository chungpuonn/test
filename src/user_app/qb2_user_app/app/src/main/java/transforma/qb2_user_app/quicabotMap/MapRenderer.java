package transforma.qb2_user_app.quicabotMap;

import android.app.Activity;
import android.opengl.GLES20;
import android.opengl.GLES32;
import android.opengl.GLSurfaceView;
import android.opengl.Matrix;
import android.util.Log;
import android.widget.TextView;

import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import transforma.qb2_user_app.ConfigurationParameters;
import transforma.qb2_user_app.R;
import transforma.qb2_user_app.UtilsOpenGl;

public class MapRenderer implements GLSurfaceView.Renderer {

    private static float eyeX = MapView.INITIAL_X, eyeY = MapView.INITIAL_Y, eyeZ = 10f; // Means we position our eye at coordinate (INITIAL_X, INITIAL_Y, -20f), below the surface
    private static float centerX = MapView.INITIAL_X, centerY = MapView.INITIAL_Y, centerZ = 0; // Means we are focusing out eyes/ looking at coordinate(-0.2, 0, 1.3)
    private static float upX = 1f, upY = 0f, upZ = 0f; // Means the axis point up is Y-axis
    private static float bot = -ConfigurationParameters.MAPVIEWRANGE, top = ConfigurationParameters.MAPVIEWRANGE, near = 9.5f, far = 10.5f; // Means the view range of our eyes, we cannot see items positioned closer than 1f (meanwhile below ar above than 0.4f) and further than 1000f


    float fdZRotation = 0f; // dz
    float fdZRotationAccu = 0;
    float fdXTranslation = 0f; // Translation in dx controlled by onTouchEvent in view
    float fdYTranslation = 0f; // dy and translation in dz not supported
    public static float fscale = 1f; // zoom scale controlled by onTouchEvent in view
    static final float ZERO = 0f;

    /** How many elements per vertex. */
    private final int mStrideBytes = 7 * 4;

    /** Offset of the position data. */
    private final int mPositionOffset = 0;

    /** Size of the position data in elements. */
    private final int mPositionDataSize = 3;

    /** Offset of the color data. */
    private final int mColorOffset = 3;

    /** Size of the color data in elements. */
    private final int mColorDataSize = 4;


    /**
     * Storing screen width/ height ratio
     */
    float ratio;

    /**
     * Stores relevant openGL asset attribute id
     */

    //Robot position renderer markers
    private int iRobotProgrammeID;
    private int iRobotTexId;
    private int iRobotPosition;
    private int iRobotTexture;
    private int iRobotMVPMatrix;

    //BGMap renderer markers
    private int iProgrammeID;
    private int iPosition;
    private int iMVPMatrix;
    private int iColor;
    private int iTexture;
    private int iTexId; //Texture we used
    private int iSize;

    private int height;

    /** Store the accumulated rotation of the object */
    private float[] mAccumulatedRotation = new float[16];

    /** Store the current rotation of the object  */
    private float[] mCurrentRotation = new float[16];



    /** Allocate storage for the final combined matrix for temporary(as it refreshes 60 times per second). This will be passed into the shader program. */
    private float[] mTemporary = new float[16];

    /** Store the projection matrix. This is used to project the scene onto a 2D viewport. */
    private float[] mProjectionMatrix = new float[16];

    /**
     * Store the view matrix. This can be thought of as a camera. This matrix transforms world space to eye space;
     * it positions things relative to our eye.
     */
    private float[] mViewMatrix = new float[16];
    private float[] mModelMatrix = new float[16];

    private Activity context;
    TextView mapLegend; //Displaying current zoom scale
    private FloatBuffer vertexBuffer = null; // Store all pointCLoud information
    private FloatBuffer robotBuffer = null; //Store robot position point information
    private int vertexCount = 0;
    private boolean updateMapLegendOrNot = false;



    public MapRenderer(Activity view, TextView mapLegend){
        context = view;
        this.mapLegend = mapLegend;
    }


    /**
     * Called when the surface is created or recreated.
     * <p>
     * Called when the rendering thread
     * starts and whenever the EGL context is lost. The EGL context will typically
     * be lost when the Android device awakes after going to sleep.
     * <p>
     * Since this method is called at the beginning of rendering, as well as
     * every time the EGL context is lost, this method is a convenient place to put
     * code to create resources that need to be created when the rendering
     * starts, and that need to be recreated when the EGL context is lost.
     * Textures are an example of a resource that you might want to create
     * here.
     * <p>
     * Note that when the EGL context is lost, all OpenGL resources associated
     * with that context will be automatically deleted. You do not need to call
     * the corresponding "glDelete" methods such as glDeleteTextures to
     * manually delete these lost resources.
     * <p>
     *
     * @param gl     the GL interface. Use <code>instanceof</code> to
     *               test if the interface supports GL11 or higher interfaces.
     * @param config the EGLConfig of the created surface. Can be used
     */
    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        GLES32.glClearColor(0.82745f, 0.55294f, ZERO, 1);

        String pointVertexShader = UtilsOpenGl.readTextFileFromRawResource(context, R.raw.map_point_vertex_shader);//Map points Vertex Shader
        String pointFragmentShader = UtilsOpenGl.readTextFileFromRawResource(context, R.raw.map_point_fragment_shader);//Map points fragment shader
        String robotVertexShader = UtilsOpenGl.readTextFileFromRawResource(context, R.raw.map_robot_pos_vertex_shader);//Robot points vertex shader
        String robotFragmentShader = UtilsOpenGl.readTextFileFromRawResource(context, R.raw.map_robot_pos_fragment_shader);//Robot points fragment shader


        //Bind all map points attributes

        iProgrammeID = UtilsOpenGl.LoadProgram(pointVertexShader, pointFragmentShader);
        iTexId = UtilsOpenGl.LoadTexture(context, R.drawable.map_tex);
        iPosition = GLES32.glGetAttribLocation(iProgrammeID, "a_Position");
        iTexture = GLES32.glGetUniformLocation(iProgrammeID, "u_texture");
        iColor = GLES32.glGetAttribLocation(iProgrammeID, "a_Color");
        iSize = GLES32.glGetAttribLocation(iProgrammeID,"a_Size");
        iMVPMatrix = GLES32.glGetUniformLocation(iProgrammeID, "u_MVPMatrix");


        //Bind all robot points attributes

        iRobotProgrammeID = UtilsOpenGl.LoadProgram(robotVertexShader, robotFragmentShader);
        iRobotTexId = UtilsOpenGl.LoadTexture(context, R.drawable.robot_position);
        iRobotPosition = GLES32.glGetAttribLocation(iRobotProgrammeID, "a_Position");
        iRobotTexture = GLES32.glGetUniformLocation(iRobotProgrammeID, "u_texture");
        iRobotMVPMatrix = GLES32.glGetUniformLocation(iProgrammeID, "u_MVPMatrix");


        //Initialise translation and transpose information to 0

        Matrix.setIdentityM(mAccumulatedRotation, 0);



    }

    /**
     * Called when the surface changed size.
     * <p>
     * Called after the surface is created and whenever
     * the OpenGL ES surface size changes.
     * <p>
     * Typically you will set your viewport here. If your camera
     * is fixed then you could also set your projection matrix here:
     * <pre class="prettyprint">
     * void onSurfaceChanged(GL10 gl, int width, int height) {
     *     gl.glViewport(0, 0, width, height);
     *     // for a fixed camera, set the projection too
     *     float ratio = (float) width / height;
     *     gl.glMatrixMode(GL10.GL_PROJECTION);
     *     gl.glLoadIdentity();
     *     gl.glFrustumf(-ratio, ratio, -1, 1, 1, 10);
     * }
     * </pre>
     *
     * @param gl     the GL interface. Use <code>instanceof</code> to
     *               test if the interface supports GL11 or higher interfaces.
     * @param width
     * @param height
     */
    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        this.height = height;
        Log.i("Height", Integer.toString(height));

        GLES32.glViewport(0, 0, width, height); //Initiate

        GLES32.glDisable(GLES32.GL_DEPTH_TEST); //This step is to make sure blending works well as enabling it conflicts with blending
        GLES32.glEnable(GLES32.GL_BLEND);

        //This blending function is particularly for our map display, it shows best result.
        //I want to make the .png texture show opacity at the points and the transparent part showing the background color.
        GLES32.glBlendFunc(GLES32.GL_ONE, GLES32.GL_ONE_MINUS_SRC_ALPHA);

        ratio =  (float) width / (float) height  ; //Calculate screen ratio
        Matrix.frustumM(mProjectionMatrix, 0, ratio * bot, - ratio * bot, bot, top, near, far); //Set our viewing range

        Matrix.setLookAtM(mViewMatrix, 0,eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ); //Set our eye position

    }

    /**
     * Called to draw the current frame.
     * <p>
     * This method is responsible for drawing the current frame.
     * <p>
     * The implementation of this method typically looks like this:
     * <pre class="prettyprint">
     * void onDrawFrame(GL10 gl) {
     *     gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
     *     //... other gl calls to render the scene ...
     * }
     * </pre>
     *
     * @param gl the GL interface. Use <code>instanceof</code> to
     *           test if the interface supports GL11 or higher interfaces.
     */
    @Override
    public void onDrawFrame(GL10 gl) {

        GLES32.glClear(GLES32.GL_COLOR_BUFFER_BIT);

        // Change viewing range by detecting change in two finger zoom event
        Matrix.frustumM(mProjectionMatrix, 0, ratio*bot/fscale, -ratio*bot/fscale, bot/fscale, top/fscale, near, far);
        // Translate eye focus point by detecting change in two finger drag event
        Matrix.setLookAtM(mViewMatrix, 0, eyeX  - fdXTranslation , eyeY + fdYTranslation, eyeZ, centerX - fdXTranslation, centerY + fdYTranslation, centerZ, upX, upY, upZ);

        //Calculate accumulated rotation
        fdZRotationAccu += fdZRotation;


        // Set a matrix that contains the current rotation.

        Matrix.setIdentityM(mCurrentRotation, 0);
        Matrix.translateM(mCurrentRotation,0,centerX,centerY,centerZ);
        Matrix.rotateM(mCurrentRotation, 0, fdZRotation, 0f, 0f, 1f);
        Matrix.translateM(mCurrentRotation,0,-centerX,-centerY,-centerZ);

        fdZRotation = 0f;

        // Multiply the current rotation by the accumulated rotation, and then set the accumulated rotation to the result.

        Matrix.multiplyMM(mTemporary, 0, mCurrentRotation, 0, mAccumulatedRotation, 0);
        System.arraycopy(mTemporary, 0, mAccumulatedRotation, 0, 16);
        // Multiply our eye info
        Matrix.multiplyMM(mTemporary,0, mViewMatrix,0,mTemporary,0);
        // Multiply view range info
        Matrix.multiplyMM(mTemporary,0, mProjectionMatrix,0, mTemporary,0);
        // Set final result



        //The lock enables the buffer wont edited when the rosBridge is updating it
        if (!QuicabotMapPage.lock && !QuicabotMapPage.firstLaunch ){
            vertexCount = QuicabotMapPage.pointCount /7;
            vertexBuffer = QuicabotMapPage.mapBuffer;
        }
        if (vertexBuffer != null){
            drawMap(); //Draw map
        }

        if (!QuicabotMapPage.robotDataLock){
            robotBuffer = QuicabotMapPage.robotBuffer;
        }
        if (robotBuffer != null) {
            drawRobot(); //Draw the robot position
        }


        updateMapLegendOrNot = !updateMapLegendOrNot;

        if (updateMapLegendOrNot) {
            context.runOnUiThread(()-> mapLegend.setText(String.format("%.1fm", top/2/fscale))); //When we use finger to make zoom, the scale is reflected on screen
        }


    }

    private void drawRobot() {



        //Draw robot sequence
        GLES32.glUseProgram(iRobotProgrammeID);
        GLES32.glActiveTexture(GLES32.GL_TEXTURE0);
        GLES32.glBindTexture(GLES32.GL_TEXTURE_2D,iRobotTexId);
        GLES32.glUniform1f(iRobotTexture,0);


        robotBuffer.position(0);
        GLES32.glVertexAttribPointer(iRobotPosition, mPositionDataSize,GLES32.GL_FLOAT, false, 12, robotBuffer);
        GLES32.glEnableVertexAttribArray(iRobotPosition);

        GLES32.glUniformMatrix4fv(iRobotMVPMatrix, 1, false, mTemporary, 0);

        //THe robot position is composed od two triangles forming an arrow
        GLES32.glDrawArrays(GLES20.GL_TRIANGLES, 0, 6);


    }


    private void drawMap(){



        GLES32.glUseProgram(iProgrammeID);
        GLES32.glActiveTexture(GLES32.GL_TEXTURE0);
        GLES32.glBindTexture(GLES32.GL_TEXTURE_2D,iTexId);
        GLES32.glUniform1f(iTexture,0);



        vertexBuffer.position(mPositionOffset);
        GLES32.glVertexAttribPointer(iPosition, mPositionDataSize,GLES32.GL_FLOAT, false, mStrideBytes, vertexBuffer);
        GLES32.glEnableVertexAttribArray(iPosition);
        vertexBuffer.position(mColorOffset);
        GLES32.glVertexAttribPointer(iColor, mColorDataSize, GLES32.GL_FLOAT, false, mStrideBytes, vertexBuffer);
        GLES32.glEnableVertexAttribArray(iColor);

        float rotationPointSizeZoom = (float)(1 + Math.abs(0.5 * Math.sin(Math.toRadians(fdZRotationAccu))));


        //As we are drawing the map by drawing the pixels we have, the pixel size should be just nice to cover the map leaving zero gap or stack
        //Therefore, the point size should take into consideration of：
        //1. Raw data resolution eg, how long a pixel should represent in real case , here is 5cm
        //2. The viewing port of our eyes on the screen eg, how much length our display is representing (bot, top), and what is the current zoom scale
        //3. The actual pixels on the screen our GLView have
        //4. When the view rotates, the distance between become sqt2 at 45 degress and back to 1 at 90 degress, therefore to avoid gaps the size fo point need to be adjusted according to rotation angle
        GLES32.glVertexAttrib1f(iSize, 1.2f * height/-(bot/fscale)/2*MapView.RESOLUTION * rotationPointSizeZoom);




        GLES32.glUniformMatrix4fv(iMVPMatrix, 1, false, mTemporary, 0);


        // Draw as many point as we have
        GLES32.glDrawArrays(GLES32.GL_POINTS, 0, vertexCount);

    }
}
