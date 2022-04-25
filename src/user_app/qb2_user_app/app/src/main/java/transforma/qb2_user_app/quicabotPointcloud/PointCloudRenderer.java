package transforma.qb2_user_app.quicabotPointcloud;

import android.content.Context;
import android.opengl.GLES32;
import android.opengl.GLSurfaceView.Renderer;
import android.opengl.Matrix;

import java.nio.FloatBuffer;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import transforma.qb2_user_app.*;
import transforma.qb2_user_app.rosmsg.InspectionResult;

/**
 * This class implements our custom renderer for rendering white pointclouds in a certain view port using OpenGL ES 3.2
 */

public class PointCloudRenderer implements Renderer {

    InspectionResult inspectionResult; //Used to get all pointCloud data

    private static float eyeX = 0f, eyeY = 0f, eyeZ = -1f; // Means we position our eye at coordinate (0, 0, -1)
    private static float centerX = -0.2f, centerY = 0f, centerZ = 1.3f; // Means we are focusing out eyes/ looking at coordinate(-0.2, 0, 1.3)
    private static float upX = 0f, upY = 1f, upZ = 0f; // Means the axis point up is Y-axis
    private static float bot = -0.4f, top = 0.4f, near = 1f, far = 100f; // Means the view range of our eyes, we cannot see items positioned closer than 1f (meanwhile below ar above than 0.4f) and further than 1000f


    static final int BYTES_PER_FLOAT = 4; // 4 bytes used to represent a float in Java
    static final float ZERO = 0f;
    float fdXRotation = 0f; // The rotation in dx controlled by onTouchEvent in view
    float fdYRotation = 0f; // dy
    float fdZRotation = 0f; // dz
    float fdXTranslation = 0f; // Translation in dx controlled by onTouchEvent in view
    float fdYTranslation = 0f; // dy and translation in dz not supported
    float fscale = 1f; // zoom scale controlled by onTouchEvent in view

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
    /**
     * Storing screen width/ height ratio
     */
    float ratio;

    /**
     * Stores relevant openGL asset attribute id
     */
    private int iProgrammeID;
    private int iPosition;
    private int iMVPMatrix;

    private Context context;
    private FloatBuffer vertexBuffer; // Store all pointCLoud information
    private int gLBufferIndex;
    private int vertexCount;

    /**
     * Construct the renderer and read in all pointCloud data
     * @param view
     */
    public PointCloudRenderer(Context view)
    {
        inspectionResult = InspectionResult.getInspectionResult();
        context = view;
        vertexBuffer = inspectionResult.buffer;
        vertexBuffer.position(0);
        vertexCount = vertexBuffer.capacity();
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
     * @param gl the GL interface. Use <code>instanceof</code> to
     * test if the interface supports GL11 or higher interfaces.
     * @param config the EGLConfig of the created surface. Can be used
     * to create matching pbuffers.
     */


    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {

        GLES32.glClearColor(ZERO, ZERO, ZERO, 1); 	//Background color
        //Reading Shader Raw String
        String pointVertexShader = UtilsOpenGl.readTextFileFromRawResource(context, R.raw.point_vertex_shader);
        String pointFragmentShader = UtilsOpenGl.readTextFileFromRawResource(context, R.raw.point_fragment_shader);

        iProgrammeID = UtilsOpenGl.LoadProgram(pointVertexShader, pointFragmentShader);
        iPosition = GLES32.glGetAttribLocation(iProgrammeID, "a_Position");
        iMVPMatrix = GLES32.glGetUniformLocation(iProgrammeID, "u_MVPMatrix");
        Matrix.setIdentityM(mAccumulatedRotation, 0);
        Matrix.setIdentityM(mModelMatrix, 0);


        // Generate as many buffers as we need.
        final int[] gLBuffer = new int[1];
        // This will give us the OpenGL handles for these buffers.
        GLES32.glGenBuffers(1, gLBuffer,0);
        // Bind to the buffer.
        GLES32.glBindBuffer(GLES32.GL_ARRAY_BUFFER, gLBuffer[0]);
        // Transfer data from RAM to the GRAM.
        GLES32.glBufferData(GLES32.GL_ARRAY_BUFFER, vertexCount * BYTES_PER_FLOAT, vertexBuffer, GLES32.GL_STATIC_DRAW);
        // IMPORTANT: Unbind from the buffer
        GLES32.glBindBuffer(GLES32.GL_ARRAY_BUFFER,0);
        gLBufferIndex = gLBuffer[0];

        //Release memory
        vertexBuffer = null;

    }

    /**
     * Called when the surface changed size.
     * <p>
     * Called after the surface is created and whenever
     * the OpenGL ES surface size changes.
     * <p>
     * Typically you will set your viewport here. If your camera
     * is fixed then you could also set your projection matrix here:
     * @param gl the GL interface. Use <code>instanceof</code> to
     * test if the interface supports GL11 or higher interfaces.
     * @param width
     * @param height
     */

    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        GLES32.glViewport(0, 0, width, height); //Initiate

        ratio =  (float) width / (float) height  ; //Calculate screen ratio

        Matrix.frustumM(mProjectionMatrix, 0, -ratio * bot, ratio * bot, bot, top, near, far); //Set our viewing range

        Matrix.setLookAtM(mViewMatrix, 0,eyeX, eyeY, eyeZ, centerX, centerY, centerZ, upX, upY, upZ); //Set our eye position and focus point
    }


    /**
     * Called to draw the current frame.
     * <p>
     * This method is responsible for drawing the current frame.
     * <p>
     * @param gl the GL interface. Use <code>instanceof</code> to
     * test if the interface supports GL11 or higher interfaces.
     */
    @Override
    public void onDrawFrame(GL10 gl) {
        // Change viewing range by detecting change in two finger zoom event
        Matrix.frustumM(mProjectionMatrix, 0, -ratio * bot /fscale, ratio * bot /fscale, bot/fscale, top/fscale, near, far);
        // Translate eye focus point by detecting change in two finger drag event
        Matrix.setLookAtM(mViewMatrix, 0, eyeX, eyeY, eyeZ, centerX - fdXTranslation, centerY + fdYTranslation, centerZ, upX, upY, upZ);

        GLES32.glClear(GLES32.GL_COLOR_BUFFER_BIT);
        GLES32.glUseProgram(iProgrammeID);
        GLES32.glBindBuffer(GLES32.GL_ARRAY_BUFFER,gLBufferIndex);
        GLES32.glVertexAttribPointer(iPosition, 3, GLES32.GL_FLOAT, false, 0, 0);
        GLES32.glEnableVertexAttribArray(iPosition);
        // Set a matrix that contains the current rotation.
        Matrix.setIdentityM(mCurrentRotation, 0);
        Matrix.translateM(mCurrentRotation,0,-0.2f,0f,1.3f);
        Matrix.rotateM(mCurrentRotation, 0, -fdXRotation, 0f, 1f, 0f);
        Matrix.rotateM(mCurrentRotation, 0, -fdYRotation, 1f, 0f, 0f);
        Matrix.rotateM(mCurrentRotation, 0, fdZRotation, 0f, 0f, 1f);
        Matrix.translateM(mCurrentRotation,0,0.2f,0f,-1.3f);
        fdYRotation = 0f;
        fdXRotation = 0f;
        fdZRotation = 0f;

        // Multiply the current rotation by the accumulated rotation, and then set the accumulated rotation to the result.
        Matrix.multiplyMM(mTemporary, 0, mCurrentRotation, 0, mAccumulatedRotation, 0);
        System.arraycopy(mTemporary, 0, mAccumulatedRotation, 0, 16);
        // Multiply our eye info
        Matrix.multiplyMM(mTemporary,0, mViewMatrix,0,mAccumulatedRotation,0);
        // Multiply view range info
        Matrix.multiplyMM(mTemporary,0, mProjectionMatrix,0, mTemporary,0);
        // Set final result
        GLES32.glUniformMatrix4fv(iMVPMatrix, 1, false, mTemporary, 0);

        GLES32.glBindBuffer(GLES32.GL_ARRAY_BUFFER,0);

        // Draw as many point as we have
        GLES32.glDrawArrays(GLES32.GL_POINTS, 0, vertexCount);

    }



}
