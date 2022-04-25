uniform mat4 u_MVPMatrix;   // A constant representing the combined model/view/projection matrix.
attribute vec4 a_Position;  // Per-vertex position information we will pass in.
attribute vec4 a_Color;
varying vec4 v_Color;
attribute float a_Size;

void main()                    
{
    v_Color = a_Color;
	gl_Position = u_MVPMatrix * a_Position;   // gl_Position is a special variable used to store the final position. Multiply the vertex by the matrix to get the final point in
    gl_PointSize = a_Size;  // PointCloud Size
}                              