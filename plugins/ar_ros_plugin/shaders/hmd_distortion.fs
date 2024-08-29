#version 120

uniform sampler2D rosImageTexture;

void main()
{
    // output_loc is the fragment location on screen from [0,1]x[0,1]
    vec2 output_loc = gl_TexCoord[0].xy;

    gl_FragColor = texture2D(rosImageTexture, output_loc); 
   
}
