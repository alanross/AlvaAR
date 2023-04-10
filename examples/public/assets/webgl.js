class WebGL2
{
    static createShader( gl, shaderSource, shaderType )
    {
        const shader = gl.createShader( shaderType );

        gl.shaderSource( shader, shaderSource );
        gl.compileShader( shader );

        if( !gl.getShaderParameter( shader, gl.COMPILE_STATUS ) )
        {
            const errorLast = gl.getShaderInfoLog( shader );
            const errorInfo = shaderSource.split( '\n' ).map( ( l, i ) => `${ i + 1 }: ${ l }` ).join( '\n' );

            gl.deleteShader( shader );

            throw new Error( `Error compiling WebGL shaders: '${ shader }': ${ errorLast }\n ${ errorInfo }` );
        }

        return shader;
    }

    static createProgram( gl, vertShaderSource, fragShaderSource )
    {
        const vertShader = WebGL2.createShader( gl, vertShaderSource, gl.VERTEX_SHADER );
        const fragShader = WebGL2.createShader( gl, fragShaderSource, gl.FRAGMENT_SHADER );

        const program = gl.createProgram();

        gl.attachShader( program, fragShader );
        gl.attachShader( program, vertShader );

        gl.linkProgram( program );
        gl.validateProgram( program ); // for debugging

        if( !gl.getProgramParameter( program, gl.LINK_STATUS ) )
        {
            gl.deleteProgram( program );
            gl.deleteShader( fragShader );
            gl.deleteShader( vertShader );

            throw new Error( 'Error linking WebGL program: ' + gl.getProgramInfoLog( program ) );
        }

        return program;
    }

    static createTexture( gl, width, height, type, flipY = false, useNearest = false )
    {
        const texId = gl.createTexture();

        gl.bindTexture( gl.TEXTURE_2D, texId );
        gl.pixelStorei( gl.UNPACK_FLIP_Y_WEBGL, flipY );

        gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE );
        gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE );
        gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, useNearest ? gl.NEAREST : gl.LINEAR );
        gl.texParameteri( gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, useNearest ? gl.NEAREST : gl.LINEAR );

        const textureType = type || gl.UNSIGNED_BYTE;
        const internalFormat = textureType === gl.FLOAT ? gl.RGBA32F : gl.RGBA;

        gl.texImage2D( gl.TEXTURE_2D, 0, internalFormat, width, height, 0, gl.RGBA, type, null );

        return texId;
    }

    static createFrameBuffer( gl, width, height, textureType = undefined, textureFlipY = false, textureUseNearest = false )
    {
        const tex = WebGL2.createTexture( gl, width, height, textureType, textureFlipY, textureUseNearest );
        const fbo = gl.createFramebuffer();

        gl.bindFramebuffer( gl.FRAMEBUFFER, fbo );
        gl.framebufferTexture2D( gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex, 0 );

        if( gl.checkFramebufferStatus( gl.FRAMEBUFFER ) !== gl.FRAMEBUFFER_COMPLETE )
        {
            throw new Error( 'Failed to bind FrameBuffer and attach texture.' );
        }

        return {
            tex: tex,
            fbo: fbo
        };
    }

    static getWebGLInfo( gl )
    {
        const info = {};

        info['WebGL renderer'] = gl.getParameter( gl.RENDERER );
        info['WebGL vendor'] = gl.getParameter( gl.VENDOR );
        info['WebGL version'] = gl.getParameter( gl.VERSION );
        info['Shading language version'] = gl.getParameter( gl.SHADING_LANGUAGE_VERSION );
        info['Unmasked renderer'] = '-';
        info['Unmasked vendor'] = '-';

        const debugInfo = gl.getExtension( 'WEBGL_debug_renderer_info' );

        if( debugInfo )
        {
            info['Unmasked renderer'] = gl.getParameter( debugInfo.UNMASKED_RENDERER_WEBGL );
            info['Unmasked vendor'] = gl.getParameter( debugInfo.UNMASKED_VENDOR_WEBGL );
        }

        return info;
    }

    static async loadShaderSource( url )
    {
        const response = await fetch( url );
        return await response.text();
    }
}

export { WebGL2 }