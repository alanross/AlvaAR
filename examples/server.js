import fs from 'fs';
import https from 'https';
import path from 'path';
import ip from 'ip';
import { Server } from 'socket.io';

const PORT = 443;
const FOLDER = './public/';

const mimeTable = new Map( [
    ['.html', 'text/html'],
    ['.css', 'text/css'],
    ['.jpg', 'image/jpeg'],
    ['.jpeg', 'image/jpeg'],
    ['.png', 'image/png'],
    ['.ico', 'image/x-icon'],
    ['.svg', 'text/plain'],
    ['.js', 'application/javascript'],
    ['.json', 'application/json'],
    ['.woff', 'application/octet-stream'],
    ['.ttf', 'application/octet-stream'],
    ['.data', 'application/octet-stream'],
    ['.obj', 'application/octet-stream'],
    ['.fbx', 'application/octet-stream'],
    ['.gltf', 'application/octet-stream'],
    ['.bin', 'application/octet-stream'],
    ['.mp4', 'video/mp4']
] );

const credentials = {
    key: fs.readFileSync( 'ssl/key.pem' ),
    cert: fs.readFileSync( 'ssl/cert.pem' )
};

class HttpsServer
{
    static requestListener( request, response )
    {
        // set CORS headers
        response.setHeader( 'Access-Control-Request-Method', '*' );
        response.setHeader( 'Access-Control-Allow-Origin', '*' );
        response.setHeader( 'Access-Control-Allow-Methods', 'OPTIONS, GET' );
        response.setHeader( 'Access-Control-Allow-Headers', '*' );
        // response.setHeader('Access-Control-Allow-Headers', 'authorization, content-type');
        // response.setHeader('Access-Control-Allow-Headers', request.header.origin );
        response.setHeader( 'Cross-Origin-Opener-Policy', 'same-origin' );
        response.setHeader( 'Cross-Origin-Embedder-Policy', 'require-corp' );

        if( request.method === 'GET' )
        {
            const q = request.url.indexOf( '?' );
            const fileUrl = request.url.substring( 0, q !== -1 ? q : request.url.length );
            const filePath = path.resolve( `${ FOLDER }${ fileUrl + (fileUrl.match( /\/$/ ) ? 'index.html' : '') }` );
            const fileExtension = path.extname( filePath );
            const mimeType = mimeTable.get( fileExtension );

            if( !mimeType )
            {
                console.log( "Unknown mime-type. Url: ", request.url );
                HttpsServer.sendUnknownMimeType( response, fileExtension );
                return;
            }

            fs.access( filePath, fs.constants.F_OK | fs.constants.R_OK, ( error ) =>
            {
                if( error )
                {
                    HttpsServer.sendFileNotFound( response );
                    return;
                }

                HttpsServer.sendFile( request, response, filePath, mimeType );
            } );
        }
    }

    static sendUnknownMimeType( response, fileExt )
    {
        response.writeHead( 500, { 'Content-Type': 'text/plain' } );
        response.write( `Error 500: Unknown MIME type for file extension: ${ fileExt }` );
        response.end();
    }

    static sendFileNotFound( response )
    {
        response.writeHead( 404, { 'Content-Type': 'text/plain' } );
        response.write( 'Error 404: Resource not found.' );
        response.end();
    }

    static sendFile( request, response, filePath, mimeType )
    {
        const stat = fs.statSync( filePath );
        const fileSize = stat.size;
        const range = request.headers.range;

        if( range )
        {
            const parts = range.replace( /bytes=/, '' ).split( '-' );
            const start = parseInt( parts[0], 10 );
            const end = parts[1] ? parseInt( parts[1], 10 ) : fileSize - 1;
            const chunksize = (end - start) + 1;
            const fileStream = fs.createReadStream( filePath, {
                start,
                end
            } );

            const head = {
                'Content-Range': `bytes ${ start }-${ end }/${ fileSize }`,
                'Accept-Ranges': 'bytes',
                'Content-Length': chunksize,
                'Content-Type': mimeType
            };

            response.writeHead( 206, head );
            fileStream.pipe( response );
        }
        else
        {
            const head = {
                'Content-Length': fileSize,
                'Content-Type': mimeType
            };

            response.writeHead( 200, head );
            fs.createReadStream( filePath ).pipe( response );
        }
    }
}

const server = https.createServer( credentials, HttpsServer.requestListener ).listen( PORT, () =>
{
    const url = `https://${ ip.address() }:${ PORT }`;

    console.log( `Server running at: \x1b[36m${ url }\x1b[0m` );
} );

const io = new Server( server );
io.on( 'connection', ( socket ) =>
{
    socket.on( 'data', ( data ) => io.emit( 'data', data ) );
} );