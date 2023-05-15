import fs from 'fs';
import https from 'https';
import express from 'express';
import ip from 'ip';
import { Server } from 'socket.io';

const SERVER_PORT = 443;
const STATIC_FOLDER = './public/';

const app = express();

app.use( ( req, response, next ) =>
{
    // response.setHeader('Access-Control-Allow-Headers', 'authorization, content-type');
    // response.setHeader('Access-Control-Allow-Headers', request.header.origin );
    response.setHeader( 'Access-Control-Allow-Headers', '*' );
    response.setHeader( 'Access-Control-Allow-Origin', '*' );
    response.setHeader( 'Access-Control-Allow-Methods', 'OPTIONS, GET' );
    response.setHeader( 'Access-Control-Request-Method', '*' );
    response.setHeader( 'Cross-Origin-Opener-Policy', 'same-origin' );
    response.setHeader( 'Cross-Origin-Embedder-Policy', 'require-corp' );
    next();
} );

app.use( express.static( STATIC_FOLDER ) );

const httpsServer = https.createServer(
    {
        key: fs.readFileSync( 'ssl/key.pem' ),
        cert: fs.readFileSync( 'ssl/cert.pem' )
    },
    app
);

httpsServer.listen( SERVER_PORT, () =>
{
    const url = `https://${ ip.address() }:${ SERVER_PORT }`;
    console.log( `Server running at: \x1b[36m${ url }\x1b[0m` );
} );

const socketServer = new Server( httpsServer );
socketServer.on( 'connection', ( socket ) =>
{
    socket.on( 'data', ( data ) => socketServer.emit( 'data', data ) );
} );