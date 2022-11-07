const rad2deg = 180.0 / Math.PI;
const deg2rad = Math.PI / 180;

function onFrame( frameTickFn, fps = 30 )
{
    const fpsInterval = ~~(1000 / fps);

    let t1 = performance.now();

    const onAnimationFrame = async () =>
    {
        const t2 = performance.now();
        const td = t2 - t1;

        if( td > fpsInterval )
        {
            t1 = t2 - (td % fpsInterval);

            if( (await frameTickFn( t2 )) === false )
            {
                return;
            }
        }

        requestAnimationFrame( onAnimationFrame );
    };

    requestAnimationFrame( onAnimationFrame );
}

function isIOS()
{
    return /iPad|iPhone|iPod/.test( navigator.platform );
}

function isMobile()
{
    try
    {
        document.createEvent( 'TouchEvent' );
        return true;
    } catch( e )
    {
        return false;
    }
}

function getScreenOrientation()
{
    let angle = -1;

    if( window.screen && window.screen.orientation )
    {
        angle = window.screen.orientation.angle;
    }
    else if( 'orientation' in window )
    {
        angle = window.orientation;
    }

    switch( angle )
    {
        case 0:
            return 'portrait';
        case 90:
            return 'landscape_left';
        case 180:
            return 'portrait';
        case 270:
            return 'landscape_right';
        case -90:
            return 'landscape_right';
    }

    return 'unknown';
}

function resize2cover( srcW, srcH, dstW, dstH )
{
    const rect = {};

    if( dstW / dstH > srcW / srcH )
    {
        const scale = dstW / srcW;
        rect.width = ~~(scale * srcW);
        rect.height = ~~(scale * srcH);
        rect.x = 0;
        rect.y = ~~((dstH - rect.height) * 0.5);
    }
    else
    {
        const scale = dstH / srcH;
        rect.width = ~~(scale * srcW);
        rect.height = ~~(scale * srcH);
        rect.x = ~~((dstW - rect.width) * 0.5);
        rect.y = 0;
    }

    return rect;
}

function createCanvas( width, height )
{
    const canvas = document.createElement( 'canvas' );

    canvas.width = width;
    canvas.height = height;

    return canvas;
}

class Camera
{
    static async Initialize( constraints = null )
    {
        if( 'facingMode' in constraints && 'deviceId' in constraints )
        {
            throw new Error( `Camera settings 'deviceId' and 'facingMode' are mutually exclusive.` );
        }

        if( 'facingMode' in constraints && ['environment', 'user'].indexOf( constraints.facingMode ) === -1 )
        {
            throw new Error( `Camera settings 'facingMode' can only be 'environment' or 'user'.` );
        }

        const setupUserMediaStream = ( permission ) =>
        {
            return new Promise( ( resolve, reject ) =>
            {
                const onSuccess = ( stream ) =>
                {
                    const track = stream.getVideoTracks()[0];

                    if( typeof track === 'undefined' )
                    {
                        reject( new Error( `Failed to access camera: Permission denied (No track).` ) );
                    }
                    else
                    {
                        const video = document.createElement( 'video' );

                        video.setAttribute( 'autoplay', 'autoplay' );
                        video.setAttribute( 'playsinline', 'playsinline' );
                        video.setAttribute( 'webkit-playsinline', 'webkit-playsinline' );
                        video.srcObject = stream;

                        video.onloadedmetadata = () =>
                        {
                            const settings = track.getSettings();

                            const tw = settings.width;
                            const th = settings.height;
                            const vw = video.videoWidth;
                            const vh = video.videoHeight;

                            if( vw !== tw || vh !== th )
                            {
                                console.warn( `Video dimensions mismatch: width: ${ tw }/${ vw }, height: ${ th }/${ vh }` );
                            }

                            video.style.width = vw + 'px';
                            video.style.height = vh + 'px';
                            video.width = vw;
                            video.height = vh;
                            video.play();

                            resolve( new Camera( video ) );
                        };
                    }
                };

                const onFailure = ( error ) =>
                {
                    switch( error.name )
                    {
                        case 'NotFoundError':
                        case 'DevicesNotFoundError':
                            reject( new Error( `Failed to access camera: Camera not found.` ) );
                            return;
                        case 'SourceUnavailableError':
                            reject( new Error( `Failed to access camera: Camera busy.` ) );
                            return;
                        case 'PermissionDeniedError':
                        case 'SecurityError':
                            reject( new Error( `Failed to access camera: Permission denied.` ) );
                            return;
                        default:
                            reject( new Error( `Failed to access camera: Rejected.` ) );
                            return;
                    }
                };

                if( permission && permission.state === 'denied' )
                {
                    reject( new Error( `Failed to access camera: Permission denied.` ) );
                    return;
                }

                navigator.mediaDevices.getUserMedia( constraints ).then( onSuccess ).catch( onFailure );
            } );
        };

        if( navigator.permissions && navigator.permissions.query )
        {
            return navigator.permissions
            .query( { name: 'camera' } )
            .then( ( permission ) =>
            {
                return setupUserMediaStream( permission );
            } )
            .catch( error =>
            {
                return setupUserMediaStream();
            } );
        }
        else
        {
            return setupUserMediaStream();
        }
    }

    constructor( videoElement )
    {
        this.el = videoElement;
        this.width = videoElement.videoWidth;
        this.height = videoElement.videoHeight;

        this._canvas = createCanvas( this.width, this.height );
        this._ctx = this._canvas.getContext( '2d', { willReadFrequently: true } );
    }

    getImageData()
    {
        this._ctx.clearRect( 0, 0, this.width, this.height );
        this._ctx.drawImage( this.el, 0, 0, this.width, this.height );

        return this._ctx.getImageData( 0, 0, this.width, this.height );
    }
}

class Video
{
    static async Initialize( url, timeout = 8000 )
    {
        return new Promise( ( resolve, reject ) =>
        {
            let tid = -1;

            const video = document.createElement( 'video' );

            video.src = url;
            video.setAttribute( 'autoplay', 'autoplay' );
            video.setAttribute( 'playsinline', 'playsinline' );
            video.setAttribute( 'webkit-playsinline', 'webkit-playsinline' );
            video.autoplay = true;
            video.muted = true;
            video.loop = true; // note: if loop is true, ended event will not fire
            video.load();

            tid = setTimeout( () =>
            {
                reject( new Error( `Failed to load video: Timed out after ${ timeout }ms.` ) );
            }, timeout );

            video.onerror = () =>
            {
                clearTimeout( tid );

                reject( new Error( `Failed to load video.` ) );
            };

            video.onabort = () =>
            {
                clearTimeout( tid );

                reject( new Error( `Failed to load video: Load aborted.` ) );
            };

            if( video.readyState >= 4 )
            {
                clearTimeout( tid );

                resolve( video );
            }
            else
            {
                video.oncanplaythrough = () =>
                {
                    clearTimeout( tid );

                    if( video.videoWidth === 0 || video.videoHeight === 0 )
                    {
                        reject( new Error( `Failed to load video: Invalid dimensions.` ) );
                    }
                    else
                    {
                        resolve( video );
                    }
                };
            }
        } ).then( video =>
        {
            video.onload = video.onabort = video.onerror = null;

            return new Video( video );
        } );
    }

    constructor( videoElement )
    {
        this.el = videoElement;
        this.width = videoElement.videoWidth;
        this.height = videoElement.videoHeight;

        this._canvas = createCanvas( this.width, this.height );
        this._ctx = this._canvas.getContext( '2d', { willReadFrequently: true } );

        this._lastTime = -1;
        this._imageData = null;
    }

    getImageData()
    {
        const t = this.el.currentTime;

        if( this._lastTime !== t )
        {
            this._lastTime = t;

            this._imageData = null;
        }

        if( this._imageData === null )
        {
            this._ctx.clearRect( 0, 0, this.width, this.height );
            this._ctx.drawImage( this.el, 0, 0, this.width, this.height );

            this._imageData = this._ctx.getImageData( 0, 0, this.width, this.height );
        }

        return this._imageData;
    }
}

export { Camera, Video, onFrame, isMobile, isIOS, getScreenOrientation, resize2cover, rad2deg, deg2rad }