import * as THREE from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/build/three.module.js';
import { OrbitControls } from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/examples/jsm/controls/OrbitControls.js';

function applyPose( obj, pose )
{
    const m = new THREE.Matrix4();
    const q = new THREE.Quaternion();
    const t = new THREE.Vector3( pose[12], pose[13], pose[14] );
    const a = new THREE.Quaternion().setFromAxisAngle( new THREE.Vector3( 1, 0, 0 ), 0 ); // axis

    m.fromArray( pose );
    q.setFromRotationMatrix( m );
    q.multiply( a );

    obj.quaternion.set( -q.x, q.y, q.z, q.w );
    obj.position.set( t.x, -t.y, -t.z );
}

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

function getDeviceOrientation()
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
    static async initialize( constraints = null )
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
    static async initialize( url, timeout = 8000 )
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

class ARCamView
{
    constructor( container, width, height, x = 0, y = 0, z = -1, scale = 0.1 )
    {
        this.renderer = new THREE.WebGLRenderer( { antialias: false, alpha: true } );
        this.renderer.setClearColor( 0, 0 );
        this.renderer.setSize( width, height );
        this.renderer.setPixelRatio( window.devicePixelRatio );

        this.camera = new THREE.PerspectiveCamera( 75, width / height, 0.1, 1000 );
        this.camera.rotation.reorder( 'YXZ' );
        this.camera.updateProjectionMatrix();

        this.object = new THREE.Mesh( new THREE.IcosahedronGeometry( 1, 0 ), new THREE.MeshNormalMaterial( { flatShading: true } ) );
        this.object.scale.set( scale, scale, scale );
        this.object.position.set( x, y, z );
        this.object.visible = false

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0x808080 ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.camera );
        this.scene.add( this.object );

        const render = () =>
        {
            this.renderer.render( this.scene, this.camera );

            window.requestAnimationFrame( render.bind( this ) );
        }

        container.appendChild( this.renderer.domElement );
        window.requestAnimationFrame( render.bind( this ) );
    }

    updateCameraPose( pose )
    {
        applyPose( this.camera, pose );

        this.object.visible = true;
    }

    lostCamera()
    {
        this.object.visible = false;
    }
}

class ARSimpleView
{
    constructor( container, width, height, mapView = null )
    {
        this.renderer = new THREE.WebGLRenderer( { antialias: false, alpha: true } );
        this.renderer.setClearColor( 0, 0 );
        this.renderer.setSize( width, height );
        this.renderer.setPixelRatio( window.devicePixelRatio );

        this.camera = new THREE.PerspectiveCamera( 75, width / height, 0.1, 1000 );
        this.camera.rotation.reorder( 'YXZ' );
        this.camera.updateProjectionMatrix();

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0x808080 ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.camera );

        this.body = document.body;

        container.appendChild( this.renderer.domElement );

        if( mapView )
        {
            this.mapView = mapView;
            this.mapView.camHelper = new THREE.CameraHelper( this.camera );
            this.mapView.scene.add( this.mapView.camHelper );
        }
    }

    updateCameraPose( pose )
    {
        applyPose( this.camera, pose );

        this.renderer.render( this.scene, this.camera );

        this.body.classList.add( "tracking" );
    }

    lostCamera()
    {
        this.body.classList.remove( "tracking" );
    }

    createObjectWithPose( pose, scale = 0.5 )
    {
        const obj = new THREE.Mesh( new THREE.IcosahedronGeometry( 1, 0 ), new THREE.MeshNormalMaterial( { flatShading: true } ) );
        obj.scale.set( scale, scale, scale );

        applyPose( obj, pose );

        this.scene.add( obj );

        if( this.mapView )
        {
            const clone = obj.clone();

            clone.scale.set( scale, scale, scale );

            this.mapView.scene.add( clone );
        }
    }
}

class ARSimpleMap
{
    constructor( container, width, height )
    {
        this.renderer = new THREE.WebGLRenderer( { antialias: false } );
        this.renderer.setClearColor( new THREE.Color( 'rgb(255, 255, 255)' ) );
        this.renderer.setPixelRatio( window.devicePixelRatio );
        this.renderer.setSize( width, height, false );
        this.renderer.domElement.style.width = width + 'px';
        this.renderer.domElement.style.height = height + 'px';

        this.camera = new THREE.PerspectiveCamera( 50, width / height, 0.01, 1000 );
        this.camera.position.set( -1, 2, 2 );

        this.controls = new OrbitControls( this.camera, this.renderer.domElement, );
        this.controls.enableDamping = true;
        this.controls.dampingFactor = 0.1;
        this.controls.minDistance = 0.1;
        this.controls.maxDistance = 1000;

        this.gridHelper = new THREE.GridHelper( 150, 100 );
        this.gridHelper.position.y = -1;

        this.axisHelper = new THREE.AxesHelper( 0.25 );

        this.camHelper = null;

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0xefefef ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.gridHelper );
        this.scene.add( this.axisHelper );

        container.appendChild( this.renderer.domElement );

        const render = () =>
        {
            this.controls.update();
            this.renderer.render( this.scene, this.camera );

            requestAnimationFrame( render );
        }

        render();
    }
}

export { ARCamView, ARSimpleView, ARSimpleMap, Camera, Video, onFrame, isMobile, getDeviceOrientation, resize2cover }
