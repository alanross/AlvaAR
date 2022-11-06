import * as THREE from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/build/three.module.js';
import { OrbitControls } from 'https://threejsfundamentals.org/threejs/resources/threejs/r132/examples/jsm/controls/OrbitControls.js';
import { AlvaARConnectorTHREE } from './alva_ar_three.js'

class ARCamView
{
    constructor( container, width, height, x = 0, y = 0, z = -1, scale = 0.1 )
    {
        this.applyPose = AlvaARConnectorTHREE.Initialize( THREE );

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
        this.object.visible = false;

        this.box = new THREE.Mesh( new THREE.BoxGeometry( 100, 100, 100, 4, 4, 4 ), new THREE.MeshBasicMaterial( { color: 0xff00ff, wireframe: true, opacity: 0.9 } ) );
        this.box.visible = false;

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0x808080 ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.camera );
        this.scene.add( this.object );
        this.scene.add( this.box );

        container.appendChild( this.renderer.domElement );

        const render = () =>
        {
            requestAnimationFrame( render.bind( this ) );

            this.renderer.render( this.scene, this.camera );
        }

        render();
    }

    updateCameraPose( pose )
    {
        this.applyPose( this.camera, pose );

        this.object.visible = true;
        this.box.visible = true;
    }

    lostCamera()
    {
        this.object.visible = false;
        this.box.visible = false;
    }
}

class ARIMUView
{
    constructor( container, width, height, x = 0, y = 0, z = -1, scale = 0.1 )
    {
        this.renderer = new THREE.WebGLRenderer( { antialias: false, alpha: true } );
        this.renderer.setClearColor( 0, 0 );
        this.renderer.setSize( width, height );
        this.renderer.setPixelRatio( window.devicePixelRatio );

        this.reticle = new THREE.Mesh( new THREE.RingBufferGeometry( 0.005, 0.01, 15 ), new THREE.MeshBasicMaterial( { color: 0xffffff } ) );
        this.reticle.position.z = -0.5;

        this.camera = new THREE.PerspectiveCamera( 75, width / height, 0.1, 1000 );
        this.camera.rotation.reorder( 'YXZ' );
        this.camera.updateProjectionMatrix();
        this.camera.add( this.reticle );

        this.box = new THREE.Mesh( new THREE.BoxGeometry( 100, 100, 100, 4, 4, 4 ), new THREE.MeshBasicMaterial( { color: 0xffff00, wireframe: true, opacity: 0.9 } ) );
        this.box.visible = false;

        this.scene = new THREE.Scene();
        this.scene.add( new THREE.AmbientLight( 0x808080 ) );
        this.scene.add( new THREE.HemisphereLight( 0x404040, 0xf0f0f0, 1 ) );
        this.scene.add( this.camera );
        this.scene.add( this.box );

        container.appendChild( this.renderer.domElement );

        const render = () =>
        {
            requestAnimationFrame( render.bind( this ) );

            this.renderer.render( this.scene, this.camera );
        }

        render();
    }

    updateCameraRotation( alpha, beta, gamma, screenAngle )
    {
        const orient = THREE.MathUtils.degToRad( screenAngle );

        const axis = new THREE.Vector3( 0, 0, 1 );
        const euler = new THREE.Euler();
        const q0 = new THREE.Quaternion();
        const q1 = new THREE.Quaternion( -Math.sqrt( 0.5 ), 0, 0, Math.sqrt( 0.5 ) ); // - PI/2 around the x-axis
        const EPS = 0.000001;

        // 'ZXY' for the device, but 'YXZ' for us
        euler.set( beta, alpha, -gamma, 'YXZ' );

        // orient the device
        this.camera.quaternion.setFromEuler( euler );

        // camera looks out the back of the device, not the top
        this.camera.quaternion.multiply( q1 );

        // adjust for screen orientation
        this.camera.quaternion.multiply( q0.setFromAxisAngle( axis, -orient ) );

        this.box.visible = true;

        this.prevQuaternion = this.prevQuaternion || new THREE.Quaternion();

        if( 8 * (1 - this.prevQuaternion.dot( this.camera.quaternion )) > EPS )
        {
            this.prevQuaternion.copy( this.camera.quaternion );
        }
    }

    lostCamera()
    {
        this.box.visible = false;
    }
}

class ARSimpleView
{
    constructor( container, width, height, mapView = null )
    {
        this.applyPose = AlvaARConnectorTHREE.Initialize( THREE );

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
        this.applyPose( this.camera, pose );

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

        this.applyPose( obj, pose );

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

export { ARCamView, ARSimpleView, ARSimpleMap, ARIMUView }