import { isIOS, getScreenOrientation, isMobile, deg2rad } from "./utils.js";

class DeviceOrientationSensor
{
    static Initialize()
    {
        return new Promise( ( resolve, reject ) =>
        {
            const finalize = () =>
            {
                if( window.isSecureContext === false )
                {
                    reject( 'DeviceOrientation is only available in secure contexts (https).' );
                    return;
                }

                if( window.DeviceOrientationEvent === undefined )
                {
                    reject( "DeviceOrientation not supported." );
                    return;
                }

                if( window.DeviceMotionEvent === undefined )
                {
                    reject( "DeviceMotion not supported." );
                    return;
                }

                resolve( new DeviceOrientationSensor() );
            }

            if( window.DeviceMotionEvent !== undefined && typeof window.DeviceMotionEvent.requestPermission === 'function' )
            {
                window.DeviceMotionEvent.requestPermission().then( state =>
                {
                    if( state === "granted" )
                    {
                        finalize();
                    }
                    else
                    {
                        reject( "Permission denied by user." );
                    }
                }, error =>
                {
                    reject( error.toString() );
                } );
            }
            else if( window.ondevicemotion !== undefined )
            {
                finalize();
            }
            else
            {
                reject( "DeviceMotion is not supported." );
            }
        } );
    }

    constructor()
    {
        this.isIOS = isIOS();
        this.isMobile = isMobile();

        this.screenOrientation = null;
        this.screenOrientationAngle = 0;

        this.deviceOrientationTimestamp = 0;
        this.deviceOrientationAlpha = 0;
        this.deviceOrientationBeta = 0;
        this.deviceOrientationGamma = 0;

        this.deviceMotionTimestamp = 0;
        this.deviceMotionInterval = 0;
        this.deviceMotionAccelerationX = 0;
        this.deviceMotionAccelerationY = 0;
        this.deviceMotionAccelerationZ = 0;
        this.deviceMotionAccelerationGX = 0;
        this.deviceMotionAccelerationGY = 0;
        this.deviceMotionAccelerationGZ = 0;
        this.deviceMotionRotationAlpha = 0;
        this.deviceMotionRotationBeta = 0;
        this.deviceMotionRotationGamma = 0;

        const handleDeviceOrientation = ( event ) =>
        {
            this.deviceOrientationAlpha = (event.alpha || 0) * deg2rad;  // compass direction
            this.deviceOrientationBeta = (event.beta || 0) * deg2rad;    // vertical tilt
            this.deviceOrientationGamma = (event.gamma || 0) * deg2rad;  // horizontal tilt
            this.deviceOrientationTimestamp = event.timeStamp;
        }

        const handleDeviceMotion = ( event ) =>
        {
            this.deviceMotionInterval = this.isIOS ? ~~(event.interval * 1000) : event.interval;
            this.deviceMotionTimestamp = event.timeStamp;

            this.deviceMotionAccelerationX = event.acceleration.x;
            this.deviceMotionAccelerationY = event.acceleration.y;
            this.deviceMotionAccelerationZ = event.acceleration.z;

            this.deviceMotionAccelerationGX = event.accelerationIncludingGravity.x;
            this.deviceMotionAccelerationGY = event.accelerationIncludingGravity.y;
            this.deviceMotionAccelerationGZ = event.accelerationIncludingGravity.z;

            this.deviceMotionRotationAlpha = (event.rotationRate.alpha || 0) * deg2rad;
            this.deviceMotionRotationBeta = (event.rotationRate.beta || 0) * deg2rad;
            this.deviceMotionRotationGamma = (event.rotationRate.gamma || 0) * deg2rad;
        }

        const handleScreenOrientation = ( event ) =>
        {
            this.screenOrientation = getScreenOrientation();

            if( this.screenOrientation === 'landscape_left' )
            {
                this.screenOrientationAngle = 90;
            }
            else if( this.screenOrientation === 'landscape_right' )
            {
                this.screenOrientationAngle = 270;
            }
            else
            {
                this.screenOrientationAngle = 0;
            }
        }

        window.addEventListener( 'devicemotion', handleDeviceMotion.bind( this ), false );
        window.addEventListener( 'deviceorientation', handleDeviceOrientation.bind( this ), false );
        window.addEventListener( 'orientationchange', handleScreenOrientation.bind( this ), false );
    }
}

export { DeviceOrientationSensor }