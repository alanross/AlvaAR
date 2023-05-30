import { deg2rad, getScreenOrientation, isIOS } from "./utils.js";

class Quaternion
{
    static fromAxisAngle( axisX = 0, axisY = 0, axisZ = 0, angle = 0 )
    {
        const angle2 = angle / 2;
        const s = Math.sin( angle2 );

        return {
            x: axisX * s,
            y: axisY * s,
            z: axisZ * s,
            w: Math.cos( angle2 )
        };
    }

    static fromEuler( x = 0, y = 0, z = 0, order = 'XYZ' )
    {
        const cos = Math.cos;
        const sin = Math.sin;

        const c1 = cos( x / 2 );
        const c2 = cos( y / 2 );
        const c3 = cos( z / 2 );

        const s1 = sin( x / 2 );
        const s2 = sin( y / 2 );
        const s3 = sin( z / 2 );

        const q = { x: 0, y: 0, z: 0, w: 1 };

        switch( order )
        {
            case 'XYZ':
                q.x = s1 * c2 * c3 + c1 * s2 * s3;
                q.y = c1 * s2 * c3 - s1 * c2 * s3;
                q.z = c1 * c2 * s3 + s1 * s2 * c3;
                q.w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'YXZ':
                q.x = s1 * c2 * c3 + c1 * s2 * s3;
                q.y = c1 * s2 * c3 - s1 * c2 * s3;
                q.z = c1 * c2 * s3 - s1 * s2 * c3;
                q.w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            case 'ZXY':
                q.x = s1 * c2 * c3 - c1 * s2 * s3;
                q.y = c1 * s2 * c3 + s1 * c2 * s3;
                q.z = c1 * c2 * s3 + s1 * s2 * c3;
                q.w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'ZYX':
                q.x = s1 * c2 * c3 - c1 * s2 * s3;
                q.y = c1 * s2 * c3 + s1 * c2 * s3;
                q.z = c1 * c2 * s3 - s1 * s2 * c3;
                q.w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            case 'YZX':
                q.x = s1 * c2 * c3 + c1 * s2 * s3;
                q.y = c1 * s2 * c3 + s1 * c2 * s3;
                q.z = c1 * c2 * s3 - s1 * s2 * c3;
                q.w = c1 * c2 * c3 - s1 * s2 * s3;
                break;

            case 'XZY':
                q.x = s1 * c2 * c3 - c1 * s2 * s3;
                q.y = c1 * s2 * c3 - s1 * c2 * s3;
                q.z = c1 * c2 * s3 + s1 * s2 * c3;
                q.w = c1 * c2 * c3 + s1 * s2 * s3;
                break;

            default:
                console.warn( 'CreateFromEuler() encountered an unknown order: ' + order );
        }

        return q;
    }

    static multiply( a, b )
    {
        const qax = a.x, qay = a.y, qaz = a.z, qaw = a.w;
        const qbx = b.x, qby = b.y, qbz = b.z, qbw = b.w;

        return {
            x: qax * qbw + qaw * qbx + qay * qbz - qaz * qby,
            y: qay * qbw + qaw * qby + qaz * qbx - qax * qbz,
            z: qaz * qbw + qaw * qbz + qax * qby - qay * qbx,
            w: qaw * qbw - qax * qbx - qay * qby - qaz * qbz,
        }
    }

    static dot( a, b )
    {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }
}

class IMU
{
    static Initialize()
    {
        return new Promise( ( resolve, reject ) =>
        {
            const finalize = () =>
            {
                if( window.isSecureContext === false )
                {
                    reject( "DeviceOrientation is only available in secure contexts (https)." );
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

                resolve( new IMU() );
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
        this.EPS = 0.000001;

        this.screenOrientation = null;
        this.screenOrientationAngle = 0;

        this.motion = [];

        this.orientation = { x: 1, y: 0, z: 0, w: 1 };
        this.worldTransform = isIOS()
            ? Quaternion.fromAxisAngle( 1, 0, 0, -Math.PI / 2 ) // -90 degrees on x-axis
            : Quaternion.fromAxisAngle( 0, 1, 0, Math.PI / 2 ); // 90 degrees on y-axis

        const handleDeviceOrientation = ( event ) =>
        {
            // axis orientation assumes device is placed on ground, screen upward
            const x = event.beta * deg2rad;    // X-axis (β) vertical tilt
            const y = event.gamma * deg2rad;   // Y-axis (γ) horizontal tilt
            const z = event.alpha * deg2rad;   // Z-axis (α) compass direction

            const orientation = Quaternion.multiply( this.worldTransform, Quaternion.fromEuler( x, y, z, 'ZXY' ) );

            if( 8 * (1 - Quaternion.dot( this.orientation, orientation )) > this.EPS )
            {
                this.orientation = orientation;
            }
        }

        const handleDeviceMotion = ( event ) =>
        {
            const gx = event.rotationRate.beta * deg2rad;   // X-axis (β) deg to rad: rad/s
            const gy = event.rotationRate.gamma * deg2rad;  // Y-axis (γ) deg to rad: rad/s
            const gz = event.rotationRate.alpha * deg2rad;  // Z-axis (α) deg to rad: rad/s

            const ax = event.acceleration.x; // (m/s^2)
            const ay = event.acceleration.y; // (m/s^2)
            const az = event.acceleration.z; // (m/s^2)

            const timestamp = Date.now();

            this.motion.push( { timestamp, gx, gy, gz, ax, ay, az } );
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

    clear()
    {
        this.motion.length = 0;
    }
}

export { IMU }